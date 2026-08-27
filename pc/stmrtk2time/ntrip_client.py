"""Minimal NTRIP v1/v2 client using only the Python standard library."""

from __future__ import annotations

import base64
import socket
from dataclasses import dataclass


class NtripError(RuntimeError):
    pass


class NtripAuthenticationError(NtripError):
    pass


class NtripMountpointError(NtripError):
    pass


class NtripConnectionLost(NtripError):
    pass


@dataclass(frozen=True)
class NtripSettings:
    host: str
    port: int
    mountpoint: str
    username: str
    password: str
    connect_timeout: float = 10.0
    read_timeout: float = 1.0
    chunk_size: int = 4096


class NtripClient:
    def __init__(self, settings: NtripSettings) -> None:
        self.settings = settings
        self._socket: socket.socket | None = None
        self._pending = b""

    def connect(self) -> None:
        self.close()
        sock = socket.create_connection(
            (self.settings.host, self.settings.port),
            timeout=self.settings.connect_timeout,
        )
        sock.settimeout(self.settings.connect_timeout)

        mountpoint = self.settings.mountpoint.lstrip("/")
        credentials = f"{self.settings.username}:{self.settings.password}"
        authorization = base64.b64encode(credentials.encode("utf-8")).decode("ascii")
        request = (
            f"GET /{mountpoint} HTTP/1.1\r\n"
            f"Host: {self.settings.host}:{self.settings.port}\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP rtk-pc-test/1.0\r\n"
            f"Authorization: Basic {authorization}\r\n"
            "Accept: */*\r\n"
            "Connection: close\r\n\r\n"
        ).encode("ascii")
        sock.sendall(request)

        self._socket = sock
        status_line, remainder = self._read_line()
        status_text = status_line.decode("latin-1", errors="replace").strip()

        if " 401 " in f" {status_text} ":
            self.close()
            raise NtripAuthenticationError("authentication failed (HTTP 401)")
        if " 404 " in f" {status_text} ":
            self.close()
            raise NtripMountpointError("mountpoint not found (HTTP 404)")
        if status_text.startswith("SOURCETABLE"):
            self.close()
            raise NtripMountpointError("caster returned its sourcetable")

        is_http_ok = status_text.startswith("HTTP/") and " 200 " in f" {status_text} "
        is_icy_ok = status_text.startswith("ICY 200")
        if not (is_http_ok or is_icy_ok):
            self.close()
            raise NtripError(f"unexpected caster response: {status_text!r}")

        if is_http_ok:
            self._pending = self._read_http_headers(remainder)
        else:
            # NTRIP v1 ICY responses may start the binary stream immediately.
            self._pending = remainder

        sock.settimeout(self.settings.read_timeout)

    def _read_line(self) -> tuple[bytes, bytes]:
        assert self._socket is not None
        buffer = bytearray()
        while len(buffer) < 16_384:
            data = self._socket.recv(1024)
            if not data:
                raise NtripConnectionLost("caster closed during handshake")
            buffer.extend(data)
            newline = buffer.find(b"\n")
            if newline >= 0:
                return bytes(buffer[: newline + 1]), bytes(buffer[newline + 1 :])
        raise NtripError("caster response header is too large")

    def _read_http_headers(self, initial: bytes) -> bytes:
        assert self._socket is not None
        buffer = bytearray(initial)
        while True:
            marker = buffer.find(b"\r\n\r\n")
            marker_length = 4
            if marker < 0:
                marker = buffer.find(b"\n\n")
                marker_length = 2
            if marker >= 0:
                return bytes(buffer[marker + marker_length :])
            if len(buffer) > 65_536:
                raise NtripError("caster response headers are too large")
            data = self._socket.recv(4096)
            if not data:
                raise NtripConnectionLost("caster closed during handshake")
            buffer.extend(data)

    def read(self) -> bytes | None:
        if self._pending:
            data, self._pending = self._pending, b""
            return data
        if self._socket is None:
            raise NtripConnectionLost("NTRIP socket is not connected")
        try:
            data = self._socket.recv(self.settings.chunk_size)
        except socket.timeout:
            return None
        if not data:
            raise NtripConnectionLost("caster closed the RTCM stream")
        return data

    def close(self) -> None:
        sock, self._socket = self._socket, None
        self._pending = b""
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def __enter__(self) -> "NtripClient":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

