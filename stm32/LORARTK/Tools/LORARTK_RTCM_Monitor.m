%% LORARTK LoRa UART Monitor - RSSI/SNR Graph + RTCM Log Viewer
clear; clc; close all;

%% 1) 설정
port = "COM5";        % PG14에 연결한 USB-UART 어댑터 COM 포트
baud = 115200;
windowSec = 30;       % 화면에 보일 시간(초)

useStmTime = false;
showRawLine = false;

%% LoRa 설정값 - STM LoRa 설정과 동일
loraSF = 7;
loraBW = 125000;
loraCRDenom = 5;
loraCRCOn = true;
loraImplicitHeader = false;
loraPreambleLen = 8;
loraLDRO = false;

%% 2) 시리얼 연결
try
    s = serialport(port, baud);
    configureTerminator(s, "LF");
    flush(s);
    disp("연결됨: " + port);
catch ME
    error("포트 연결 실패: %s", ME.message);
end

%% 3) UI 구성
% --- [창 1] 그래프 화면: 기존 코드와 동일 ---
fig1 = figure('Name','LoRa Data Viewer - Graphs', ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Position', [100, 200, 600, 600]);

t = tiledlayout(fig1, 2, 1, 'Padding','compact'); %#ok<NASGU>

recvTextBox = annotation(fig1, 'textbox', [0.1, 0.92, 0.8, 0.07], ...
    'String', 'Waiting for Data...', ...
    'EdgeColor', 'k', ...
    'LineWidth', 2, ...
    'BackgroundColor', '#FFFFDD', ...
    'HorizontalAlignment', 'center', ...
    'FontSize', 12, ...
    'FontWeight', 'bold');

% --- 위쪽 RSSI 그래프 ---
ax1 = nexttile;
hR = animatedline(ax1, 'Color', '#0072BD', 'LineWidth', 1.5);
grid(ax1, 'on');
title(ax1, '배경 노이즈 (RSSI)');
ylabel(ax1, 'dBm');
axis(ax1, 'auto y');

% --- 아래쪽 패킷 RSSI 그래프 ---
ax2 = nexttile;
hold(ax2, 'on');

hOk  = animatedline(ax2, ...
    'LineStyle','none', ...
    'Marker','o', ...
    'MarkerFaceColor','g', ...
    'MarkerEdgeColor','k', ...
    'MarkerSize',8);

hBad = animatedline(ax2, ...
    'LineStyle','none', ...
    'Marker','x', ...
    'Color','r', ...
    'LineWidth',2, ...
    'MarkerSize',10);

grid(ax2, 'on');
title(ax2, '패킷 수신 & 데이터');
ylabel(ax2, 'Packet RSSI (dBm)');
legend(ax2, [hOk, hBad], {'수신 성공', 'CRC 에러'}, 'Location','southeast');
axis(ax2, 'auto y');

% --- [창 2] RTCM 로그 뷰어 화면 ---
fig2 = figure('Name','LORARTK - RTCM Logs', ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Position', [750, 120, 900, 760]);

logListBox = uicontrol(fig2, ...
    'Style', 'listbox', ...
    'Units', 'normalized', ...
    'Position', [0.03 0.03 0.94 0.94], ...
    'FontSize', 10, ...
    'FontName', 'Consolas');

logLines = {};
maxLogLines = 500;

% 같은 SEQ의 A2 fragment를 모아 완전한 RTCM frame으로 복원한다.
rtcmStates = containers.Map('KeyType', 'char', 'ValueType', 'any');
t0 = tic;

%% 4) 메인 루프
while ishandle(fig1) && ishandle(fig2)

    if s.NumBytesAvailable > 0
        try
            lineData = strtrim(readline(s));

            if showRawLine
                disp("RAW: " + lineData);
            end

            if strlength(lineData) < 3
                continue;
            end

            parts = split(lineData, ",");

            if useStmTime && length(parts) >= 2
                t_now = str2double(parts(2)) / 1000;
                if isnan(t_now)
                    t_now = toc(t0);
                end
            else
                t_now = toc(t0);
            end

            tag = strtrim(parts(1));
            newLog = {};

            %% -------- R 라인 처리: 기존 그래프 동작 유지 --------
            if tag == "R" && length(parts) >= 3
                rssi = str2double(parts(3));

                if ~isnan(rssi)
                    addpoints(hR, t_now, rssi);
                end

            %% -------- P 라인 처리: 기존 그래프 + TTGO packet 저장 --------
            elseif tag == "P" && length(parts) >= 5
                pkt_rssi = str2double(parts(3));
                snr_num  = str2double(parts(4)) / 100;
                ok       = str2double(parts(5));

                if isnan(pkt_rssi) || isnan(ok)
                    continue;
                end

                if isnan(snr_num)
                    snr_num = 0;
                end

                currentTimeStr = datestr(now, 'HH:MM:SS');
                packetLogLines = {};
                payloadLen = 0;
                airtime_ms = NaN;
                payloadSymbNb = NaN;
                tSym_ms = NaN;
                rawBytes = uint8([]);
                rawHex = '';

                if length(parts) >= 6
                    raw_msg = join(parts(6:end), ",");
                    rawHex = regexprep(char(raw_msg), '\s+', '');
                    if mod(length(rawHex), 2) == 0 && ~isempty(rawHex)
                        rawBytes = uint8(sscanf(rawHex, '%2x')');
                    end

                    payloadLen = length(rawBytes);
                    [airtime_ms, payloadSymbNb, tSym_ms] = loraAirtimeMs( ...
                        payloadLen, loraSF, loraBW, loraCRDenom, ...
                        loraCRCOn, loraImplicitHeader, loraLDRO, loraPreambleLen);
                end

                % 그래프용 동작은 기존 코드와 동일
                if ok == 1
                    addpoints(hOk, t_now, pkt_rssi);
                    addpoints(hR, t_now, pkt_rssi);

                    recvTextBox.String = sprintf( ...
                        'RX [%.2fs]: OK (RSSI: %.0f, SNR: %.2f, Airtime: %.2f ms)', ...
                        t_now, pkt_rssi, snr_num, airtime_ms);
                    recvTextBox.BackgroundColor = '#CCFFCC';
                else
                    addpoints(hBad, t_now, pkt_rssi);

                    recvTextBox.String = sprintf( ...
                        'CRC ERROR! (RSSI: %.0f, SNR: %.2f)', ...
                        pkt_rssi, snr_num);
                    recvTextBox.BackgroundColor = '#FFCCCC';
                end

                % 여기부터 로그창 내용만 LORARTK/RTCM용으로 변경
                newLog{end+1} = '----------------------------------------';
                if ok == 1
                    newLog{end+1} = sprintf('[%s] LoRa PACKET RX', currentTimeStr);
                else
                    newLog{end+1} = sprintf('[%s] LoRa PHY CRC ERROR', currentTimeStr);
                end
                newLog{end+1} = sprintf('RSSI / SNR : %.0f dBm / %.2f dB', pkt_rssi, snr_num);
                newLog{end+1} = sprintf('LoRa Length: %d bytes, Airtime: %.2f ms', payloadLen, airtime_ms);

                if ok == 1 && length(rawBytes) >= 2
                    packetType = double(rawBytes(1));
                    sequence = double(rawBytes(2));
                    key = sprintf('%u', sequence);

                    if packetType == hex2dec('A1')
                        rtcmFrame = rawBytes(3:end);
                        state = newRtcmState(1);
                        state.fragments{1} = rtcmFrame;
                        state.received(1) = true;
                        rtcmStates(key) = state;

                        info = inspectRtcm3(rtcmFrame);
                        packetLogLines{end+1} = 'TTGO Type  : 0xA1 (Single RTCM)';
                        packetLogLines{end+1} = sprintf('Sequence   : %u', sequence);
                        packetLogLines{end+1} = sprintf('RTCM bytes : %d', length(rtcmFrame));
                        if ~isnan(info.messageType)
                            packetLogLines{end+1} = sprintf('RTCM Type  : %u (%s)', ...
                                info.messageType, rtcmTypeText(info.messageType));
                        end

                    elseif packetType == hex2dec('A2') && length(rawBytes) >= 4
                        fragmentIndex = double(rawBytes(3));
                        fragmentCount = double(rawBytes(4));
                        fragmentData = rawBytes(5:end);

                        if fragmentCount > 0 && fragmentIndex < fragmentCount
                            if isKey(rtcmStates, key)
                                state = rtcmStates(key);
                                if state.count ~= fragmentCount
                                    state = newRtcmState(fragmentCount);
                                end
                            else
                                state = newRtcmState(fragmentCount);
                            end
                            state.fragments{fragmentIndex + 1} = fragmentData;
                            state.received(fragmentIndex + 1) = true;
                            rtcmStates(key) = state;
                        end

                        packetLogLines{end+1} = 'TTGO Type  : 0xA2 (RTCM Fragment)';
                        packetLogLines{end+1} = sprintf('Sequence   : %u', sequence);
                        packetLogLines{end+1} = sprintf('Fragment   : %u / %u', ...
                            fragmentIndex + 1, fragmentCount);
                        packetLogLines{end+1} = sprintf('Fragment bytes: %d', length(fragmentData));

                        if fragmentIndex == 0
                            info = inspectRtcm3(fragmentData);
                            if ~isnan(info.messageType)
                                packetLogLines{end+1} = sprintf('RTCM Type  : %u (%s)', ...
                                    info.messageType, rtcmTypeText(info.messageType));
                                packetLogLines{end+1} = sprintf('Expected full frame: %d bytes', ...
                                    info.expectedLength);
                            end
                        end
                    else
                        packetLogLines{end+1} = sprintf('Unknown TTGO packet type: 0x%02X', packetType);
                        packetLogLines{end+1} = sprintf('Sequence: %u', sequence);
                    end

                    packetLogLines{end+1} = sprintf('LoRa RAW HEX: %s', rawHex);
                elseif ok == 1
                    packetLogLines{end+1} = 'TTGO packet parsing failed: payload too short';
                end

                newLog = [newLog, packetLogLines]; %#ok<AGROW>

                if ok == 1
                    newLog{end+1} = sprintf('SF/BW/CR   : SF%d / %.0f kHz / 4/%d', ...
                        loraSF, loraBW/1000, loraCRDenom);
                    newLog{end+1} = sprintf('Symbol      : %.3f ms, PayloadSym: %d', ...
                        tSym_ms, payloadSymbNb);
                end

            %% -------- T 라인: STM에서 RTCM 검증을 끝낸 최종 결과 --------
            elseif tag == "T" && length(parts) >= 9
                sequence = str2double(parts(3));
                stmType = str2double(parts(4));
                stmLength = str2double(parts(5));
                stmCrcOk = str2double(parts(6));
                fragmentCount = str2double(parts(7));
                forwarded = str2double(parts(8));
                status = char(strtrim(join(parts(9:end), ",")));
                currentTimeStr = datestr(now, 'HH:MM:SS');

                key = sprintf('%u', sequence);
                assembled = uint8([]);
                missing = [];
                if isKey(rtcmStates, key)
                    state = rtcmStates(key);
                    missing = find(~state.received) - 1;
                    if isempty(missing)
                        for fragmentIndex = 1:state.count
                            assembled = [assembled, state.fragments{fragmentIndex}]; %#ok<AGROW>
                        end
                    end
                    remove(rtcmStates, key);
                end

                info = inspectRtcm3(assembled);
                newLog{end+1} = '============================================================';
                if stmCrcOk == 1 && forwarded == 1
                    newLog{end+1} = sprintf('[%s] RTCM RECEIVED -> UM982 FORWARDED', currentTimeStr);
                else
                    newLog{end+1} = sprintf('[%s] RTCM RECEIVE RESULT: %s', currentTimeStr, status);
                end
                newLog{end+1} = sprintf('Sequence      : %u', sequence);
                newLog{end+1} = sprintf('RTCM Type     : %u (%s)', stmType, rtcmTypeText(stmType));
                newLog{end+1} = sprintf('Frame Length  : %u bytes', stmLength);
                newLog{end+1} = sprintf('LoRa Packets  : %u', fragmentCount);
                newLog{end+1} = sprintf('STM CRC24Q    : %s', yesNo(stmCrcOk));
                newLog{end+1} = sprintf('UM982 Forward : %s', yesNo(forwarded));
                newLog{end+1} = sprintf('Status        : %s', status);

                if ~isempty(missing)
                    newLog{end+1} = sprintf('Missing fragment index: %s', mat2str(missing));
                elseif ~isempty(assembled)
                    newLog{end+1} = sprintf('MATLAB assembled: %d bytes', length(assembled));
                    if info.validHeader
                        newLog{end+1} = sprintf('Header payload: %u bytes / expected frame: %u bytes', ...
                            info.payloadLength, info.expectedLength);
                    end
                    if ~isnan(info.receivedCRC)
                        newLog{end+1} = sprintf('CRC RX / CALC : %06X / %06X (%s)', ...
                            info.receivedCRC, info.calculatedCRC, yesNo(info.crcOk));
                    end
                    newLog{end+1} = 'RTCM RAW HEX:';
                    hexLog = formatHexLines(assembled, 32);
                    newLog = [newLog, hexLog]; %#ok<AGROW>
                else
                    newLog{end+1} = 'RTCM RAW HEX를 복원하지 못함 (P line 누락 또는 PHY CRC error)';
                end
                newLog{end+1} = '============================================================';
            end

            %% 로그창 갱신
            for k = 1:length(newLog)
                logLines{end+1} = char(newLog{k}); %#ok<SAGROW>
            end

            if length(logLines) > maxLogLines
                logLines(1 : length(logLines)-maxLogLines) = [];
            end

            if ~isempty(newLog)
                set(logListBox, 'String', logLines);
                set(logListBox, 'Value', length(logLines));
            end

            %% x축 슬라이딩: 기존 동작 유지
            xlim(ax1, [t_now - windowSec, t_now]);
            xlim(ax2, [t_now - windowSec, t_now]);

        catch ME
            disp("루프 에러 발생: " + ME.message);
        end
    end

    drawnow limitrate;
end

clear s;

%% 5) LoRa Airtime 계산 함수 - 기존 코드와 동일
function [airtime_ms, payloadSymbNb, tSym_ms] = loraAirtimeMs(payloadLen, sf, bw, crDenom, crcOn, implicitHeader, lowDataRateOptimize, preambleLen)
    ih  = double(implicitHeader);
    crc = double(crcOn);
    de  = double(lowDataRateOptimize);
    cr = crDenom - 4;
    tSym_ms = (2^sf / bw) * 1000;
    payloadSymbNb = 8 + max( ...
        ceil((8*payloadLen - 4*sf + 28 + 16*crc - 20*ih) ...
        / (4 * (sf - 2*de))) * (cr + 4), 0);
    tPreamble_ms = (preambleLen + 4.25) * tSym_ms;
    tPayload_ms  = payloadSymbNb * tSym_ms;
    airtime_ms = tPreamble_ms + tPayload_ms;
end

%% 6) RTCM/TTGO 로그 처리 함수
function state = newRtcmState(fragmentCount)
    state.count = fragmentCount;
    state.received = false(1, fragmentCount);
    state.fragments = cell(1, fragmentCount);
end

function info = inspectRtcm3(bytes)
    info.validHeader = false;
    info.messageType = NaN;
    info.payloadLength = NaN;
    info.expectedLength = NaN;
    info.receivedCRC = NaN;
    info.calculatedCRC = NaN;
    info.crcOk = false;

    if length(bytes) < 3 || bytes(1) ~= hex2dec('D3') || bitand(bytes(2), 252) ~= 0
        return;
    end

    info.validHeader = true;
    info.payloadLength = double(bitand(bytes(2), 3)) * 256 + double(bytes(3));
    info.expectedLength = info.payloadLength + 6;

    if length(bytes) >= 5
        info.messageType = double(bytes(4)) * 16 + floor(double(bytes(5)) / 16);
    end

    if length(bytes) == info.expectedLength && length(bytes) >= 6
        info.receivedCRC = double(bytes(end-2)) * 65536 + ...
            double(bytes(end-1)) * 256 + double(bytes(end));
        info.calculatedCRC = double(crc24q(bytes(1:end-3)));
        info.crcOk = info.receivedCRC == info.calculatedCRC;
    end
end

function crc = crc24q(bytes)
    crc = uint32(0);
    polynomial = uint32(hex2dec('1864CFB'));
    topBit = uint32(hex2dec('1000000'));

    for byteIndex = 1:length(bytes)
        crc = bitxor(crc, bitshift(uint32(bytes(byteIndex)), 16));
        for bitIndex = 1:8 %#ok<NASGU>
            crc = bitshift(crc, 1);
            if bitand(crc, topBit) ~= 0
                crc = bitxor(crc, polynomial);
            end
        end
    end
    crc = bitand(crc, uint32(hex2dec('FFFFFF')));
end

function lines = formatHexLines(bytes, bytesPerLine)
    lines = {};
    for startIndex = 1:bytesPerLine:length(bytes)
        stopIndex = min(startIndex + bytesPerLine - 1, length(bytes));
        hex = sprintf('%02X', bytes(startIndex:stopIndex));
        lines{end+1} = sprintf('  %04d: %s', startIndex - 1, hex); %#ok<AGROW>
    end
end

function text = rtcmTypeText(messageType)
    switch double(messageType)
        case 1004
            text = 'GPS L1/L2 Legacy Observation';
        case 1005
            text = 'Reference Station ARP';
        case 1006
            text = 'Reference Station ARP + Height';
        case 1012
            text = 'GLONASS L1/L2 Legacy Observation';
        case 1077
            text = 'GPS MSM7';
        case 1087
            text = 'GLONASS MSM7';
        case 1097
            text = 'Galileo MSM7';
        case 1127
            text = 'BeiDou MSM7';
        case 1230
            text = 'GLONASS Code-Phase Bias';
        otherwise
            text = 'Other / Unknown';
    end
end

function text = yesNo(value)
    if value == 1
        text = 'YES';
    else
        text = 'NO';
    end
end
