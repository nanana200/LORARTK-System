#include <lora.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////////////////////////// 2. 전역 변수 + 프로토타입

char data[200]="";  				// sprintf로 시리얼로 보내기 위한 문자열을 저장하는 배열 --> 또한 뒤에서 HAL_UART_Transmit가 원라는 시작주소의 자료형 타입이 uint8_t여서 char말고 uint8로 선언
uint8_t rxflag = 0;
int rcv = 0;
int r_byte = 0;

static uint32_t last_rssi_tick = 0;
static uint8_t rx_buf[256];

static void uart_print_hex(uint8_t *buf, uint8_t len);
static int16_t lora_get_current_rssi(void);
static int16_t lora_get_packet_rssi(void);
static int16_t lora_get_packet_snr_x100(void);


////////////////////////////////////////////////////////////////////////////////////////////////

// --- 함수 구현부 (main.c에서 잘라내기해서 붙여넣기) ---

void lora_log(const char *fmt, ...) {          //printf처럼 그냥 문자열 프린트하고싶을떄
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
}


void lora_write(uint8_t adr,uint8_t data)
{
	uint8_t tx_buffer[2] = {(write_bit|adr) , data};
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_RESET);  //CS LOW
	HAL_SPI_Transmit(&hspi1, tx_buffer, 2, 100);
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_SET); //CS HIGH
}

void lora_write_burst(uint8_t adr, uint8_t data1, uint8_t data2, uint8_t data3) //레지스터 길이가 2byte보다 길어서 한번에 BURST access방식으로 레지스터에 넣어줌(주소가 이어진 레지스터의 상황에서만 가능)
{
	uint8_t tx_buffer[4] = {(write_bit|adr) , data1 , data2 , data3};
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_RESET);
	if(data3 != no_more)  HAL_SPI_Transmit(&hspi1, tx_buffer, 4, 100);
	else if(data3==no_more)  HAL_SPI_Transmit(&hspi1, tx_buffer, 3, 100); // 3개까지 burst하고싶지않을때 no_more 인자 주면됨
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_SET);
}

uint8_t lora_read(uint8_t adr) //레지스터 접근해서 읽을때
{
	uint8_t rx_buffer[2] = {(read_bit|adr) , 0x00}; //차피 레지스터로 mosi로 1bit보내면 miso로 1bit 나오니깐 결국 이 배열엔는 보낸자리에 새 데이터가 차는거니깐 배열한개로 tx로 보내면서 rx로 보낸자리ㅣ를 채우는 방식으로 가능
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, rx_buffer, rx_buffer ,2, 100);
	//HAL_SPI_TransmitReceive함수가 spi.tranfer처럼 보내고 바로 반환값을 3번쨰 인자에 반환하는함수임
	// 이 함수 들가면 2,3번째 인자로 주소값을 줘야하므로 바로 값을 못넣고 변수나,문자열에 넣어서 주소를 넘겨줘야한다.
	//4번째 인자는 몇 바이트 보내고 통신종료할지--> 이거는 레지스터마다 다르니깐 바꿔주기ㅣ******
	//이떄 들어가는 tx,rx의 인자는 tx는 뭐 알아서 하면되는데 rx는 무조건 배열로 해야한다
	//왜냐하면 rx는 2번째 byte타이밍에서 받아오는데 그럴려면 변수로 하면 변수에는 첫 byte에오는 더미데이터를 받아서 두번째 byte타이밍에 오는 값을 그 변수가 못받음 -->받으려면 배열선언해서 두번째 배열요소에 그 응답이 저장되니깐 그걸 읽으면된다
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_SET);

	//sprintf(data, "read data: 0x%02X\r\n",rx_buffer[1]); //시리얼로 보낼 문자열을 data라는 문자열에 담음
	//HAL_UART_Transmit(&huart6, (uint8_t*)data, strlen(data), 100); // uart로 보내기

	return rx_buffer[1];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// lora 기본 세팅 관련

void lora_setup(void)
{
	lora_write(0x09, 0x8F); //출력 관련 세팅 : PA_BOOST 핀(1)--> 안테나쪽으로 출력연결+고출력, MAX POWER : 000 , OUTPUTPOWER : 1111(17dBm)  --> 1000 1111 = 0x8F
	lora_write(0x0B, 0x31); // 과전류 방지 전류 보호회로 세팅(OCP) --> 0011 0001 --> 140mA로 제한

	lora_write(0x11,0x00); //인터럽트 마스크 안해줘서 다 허용(tx 인터럽트 mask 안해주는겸)
	lora_write(0x12, 0xFF); // 안전빵으로 인터럽트 flag 한번씩 다 clear
}


void lora_freq(void) //주파수 922.1MHz로 설정  ( 922100000 / 61.03515625 = 15107686 = E68666)
{
    //주의 : 주파수(LoRa Frequency)를 변경하려면, 칩은 반드시 Sleep 모드나 Standby 모드 상태여야 한다."
    lora_write(0x01,sleep);
    HAL_Delay(10);
    lora_write_burst(0x06,0XE6,0X86,0X66); // 주소, RegFrfMsb, RegFrfMid, RegFrfLsb (burst access로 함)
    //lora_read(0x06);
    //lora_read(0x07);
    //lora_read(0x08);
}

void packet_set(void) //LORA링크 성격 설정 +  속도·신뢰성·수신 동작 방식을 설정 - 이 레지스터들 쓸때 무조건 SLEEP이나 STDBY상태여야함
{
    //sprintf(data, "BW(대역폭):125kHz, CR(부호화율):4/5, Explicit Header mode\r\n  SF->9, 통신방식 : 패킷, CRC mode : ON, \r\n\n"); //시리얼로 보낼 문자열을 data라는 문자열에 담음
    //HAL_UART_Transmit(&huart6, (uint8_t*)data, strlen(data), 100); // uart로 보내기

    lora_write(0x1D,0X72);  //RegModemConfig1(0x1D) -> 0111 0010 = 0X72 (BW(대역폭):125kHz, CR(부호화율):4/5, Explicit Header mode)
    //lora_read(0x1D);

    lora_write(0x1E,0X74);  //LoRa의 속도·신뢰성·수신 동작 방식을 정하는 핵심 레지스터  - 이 레지스터 쓸때 무조건 SLEEP이나 STDBY상태여야함
                            //RegModemConfig2(0x1E)_ -> 0111 0100 = 0X94 (SF->7(통신거리,속도),  통신방식 : 패킷으로(분석용x), CRC모드 쓰기, RX TIMEOUT : 0(이건 일단은 0으로 나중에 최적화부분 비트여서))
    //lora_read(0x1E);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  rx관련
void rx_set(void)
{
	lora_write(0x01,stanby); //sleep,stanby모드여야한다*************
	lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어

	lora_write(0x0F,0X00); //수신 데이터(Rx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x00)
	lora_write(0x0D, 0x00); ////RegFifoAddrPtr : 데이터 읽고 쓸 위치 설정
	lora_write(0x40, 0x00); // D0을 rxdone의 인터럽트로 쓰겠다
	lora_write(0x01, 0x85); // 000 0101 -> 0x85(RXCONTINOUS 모드) -------> 이거 하면 바로 수신시작
}


void rx_read(void) ////////////////////////////////////////////////// 테라텀에서 확인하는 함수
{

	rxflag = lora_read(0x12); //rxdone 계속 보기
	//lora_log(".\r\n");

	if(rxflag&0x20) //PayloadCrcError발생시 탈출
	{
		lora_log("PayloadCrcError\r\n");
		lora_write(0x01,stanby);
		lora_write(0x12, 0xFF);
		return;
	}


	if(rxflag & 0x40)  // 무언가 수신이 됬다면
	{
		lora_write(0x01,stanby); //수신 끝나서 stanby로
		r_byte = lora_read(0x13); // RegRxNbBytes로 수신되니 데이터 길이(byte수) 있음
		lora_write(0x0D, lora_read(0x10));//RegFifoAddrPtr(0x0D)로 수신데이터 읽기 위해서 포인터를  RegFifoRxCurrentAddr(0x10)이용해서 → 가장 마지막으로 수신된 패킷의 FIFO 시작 주소로 이동
		lora_log("receive byte : %d\r\n",r_byte);
		lora_log("rx_success!!!\r\n receive data : ");
	}

	 for(int i = 0; i < r_byte; i++) //받은 한 패킷길이만큼 읽기
	 {
		 rcv=lora_read(0x00); // RegFifo : 포인터로 가르킨 주소 0x00에서 데이터를 읽음 -> 포인터가 알아서 1씩 증가함
		 lora_log(" %c",rcv);
	 }


	 lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어
}

/*
 * RTCM 브리지용 바이너리 수신 함수.
 * 기존 rx_read()의 FIFO 읽기 순서를 그대로 사용하되 문자열 변환 없이 원본 바이트를 반환한다.
 */
int lora_receive_packet(uint8_t *buffer, uint8_t *length, uint8_t *irq_flags)
{
	uint8_t irq = lora_read(0x12); // RegIrqFlags
	uint8_t received_length;
	uint8_t fifo_address = 0x00U;

	*irq_flags = irq;
	*length = 0U;
	if ((irq & 0x20U) != 0U) // PayloadCrcError
	{
		lora_write(0x01, stanby);
		lora_write(0x12, 0xFF);
		return -1;
	}
	if ((irq & 0x40U) == 0U) // RxDone이 아닌 DIO 이벤트
	{
		lora_write(0x12, 0xFF);
		return 0;
	}

	lora_write(0x01, stanby);
	received_length = lora_read(0x13); // RegRxNbBytes
	lora_write(0x0D, lora_read(0x10)); // FIFO pointer = RegFifoRxCurrentAddr
	/* FIFO는 burst read로 한 번에 읽어서 다음 LoRa 패킷까지의 비수신 시간을 줄인다. */
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &fifo_address, 1U, 100U);
	HAL_SPI_Receive(&hspi1, buffer, received_length, 100U);
	HAL_GPIO_WritePin(GPIOD, LORA_CS_Pin, GPIO_PIN_SET);
	*length = received_length;
	lora_write(0x12, 0xFF);
	return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 송수신 관련 set

// 송수신 위한 stanby 모드 변경 + rx,tx주고 받을때 어떤 메모리 쓸지 세팅
void fifo_set(void) //payload 전송전 세팅
{
    lora_write(0x01,stanby); //fifo메모리는 sleep에서 초기화되서 무조건 sleep에서는 하면 안됨, fifo는 stanby에서만 접근 가능


    lora_write_burst(0x0E,0X80,0X00,no_more); //송수신데이터 저장 메모리 위치 설정
    //lora_write(0x0E,0X80);  //RegFifoTxBaseAddr : 송신 데이터(Tx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x80)
    //lora_write(0x0F,0X00);  //RegFifoRxBaseAddr : 수신 데이터(Rx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x00)
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* 이 프로젝트는 드론 수신 전용이므로 LoRa 송신 함수는 포함하지 않는다. */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  매트랩 코드보는용 함수들


void rssi_graph(void) ///////////////////////////////////////////////////////// 매트랩 수신 보는용 함수
{
    uint8_t irq = lora_read(0x12);   // RegIrqFlags
    uint32_t now = HAL_GetTick();
    int16_t pkt_rssi = lora_get_packet_rssi();
    int16_t snr_x100 = lora_get_packet_snr_x100();

    /* CRC 에러 */
    if(irq & 0x20)
    {
        char line[64];
        snprintf(line, sizeof(line), "P,%lu,%d,%d,0,\r\n", now, pkt_rssi, snr_x100);
        HAL_UART_Transmit(&huart6, (uint8_t*)line, strlen(line), 100);

        lora_write(0x01, stanby);
        lora_write(0x12, 0xFF);
        r_byte = 0;
        return;
    }

    /* RX done */
    if(irq & 0x40)
    {
        lora_write(0x01, stanby);

        r_byte = lora_read(0x13);                    // RegRxNbBytes
        lora_write(0x0D, lora_read(0x10));           // FIFO pointer = current RX start

        if(r_byte > sizeof(rx_buf)) r_byte = sizeof(rx_buf);

        for(int i = 0; i < r_byte; i++)
        {
            rx_buf[i] = lora_read(0x00);            // RegFifo
        }

        char head[64];
        snprintf(head, sizeof(head), "P,%lu,%d,%d,1,", now, pkt_rssi, snr_x100);
        HAL_UART_Transmit(&huart6, (uint8_t*)head, strlen(head), 100);

        uart_print_hex(rx_buf, r_byte);

        HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 100);
    }

    lora_write(0x12, 0xFF);
    r_byte = 0;
}

void uart_send_noise_rssi(void)
{
    uint32_t now = HAL_GetTick();

    if(now - last_rssi_tick >= 50)   // 매트랩에 찍기위한 값들을 보내는 주기
    {
        last_rssi_tick = now;

        char line[64];
        int16_t rssi = lora_get_current_rssi();

        snprintf(line, sizeof(line), "R,%lu,%d\r\n", now, rssi);
        HAL_UART_Transmit(&huart6, (uint8_t*)line, strlen(line), 100);
    }
}

static void uart_print_hex(uint8_t *buf, uint8_t len)
{
    char hx[4];

    for(uint8_t i = 0; i < len; i++)
    {
        snprintf(hx, sizeof(hx), "%02X", buf[i]);
        HAL_UART_Transmit(&huart6, (uint8_t*)hx, strlen(hx), 100);
    }
}

static int16_t lora_get_current_rssi(void)
{
    /* HF port(779MHz 이상) 기준 */
    return (int16_t)lora_read(0x1B) - 157;
}

static int16_t lora_get_packet_rssi(void)
{
    return (int16_t)lora_read(0x1A) - 157;
}

static int16_t lora_get_packet_snr_x100(void)
{
    int8_t snr_raw = (int8_t)lora_read(0x19);
    return (int16_t)snr_raw * 25;   // 0.25dB * 100
}

/*
 * LORARTK에서는 RTCM 처리를 위해 FIFO를 먼저 읽으므로 기존 rssi_graph()가
 * FIFO를 다시 소비하면 안 된다. 이미 읽어 둔 packet으로 원본과 동일한
 * P,<tick>,<RSSI>,<SNRx100>,<CRC_OK>,<HEX> 형식을 PG14(USART6_TX)에 출력한다.
 */
void rssi_graph_packet(uint8_t crc_ok, const uint8_t *buffer, uint8_t length)
{
    uint32_t now = HAL_GetTick();
    int16_t pkt_rssi = lora_get_packet_rssi();
    int16_t snr_x100 = lora_get_packet_snr_x100();
    char head[64];

    snprintf(head, sizeof(head), "P,%lu,%d,%d,%u,",
             (unsigned long)now, pkt_rssi, snr_x100, crc_ok ? 1U : 0U);
    HAL_UART_Transmit(&huart6, (uint8_t*)head, strlen(head), 100);
    if ((crc_ok != 0U) && (buffer != NULL))
    {
        uart_print_hex((uint8_t *)buffer, length);
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2U, 100);
}

/*
 * MATLAB 로그창용 RTCM 최종 결과.
 * R/P 그래프 프로토콜은 그대로 두고, 완전한 RTCM의 검증 결과만 T 라인으로 추가한다.
 * T,tick,seq,type,frame_len,crc_ok,fragment_count,forwarded,status
 */
void lora_rtcm_debug_result(uint8_t sequence, uint16_t rtcm_type,
                            uint16_t frame_length, uint8_t crc_ok,
                            uint8_t fragment_count, uint8_t forwarded,
                            const char *status)
{
    char line[112];

    snprintf(line, sizeof(line), "T,%lu,%u,%u,%u,%u,%u,%u,%s\r\n",
             (unsigned long)HAL_GetTick(), sequence, rtcm_type, frame_length,
             crc_ok ? 1U : 0U, fragment_count, forwarded ? 1U : 0U, status);
    HAL_UART_Transmit(&huart6, (uint8_t*)line, strlen(line), 100);
}

int16_t lora_packet_rssi_dbm(void)
{
    return lora_get_packet_rssi();
}

int16_t lora_packet_snr_x100(void)
{
    return lora_get_packet_snr_x100();
}
