/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 100Hz Speed Loop and 50Hz Deterministic Telemetry Stream
  * (Position Control handling offloaded to Jetson)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DIFF_WINDOW 5

typedef struct
{
    float kp;
    float ki;
    float integral;
    float outMin;
    float outMax;
} PI_Controller;

typedef enum {
    STATE_SOF1 = 0,
    STATE_SOF2,
    STATE_SEQ,
    STATE_LEN,
    STATE_PAYLOAD,
    STATE_CRC_H,
    STATE_CRC_L
} ParseState;

typedef struct {
    int16_t enc1;
    int16_t enc2;
    uint8_t status1;
    uint8_t status2;
    uint16_t volt;
    uint16_t curr;
} TelemetrySnapshot;

volatile TelemetrySnapshot snapshot;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR1_ADDR      (88<<1)
#define MOTOR2_ADDR      (89<<1)

#define REG_COMMAND      0x00
#define REG_STATUS       0x01
#define REG_SPEED        0x02
#define REG_ACCELERATION 0x03

#define RX_DMA_BUF_SIZE  64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint16_t u_left = 0;

HAL_StatusTypeDef status;

static int16_t d1_buf[DIFF_WINDOW] = {0};
static int16_t d2_buf[DIFF_WINDOW] = {0};

static uint8_t diff_idx = 0;

static int32_t d1_sum = 0;
static int32_t d2_sum = 0;

volatile int16_t enc1_prev = 0;
volatile int16_t enc2_prev = 0;

volatile int16_t enc1_count = 0;
volatile int16_t enc2_count = 0;

// Targets commanded by the Jetson's 50Hz position loop
volatile float rpm1_setpoint = 0.0f;
volatile float rpm2_setpoint = 0.0f;

volatile uint8_t md1_status = 0;
volatile uint8_t md2_status = 0;

volatile uint16_t adc_raw_buffer[2] = {0, 0};

volatile uint8_t uart_tx_ready = 1;
volatile uint8_t tx_pending = 0;

volatile uint16_t debug_received_crc = 0;
volatile uint16_t debug_computed_crc = 0;

volatile float rmp_g1 = 0;
volatile float rmp_g2 = 0;

volatile int16_t d1_g = 0;
volatile int16_t d2_g = 0;

volatile uint8_t speed_control_flag = 0;

uint8_t dma_rx_buffer[RX_DMA_BUF_SIZE];
uint16_t last_dma_read_ptr = 0;

uint8_t parse_idx = 0;
uint8_t expected_payload_len = 0;
ParseState parser_state = STATE_SOF1;

uint8_t tx_seq_counter = 0;
volatile uint8_t telemetry_loop_counter = 0;
volatile uint32_t uart_rx_led_until = 0;


PI_Controller pi1 = { .kp = 1.0f, .ki = 30.0f, .integral = 0.0f, .outMin = -255.0f, .outMax = 255.0f };
PI_Controller pi2 = { .kp = 1.0f, .ki = 30.0f, .integral = 0.0f, .outMin = -255.0f, .outMax = 255.0f };

static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB5, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x0235, 0x1214, 0x2277, 0x3256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEBBF, 0xFB9E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E54, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Compute_CRC16(const uint8_t *data, uint16_t len);
float PI_Update(PI_Controller *pi, float ref, float meas, float dt);
void MD03_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t MD03_ReadStatus(uint8_t addr);
void MD03_SetMotor(uint8_t addr, int16_t command);
void SpeedControlLoop(void);
void ParseValidatedPayload(const uint8_t *payload, uint8_t length);
void ProcessSerialByte(uint8_t byte);
void CheckForInboundPackets(void);
void SendTelemetryToROS(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t Compute_CRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

static inline int16_t MovingAverageDiff(int16_t new_d,
                                         int16_t *buf,
                                         int32_t *sum,
                                         uint8_t idx)
{
    *sum -= buf[idx];
    *sum += new_d;
    buf[idx] = new_d;

    return (int16_t)(*sum / DIFF_WINDOW);
}

float PI_Update(PI_Controller *pi, float ref, float meas, float dt) {
    float error = ref - meas;
//    float p_term = pi->kp * error;
    float p_term = - pi->kp * meas;
    float i_update = error * pi->ki * dt;

    // 1. Calculate tentative total output
    float out = p_term + pi->integral + i_update;

    // 2. Anti-Windup: Conditional Integration
    // If output is saturated AND the error is pushing it further into saturation,
    // do NOT accumulate the integral. Otherwise, integrate normally.
    if ((out > pi->outMax && error > 0) || (out < pi->outMin && error < 0)) {
        // Anti-windup active: do nothing to pi->integral
    } else {
        // Normal operation or pulling out of saturation
        pi->integral += i_update;
    }

    // 3. Optional safety clamp for the integral state itself
//    if (pi->integral > pi->outMax) pi->integral = pi->outMax;
//    if (pi->integral < pi->outMin) pi->integral = pi->outMin;

    // 4. Clamp the final output being sent to the motor
    if (out > pi->outMax) out = pi->outMax;
    if (out < pi->outMin) out = pi->outMin;

    return out;
}

void MD03_Write(uint8_t addr, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
}

uint8_t MD03_ReadStatus(uint8_t addr) {
    uint8_t status = 0;
    HAL_I2C_Mem_Read(&hi2c1, addr, REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status, 1, 10);
    return status;
//	return 0;
}

void MD03_SetMotor(uint8_t addr, int16_t command) {
    uint8_t dir = 1;
    if (command < 0) { dir = 2; command = -command; }
    if (command > 255) command = 255;
    MD03_Write(addr, REG_COMMAND, dir);
    MD03_Write(addr, REG_SPEED, (uint8_t)command);
    u_left = command;
}

/* Speed Tracking Engine (Executes at 100Hz inside Hardware Counter Interrupt) */
void SpeedControlLoop(void)
{
    int16_t enc1_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t enc2_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);

    enc1_count = enc1_now;
    enc2_count = enc2_now;

    int16_t d1_raw = enc1_now - enc1_prev;
    int16_t d2_raw = enc2_now - enc2_prev;

    int16_t d1 = MovingAverageDiff(d1_raw, d1_buf, &d1_sum, diff_idx);
    int16_t d2 = MovingAverageDiff(d2_raw, d2_buf, &d2_sum, diff_idx);

    enc1_prev = enc1_now;
    enc2_prev = enc2_now;

    diff_idx++;
    if (diff_idx >= DIFF_WINDOW)
        diff_idx = 0;

//    float rpm1 = ((float)d1 * 10.0f) / 144.0f;
//    float rpm2 = -((float)d2 * 10.0f) / 144.0f;

    float rpm1 = ((float)d1 * 600) / (144.0f * 66.72);
    float rpm2 = -((float)d2 * 600) / (144.0f * 66.72);


    rmp_g1 = rpm1;
    rmp_g2 = rpm2;

    int16_t out1 = PI_Update(&pi1, rpm1_setpoint, rpm1, 0.01f);
    int16_t out2 = -PI_Update(&pi2, rpm2_setpoint, rpm2, 0.01f);

    MD03_SetMotor(MOTOR1_ADDR, (int16_t)out1);
    MD03_SetMotor(MOTOR2_ADDR, (int16_t)out2);

    md1_status = MD03_ReadStatus(MOTOR1_ADDR);
    md2_status = MD03_ReadStatus(MOTOR2_ADDR);

    snapshot.enc1 = enc1_now;
      snapshot.enc2 = enc2_now;
      snapshot.status1 = md1_status;
      snapshot.status2 = md2_status;
      snapshot.volt = adc_raw_buffer[0];
      snapshot.curr = adc_raw_buffer[1];
}

void ParseValidatedPayload(const uint8_t *payload, uint8_t length) {
    // Process inbound velocity updates sent down from the Jetson's control framework
    if(length == 5 && payload[0] == 0x00) {
        int16_t r1 = (int16_t)((payload[1] << 8) | payload[2]);
        int16_t r2 = (int16_t)((payload[3] << 8) | payload[4]);
        rpm1_setpoint = (float)r1 / 10.0f;
        rpm2_setpoint = (float)r2 / 10.0f;
    }
}

volatile int crc_fail_cnt = 0;
void ProcessSerialByte(uint8_t byte) {
    static uint8_t crc_check_payload[16];
    static uint8_t current_seq = 0;
    static uint16_t received_crc = 0;

    switch (parser_state) {
        case STATE_SOF1:
            if (byte == 0xAA) parser_state = STATE_SOF2;
            break;
        case STATE_SOF2:
            parser_state = (byte == 0x55) ? STATE_SEQ : STATE_SOF1;
            break;
        case STATE_SEQ:
            current_seq = byte;
            parser_state = STATE_LEN;
            break;
        case STATE_LEN:
            expected_payload_len = byte;
            if (expected_payload_len <= 12) {
                parse_idx = 0;
                parser_state = (expected_payload_len == 0) ? STATE_CRC_H : STATE_PAYLOAD;
            } else {
                parser_state = STATE_SOF1;
            }
            break;
        case STATE_PAYLOAD:
            crc_check_payload[parse_idx++] = byte;
            if (parse_idx >= expected_payload_len) parser_state = STATE_CRC_H;
            break;
        case STATE_CRC_H:
            received_crc = (byte << 8);
            parser_state = STATE_CRC_L;
            break;
        case STATE_CRC_L:
            received_crc |= byte;

            uint8_t verification_buf[20];
            verification_buf[0] = current_seq;
            verification_buf[1] = expected_payload_len;
            memcpy(&verification_buf[2], crc_check_payload, expected_payload_len);

            debug_received_crc = received_crc;
            debug_computed_crc = Compute_CRC16(verification_buf, expected_payload_len + 2);

            if (debug_received_crc == debug_computed_crc) {
                ParseValidatedPayload(crc_check_payload, expected_payload_len);
            }
            else
            {
            	crc_fail_cnt++;
            }
            parser_state = STATE_SOF1;
            break;
    }
}

void CheckForInboundPackets(void) {
    uint16_t current_dma_write_ptr = RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    while (last_dma_read_ptr != current_dma_write_ptr) {
    	uint8_t byte_to_process = dma_rx_buffer[last_dma_read_ptr];
        ProcessSerialByte(byte_to_process);
        last_dma_read_ptr = (last_dma_read_ptr + 1) % RX_DMA_BUF_SIZE;

        // UART RX indication
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        uart_rx_led_until = HAL_GetTick() + 80;   // keep LED on 80 ms
    }
}

static inline void CaptureSnapshot(void)
{
    TelemetrySnapshot tmp;

    __disable_irq();
    tmp.enc1 = enc1_count;
    tmp.enc2 = enc2_count;
    tmp.status1 = md1_status;
    tmp.status2 = md2_status;
    tmp.volt = adc_raw_buffer[0];
    tmp.curr = adc_raw_buffer[1];
    __enable_irq();

    snapshot = tmp;   // single atomic struct copy
}

/* Streams telemetry values over UART DMA directly inside the 50Hz interrupt phase */
void SendTelemetryToROS(void)
{
    if (!uart_tx_ready)
        return;

    uart_tx_ready = 0;

    static uint8_t tx_frame[16];

//    TelemetrySnapshot s = snapshot;  // atomic read (single copy)

    TelemetrySnapshot s;
    s = snapshot;

    tx_frame[0] = 0xAA;
    tx_frame[1] = 0x55;
    tx_frame[2] = tx_seq_counter++;

    tx_frame[3] = 10;

    tx_frame[4] = (s.enc1 >> 8) & 0xFF;
    tx_frame[5] = s.enc1 & 0xFF;

    tx_frame[6] = (s.enc2 >> 8) & 0xFF;
    tx_frame[7] = s.enc2 & 0xFF;

    tx_frame[8] = s.status1;
    tx_frame[9] = s.status2;

    tx_frame[10] = (s.volt >> 8) & 0xFF;
    tx_frame[11] = s.volt & 0xFF;

    tx_frame[12] = (s.curr >> 8) & 0xFF;
    tx_frame[13] = s.curr & 0xFF;

    uint16_t crc = Compute_CRC16(&tx_frame[2], 12);

    tx_frame[14] = (crc >> 8) & 0xFF;
    tx_frame[15] = crc & 0xFF;

    HAL_UART_Transmit_DMA(&huart2, tx_frame, 16);
}

/* Master Hardware ISR router */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        // 1. Core loop steps at 100 Hz
    	speed_control_flag = 1;
//        SpeedControlLoop();

        CaptureSnapshot();   // <-- ADD THIS HERE

        if (++telemetry_loop_counter >= 2) {
               telemetry_loop_counter = 0;
               tx_pending = 1;
           }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

//  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0);
  HAL_Delay(10000);

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim6);

//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, 2);
  HAL_UART_DMAStop(&huart2);
  memset(dma_rx_buffer, 0, RX_DMA_BUF_SIZE);
  last_dma_read_ptr = 0;
  parser_state = STATE_SOF1;
  status = HAL_UART_Receive_DMA(&huart2, dma_rx_buffer, RX_DMA_BUF_SIZE);
  MD03_Write(MOTOR1_ADDR, REG_ACCELERATION, 0);
  MD03_Write(MOTOR2_ADDR, REG_ACCELERATION, 0);

  uint32_t lastTime = 0;
  uint8_t reg_state = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  CheckForInboundPackets();
	  if (tx_pending && uart_tx_ready) {
	      tx_pending = 0;
	      SendTelemetryToROS();
	  }

	  if(speed_control_flag)
	  {
		  SpeedControlLoop();
		  speed_control_flag = 0;
	  }

	  // Turn LED off after timeout
	  if ((int32_t)(HAL_GetTick() - uart_rx_led_until) >= 0)
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  }

//	  if ((HAL_GetTick() - lastTime) >= 2000)   // 2000 ms = 2 seconds
//	  {
//		  lastTime = HAL_GetTick();
//		  if(reg_state++ > 3) reg_state = 0;
//		  // Code executed every 2 seconds
//	  }
//
//	  switch(reg_state)
//	  {
//	  case 0:
//	  case 2:
//		  rpm1_setpoint = 0;
//		  rpm2_setpoint = 0;
//		  break;
//	  case 1:
//		  rpm1_setpoint = rpm2_setpoint = 80;
//		  break;
//	  case 3:
//		  rpm1_setpoint = rpm2_setpoint = -80;
//		  break;
//	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart_tx_ready = 1;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
