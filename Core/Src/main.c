/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GPS_RX_BUFFER_SIZE 128
#define MAX_UBX_PAYLOAD 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
uint8_t mag[6];
uint8_t gyro[6];
uint8_t accel[6];

uint8_t bar[3];

uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
volatile uint8_t gps_rx_ready = 0;
volatile uint16_t gps_rx_size = 0;
volatile uint32_t gps_uart_error = 0;
uint8_t test_byte;

uint8_t gps_config_payload[] = {
  // VALSET header
  0x00, // Version
  0x01, // Layers: RAM
  0x00, 0x00, //  Reserved

  // CFG-UART1OUTPROT-UBX = 1
  0x01, 0x00, 0x74, 0x10,
  0x01, // true

  // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1
  0x07, 0x00, 0x91, 0x20,
  0x01 // true
};

uint8_t rx_address[5] = {
  0x6A, 0x65, 0x6C, 0x6C, 0x79 // jelly
};

// magnetometer config
uint8_t mag_config1 = 0x7C;
uint8_t mag_config2 = 0x00;
uint8_t mag_config3 = 0x00;
uint8_t mag_config4 = 0x0C;
uint8_t mag_config5 = 0x40;

// gyro config
uint8_t gyro_config1 = 0x40;

// accel config
uint8_t accel_config1 = 0x40;

// IMU shared config
uint8_t shared_config1 = 0x44;

HAL_StatusTypeDef mag_status;
HAL_StatusTypeDef gyro_status;
HAL_StatusTypeDef accel_status;
HAL_StatusTypeDef bar_status;
HAL_StatusTypeDef gps_status;

extern USBD_HandleTypeDef hUsbDeviceFS;

void debug_print(const char* message) {
  static uint8_t tx_buf[256];
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  uint32_t start = HAL_GetTick();

  if (hcdc == NULL) {
    return;
  }
  while (hcdc->TxState != 0) {
    if (HAL_GetTick() - start > 20) {
      return;
    }
  }

  size_t len = strlen(message);
  if (len > sizeof(tx_buf)) {
    len = sizeof(tx_buf);
  }
  memcpy(tx_buf, message, len);
  CDC_Transmit_FS(tx_buf, (uint16_t)len);
}

HAL_StatusTypeDef BMP581_ReadPressure(uint8_t *data) {
  uint8_t tx[5] = {0x20 | 0x80, 0, 0, 0, 0};
  uint8_t rx[5] = {0};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  if (status == HAL_OK) {
    data[0] = rx[2];
    data[1] = rx[3];
    data[2] = rx[4];
  }

  return status;
}

HAL_StatusTypeDef BMP581_ReadChipID(uint8_t *chip_id) {
  uint8_t tx[3] = {0x01 | 0x80, 0, 0};
  uint8_t rx[3] = {0};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  if (status == HAL_OK) {
    *chip_id = rx[2];
  }

  return status;
}

HAL_StatusTypeDef BMP581_WriteRegister(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg, data};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, tx, 2, 100);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  
  return status;
}

volatile uint32_t gps_error_count = 0;
volatile uint32_t gps_bytes_total = 0;

static HAL_StatusTypeDef GPS_StartRx(void) {
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, gps_rx_buffer, GPS_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
  return status;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART6) {
    gps_rx_size = Size;
    gps_bytes_total += Size;
    gps_rx_ready = 1;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6) {
    // gps_uart_error = HAL_UART_GetError(huart);
    gps_uart_error |= HAL_UART_GetError(huart);
    gps_error_count++;
    GPS_StartRx();
  }
}

HAL_StatusTypeDef Send_UBX_Frame(const uint8_t *payload, uint16_t payload_len) {
    if (payload_len > MAX_UBX_PAYLOAD) {
        return HAL_ERROR;
    }

    uint8_t frame[2 + 1 + 1 + 2 + MAX_UBX_PAYLOAD + 2];

    uint16_t i = 0;

    // Synchronization characters
    frame[i++] = 0xB5;
    frame[i++] = 0x62;

    // Class and ID: UBX-CFG-VALSET
    frame[i++] = 0x06;
    frame[i++] = 0x8A;

    // Payload length (little-endian)
    frame[i++] = payload_len & 0xFF;
    frame[i++] = (payload_len >> 8) & 0xFF;

    // Payload
    for (uint16_t j = 0; j < payload_len; j++) {
        frame[i++] = payload[j];
    }

    // Calculate UBX checksum
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    // Checksum covers class, ID, length, and payload.
    // It excludes the two synchronization bytes.
    for (uint16_t j = 2; j < i; j++) {
        ck_a += frame[j];
        ck_b += ck_a;
    }

    frame[i++] = ck_a;
    frame[i++] = ck_b;

    return HAL_UART_Transmit(&huart6, frame, i, 1000);
}

uint8_t NRF_ReadRegister(uint8_t reg) {
  uint8_t command = reg & 0x1F;
  uint8_t value = 0;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &command, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi3, &value, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  return value;
}

void NRF_WriteRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {
    (reg & 0x1F) | 0x20, value
  };

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, buffer, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void NRF_WriteRegisterBytes(uint8_t reg, uint8_t *data, uint8_t length) {
  uint8_t command = (reg & 0x1F) | 0x20;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &command, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi3, data, length, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void NRF_ReadPayload(uint8_t *buffer, uint8_t length) {
  uint8_t command = 0x61;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &command, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi3, buffer, length, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

static uint8_t NRF_ReadStatus(void) {
  uint8_t command = 0xFF; /* NOP returns STATUS without changing the radio. */
  uint8_t status = 0;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi3, &command, &status, 1, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  return status;
}

static void NRF_ClearInterrupts(void) {
  NRF_WriteRegister(0x07, 0x70); /* RX_DR, TX_DS, MAX_RT clear on write-1. */
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // configure magnetometer
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0x3D, // I2C address
    0x20, // control register 1
    I2C_MEMADD_SIZE_8BIT, // register size
    &mag_config1, // data to write
    1, // length
    100 // timeout
  );
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0x3D, // I2C address
    0x21, // control register 2
    I2C_MEMADD_SIZE_8BIT, // register size
    &mag_config2, // data to write
    1, // length
    100 // timeout
  );
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0x3D, // I2C address
    0x22, // control register 3
    I2C_MEMADD_SIZE_8BIT, // register size
    &mag_config3, // data to write
    1, // length
    100 // timeout
  );
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0x3D, // I2C address
    0x23, // control register 4
    I2C_MEMADD_SIZE_8BIT, // register size
    &mag_config4, // data to write
    1, // length
    100 // timeout
  );
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0x3D, // I2C address
    0x24, // control register 5
    I2C_MEMADD_SIZE_8BIT, // register size
    &mag_config5, // data to write
    1, // length
    100 // timeout
  );

  // configure gyro
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0xD5, // I2C address
    0x11, // CTRL2_G
    I2C_MEMADD_SIZE_8BIT, // register size
    &gyro_config1, // data to write
    1, // length
    100 // timeout
  );

  // configure accel
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0xD5, // I2C address
    0x10, // CTRL1_XL
    I2C_MEMADD_SIZE_8BIT, // register size
    &accel_config1, // data to write
    1, // length
    100 // timeout
  );

  // IMU shared config
  HAL_I2C_Mem_Write(
    &hi2c1, // I2C handle
    0xD5, // I2C address
    0x12, // CTRL3_C
    I2C_MEMADD_SIZE_8BIT, // register size
    &shared_config1, // data to write
    1, // length
    100 // timeout
  );

  BMP581_WriteRegister(0x36, 0x40);
  BMP581_WriteRegister(0x37, 0x01);

  NRF_WriteRegister(0x00, 0x0F);
  NRF_WriteRegister(0x01, 0x01);
  NRF_WriteRegister(0x02, 0x01);
  NRF_WriteRegister(0x03, 0x03);
  NRF_WriteRegister(0x04, 0x23);
  NRF_WriteRegister(0x05, 0x4C);
  NRF_WriteRegister(0x06, 0x06);
  NRF_WriteRegisterBytes(0x0A, rx_address, sizeof(rx_address));
  NRF_WriteRegister(0x11, 0x10);
  NRF_ClearInterrupts();

  gps_status = Send_UBX_Frame(gps_config_payload, sizeof(gps_config_payload));

  /* CONFIG.PWR_UP needs at least 1.5 ms before RX operation. */
  HAL_Delay(2000);

  // RF Debugging
  /*
  uint8_t status = NRF_ReadStatus();

  char buffer[64];

  snprintf(buffer, sizeof(buffer),
          "NRF STATUS: 0x%02X\r\n", status);

  debug_print(buffer);

  HAL_Delay(2000);

  uint8_t config = NRF_ReadRegister(0x00);
  uint8_t channel = NRF_ReadRegister(0x05);
  uint8_t rf_setup = NRF_ReadRegister(0x06);

  char buffer_[100];

  snprintf(buffer_, sizeof(buffer_),
          "CONFIG: %02X | RF_CH: %02X | RF_SETUP: %02X\r\n",
          config, channel, rf_setup);

  debug_print(buffer_);
  */

  /* CE is wired to PC1 in this project; high enables continuous RX. */
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  if (GPS_StartRx() != HAL_OK) {
    debug_print("Receive to Idle DMA failed\r\n");
    Error_Handler();
  } else {
    debug_print("Receive to Idle DMA succeeded\r\n");
  }
  HAL_Delay(2000);

  //char buffer[128];
  //snprintf(buffer, sizeof(buffer), "Hello world! Testing serial output.\r\n");
  //debug_print(buffer);

  debug_print("Hello world! Testing serial output.\r\n");
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /*
    mag_status = HAL_I2C_Mem_Read(
      &hi2c1, // I2C handle
      0x3D, // I2C address
      0x28 | 0x80, // register address
      I2C_MEMADD_SIZE_8BIT, // register size
      (uint8_t *)mag, // data pointer
      6, // length
      100 // timeout
    ) == HAL_OK ? 0 : -1;
    if (mag_status != 0) {
      debug_print("Error reading magnetometer\r\n");
    } else {
      int16_t x = (int16_t)((mag[1] << 8) | mag[0]);
      int16_t y = (int16_t)((mag[3] << 8) | mag[2]);
      int16_t z = (int16_t)((mag[5] << 8) | mag[4]);

      char buffer[128];
      snprintf(buffer, sizeof buffer,
              "Mag X: %d, "
              "Mag Y: %d, "
              "Mag Z: %d\r\n",
              x, y, z);
      debug_print(buffer);
    }

    gyro_status = HAL_I2C_Mem_Read(
      &hi2c1,
      0xD5,
      0x22,
      I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)gyro,
      6,
      100
    ) == HAL_OK ? 0 : -1;
    if (gyro_status != 0) {
      debug_print("Error reading gyro\r\n");
    } else {
      int16_t x = (int16_t)((gyro[1] << 8) | gyro[0]);
      int16_t y = (int16_t)((gyro[3] << 8) | gyro[2]);
      int16_t z = (int16_t)((gyro[5] << 8) | gyro[4]);

      int16_t x_converted = (int16_t)((float)x * 0.00875);
      int16_t y_converted = (int16_t)((float)y * 0.00875);
      int16_t z_converted = (int16_t)((float)z * 0.00875);

      char buffer[128];
      snprintf(buffer, sizeof buffer,
              "Gyro X: %d, "
              "Gyro Y: %d, "
              "Gyro Z: %d\r\n",
              x_converted, y_converted, z_converted);
      debug_print(buffer);
    }
    
    accel_status = HAL_I2C_Mem_Read(
      &hi2c1,
      0xD5,
      0x28,
      I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)accel,
      6,
      100
    ) == HAL_OK ? 0 : -1;
    if (accel_status != 0) {
      debug_print("Error reading accel\r\n");
    } else {
      int16_t x = (int16_t)((accel[1] << 8) | accel[0]);
      int16_t y = (int16_t)((accel[3] << 8) | accel[2]);
      int16_t z = (int16_t)((accel[5] << 8) | accel[4]);

      int16_t x_converted = (int16_t)((float)x * 0.061);
      int16_t y_converted = (int16_t)((float)y * 0.061);
      int16_t z_converted = (int16_t)((float)z * 0.061);

      char buffer[128];
      snprintf(buffer, sizeof buffer,
              "Accel X: %d, "
              "Accel Y: %d, "
              "Accel Z: %d\r\n",
              x_converted, y_converted, z_converted);
      debug_print(buffer);
    }
    */
    /*
    bar_status = BMP581_ReadPressure(bar) == HAL_OK ? 0 : -1;
    if (bar_status != 0) {
      debug_print("Error reading pressure sensor\r\n");
    } else {
      uint32_t raw_pressure = ((uint32_t)bar[2] << 16) | ((uint32_t)bar[1] << 8) | (uint32_t)bar[0];
      uint16_t pressure_pa = (uint16_t)((raw_pressure / 64.0f) * 100); // pascals

      char buffer[16];
      snprintf(buffer, sizeof(buffer), "P: %d\r\n", pressure_pa);
      debug_print(buffer);
    }
    */

    /*
    uint8_t status = NRF_ReadStatus();
    if ((status & 0x40U) != 0U) { // RX_DR
      while ((NRF_ReadRegister(0x17) & 0x01U) == 0U) { // RX FIFO not empty
        uint8_t payload[16];
        char line[56];
        NRF_ReadPayload(payload, sizeof(payload));
        int n = snprintf(line, sizeof(line), "RX:");
        for (uint8_t i = 0; i < sizeof(payload) && n < (int)sizeof(line) - 4; ++i) {
          n += snprintf(line + n, sizeof(line) - (size_t)n, " %02X", payload[i]);
        }
        snprintf(line + n, sizeof(line) - (size_t)n, "\r\n");
        debug_print(line);
      }
      NRF_ClearInterrupts();

      debug_print("We are running!\r\n");
    }
    */
    
    if (gps_rx_ready) {
      uint16_t size;

      __disable_irq();
      size = gps_rx_size;
      gps_rx_ready = 0;
      __enable_irq();

      char buffer[GPS_RX_BUFFER_SIZE + 1];

      memcpy(buffer, gps_rx_buffer, size);
      buffer[size] = '\0';

      GPS_StartRx();

      debug_print(buffer);
    }

    static uint32_t last_status_tick = 0;
    if (HAL_GetTick() - last_status_tick >= 1000) {
      last_status_tick = HAL_GetTick();

      uint32_t err_bits, err_count;
      __disable_irq();
      err_bits = gps_uart_error;
      err_count = gps_error_count;
      gps_uart_error = 0;
      __enable_irq();

      char status_buffer[96];
      snprintf(status_buffer, sizeof(status_buffer), "\r\n[GPS] bytes:%" PRIu32 " | errors: %" PRIu32 " | last err bits: 0x%02" PRIX32 " | tx: %d\r\n", gps_bytes_total, err_count, err_bits, (int)gps_status);
      debug_print(status_buffer);
    }
    
    /*
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, &test_byte, 1, 1000);

    if (status == HAL_OK) {
      char msg[10];
      snprintf(msg, sizeof(msg), "%02X\r\n", test_byte);
      debug_print(msg);
    } else if (status == HAL_TIMEOUT) {
      debug_print("UART timeout\r\n");
    } else {
      debug_print("UART receive error\r\n");
    }
    */
    /*
    char debug_buffer[96];

    snprintf(
        debug_buffer,
        sizeof(debug_buffer),
        "UART State: 0x%02X | UART Error: 0x%08lX | DMA Count: %lu\r\n",
        (unsigned int)HAL_UART_GetState(&huart6),
        (unsigned long)HAL_UART_GetError(&huart6),
        (unsigned long)__HAL_DMA_GET_COUNTER(huart6.hdmarx)
    );

    debug_print(debug_buffer);
    */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC4 PC5 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA13 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB15
                           PB3 PB4 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
