/*
 * sensor_manager.c
 *
 *  Created on: Feb 23, 2026
 *      Author: andrewchisholm
 */
#include "sensor_manager.h"
#include "main.h"
#include "spi.h"
#include <string.h>


// --- Private Constants ---
#define AS5048A_SPI        		hspi3
#define BMI270_SPI            	hspi4

#define AS5048A_CMD_READ   		0xFFFF

#define BMI270_READ_BIT    		0x80
#define BMI270_REG_ID          	0x00 // chip ID code location, code = 0x24
#define BMI270_DATA_ADDR   		(0x0C | BMI270_READ_BIT)
#define BMI270_REG_INTERNAL_ST 	0x21
#define BMI270_CHIP_ID			0x24
#define BMI270_ACC_CONF			0x40
#define BMI270_REG_ACC_RANGE	0x41
#define BMI270_GYR_CONF			0x42
#define BMI270_REG_GYR_RANGE	0x43
#define BMI270_REG_INIT_CTRL   	0x59
#define BMI270_REG_INIT_DATA   	0x5E
#define BMI270_NV_CONF			0x70
#define BMI270_REG_PWR_CONF    	0x7C
#define BMI270_REG_PWR_CTRL    	0x7D
#define BMI270_REG_CMD			0x7E


#define BMI270_ACCEL_2G_LSB    	16384.0f
#define GRAVITY_MSS            	9.81f
#define ACCEL_SCALE            	(GRAVITY_MSS / BMI270_ACCEL_2G_LSB)

// --- Private Variables ---
__attribute__((section(".RAM_D2"))) volatile uint16_t enc_rx_raw[2];
static uint16_t enc_tx_cmd[2] = {AS5048A_CMD_READ, 0}; // {AS5048A_CMD_READ, 0};
// BMI270 SPI requires an extra dummy byte after the address
static uint8_t  imu_tx_buf[14] = {BMI270_DATA_ADDR, 0x00};
__attribute__((section(".RAM_D2"))) volatile uint8_t  imu_rx_raw[14];
static SensorData_t live_data;
static SensorData_t validated_data;
uint8_t error_reg = 234;
extern uint8_t debug_array[3];
uint8_t ID_returned = 0;
static uint32_t last_ready_time = 0;

uint8_t toggle_count = 0;

// Synchronization Flags
static volatile bool enc_done = false;
static volatile bool imu_done = false;

// Data array to initialise BMI270, found in bmi_array.c
extern const uint8_t bmi270_config_file[];

// Private Prototypes
static bool AS5048A_CheckParity(uint16_t v);
void 		BMI270_Init(void);
uint8_t 	BMI270_Read_Reg(uint8_t reg);
void 		BMI270_Write_Reg(uint8_t reg, uint8_t data);

// Public Functions
void SENSOR_Init(void) {
    memset(&live_data, 0, sizeof(SensorData_t));
    memset(&validated_data, 0, sizeof(SensorData_t));

    HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_SET);

    // 1. Initialize the IMU (The "Blob" load)
    BMI270_Init();

    // 2. Prime the AS5048A pipeline
    // Send one read command now so the first 2ms interrupt has data waiting
    uint16_t dummy_rx;
    HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&AS5048A_SPI, (uint8_t*)&enc_tx_cmd, (uint8_t*)&dummy_rx, 1, 10);
    HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_SET);
}

/**
 * When the timer expires, reset fresh data
 * flags and hand the rest of the execution
 * to SENSOR_StartAcquisition.
 */
void SENSOR_TimerCallback(void) {
    /* 1. SOFT RESET: Clear flags at the start of the 2ms window.
       If the previous cycle failed, this "unlocks" the logic. */
    enc_done = false;
    imu_done = false;

    /* 2. Start the hardware acquisition */
    SENSOR_StartAcquisition();
}

/**
 * Acquire new data for both sensors
 *
 *
 */
void SENSOR_StartAcquisition(void) {

	if (toggle_count % 250 == 0){
	toggle_count = 0;
	HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin); // TODO remove this debug device
	}

	toggle_count++;

    HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&AS5048A_SPI, (uint8_t*)&enc_tx_cmd, (uint8_t*)&enc_rx_raw, 2);

    HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&BMI270_SPI, imu_tx_buf, (uint8_t*)imu_rx_raw, 14); // TODO this acquires more data than used, there are maybe 40us available to be saved
}

/**
 * Handle DMA completion for both SPIs
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) {
        HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_SET);
        enc_done = true;
    }
    else if (hspi->Instance == SPI4) {
        HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_SET);
        imu_done = true;
    }

    /* 3. The "AND" Gate: Only triggers if both finished since SENSOR_TimerCallback was called */
    if (enc_done && imu_done) {
        SENSOR_OnDataReady();
    }
}

void SENSOR_OnDataReady(void) {
    // 0. Capture timestamp IMMEDIATELY when the callback fires
    // The timestamp reflects the
    // moment the DMA finished.
    uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim5);
    uint32_t delta_us = current_time - last_ready_time;
    last_ready_time = current_time;

    // 1. A. Tell CPU to refresh DMA data.
    SCB_InvalidateDCache_by_Addr((uint32_t*)imu_rx_raw, 14);
    SCB_InvalidateDCache_by_Addr((uint32_t*)&enc_rx_raw, 2);

    // 2. Encoder Validation
    bool parity_ok = AS5048A_CheckParity(enc_rx_raw[0]);
    bool error_bit = (enc_rx_raw[0] & 0x4000) != 0;

    if(error_bit) {
        // PAR=0, RW=1, Addr=0x0001 -> 0x4001
        uint16_t clear_cmd = 0x4001;
        uint16_t nop_cmd = 0x0000;
        uint16_t rx_dummy;

        HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&AS5048A_SPI, (uint8_t*)&clear_cmd, (uint8_t*)&rx_dummy, 1, 10);
        HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_SET);

        // Required delay between frames >= 350 ns
        for(volatile int i=0; i<200; i++);

        // Second frame to finish the clear operation
        HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&AS5048A_SPI, (uint8_t*)&nop_cmd, (uint8_t*)&rx_dummy, 1, 10);
        HAL_GPIO_WritePin(AS5048A_CSn_GPIO_Port, AS5048A_CSn_Pin, GPIO_PIN_SET);
    }

    if (parity_ok && !error_bit) {

	// 1. Apply Offset
	int32_t raw_centered = (int32_t)(enc_rx_raw[0] & 0x3FFF) - AS5048A_ZERO_OFFSET;

	// 2.A. Handle Rollover (Wrapping to [-8192, 8191])
	if (raw_centered > (AS5048A_RANGE / 2)) raw_centered -= AS5048A_RANGE;
	if (raw_centered < -(AS5048A_RANGE / 2)) raw_centered += AS5048A_RANGE;

	// 2.B. Convert to Radians
	live_data.raw_encoder = raw_centered;
	live_data.angle_rad = (float)live_data.raw_encoder * (2.0f * 3.14159265f / (AS5048A_RANGE * AS5048A_DIRECTION));
	live_data.encoder_valid = true;
    }

    else {
        live_data.encoder_valid = false;
        // Keep the previous angle_rad to avoid jumping to 0.0
    }

    // 3. IMU: Mapping BMI270 bytes (Filling in the placeholders)
    int16_t raw_ax = (int16_t)((imu_rx_raw[3] << 8) | imu_rx_raw[2]);
    int16_t raw_gz = (int16_t)((imu_rx_raw[13] << 8) | imu_rx_raw[12]);

    live_data.accel_x_mps2 = (float)raw_ax * (GRAVITY_MSS / BMI270_ACCEL_2G_LSB);
    live_data.gyro_z_rps   = (float)raw_gz * ((3.14159f / 180.0f) / 131.2f) * BMI270_DIRECTION;
    live_data.imu_valid    = true; // Add specific BMI270 error checks if needed

    // 4. Update and Snapshot
    live_data.timestamp_us = delta_us;

    // Only copy if at least one sensor is healthy
    if (live_data.encoder_valid || live_data.imu_valid) {
        memcpy(&validated_data, &live_data, sizeof(SensorData_t));
    }
}

// TODO remove if not used
void SENSOR_GetLatest(SensorData_t *out_data) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(out_data, &validated_data, sizeof(SensorData_t));
    __set_PRIMASK(primask);
}

void BMI270_Init(void) {

	// Empirically, the BMI270 seems to be MUCH more reliable in SPI mode 3,
	// use CPOL: High, CPHA: 2 Edge settings

	// 0. Soft Reset - empirically not needed provided CPOL: high, CPHA: 2 edge selected in .ioc
	//BMI270_Write_Reg(BMI270_REG_CMD, 0xB6);  // Soft Reset
	//HAL_Delay(50);

	// 1. Dummy read of ID
	uint8_t reg_addr = BMI270_REG_ID;
	BMI270_Read_Reg(reg_addr);
	HAL_Delay(3);

	// 2. Live read of ID
	ID_returned = BMI270_Read_Reg(reg_addr); // still the same address as the dummy read
	if (ID_returned != BMI270_CHIP_ID){while(1){HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin); HAL_Delay(30);}}
	HAL_Delay(1);

	// 3. Disable PWR_CONF, sleep > 450us
	reg_addr = BMI270_REG_PWR_CONF;
	uint8_t tx_val = 0x00;
	BMI270_Write_Reg(reg_addr, tx_val);
	HAL_Delay(1);

	// 4. Prep for data load: INIT_CTRL = 0
	reg_addr = BMI270_REG_INIT_CTRL;
	tx_val = 0x00;
	BMI270_Write_Reg(reg_addr, tx_val);
	HAL_Delay(30);

	// 5. Write to data load address
	reg_addr = BMI270_REG_INIT_DATA;
	HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, &reg_addr, 1, 1000);
	HAL_SPI_Transmit(&hspi4, bmi270_config_file, 8192, 1000);
	HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	// 6. Complete Config Load
	reg_addr = BMI270_REG_INIT_CTRL;
	tx_val = 0x01;
	BMI270_Write_Reg(reg_addr, tx_val);
	HAL_Delay(50);

	// 7. Confirm Status
	reg_addr = BMI270_REG_INTERNAL_ST;
	if (BMI270_Read_Reg(reg_addr) == 0x01){}
	//{for (int i = 0; i < 2; i++){HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin); HAL_Delay(500);}}
	else
	{while(1){HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin); HAL_Delay(100);}}

	 // 8. Enable the sensor
	reg_addr = BMI270_REG_PWR_CTRL;
	tx_val = 0x0E;
	BMI270_Write_Reg(reg_addr, tx_val);
	HAL_Delay(10);

    // 9.a. ACC_CONF
    reg_addr = BMI270_ACC_CONF;
    tx_val = 0xAB;
    BMI270_Write_Reg(reg_addr, tx_val);
    HAL_Delay(10);

    // 9.b. ACC_RANGE
    reg_addr = BMI270_REG_ACC_RANGE;
    tx_val = 0x00;
    BMI270_Write_Reg(reg_addr, tx_val);
    HAL_Delay(10);

    // 10.a. GYR_CONF
    reg_addr = BMI270_GYR_CONF;
    tx_val = 0xEB;
    BMI270_Write_Reg(reg_addr, tx_val);
    HAL_Delay(10);

    // 10.b. GYR_CONF
    reg_addr = BMI270_REG_GYR_RANGE;
    tx_val = 0x03; // 0x03 = +/-250 degrees/second (131.2 LSB/dps), 0x04 = +/-125 dps (262.4 LSB/dps)
    BMI270_Write_Reg(reg_addr, tx_val);
    HAL_Delay(10);

    // 11. PWR_CONF
    reg_addr = BMI270_REG_PWR_CONF;
    tx_val = 0x02;
    BMI270_Write_Reg(reg_addr, tx_val);
    HAL_Delay(10);

    // 12. Toggle LEDs to show completion
    for (int i = 0; i < 10; i++)
    {
    	HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin); HAL_Delay(30);
    	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin); HAL_Delay(30);
    	HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin); HAL_Delay(30);
    	HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin); HAL_Delay(30);
    }
}

void BMI270_Write_Reg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg & 0x7F, data }; // Clear bit 7 for Write
    HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&BMI270_SPI, tx, 2, 10);
    HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_SET);
}

uint8_t BMI270_Read_Reg(uint8_t reg) {
    uint8_t tx[3] = { reg | 0x80, 0x00, 0x00 }; // Addr, Dummy, Clock for Rx
    uint8_t rx[3] = {0};

    HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&BMI270_SPI, tx, rx, 3, 10);
    HAL_GPIO_WritePin(BMI270_CSn_GPIO_Port, BMI270_CSn_Pin, GPIO_PIN_SET);
    debug_array[0] = rx[0];
    debug_array[1] = rx[1];
    debug_array[2] = rx[2];

    return rx[2]; // The valid data is in the 3rd byte because of the dummy byte
}

static bool AS5048A_CheckParity(uint16_t v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (v & 1) == 0; // Returns true if parity is even (correct)
}
