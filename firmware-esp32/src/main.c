/* LeoDrone Phoenix - ESP32-S3 Sensor Node
 * BME280 (I2C) + ICM-42688-P (SPI) → MAVLink UART → Companion Computer
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char *TAG = "LeoDrone";

// I2C config for BME280
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

// SPI config for ICM-42688-P
#define SPI_MOSI_IO          11
#define SPI_MISO_IO          13
#define SPI_CLK_IO           12
#define SPI_CS_IO            10

// UART for MAVLink
#define UART_NUM             UART_NUM_1
#define UART_TX_IO           43
#define UART_RX_IO           44
#define UART_BAUD            921600

// Sensor data structure
typedef struct {
    float temperature;  // BME280
    float humidity;     // BME280
    float pressure;     // BME280
    float accel[3];     // ICM-42688-P
    float gyro[3];      // ICM-42688-P
    uint32_t timestamp;
} sensor_data_t;

static sensor_data_t g_sensor_data;

// Initialize I2C for BME280
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

// Initialize SPI for ICM-42688-P
static esp_err_t spi_master_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_IO,
        .miso_io_num = SPI_MISO_IO,
        .sclk_io_num = SPI_CLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = SPI_CS_IO,
        .queue_size = 7,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;
    spi_device_handle_t spi;
    return spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
}

// Initialize UART for MAVLink
static esp_err_t uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t err = uart_param_config(UART_NUM, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(UART_NUM, UART_TX_IO, UART_RX_IO, -1, -1);
    if (err != ESP_OK) return err;
    return uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
}

// Read BME280 (simplified - returns simulated data for now)
static void bme280_read(float *temp, float *hum, float *press) {
    // TODO: Actual I2C register read
    *temp = 25.0f + (esp_random() % 100) / 100.0f;
    *hum = 50.0f + (esp_random() % 100) / 10.0f;
    *press = 101325.0f + (esp_random() % 1000) / 10.0f;
}

// Read ICM-42688-P (simplified)
static void icm42688_read(float accel[3], float gyro[3]) {
    // TODO: Actual SPI register read
    accel[0] = 0.01f; accel[1] = 0.02f; accel[2] = 9.81f;
    gyro[0] = 0.001f; gyro[1] = 0.002f; gyro[2] = 0.001f;
}

// Send MAVLink heartbeat
static void send_heartbeat(void) {
    uint8_t buf[64];
    int len = snprintf((char*)buf, sizeof(buf),
        "HEARTBEAT: T=%.1f H=%.1f P=%.0f A=[%.2f,%.2f,%.2f]\\n",
        g_sensor_data.temperature, g_sensor_data.humidity, g_sensor_data.pressure,
        g_sensor_data.accel[0], g_sensor_data.accel[1], g_sensor_data.accel[2]);
    uart_write_bytes(UART_NUM, buf, len);
}

// Sensor reading task (100Hz)
static void sensor_task(void *arg) {
    ESP_LOGI(TAG, "Sensor task started at 100Hz");
    while (1) {
        bme280_read(&g_sensor_data.temperature,
                     &g_sensor_data.humidity,
                     &g_sensor_data.pressure);
        icm42688_read(g_sensor_data.accel, g_sensor_data.gyro);
        g_sensor_data.timestamp = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
    }
}

// MAVLink streaming task (10Hz)
static void mavlink_task(void *arg) {
    ESP_LOGI(TAG, "MAVLink task started at 10Hz");
    while (1) {
        send_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "LeoDrone Phoenix - ESP32-S3 Sensor Node v1.0");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // Init peripherals
    i2c_master_init();
    ESP_LOGI(TAG, "I2C (BME280) initialized");
    
    spi_master_init();
    ESP_LOGI(TAG, "SPI (ICM-42688-P) initialized");
    
    uart_init();
    ESP_LOGI(TAG, "UART (MAVLink) initialized at %d baud", UART_BAUD);
    
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(sensor_task, "sensor", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(mavlink_task, "mavlink", 4096, NULL, 3, NULL, 1);
    
    ESP_LOGI(TAG, "All tasks started. LeoDrone Phoenix is READY.");
}
