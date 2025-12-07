// MLX90640_I2C_Driver.c - implementação para ESP32 / ESP-IDF

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "MLX90640_I2C_Driver.h"

#define TAG "MLX90640_I2C"

// Ajuste aqui se quiser usar outros pinos:
#define MLX_I2C_PORT   I2C_NUM_0
#define MLX_I2C_SDA    21
#define MLX_I2C_SCL    22
#define MLX_I2C_FREQ   400000   // 400 kHz

#define I2C_TIMEOUT_TICKS  pdMS_TO_TICKS(5000)

void MLX90640_I2CInit(void)
{
    static bool initialized = false;
    if (initialized) {
        return;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MLX_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MLX_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MLX_I2C_FREQ,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(MLX_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(MLX_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    initialized = true;
    ESP_LOGI(TAG, "I2C inicializado (port=%d, SDA=%d, SCL=%d, freq=%d)",
             MLX_I2C_PORT, MLX_I2C_SDA, MLX_I2C_SCL, MLX_I2C_FREQ);
}

int MLX90640_I2CGeneralReset(void)
{
    // Muitos exemplos deixam isso como no-op. Não é obrigatório.
    // Se quiser implementar um reset geral I2C, pode fazer um broadcast aqui.
    return MLX90640_NO_ERROR;
}

int MLX90640_I2CRead(uint8_t slaveAddr,
                     uint16_t startAddress,
                     uint16_t nMemAddressRead,
                     uint16_t *data)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd;

    // Primeiro: escreve o endereço de memória (register address, 16 bits)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (startAddress >> 8) & 0xFF, true); // MSB
    i2c_master_write_byte(cmd, startAddress & 0xFF, true);        // LSB
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(MLX_I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2CRead: erro ao enviar addr (0x%04X): %s",
                 startAddress, esp_err_to_name(err));
        return -MLX90640_I2C_WRITE_ERROR;
    }

    // Depois: lê 2 * nMemAddressRead bytes
    int bytes_to_read = nMemAddressRead * 2;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | I2C_MASTER_READ, true);

    for (int i = 0; i < bytes_to_read; i++) {
        uint8_t byte = 0;
        i2c_ack_type_t ack = (i < (bytes_to_read - 1)) ? I2C_MASTER_ACK : I2C_MASTER_NACK;
        i2c_master_read_byte(cmd, &byte, ack);

        if (i % 2 == 0) {
            data[i / 2] = ((uint16_t)byte) << 8;   // MSB
        } else {
            data[i / 2] |= byte;                   // LSB
        }
    }

    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(MLX_I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2CRead: erro ao ler (addr 0x%04X, n=%d): %s",
                 startAddress, nMemAddressRead, esp_err_to_name(err));
        return -MLX90640_I2C_NACK_ERROR;
    }

    return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr,
                      uint16_t writeAddress,
                      uint16_t dataWord)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    uint8_t addr_msb = (writeAddress >> 8) & 0xFF;
    uint8_t addr_lsb = writeAddress & 0xFF;
    uint8_t data_msb = (dataWord >> 8) & 0xFF;
    uint8_t data_lsb = dataWord & 0xFF;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr_msb, true);
    i2c_master_write_byte(cmd, addr_lsb, true);
    i2c_master_write_byte(cmd, data_msb, true);
    i2c_master_write_byte(cmd, data_lsb, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(MLX_I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2CWrite: erro addr=0x%04X data=0x%04X: %s",
                 writeAddress, dataWord, esp_err_to_name(err));
        return -MLX90640_I2C_WRITE_ERROR;
    }

    return MLX90640_NO_ERROR;
}

void MLX90640_I2CFreqSet(int freq)
{
    // A lib original permite mudar a frequência.
    // Aqui pode simplesmente ignorar ou, se quiser,
    // alterar o clock. Por simplicidade, deixo como no-op.
    (void)freq;
}
