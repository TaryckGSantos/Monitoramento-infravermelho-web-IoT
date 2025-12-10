// esp32_sensor_main.c
// ESP32 #1 (SENSOR): lê GY-MCU90640 via UART, normaliza 24x32 em 8 bits e envia o frame via UDP para a ESP32 #2 (SERVIDOR).

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#define TAG "MLX-SENSOR"

// MLX90640 parâmetros
#define MLX_ROWS   24
#define MLX_COLS   32
#define MLX_PIXELS (MLX_ROWS * MLX_COLS) 
#define MLX_FRAME_BYTES (MLX_PIXELS * 2)  

// UART do módulo GY-MCU90640
#define MLX_UART_NUM   UART_NUM_1
#define MLX_UART_TXPIN GPIO_NUM_17
#define MLX_UART_RXPIN GPIO_NUM_16
#define MLX_UART_BAUD  115200
#define MLX_UART_BUF   2048

// Protocolo GY-MCU90640
#define GY_HEADER_1    0x5A
#define GY_HEADER_2    0x5A

// Buffer para reconstrução do frame completo
static uint8_t  frame_buffer[2048];
static int      buffer_idx = 0;
static int      state = 0; 

// 24x32 em 8 bits (normalizado) – será enviado via UDP
static uint8_t  mlx_frame_u8[MLX_PIXELS];

// 24x32 em 16 bits (valores brutos)
static uint16_t mlx_raw16[MLX_PIXELS];

// WIFI (STA simples, SSID/senha fixos)
static void wifi_init_sta(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = "imperadorpaulo",
            .password = "12345678",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "WiFi STA inicializado; conectando ao AP \"%s\"", wifi_config.sta.ssid);
}

// UDP CLIENTE (envia para ESP32 servidor)
#define SERVER_IP  "172.21.119.51"
#define UDP_PORT   5005

static int udp_sock = -1;
static struct sockaddr_in server_addr;

static void udp_client_init(void)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Erro ao criar socket UDP client");
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port   = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    ESP_LOGI(TAG, "UDP client pronto para enviar para %s:%d", SERVER_IP, UDP_PORT);
}

static void send_frame_to_server(const uint8_t *frame)
{
    if (udp_sock < 0) {
        return;
    }

    int err = sendto(udp_sock, frame, MLX_PIXELS, 0,
                     (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Erro sendto UDP: %d", err);
    } else if (err != MLX_PIXELS) {
        ESP_LOGW(TAG, "sendto enviou tamanho diferente: %d", err);
    }
}

// UART do módulo GY-MCU90640
static void mlx_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = MLX_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        MLX_UART_NUM,
        MLX_UART_BUF, 
        0,   
        0,     
        NULL,      
        0           
    ));

    ESP_ERROR_CHECK(uart_param_config(MLX_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        MLX_UART_NUM,
        MLX_UART_TXPIN,
        MLX_UART_RXPIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_LOGI(TAG, "UART inicializada (num=%d, TX=%d, RX=%d, baud=%d)",
             MLX_UART_NUM, MLX_UART_TXPIN, MLX_UART_RXPIN, MLX_UART_BAUD);
}

// Task que lê bytes da UART e monta frames 24x32 (16 bits) → 8 bits + envia via UDP
static void uart_reader_task(void *arg)
{
    uint8_t chunk[64];

    while (1) {
        int len = uart_read_bytes(MLX_UART_NUM, chunk, sizeof(chunk), pdMS_TO_TICKS(10));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; i++) {
            uint8_t byte = chunk[i];

            switch (state) {
                case 0: // Header 1
                    if (byte == GY_HEADER_1) {
                        state = 1;
                        buffer_idx = 0;
                    }
                    break;

                case 1: // Header 2
                    if (byte == GY_HEADER_2) {
                        state = 2;
                    } else {
                        state = 0;
                    }
                    break;

                case 2:
                    // Acumula alguns bytes de configuração e pula para estado 3
                    frame_buffer[buffer_idx++] = byte;
                    if (buffer_idx >= 4) {
                        state = 3;
                    }
                    break;

                case 3: // Lendo dados de pixels
                    frame_buffer[buffer_idx++] = byte;

                    // Espera 4 bytes de config + 1536 de dados = 1540 (sem checksum)
                    if (buffer_idx >= (4 + MLX_FRAME_BYTES)) {
                        uint8_t *pData = &frame_buffer[4];

                        int16_t min_temp = 32000;
                        int16_t max_temp = -10000;

                        for (int p = 0; p < MLX_PIXELS; p++) {
                            uint8_t lo = pData[2 * p];
                            uint8_t hi = pData[2 * p + 1];
                            int16_t raw = (int16_t)((hi << 8) | lo);

                            mlx_raw16[p] = raw;

                            if (raw > -4000 && raw < 100000) {
                                if (raw < min_temp) min_temp = raw;
                                if (raw > max_temp) max_temp = raw;
                            }
                        }

                        float span = (max_temp - min_temp);
                        if (span <= 0) span = 1;

                        for (int p = 0; p < MLX_PIXELS; p++) {
                            int16_t val = (int16_t)mlx_raw16[p];

                            if (val < min_temp) val = min_temp;
                            if (val > max_temp) val = max_temp;

                            float normalized = ((float)(val - min_temp) / span) * 255.0f;
                            mlx_frame_u8[p] = (uint8_t)normalized;
                        }

                        ESP_LOGI(TAG, "Frame OK: Min=%d Max=%d", min_temp, max_temp);

                        // Envia o frame normalizado para a ESP32 SERVIDOR
                        send_frame_to_server(mlx_frame_u8);

                        // Reset para próximo frame
                        buffer_idx = 0;
                        state = 0;
                    }
                    break;
            }
        }
    }
}

// app_main
void app_main(void)
{
    memset(mlx_frame_u8, 0, sizeof(mlx_frame_u8));

    // Wi-Fi (STA) + UDP client
    wifi_init_sta();
    udp_client_init();

    // UART do módulo MLX
    mlx_uart_init();

    // Task que lê o sensor e envia via UDP
    xTaskCreate(uart_reader_task, "uart_reader_task", 4096, NULL, 7, NULL);
}
