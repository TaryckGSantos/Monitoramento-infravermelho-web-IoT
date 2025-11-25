// station_example_main.c (ESP-IDF v5.4.2)
// ESP32 → recebe frames 24x32 via UART e envia em JSON via WebSocket /ws

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

#include "esp_http_server.h"    // HTTP server + WebSocket
#include "driver/uart.h"
#include "driver/gpio.h"

#define TAG "MLX-WS"

#define MLX_ROWS   24
#define MLX_COLS   32
#define MLX_PIXELS (MLX_ROWS * MLX_COLS)

// -----------------------------
// UART do módulo GY-MCU90640
// (ajuste os pinos se o seu mapeamento anterior for diferente)
// -----------------------------
#define MLX_UART_NUM   UART_NUM_1
#define MLX_UART_TXPIN GPIO_NUM_17   // TX da ESP32  -> RX do módulo
#define MLX_UART_RXPIN GPIO_NUM_16   // RX da ESP32  -> TX do módulo
#define MLX_UART_BAUD  115200
#define MLX_UART_BUF   2048

// Definições do protocolo GY-MCU90640
#define GY_HEADER_1    0x5A
#define GY_HEADER_2    0x5A
#define GY_MODE_TEMP   0x14 // Modo de saída de temperatura
#define MLX_PIXELS     768  // 24 * 32

// Buffer para reconstrução
static uint8_t  frame_buffer[2048]; 
static int      buffer_idx = 0;
static int      state = 0; // 0:Busca H1, 1:Busca H2, 2:Busca Mode, 3:Lê Dados

#define MLX_FRAME_BYTES (MLX_PIXELS * 2)    // 24*32*2 = 1536 bytes

// 24x32 em 8 bits (vai pro WebSocket)
static uint8_t  mlx_frame_u8[MLX_PIXELS];

// 24x32 em 16 bits (valores brutos)
static uint16_t mlx_raw16[MLX_PIXELS];

static uint8_t  uart_rxbuf[MLX_UART_BUF];
// frame bruto vindo da UART (16 bits/pixel)
static uint8_t  uart_framebuf[MLX_FRAME_BYTES];

// -----------------------------
// HTTP + WEBSOCKET
// -----------------------------
static httpd_handle_t ws_server = NULL;
static int ws_client_fd = -1; // socket do cliente atual

static char json_buf[4096];

// Handler simples pra HTTP GET /
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP GET / recebido");
    const char *resp = "OK - HTTP server esta vivo\n";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL,
};

// Handler de /ws
static esp_err_t ws_handler(httpd_req_t *req)
{
    // 1ª chamada: handshake
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket OK");
        ws_client_fd = httpd_req_to_sockfd(req);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // Primeiro descobre o tamanho
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame (len) = %d", ret);
        return ret;
    }

    if (ws_pkt.len > 0) {
        uint8_t *buf = calloc(1, ws_pkt.len + 1);
        if (!buf) {
            ESP_LOGE(TAG, "Falha calloc");
            return ESP_ERR_NO_MEM;
        }

        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame (data) = %d", ret);
            free(buf);
            return ret;
        }

        ESP_LOGI(TAG, "Recebi do cliente WS: %s", (char *)ws_pkt.payload);
        free(buf);
    }

    return ESP_OK;
}

// Inicia HTTP server + WebSocket
static httpd_handle_t start_websocket_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao iniciar httpd");
        return NULL;
    }

    // 1. Handler normal da raiz
    httpd_register_uri_handler(server, &root_uri);

    // 2. Handler de WebSocket
    static const httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .user_ctx     = NULL,
        .is_websocket = true
    };

    httpd_register_uri_handler(server, &ws_uri);

    ESP_LOGI(TAG, "Servidor HTTP + WebSocket rodando na porta 80");
    return server;
}

// Envia 1 frame pro cliente conectado
static void send_frame_over_ws(const uint8_t *frame)
{
    if (!ws_server || ws_client_fd < 0) {
        ESP_LOGW(TAG, "Nenhum cliente WebSocket conectado");
        return;
    }

    int pos = 0;
    char *json = json_buf;

    pos += snprintf(json + pos, sizeof(json_buf) - pos,
                    "{\"rows\":24,\"cols\":32,\"frame\":[");

    for (int i = 0; i < MLX_PIXELS; i++) {
        pos += snprintf(json + pos, sizeof(json_buf) - pos,
                        "%s%u", (i ? "," : ""), frame[i]);
        if (pos >= (int)sizeof(json_buf) - 10) {
            break;
        }
    }

    pos += snprintf(json + pos, sizeof(json_buf) - pos, "]}");

    httpd_ws_frame_t pkt = {
        .payload = (uint8_t *)json,
        .len     = pos,
        .type    = HTTPD_WS_TYPE_TEXT,
        .final   = true
    };

    esp_err_t ret = httpd_ws_send_frame_async(ws_server, ws_client_fd, &pkt);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao enviar frame assíncrono: %s", esp_err_to_name(ret));

        if (ret == ESP_ERR_INVALID_STATE || ret == ESP_ERR_INVALID_ARG) {
            ws_client_fd = -1;
        }
    }
}

// Task que envia o último frame a cada ~200 ms
static void frame_sender_task(void *arg)
{
    while (1) {
        send_frame_over_ws(mlx_frame_u8);
        vTaskDelay(pdMS_TO_TICKS(200)); // ~5 fps para o navegador
    }
}

// -----------------------------
// UART do módulo GY-MCU90640
// -----------------------------
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
        MLX_UART_BUF,   // rx_buffer_size
        0,              // tx_buffer_size (0 = sem buffer)
        0,              // queue_size
        NULL,           // uart_queue
        0               // intr_alloc_flags
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

// Task que lê bytes da UART e monta frames 24x32 (16 bits) → escala para 0..255
static void uart_reader_task(void *arg)
{
    uint8_t chunk[64]; // Ler em pequenos pedaços ou byte a byte
    
    while (1) {
        int len = uart_read_bytes(MLX_UART_NUM, chunk, sizeof(chunk), pdMS_TO_TICKS(10));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            uint8_t byte = chunk[i];

            switch (state) {
                case 0: // Header 1
                    if (byte == GY_HEADER_1) state = 1;
                    break;
                
                case 1: // Header 2
                    if (byte == GY_HEADER_2) state = 2;
                    else state = 0; // Falso alarme, reseta
                    break;

                case 2: // High Byte do Tamanho ou Modo
                    // O GY-MCU90640 varia um pouco entre versões, mas geralmente
                    // após 5A 5A vem 02 06 (tamanho) ou o modo. 
                    // Vamos pular os bytes de configuração e ir direto ler o payload.
                    // O jeito mais seguro é assumir que após 5A 5A vem 4 bytes de config
                    // e depois os dados.
                    
                    // Simplificação: Vamos acumular no buffer até ter o frame todo
                    frame_buffer[buffer_idx++] = byte;
                    if (buffer_idx >= 4) { // Já passamos o cabeçalho de config (4 bytes aprox)
                         state = 3;
                    }
                    break;

                case 3: // Lendo dados de pixels
                    frame_buffer[buffer_idx++] = byte;
                    
                    // Precisamos de 1536 bytes de dados REAIS
                    // O offset é geralmente 4 bytes após o 5A 5A.
                    // Total esperado: 4 (config) + 1536 (pixels) + 1 (checksum) = 1541 bytes
                    if (buffer_idx >= (4 + 1536)) {
                        
                        // --- Processamento do Frame Completo ---
                        
                        // O array de pixels começa no índice 4 do frame_buffer
                        uint8_t *pData = &frame_buffer[4]; 
                        
                        // Variaveis para auto-scale
                        int16_t min_temp = 32000;
                        int16_t max_temp = -10000;

                        for (int p = 0; p < MLX_PIXELS; p++) {
                            // A fórmula do GY-MCU90640 geralmente é:
                            // Temp = (High << 8) | Low
                            // A temperatura vem em Centígrados * 100 (ex: 3055 = 30.55°C)
                            
                            uint8_t lo = pData[2 * p];
                            uint8_t hi = pData[2 * p + 1];
                            int16_t raw = (int16_t)((hi << 8) | lo);
                            
                            // Correção de temperatura (opcional divide por 100 no JS ou aqui)
                            // Para visualização 0-255, usamos o valor cru mesmo
                            
                            mlx_raw16[p] = raw; 

                            // Filtro básico de ruído: Ignorar valores absurdos (ex: 0 absolute ou erro)
                            // Sensores térmicos as vezes dão 0 ou max int em erro.
                            if (raw > -4000 && raw < 100000) { 
                                if (raw < min_temp) min_temp = raw;
                                if (raw > max_temp) max_temp = raw;
                            }
                        }

                        // Calcular span para normalização
                        float span = (max_temp - min_temp);
                        if (span <= 0) span = 1;

                        // Converter para 8 bits (Heatmap grayscale)
                        for (int p = 0; p < MLX_PIXELS; p++) {
                             int16_t val = (int16_t)mlx_raw16[p];
                             
                             // Clamp para evitar estouro por ruído
                             if (val < min_temp) val = min_temp;
                             if (val > max_temp) val = max_temp;

                             float normalized = ((float)(val - min_temp) / span) * 255.0f;
                             mlx_frame_u8[p] = (uint8_t)normalized;
                        }
                        
                        ESP_LOGI(TAG, "Frame OK: Min=%d Max=%d", min_temp, max_temp);

                        // Resetar para o próximo frame
                        buffer_idx = 0;
                        state = 0;
                    }
                    break;
            }
        }
    }
}

// -----------------------------
// WIFI (STA simples, com SSID/senha fixos)
// -----------------------------
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
            .ssid     = "Melk",
            .password = "GMUH2021*",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "WiFi STA inicializado; conectando ao AP \"%s\"", wifi_config.sta.ssid);
}

// -----------------------------
// app_main
// -----------------------------
void app_main(void)
{
    // Zera frame inicial
    memset(mlx_frame_u8, 0, sizeof(mlx_frame_u8));

    // Wi-Fi + HTTP + WebSocket
    wifi_init_sta();

    ws_server = start_websocket_server();
    if (!ws_server) {
        ESP_LOGE(TAG, "Falha ao iniciar servidor HTTP/WS");
        return;
    }
    ESP_LOGI(TAG, "Servidor WebSocket pronto em ws://<IP-DA-ESP>/ws");

    // UART do módulo
    mlx_uart_init();

    // Tasks
    xTaskCreate(uart_reader_task,  "uart_reader_task",  4096, NULL, 7, NULL);
    xTaskCreate(frame_sender_task, "frame_sender_task", 4096, NULL, 5, NULL);
}
