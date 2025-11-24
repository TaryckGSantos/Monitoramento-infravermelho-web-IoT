// station_example_main.c (ESP-IDF v5.4.2)
// WebSocket /ws mandando 3 "frames" fake 24x32 em JSON

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

#include "esp_http_server.h" // HTTP server + WebSocket

#define TAG "MLX-SIM"

#define MLX_ROWS 24
#define MLX_COLS 32
#define MLX_PIXELS (MLX_ROWS * MLX_COLS)

// -----------------------------
// FRAMES FAKE
// -----------------------------
static uint8_t frame0[MLX_PIXELS];
static uint8_t frame1[MLX_PIXELS];
static uint8_t frame2[MLX_PIXELS];

static httpd_req_t *ws_req = NULL;
static char json_buf[4096];

static void init_fake_frames(void)
{
    // Gradiente horizontal
    for (int r = 0; r < MLX_ROWS; r++) {
        for (int c = 0; c < MLX_COLS; c++) {
            frame0[r * MLX_COLS + c] = (uint8_t)(c * 255 / (MLX_COLS - 1));
        }
    }

    // Gradiente vertical
    for (int r = 0; r < MLX_ROWS; r++) {
        for (int c = 0; c < MLX_COLS; c++) {
            frame1[r * MLX_COLS + c] = (uint8_t)(r * 255 / (MLX_ROWS - 1));
        }
    }

    // Checkerboard
    for (int r = 0; r < MLX_ROWS; r++) {
        for (int c = 0; c < MLX_COLS; c++) {
            frame2[r * MLX_COLS + c] = ((r / 4 + c / 4) & 1) ? 220 : 40;
        }
    }
}

// -----------------------------
// HTTP + WEBSOCKET
// -----------------------------
static httpd_handle_t ws_server = NULL;
static int ws_client_fd = -1; // socket do cliente atual

// Handler simples pra HTTP GET /
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP GET / recebido");
    const char *resp = "OK - HTTP server esta vivo\n";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL,
};

// Handler de /ws
static esp_err_t ws_handler(httpd_req_t *req)
{
    // 1ª chamada: handshake
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket OK");
        ws_req = req;
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
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
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

    pos += snprintf(json + pos, 4096 - pos,
                    "{\"rows\":24,\"cols\":32,\"frame\":[");

    for (int i = 0; i < MLX_PIXELS; i++) {
        pos += snprintf(json + pos, 4096 - pos,
                        "%s%u", (i ? "," : ""), frame[i]);
        if (pos >= 4096 - 10) break;
    }

    pos += snprintf(json + pos, 4096 - pos, "]}");

    httpd_ws_frame_t pkt = {
        .payload = (uint8_t *)json,
        .len = pos,
        .type = HTTPD_WS_TYPE_TEXT,
        .final = true
    };

    esp_err_t ret = httpd_ws_send_frame_async(ws_server, ws_client_fd, &pkt);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao enviar frame assíncrono: %s", esp_err_to_name(ret));

        if (ret == ESP_ERR_INVALID_STATE || ret == ESP_ERR_INVALID_ARG) {
            ws_client_fd = -1;
        }
    }
}

// Task que envia frame a cada 1s
static void frame_sender_task(void *arg)
{
    uint8_t *frames[3] = { frame0, frame1, frame2 };
    int idx = 0;

    while (1) {
        send_frame_over_ws(frames[idx]);
        idx = (idx + 1) % 3;
        vTaskDelay(pdMS_TO_TICKS(1000));
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
            .ssid = "Melk",
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
    init_fake_frames();
    wifi_init_sta();

    ws_server = start_websocket_server();

    if (ws_server) {
        ESP_LOGI(TAG, "Servidor WebSocket pronto em ws://<IP-DA-ESP>/ws");
    }

    xTaskCreate(frame_sender_task, "frame_sender_task", 4096, NULL, 5, NULL);
}
