// esp32_server_main.c
// recebe frames 24x32 em 8 bits via UDP, guarda em mlx_frame_u8 e envia em JSON via WebSocket /ws para o navegador.

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
#include "esp_timer.h"   
#include "esp_sntp.h"       

#include "esp_http_server.h"    // HTTP server + WebSocket
#include "esp_http_client.h"    

#include "esp_crt_bundle.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#define TAG "MLX-WS-SERVER"

#define MLX_ROWS   24
#define MLX_COLS   32
#define MLX_PIXELS (MLX_ROWS * MLX_COLS)

#define FIREBASE_BODY_MAX 24000

static uint8_t mlx_frame_u8[MLX_PIXELS];

// API KEY 
#define FIREBASE_API_KEY "AIzaSyBylZrRkFf0xTUNqaCx7bMogndO0qEEQ_8"

// Endpoint documents.create da coleção "frames"
#define FIREBASE_URL "https://firestore.googleapis.com/v1/projects/" \
                     "mlx-thermal-monitor/databases/(default)/documents/frames?key=" FIREBASE_API_KEY

static void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "GOT IP: " IPSTR ", GW: " IPSTR ", MASK: " IPSTR,
             IP2STR(&ip_info->ip),
             IP2STR(&ip_info->gw),
             IP2STR(&ip_info->netmask));
}

// SNTP | acertar data/hora via NTP
static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Hora sincronizada via SNTP");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Inicializando SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "br.pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

// Envia um frame 24x32 (8 bits) para Firestore
static void firebase_send_frame(const uint8_t *frame)
{
    // {
    //   "fields": {
    //     "rows": {"integerValue":"24"},
    //     "cols": {"integerValue":"32"},
    //     "timestamp": {"stringValue":"boot_us=123456789"},
    //     "frame": {
    //       "arrayValue": {
    //         "values":[{"integerValue":"0"}, ...]
    //       }
    //     }
    //   }
    // }

    // timestamp em ISO UTC
    time_t now;
    time(&now);

    struct tm tm_utc;
    gmtime_r(&now, &tm_utc);

    char ts_str[64];
    strftime(ts_str, sizeof(ts_str), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);

    char *body = malloc(FIREBASE_BODY_MAX);
    if (!body) {
        ESP_LOGE(TAG, "firebase_send_frame: malloc falhou");
        return;
    }

    int pos = 0;
    pos += snprintf(body + pos, FIREBASE_BODY_MAX - pos,
                    "{"
                    "\"fields\":{"
                    "\"rows\":{\"integerValue\":\"%d\"},"
                    "\"cols\":{\"integerValue\":\"%d\"},"
                    "\"timestamp\":{\"timestampValue\":\"%s\"},"
                    "\"frame\":{"
                        "\"arrayValue\":{"
                        "\"values\":[",
                    MLX_ROWS, MLX_COLS, ts_str);

    for (int i = 0; i < MLX_PIXELS && pos < (FIREBASE_BODY_MAX - 100); i++) {
        pos += snprintf(body + pos, FIREBASE_BODY_MAX - pos,
                        "%s{\"integerValue\":\"%u\"}",
                        (i ? "," : ""), frame[i]);
    }

    pos += snprintf(body + pos, FIREBASE_BODY_MAX - pos,
                    "]"
                    "}"
                    "}"
                    "}"
                    "}");

    esp_http_client_config_t config = {
        .url = FIREBASE_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .cert_pem = NULL,                   
        .skip_cert_common_name_check = true  
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "firebase_send_frame: esp_http_client_init falhou");
        free(body);
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, pos);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "Firebase POST OK, HTTP status = %d", status);
    } else {
        ESP_LOGE(TAG, "Erro ao enviar para Firebase: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(body);
}

// Task que a cada ~10 s envia o último frame para o Firebase
static void firebase_uploader_task(void *arg)
{
    uint8_t local_copy[MLX_PIXELS];

    time_t now = 0;
    struct tm timeinfo = {0};

    while (1) {
        time(&now);
        gmtime_r(&now, &timeinfo);
        if (timeinfo.tm_year >= (2020 - 1900)) {
            break; 
        }
        ESP_LOGI(TAG, "Aguardando sincronizacao SNTP...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    while (1) {
        memcpy(local_copy, mlx_frame_u8, MLX_PIXELS);
        firebase_send_frame(local_copy);
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10 s
    }
}

// HTTP + WEBSOCKET
static httpd_handle_t ws_server = NULL;
static int ws_client_fd = -1;

static char json_buf[4096];

// Handler simples pra HTTP GET
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
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket OK");
        ws_client_fd = httpd_req_to_sockfd(req);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

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

    httpd_register_uri_handler(server, &root_uri);

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
        // Nenhum cliente WebSocket conectado
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

// Task que envia o último frame
static void frame_sender_task(void *arg)
{
    while (1) {
        send_frame_over_ws(mlx_frame_u8);
        vTaskDelay(pdMS_TO_TICKS(200)); // ~5 fps
    }
}

// WIFI (STA simples, com SSID/senha fixos)
static void wifi_init_sta(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &on_got_ip,
        NULL,
        NULL
    ));


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

// UDP SERVIDOR (recebe frames do SENSOR)
#define UDP_PORT 5005

static void udp_receiver_task(void *arg)
{
    int sock = -1;
    struct sockaddr_in dest_addr;

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(UDP_PORT);

    while (1) {
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGE(TAG, "Erro ao criar socket UDP");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Erro bind UDP: %d", err);
                close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            ESP_LOGI(TAG, "UDP servidor escutando na porta %d", UDP_PORT);
        }

        uint8_t buf[MLX_PIXELS];
        int len = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);

        if (len < 0) {
            ESP_LOGE(TAG, "Erro recvfrom: %d", len);
            close(sock);
            sock = -1;
            continue;
        }

        if (len == MLX_PIXELS) {
            memcpy(mlx_frame_u8, buf, MLX_PIXELS);
            ESP_LOGI(TAG, "Frame recebido via UDP (%d bytes)", len);
        } else {
            ESP_LOGW(TAG, "Pacote UDP tamanho inesperado: %d", len);
        }

    }
}

// app_main
void app_main(void)
{
    memset(mlx_frame_u8, 0, sizeof(mlx_frame_u8));

    // Wi-Fi + HTTP + WebSocket
    wifi_init_sta();

    // começa a sincronizar o relógio
    initialize_sntp(); 

    ws_server = start_websocket_server();
    if (!ws_server) {
        ESP_LOGE(TAG, "Falha ao iniciar servidor HTTP/WS");
        return;
    }

    ESP_LOGI(TAG, "Servidor WebSocket pronto em ws://<IP-DA-ESP-SERVIDOR>/ws");

    // Task que recebe frames via UDP
    xTaskCreate(udp_receiver_task, "udp_receiver_task", 4096, NULL, 6, NULL);

    // Task que envia frames para o navegador via WebSocket
    xTaskCreate(frame_sender_task, "frame_sender_task", 4096, NULL, 5, NULL);

    // Task que envia um frame a cada 10 s para o Firebase
    xTaskCreate(firebase_uploader_task, "firebase_uploader_task", 6144, NULL, 4, NULL);
}
