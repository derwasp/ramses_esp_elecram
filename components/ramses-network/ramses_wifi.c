/********************************************************************
 * ramses_esp
 * ramses_wifi.c
 *
 * (C) 2023 Peter Price
 *
 * WiFi Station
 *
 */
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

static const char* TAG = "WIFI";
#include "esp_log.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "ramses_wifi.h"
#include "wifi_cmd.h"

#define WIFI_CONNECT_TIMEOUT_MS 20000
#define WIFI_RETRY_BACKOFF_MS 5000
#define WIFI_RESTART_DELAY_MS 250
#define WIFI_LINK_HEALTH_CHECK_MS 5000
#define WIFI_LINK_HEALTH_MAX_FAILS 3
#define WIFI_MANAGER_TICK_MS 250
#define WIFI_CONFIG_ERROR_STREAK_LIMIT 3

#define WIFI_REASON_CONNECT_TIMEOUT (-1)
#define WIFI_REASON_LINK_HEALTH (-2)
#define WIFI_REASON_LOST_IP (-3)
#define WIFI_REASON_API_ERROR (-4)
#define WIFI_REASON_MANUAL_RESTART (-5)
#define WIFI_REASON_EVENT_OVERFLOW (-6)

#define WIFI_CONNECTED_BIT BIT0

/* Map configuration to internal constants */
#if CONFIG_WPA3_SAE_PWE_HUNT_AND_PECK
#define SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define H2E_IDENTIFIER ""
#elif CONFIG_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define H2E_IDENTIFIER CONFIG_WIFI_PW_ID
#elif CONFIG_WPA3_SAE_PWE_BOTH
#define SAE_MODE WPA3_SAE_PWE_BOTH
#define H2E_IDENTIFIER CONFIG_WIFI_PW_ID
#endif

#if CONFIG_WIFI_AUTH_OPEN
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_WIFI_AUTH_WEP
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_WIFI_AUTH_WPA_PSK
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_WIFI_AUTH_WPA2_PSK
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_WIFI_AUTH_WPA_WPA2_PSK
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_WIFI_AUTH_WPA3_PSK
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_WIFI_AUTH_WPA2_WPA3_PSK
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_WIFI_AUTH_WAPI_PSK
#define SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define WIFI_STATE_LIST                     \
    WIFI_STATE(WIFI_IDLE, "Idle")           \
    WIFI_STATE(WIFI_STOPPED, "Stopped")     \
    WIFI_STATE(WIFI_STARTING, "Starting")   \
    WIFI_STATE(WIFI_CONNECTING, "Connecting") \
    WIFI_STATE(WIFI_ONLINE, "Online")       \
    WIFI_STATE(WIFI_BACKOFF, "Backoff")

#define WIFI_STATE(_e, _t) _e,
enum wifi_state {
    WIFI_STATE_LIST
        WIFI_STATE_MAX
};
#undef WIFI_STATE

static char const* wifi_state_text(enum wifi_state state)
{
    static char const* const state_text[WIFI_STATE_MAX] = {
#define WIFI_STATE(_e, _t) _t,
        WIFI_STATE_LIST
#undef WIFI_STATE
    };

    if (state < WIFI_STATE_MAX)
        return state_text[state];

    return "Unknown";
}

#define SSID_LEN 33
#define PASSWORD_LEN 65

enum wifi_msg_type {
    WIFI_MSG_SET_SSID,
    WIFI_MSG_SET_PASSWORD,
    WIFI_MSG_RESTART,
    WIFI_MSG_STATUS,
    WIFI_MSG_EVT_STA_START,
    WIFI_MSG_EVT_STA_DISCONNECTED,
    WIFI_MSG_EVT_GOT_IP,
    WIFI_MSG_EVT_LOST_IP,
};

struct wifi_msg {
    enum wifi_msg_type type;
    uint32_t generation;
    union wifi_msg_param {
        char ssid[SSID_LEN];
        char password[PASSWORD_LEN];
        struct {
            int reason;
        } disconnected;
        struct {
            esp_netif_ip_info_t ip_info;
        } got_ip;
    } param;
};

struct wifi_data {
    BaseType_t coreID;
    TaskHandle_t task;
    QueueHandle_t queue;

    enum wifi_state state;

    EventGroupHandle_t event_group;
    esp_event_handler_instance_t any_id;
    esp_event_handler_instance_t got_ip;
    esp_event_handler_instance_t lost_ip;

    wifi_config_t station_config;
    bool driver_started;
    uint32_t generation;
    uint32_t retry_num;
    TickType_t connect_deadline;
    TickType_t retry_after_tick;
    TickType_t link_health_check_tick;
    uint8_t link_health_failures;
    bool suppress_disconnect_once;
    bool suppress_lost_ip_once;
    bool event_overflow;
    uint8_t config_error_streak;
    int last_reason;
    esp_netif_ip_info_t ip_info;
    bool has_ip;
};

static struct wifi_data* wifi_ctxt(void)
{
    static struct wifi_data wifi = {
        .state = WIFI_IDLE,
        .station_config = {
            .sta = {
                .ssid = CONFIG_WIFI_SSID,
                .password = CONFIG_WIFI_PASSWORD,
                /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
                 * If you want to connect the device to deprecated WEP/WPA networks, please set the threshold value
                 * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching those standards.
                 */
                .threshold.authmode = SCAN_AUTH_MODE_THRESHOLD,
                .sae_pwe_h2e = SAE_MODE,
                .sae_h2e_identifier = H2E_IDENTIFIER,
            },
        },
    };

    static struct wifi_data* ctxt = NULL;
    if (!ctxt)
        ctxt = &wifi;

    return ctxt;
}

static bool wifi_has_config(struct wifi_data* ctxt)
{
    return ctxt && ctxt->station_config.sta.ssid[0] != '\0';
}

static void wifi_set_state(struct wifi_data* ctxt, enum wifi_state new_state)
{
    if (ctxt && ctxt->state != new_state) {
        ESP_LOGI(TAG, "state %s->%s", wifi_state_text(ctxt->state), wifi_state_text(new_state));
        ctxt->state = new_state;
    }
}

static void wifi_log_if_error(const char* op, esp_err_t err)
{
    if (err != ESP_OK)
        ESP_LOGW(TAG, "%s failed: %s (%d)", op, esp_err_to_name(err), (int)err);
}

static void wifi_copy_ssid_text(struct wifi_data* ctxt, char* buf, size_t buf_size)
{
    size_t len;

    if (!buf || buf_size == 0) {
        return;
    }

    if (!ctxt) {
        buf[0] = '\0';
        return;
    }

    len = strnlen((char const*)ctxt->station_config.sta.ssid, sizeof(ctxt->station_config.sta.ssid));
    if (len >= buf_size)
        len = buf_size - 1;

    memcpy(buf, ctxt->station_config.sta.ssid, len);
    buf[len] = '\0';
}

static char const* wifi_reason_text(int reason)
{
    switch (reason) {
    case WIFI_REASON_CONNECT_TIMEOUT:
        return "connect-timeout";
    case WIFI_REASON_LINK_HEALTH:
        return "link-health";
    case WIFI_REASON_LOST_IP:
        return "lost-ip";
    case WIFI_REASON_API_ERROR:
        return "api-error";
    case WIFI_REASON_MANUAL_RESTART:
        return "manual-restart";
    case WIFI_REASON_EVENT_OVERFLOW:
        return "event-overflow";
    default:
        return "disconnect";
    }
}

static bool wifi_reason_is_config_error(int reason)
{
    switch (reason) {
#ifdef WIFI_REASON_AUTH_EXPIRE
    case WIFI_REASON_AUTH_EXPIRE:
#endif
#ifdef WIFI_REASON_AUTH_FAIL
    case WIFI_REASON_AUTH_FAIL:
#endif
#ifdef WIFI_REASON_NOT_AUTHED
    case WIFI_REASON_NOT_AUTHED:
#endif
#ifdef WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
#endif
#ifdef WIFI_REASON_HANDSHAKE_TIMEOUT
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
#endif
#ifdef WIFI_REASON_802_1X_AUTH_FAILED
    case WIFI_REASON_802_1X_AUTH_FAILED:
#endif
#ifdef WIFI_REASON_AKMP_INVALID
    case WIFI_REASON_AKMP_INVALID:
#endif
#ifdef WIFI_REASON_UNSUPP_RSN_IE_VERSION
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
#endif
#ifdef WIFI_REASON_INVALID_RSN_IE_CAP
    case WIFI_REASON_INVALID_RSN_IE_CAP:
#endif
#ifdef WIFI_REASON_IE_IN_4WAY_DIFFERS
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:
#endif
#ifdef WIFI_REASON_GROUP_CIPHER_INVALID
    case WIFI_REASON_GROUP_CIPHER_INVALID:
#endif
#ifdef WIFI_REASON_PAIRWISE_CIPHER_INVALID
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
#endif
        return true;

    default:
        return false;
    }
}

static void wifi_clear_link_state(struct wifi_data* ctxt)
{
    if (!ctxt)
        return;

    ctxt->connect_deadline = 0;
    ctxt->link_health_failures = 0;
    ctxt->has_ip = false;
    memset(&ctxt->ip_info, 0, sizeof(ctxt->ip_info));

    if (ctxt->event_group)
        xEventGroupClearBits(ctxt->event_group, WIFI_CONNECTED_BIT);
}

static BaseType_t wifi_msg_post(struct wifi_msg* msg, TickType_t wait_ticks)
{
    struct wifi_data* ctxt = wifi_ctxt();

    if (!ctxt || !ctxt->queue)
        return pdFALSE;

    return xQueueSend(ctxt->queue, msg, wait_ticks);
}

static void wifi_msg_post_event(struct wifi_data* ctxt, struct wifi_msg* msg)
{
    if (!ctxt || !ctxt->queue || !msg)
        return;

    if (xQueueSend(ctxt->queue, msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        ctxt->event_overflow = true;
        ESP_LOGW(TAG, "dropping event %d", msg->type);
    }
}

bool wifi_is_connected(void)
{
    struct wifi_data* ctxt = wifi_ctxt();

    if (!ctxt || !ctxt->event_group)
        return false;

    return (xEventGroupGetBits(ctxt->event_group) & WIFI_CONNECTED_BIT) != 0;
}

static void wifi_store_ssid(struct wifi_data* ctxt, char const* ssid)
{
    size_t len;

    if (!ctxt || !ssid)
        return;

    memset(ctxt->station_config.sta.ssid, 0, sizeof(ctxt->station_config.sta.ssid));
    len = strnlen(ssid, sizeof(ctxt->station_config.sta.ssid));
    memcpy(ctxt->station_config.sta.ssid, ssid, len);
}

static void wifi_store_password(struct wifi_data* ctxt, char const* password)
{
    size_t len;

    if (!ctxt || !password)
        return;

    memset(ctxt->station_config.sta.password, 0, sizeof(ctxt->station_config.sta.password));
    len = strnlen(password, sizeof(ctxt->station_config.sta.password));
    memcpy(ctxt->station_config.sta.password, password, len);
}

void wifi_set_ssid(char* ssid)
{
    struct wifi_msg msg = {
        .type = WIFI_MSG_SET_SSID,
    };

    if (!ssid)
        return;

    strncpy(msg.param.ssid, ssid, sizeof(msg.param.ssid) - 1);
    msg.param.ssid[sizeof(msg.param.ssid) - 1] = '\0';
    (void)wifi_msg_post(&msg, portTICK_PERIOD_MS);
}

void wifi_set_password(char* password)
{
    struct wifi_msg msg = {
        .type = WIFI_MSG_SET_PASSWORD,
    };

    if (!password)
        return;

    strncpy(msg.param.password, password, sizeof(msg.param.password) - 1);
    msg.param.password[sizeof(msg.param.password) - 1] = '\0';
    (void)wifi_msg_post(&msg, portTICK_PERIOD_MS);
}

void wifi_restart(void)
{
    struct wifi_msg msg = {
        .type = WIFI_MSG_RESTART,
    };

    (void)wifi_msg_post(&msg, portTICK_PERIOD_MS);
}

void wifi_status(void)
{
    struct wifi_msg msg = {
        .type = WIFI_MSG_STATUS,
    };

    (void)wifi_msg_post(&msg, portTICK_PERIOD_MS);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    struct wifi_data* ctxt = arg;
    struct wifi_msg msg = {
        .generation = ctxt ? ctxt->generation : 0,
    };

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        msg.type = WIFI_MSG_EVT_STA_START;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        msg.type = WIFI_MSG_EVT_STA_DISCONNECTED;
        if (event_data) {
            wifi_event_sta_disconnected_t* disconn = (wifi_event_sta_disconnected_t*)event_data;
            msg.param.disconnected.reason = disconn->reason;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        msg.type = WIFI_MSG_EVT_GOT_IP;
        if (event_data) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            msg.param.got_ip.ip_info = event->ip_info;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        msg.type = WIFI_MSG_EVT_LOST_IP;
    } else {
        return;
    }

    wifi_msg_post_event(ctxt, &msg);
}

static void wifi_check_station(struct wifi_data* ctxt)
{
    wifi_config_t config;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    cfg.wifi_task_core_id = ctxt->coreID;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &config));

    if (config.sta.ssid[0] != '\0') {
        memcpy(ctxt->station_config.sta.ssid, config.sta.ssid, sizeof(ctxt->station_config.sta.ssid));
        ESP_LOGI(TAG, "SSID retrieved from NVS");
    }
    if (config.sta.password[0] != '\0') {
        memcpy(ctxt->station_config.sta.password, config.sta.password, sizeof(ctxt->station_config.sta.password));
        ESP_LOGI(TAG, "PASSWORD retrieved from NVS");
    }
}

static void wifi_create(struct wifi_data* ctxt)
{
    ctxt->event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, ctxt, &ctxt->any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, ctxt, &ctxt->got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, ctxt, &ctxt->lost_ip));
}

static void wifi_reset_driver_generation(struct wifi_data* ctxt, bool restart_driver)
{
    if (!ctxt)
        return;

    ctxt->generation++;
    ctxt->suppress_disconnect_once = true;
    ctxt->suppress_lost_ip_once = true;

    if (!ctxt->driver_started)
        return;

    if (restart_driver) {
        wifi_log_if_error("esp_wifi_stop", esp_wifi_stop());
        ctxt->driver_started = false;
    } else {
        wifi_log_if_error("esp_wifi_disconnect", esp_wifi_disconnect());
    }
}

static void wifi_begin_connect(struct wifi_data* ctxt)
{
    char ssid[SSID_LEN];
    TickType_t now = xTaskGetTickCount();
    esp_err_t err;

    /* Clear stale suppression flags from previous managed transitions. */
    ctxt->suppress_disconnect_once = false;
    ctxt->suppress_lost_ip_once = false;

    if (!wifi_has_config(ctxt)) {
        wifi_clear_link_state(ctxt);
        wifi_set_state(ctxt, WIFI_STOPPED);
        return;
    }

    wifi_copy_ssid_text(ctxt, ssid, sizeof(ssid));
    printf("# Attempting to connect to SSID:%s\n", ssid);

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_mode failed: %s (%d)", esp_err_to_name(err), (int)err);
        ctxt->last_reason = WIFI_REASON_API_ERROR;
        ctxt->retry_after_tick = now + pdMS_TO_TICKS(WIFI_RETRY_BACKOFF_MS);
        wifi_set_state(ctxt, WIFI_BACKOFF);
        return;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &ctxt->station_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_config failed: %s (%d)", esp_err_to_name(err), (int)err);
        ctxt->last_reason = WIFI_REASON_API_ERROR;
        ctxt->retry_after_tick = now + pdMS_TO_TICKS(WIFI_RETRY_BACKOFF_MS);
        wifi_set_state(ctxt, WIFI_BACKOFF);
        return;
    }

    wifi_clear_link_state(ctxt);
    ctxt->connect_deadline = now + pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS);
    ctxt->link_health_check_tick = now + pdMS_TO_TICKS(WIFI_LINK_HEALTH_CHECK_MS);

    if (!ctxt->driver_started) {
        err = esp_wifi_start();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_start failed: %s (%d)", esp_err_to_name(err), (int)err);
            ctxt->last_reason = WIFI_REASON_API_ERROR;
            ctxt->retry_after_tick = now + pdMS_TO_TICKS(WIFI_RETRY_BACKOFF_MS);
            wifi_set_state(ctxt, WIFI_BACKOFF);
            return;
        }

        ctxt->driver_started = true;
        wifi_set_state(ctxt, WIFI_STARTING);
        return;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_connect failed: %s (%d)", esp_err_to_name(err), (int)err);
        ctxt->last_reason = WIFI_REASON_API_ERROR;
        ctxt->retry_after_tick = now + pdMS_TO_TICKS(WIFI_RETRY_BACKOFF_MS);
        wifi_set_state(ctxt, WIFI_BACKOFF);
        return;
    }

    wifi_set_state(ctxt, WIFI_CONNECTING);
}

static void wifi_schedule_retry(struct wifi_data* ctxt, int reason, char const* source, bool reset_link)
{
    TickType_t now = xTaskGetTickCount();
    bool restart_driver = false;
    bool config_error;

    if (!ctxt)
        return;

    ctxt->last_reason = reason;
    wifi_clear_link_state(ctxt);
    config_error = wifi_reason_is_config_error(reason);
    if (config_error) {
        if (ctxt->config_error_streak < UINT8_MAX)
            ctxt->config_error_streak++;
    } else {
        ctxt->config_error_streak = 0;
    }

    if (config_error && ctxt->config_error_streak >= WIFI_CONFIG_ERROR_STREAK_LIMIT) {
        ESP_LOGE(TAG,
            "WiFi auth/config failure streak %u (%s/%d), stopping until manual restart or credential update",
            (unsigned int)ctxt->config_error_streak, wifi_reason_text(reason), reason);
        wifi_reset_driver_generation(ctxt, true);
        ctxt->retry_num = 0;
        wifi_set_state(ctxt, WIFI_STOPPED);
        return;
    }

    ctxt->retry_num++;
    if (ctxt->retry_num >= CONFIG_WIFI_MAXIMUM_RETRY) {
        restart_driver = true;
        ctxt->retry_num = 0;
    }

    if (restart_driver) {
        ESP_LOGW(TAG, "WiFi failure (%s, reason=%s/%d), restarting driver after %d attempts",
            source ? source : "unknown", wifi_reason_text(reason), reason, CONFIG_WIFI_MAXIMUM_RETRY);
        wifi_reset_driver_generation(ctxt, true);
    } else {
        if (reset_link)
            wifi_reset_driver_generation(ctxt, false);

        ESP_LOGW(TAG, "WiFi failure (%s, reason=%s/%d), retry %u/%d in %d ms",
            source ? source : "unknown", wifi_reason_text(reason), reason,
            (unsigned int)ctxt->retry_num, CONFIG_WIFI_MAXIMUM_RETRY, WIFI_RETRY_BACKOFF_MS);
    }

    ctxt->retry_after_tick = now + pdMS_TO_TICKS(WIFI_RETRY_BACKOFF_MS);
    wifi_set_state(ctxt, WIFI_BACKOFF);
}

static void wifi_request_restart(struct wifi_data* ctxt)
{
    TickType_t now = xTaskGetTickCount();

    ctxt->retry_num = 0;
    ctxt->config_error_streak = 0;
    ctxt->last_reason = WIFI_REASON_MANUAL_RESTART;
    wifi_clear_link_state(ctxt);
    wifi_reset_driver_generation(ctxt, true);
    ctxt->retry_after_tick = now + pdMS_TO_TICKS(WIFI_RESTART_DELAY_MS);
    wifi_set_state(ctxt, WIFI_BACKOFF);
}

static bool wifi_link_health_unhealthy(struct wifi_data* ctxt)
{
    TickType_t now = xTaskGetTickCount();
    wifi_ap_record_t ap_info;
    esp_err_t err;

    if ((int32_t)(now - ctxt->link_health_check_tick) < 0)
        return false;

    ctxt->link_health_check_tick = now + pdMS_TO_TICKS(WIFI_LINK_HEALTH_CHECK_MS);

    err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err == ESP_OK) {
        ctxt->link_health_failures = 0;
        return false;
    }

    ctxt->link_health_failures++;
    ESP_LOGW(TAG, "WiFi link health check failed: %s (%d), count %u/%u",
        esp_err_to_name(err), (int)err,
        (unsigned int)ctxt->link_health_failures, WIFI_LINK_HEALTH_MAX_FAILS);

    if (ctxt->link_health_failures >= WIFI_LINK_HEALTH_MAX_FAILS) {
        ctxt->link_health_failures = 0;
        return true;
    }

    return false;
}

static void wifi_print_status(struct wifi_data* ctxt)
{
    char ssid[SSID_LEN];
    int32_t retry_ms = 0;

    wifi_copy_ssid_text(ctxt, ssid, sizeof(ssid));
    if (ctxt->state == WIFI_BACKOFF) {
        retry_ms = (int32_t)pdTICKS_TO_MS(ctxt->retry_after_tick - xTaskGetTickCount());
        if (retry_ms < 0)
            retry_ms = 0;
    }

    printf("# WiFi state: %s\n", wifi_state_text(ctxt->state));
    printf("# WiFi SSID: %s\n", ssid[0] ? ssid : "<unset>");
    printf("# WiFi connected: %s\n", wifi_is_connected() ? "yes" : "no");
    printf("# WiFi driver started: %s\n", ctxt->driver_started ? "yes" : "no");
    printf("# WiFi retries: %u/%d\n", (unsigned int)ctxt->retry_num, CONFIG_WIFI_MAXIMUM_RETRY);
    printf("# WiFi config error streak: %u/%u\n",
        (unsigned int)ctxt->config_error_streak, WIFI_CONFIG_ERROR_STREAK_LIMIT);
    printf("# WiFi last reason: %s (%d)\n", wifi_reason_text(ctxt->last_reason), ctxt->last_reason);
    printf("# WiFi generation: %u\n", (unsigned int)ctxt->generation);
    printf("# WiFi event overflow pending: %s\n", ctxt->event_overflow ? "yes" : "no");
    if (ctxt->has_ip) {
        printf("# WiFi IP: " IPSTR "\n", IP2STR(&ctxt->ip_info.ip));
    } else {
        printf("# WiFi IP: <none>\n");
    }
    if (ctxt->state == WIFI_BACKOFF)
        printf("# WiFi next retry in: %" PRId32 " ms\n", retry_ms);
}

static void wifi_handle_sta_start(struct wifi_data* ctxt, struct wifi_msg const* msg)
{
    esp_err_t err;

    if (!ctxt || !msg || msg->generation != ctxt->generation)
        return;

    if (ctxt->state != WIFI_STARTING)
        return;

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_connect failed after start: %s (%d)", esp_err_to_name(err), (int)err);
        ctxt->last_reason = WIFI_REASON_API_ERROR;
        ctxt->retry_after_tick = xTaskGetTickCount() + pdMS_TO_TICKS(WIFI_RETRY_BACKOFF_MS);
        wifi_set_state(ctxt, WIFI_BACKOFF);
        return;
    }

    wifi_set_state(ctxt, WIFI_CONNECTING);
}

static void wifi_handle_got_ip(struct wifi_data* ctxt, struct wifi_msg const* msg)
{
    char ssid[SSID_LEN];

    if (!ctxt || !msg || msg->generation != ctxt->generation)
        return;

    ctxt->ip_info = msg->param.got_ip.ip_info;
    ctxt->has_ip = true;
    ctxt->retry_num = 0;
    ctxt->config_error_streak = 0;
    ctxt->link_health_failures = 0;
    ctxt->connect_deadline = 0;
    ctxt->link_health_check_tick = xTaskGetTickCount() + pdMS_TO_TICKS(WIFI_LINK_HEALTH_CHECK_MS);
    ctxt->suppress_disconnect_once = false;
    ctxt->suppress_lost_ip_once = false;

    if (ctxt->event_group)
        xEventGroupSetBits(ctxt->event_group, WIFI_CONNECTED_BIT);

    wifi_copy_ssid_text(ctxt, ssid, sizeof(ssid));
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&ctxt->ip_info.ip));
    printf("# Connected to SSID:%s\n", ssid);
    wifi_set_state(ctxt, WIFI_ONLINE);
}

static void wifi_handle_disconnected(struct wifi_data* ctxt, struct wifi_msg const* msg)
{
    char ssid[SSID_LEN];
    bool was_connecting;

    if (!ctxt || !msg || msg->generation != ctxt->generation)
        return;

    if (ctxt->suppress_disconnect_once) {
        ctxt->suppress_disconnect_once = false;
        ESP_LOGI(TAG, "ignoring managed disconnect (reason=%d)", msg->param.disconnected.reason);
        return;
    }

    was_connecting = (ctxt->state == WIFI_STARTING || ctxt->state == WIFI_CONNECTING);
    wifi_copy_ssid_text(ctxt, ssid, sizeof(ssid));

    ESP_LOGW(TAG, "station disconnected (reason=%d)", msg->param.disconnected.reason);
    if (was_connecting)
        printf("# Failed to connect to SSID:%s\n", ssid);

    wifi_schedule_retry(ctxt, msg->param.disconnected.reason, "disconnect", false);
}

static void wifi_handle_lost_ip(struct wifi_data* ctxt, struct wifi_msg const* msg)
{
    if (!ctxt || !msg || msg->generation != ctxt->generation)
        return;

    if (ctxt->suppress_lost_ip_once) {
        ctxt->suppress_lost_ip_once = false;
        ESP_LOGI(TAG, "ignoring managed lost-ip event");
        return;
    }

    ESP_LOGW(TAG, "lost IP, forcing reconnect");
    wifi_schedule_retry(ctxt, WIFI_REASON_LOST_IP, "lost-ip", true);
}

static void wifi_handle_message(struct wifi_data* ctxt, struct wifi_msg const* msg)
{
    if (!ctxt || !msg)
        return;

    switch (msg->type) {
    case WIFI_MSG_SET_SSID:
        wifi_store_ssid(ctxt, msg->param.ssid);
        printf("# WiFi SSID updated, run 'wifi restart' to apply\n");
        break;

    case WIFI_MSG_SET_PASSWORD:
        wifi_store_password(ctxt, msg->param.password);
        printf("# WiFi password updated, run 'wifi restart' to apply\n");
        break;

    case WIFI_MSG_RESTART:
        printf("# WiFi restart requested\n");
        wifi_request_restart(ctxt);
        break;

    case WIFI_MSG_STATUS:
        wifi_print_status(ctxt);
        break;

    case WIFI_MSG_EVT_STA_START:
        wifi_handle_sta_start(ctxt, msg);
        break;

    case WIFI_MSG_EVT_STA_DISCONNECTED:
        wifi_handle_disconnected(ctxt, msg);
        break;

    case WIFI_MSG_EVT_GOT_IP:
        wifi_handle_got_ip(ctxt, msg);
        break;

    case WIFI_MSG_EVT_LOST_IP:
        wifi_handle_lost_ip(ctxt, msg);
        break;

    default:
        break;
    }
}

static void wifi_poll(struct wifi_data* ctxt)
{
    TickType_t now = xTaskGetTickCount();

    if (!ctxt)
        return;

    if (ctxt->event_overflow && ctxt->state != WIFI_IDLE && ctxt->state != WIFI_STOPPED) {
        ctxt->event_overflow = false;
        ESP_LOGW(TAG, "event queue overflow detected, forcing reconnect cycle");
        wifi_schedule_retry(ctxt, WIFI_REASON_EVENT_OVERFLOW, "event-queue-overflow", true);
        return;
    }

    switch (ctxt->state) {
    case WIFI_IDLE:
    case WIFI_STOPPED:
        break;

    case WIFI_STARTING:
    case WIFI_CONNECTING:
        if (ctxt->connect_deadline != 0 && (int32_t)(now - ctxt->connect_deadline) >= 0) {
            ESP_LOGW(TAG, "WiFi connect attempt timed out");
            wifi_schedule_retry(ctxt, WIFI_REASON_CONNECT_TIMEOUT, "connect-timeout", true);
        }
        break;

    case WIFI_ONLINE:
        if (wifi_link_health_unhealthy(ctxt)) {
            ESP_LOGW(TAG, "WiFi link health watchdog triggered recovery");
            wifi_schedule_retry(ctxt, WIFI_REASON_LINK_HEALTH, "health-check", true);
        }
        break;

    case WIFI_BACKOFF:
        if ((int32_t)(now - ctxt->retry_after_tick) >= 0)
            wifi_begin_connect(ctxt);
        break;

    default:
        break;
    }
}

static void Wifi(void* param)
{
    struct wifi_data* ctxt = param;

    ESP_LOGI(TAG, "Task Started");

    ctxt->queue = xQueueCreate(24, sizeof(struct wifi_msg));

    wifi_check_station(ctxt);
    wifi_create(ctxt);
    wifi_clear_link_state(ctxt);

    if (wifi_has_config(ctxt)) {
        ctxt->retry_after_tick = xTaskGetTickCount();
        wifi_set_state(ctxt, WIFI_BACKOFF);
    } else {
        wifi_set_state(ctxt, WIFI_STOPPED);
    }

    for (;;) {
        struct wifi_msg msg;

        if (xQueueReceive(ctxt->queue, &msg, pdMS_TO_TICKS(WIFI_MANAGER_TICK_MS)) == pdTRUE) {
            wifi_handle_message(ctxt, &msg);
            while (xQueueReceive(ctxt->queue, &msg, 0) == pdTRUE)
                wifi_handle_message(ctxt, &msg);
        }

        wifi_poll(ctxt);
#if CONFIG_IDF_TARGET_ESP32C6
        vTaskDelay(pdMS_TO_TICKS(1));
#endif
    }
}

WIFI_HNDL ramses_wifi_init(BaseType_t coreID)
{
    struct wifi_data* ctxt = wifi_ctxt();

    ctxt->coreID = coreID;
    esp_log_level_set(TAG, CONFIG_WIFI_LOG_LEVEL);

    wifi_register();
    xTaskCreatePinnedToCore(Wifi, "WiFi", 4096, ctxt, 10, &ctxt->task, ctxt->coreID);

    return ctxt;
}
