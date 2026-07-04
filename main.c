#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "driver/usb_serial_jtag.h"

#define DEBUG_MODE        0
#define N_DEVICES         4
#define WIFI_CHANNEL      11
#define ESPNOW_RATE       WIFI_PHY_RATE_MCS0_LGI

#define CONFIG_CSI_LOG_BINARY 1

#if CONFIG_CSI_LOG_BINARY
#define BOOT_LOG(...)   /* silenced in binary mode */
#else
#define BOOT_LOG(...)   esp_rom_printf(__VA_ARGS__)
#endif


// =========================================================================
// FIX #1: Thêm round_id vào header binary để đảm bảo nhất quán
// (packed struct giống spec trong PDF)
// =========================================================================
typedef struct __attribute__((packed)) {
    uint32_t magic;      // 0x43534921 ("!ISC" little-endian -> "CSI!")
    uint32_t round_id;
    uint8_t  tx_id;
    uint8_t  rx_id;
    int8_t   rssi;
    uint16_t csi_len;
} binary_csi_header_t;

static uint8_t mac_table[N_DEVICES][6] = {
    {0x88, 0x56, 0xA6, 0x3A, 0x0E, 0x2C},
    {0x88, 0x56, 0xA6, 0x3A, 0xE3, 0x68},
    {0x88, 0x56, 0xA6, 0x3A, 0xF7, 0x98},
    {0x88, 0x56, 0xA6, 0x3A, 0xF2, 0x18},
};

static uint8_t g_device_id = 0xFF;
static volatile uint32_t g_round_id = 0xFFFFFFFF;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  tx_id;
    uint32_t round_id;
    uint16_t seq;
} probe_pkt_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  tx_id;
    uint16_t seq;
} ack_pkt_t;

typedef struct {
    uint8_t tx_id;
    uint8_t rx_id;
} tdma_slot_t;

#define N_SLOTS 12

static const tdma_slot_t schedule_table[N_SLOTS] = {
    {0, 1}, {0, 2}, {0, 3},
    {1, 0}, {1, 2}, {1, 3},
    {2, 0}, {2, 1}, {2, 3},
    {3, 0}, {3, 1}, {3, 2}
};

static volatile uint8_t g_slot_idx = 0xFF;
static uint32_t g_slot_us = 2000;
static esp_timer_handle_t slot_timer = NULL;
static TaskHandle_t tx_task_handle = NULL;

static int mac_to_id(const uint8_t *mac) {
    for (int i = 0; i < N_DEVICES; i++) {
        if (memcmp(mac, mac_table[i], 6) == 0) return i;
    }
    return -1;
}

// =========================================================================
// FIX #1 (tiếp theo): Thêm round_id vào csi_record_t để capture ngay
// tại thời điểm nhận CSI, tránh race condition khi dequeue
// =========================================================================
typedef struct {
    uint8_t  tx_id;
    uint8_t  rx_id;
    uint32_t round_id;   // <-- THÊM MỚI: capture tại wifi_csi_cb
    int64_t  ts_us;
    int8_t   rssi;
    uint16_t n_bytes;
    int8_t   raw[256];
} csi_record_t;

static QueueHandle_t csi_queue;
static QueueHandle_t ack_queue = NULL;

// =========================================================================
// CRC16-CCITT cho data integrity trên USB serial
// =========================================================================
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static void wifi_csi_cb(void *ctx, wifi_csi_info_t *info) {
    if (!info || !info->buf) return;

    // Lọc nhiễu: bỏ qua gói quá ngắn (ACK, rác)
    if (info->len < 64) return;

    int tx_id = mac_to_id(info->mac);
    if (tx_id < 0 || tx_id == g_device_id) return;

    int64_t now_us = esp_timer_get_time();
    static int64_t last_csi_ts[N_DEVICES] = {0, 0, 0, 0};
    if (now_us - last_csi_ts[tx_id] < 1000) return;  // debounce 1ms cho slot 3.5ms
    last_csi_ts[tx_id] = now_us;

    csi_record_t rec;
    rec.tx_id    = (uint8_t)tx_id;
    rec.rx_id    = (uint8_t)g_device_id;
    rec.ts_us    = now_us;
    rec.rssi     = info->rx_ctrl.rssi;

    // =========================================================================
    // FIX #1: Gắn round_id NGAY TẠI ĐÂY (trong callback), không phải khi dequeue
    // =========================================================================
    rec.round_id = (g_round_id != 0xFFFFFFFF) ? g_round_id : 0;

    int n_bytes = info->len;
    if (n_bytes > 256) n_bytes = 256;

    // =========================================================================
    // FIX #3: Kiểm tra first_word_invalid — zero hóa 4 byte đầu nếu cờ bật
    // =========================================================================
    memcpy(rec.raw, info->buf, n_bytes);
    if (info->first_word_invalid) {
        rec.raw[0] = 0;
        rec.raw[1] = 0;
        rec.raw[2] = 0;
        rec.raw[3] = 0;
    }
    rec.n_bytes = (uint16_t)n_bytes;

    xQueueSend(csi_queue, &rec, 0);
}

static void csi_print_task(void *arg) {
    csi_record_t rec;
    while (1) {
        if (xQueueReceive(csi_queue, &rec, portMAX_DELAY) == pdTRUE) {
#if CONFIG_CSI_LOG_BINARY
            // =========================================================================
            // FIX #1: Dùng rec.round_id (đã capture tại callback), không đọc lại global
            // =========================================================================
            binary_csi_header_t header = {
                .magic    = 0x43534921,
                .round_id = rec.round_id,   // <-- dùng từ record, không phải g_round_id
                .tx_id    = rec.tx_id,
                .rx_id    = rec.rx_id,
                .rssi     = rec.rssi,
                .csi_len  = rec.n_bytes,
            };
            usb_serial_jtag_write_bytes((const char*)&header, sizeof(header), pdMS_TO_TICKS(50));
            usb_serial_jtag_write_bytes((const char*)rec.raw, rec.n_bytes, pdMS_TO_TICKS(50));

            // CRC16 over header + CSI data for integrity verification
            uint8_t crc_buf[sizeof(binary_csi_header_t) + 256];
            memcpy(crc_buf, &header, sizeof(header));
            memcpy(crc_buf + sizeof(header), rec.raw, rec.n_bytes);
            uint16_t crc = crc16_ccitt(crc_buf, sizeof(header) + rec.n_bytes);
            usb_serial_jtag_write_bytes((const char*)&crc, sizeof(crc), pdMS_TO_TICKS(50));
#else
            static char print_buf[1000];
            int offset = snprintf(print_buf, sizeof(print_buf),
                                  "CSI,%lu,%d,%d,%lld,%d,",
                                  (unsigned long)rec.round_id,
                                  rec.tx_id, rec.rx_id,
                                  rec.ts_us, rec.rssi);
            for (int i = 0; i < rec.n_bytes; i++) {
                sprintf(print_buf + offset, "%02X", (uint8_t)rec.raw[i]);
                offset += 2;
            }
            sprintf(print_buf + offset, "\n");
            printf("%s", print_buf);
#endif
        }
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len) {
    if (len < 1) return;

    if (data[0] == 1 && len >= (int)sizeof(probe_pkt_t)) {
        probe_pkt_t pkt;
        memcpy(&pkt, data, sizeof(pkt));
        g_round_id = pkt.round_id;

        ack_pkt_t ack = {
            .type   = 2,
            .tx_id  = g_device_id,
            .seq    = pkt.seq,
        };
        esp_now_send(recv_info->src_addr, (uint8_t *)&ack, sizeof(ack));

        // P3: Only re-sync on NEW round to avoid mid-round reset
        if (pkt.tx_id == 0 && g_device_id != 0) {
            static uint32_t last_sync_round = 0xFFFFFFFF;
            if (pkt.round_id != last_sync_round) {
                last_sync_round = pkt.round_id;
                // P4: Disable interrupts to prevent ISR race on g_slot_idx
                portDISABLE_INTERRUPTS();
                g_slot_idx = g_device_id - 1;
                portENABLE_INTERRUPTS();
                if (slot_timer != NULL) {
                    esp_timer_stop(slot_timer);
                    esp_timer_start_periodic(slot_timer, g_slot_us);
                }
            }
        }
    } else if (data[0] == 2 && len >= (int)sizeof(ack_pkt_t)) {
        ack_pkt_t ack;
        memcpy(&ack, data, sizeof(ack));
        if (ack_queue != NULL) {
            xQueueSend(ack_queue, &ack.seq, 0);
        }
    }
}

static esp_err_t send_to_node(uint8_t dest_id, const uint8_t *data, size_t len) {
    if (dest_id >= N_DEVICES || dest_id == g_device_id)
        return ESP_ERR_INVALID_ARG;
    return esp_now_send(mac_table[dest_id], data, len);
}

static void send_probe_in_slot(uint8_t dst) {
    static uint16_t seq = 0;
    probe_pkt_t pkt = {
        .type     = 1,
        .tx_id    = g_device_id,
        .round_id = g_round_id,
        .seq      = seq++,
    };

    for (int retry = 0; retry <= 1; retry++) {
        if (ack_queue != NULL) xQueueReset(ack_queue);
        send_to_node(dst, (const uint8_t *)&pkt, sizeof(pkt));

        int64_t start = esp_timer_get_time();
        bool got_ack  = false;
        while (esp_timer_get_time() - start < 500) {
            uint16_t ack_seq;
            if (ack_queue != NULL &&
                xQueueReceive(ack_queue, &ack_seq, 0) == pdTRUE) {
                if (ack_seq == pkt.seq) { got_ack = true; break; }
            }
            taskYIELD();  // P5: yield to WiFi task to process ACK
        }
        if (got_ack) break;
    }
}

static void tx_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t current_slot = g_slot_idx;
        if (current_slot < N_SLOTS &&
            schedule_table[current_slot].tx_id == g_device_id) {
            send_probe_in_slot(schedule_table[current_slot].rx_id);
        }
    }
}

static void on_slot_timer(void *arg) {
    if (g_slot_idx == 0xFF) {
        if (g_device_id == 0) {
            g_slot_idx = 0;
            g_round_id = 0;
        } else {
            return;
        }
    } else {
        g_slot_idx = (g_slot_idx + 1) % N_SLOTS;
        if (g_slot_idx == 0 && g_device_id == 0) {
            g_round_id++;
        }
    }

    if (schedule_table[g_slot_idx].tx_id == g_device_id) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (tx_task_handle != NULL) {
            vTaskNotifyGiveFromISR(tx_task_handle, &xHigherPriorityTaskWoken);
        }
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void wifi_csi_init(void) {
    // =========================================================================
    // FIX #2: ltf_merge_en = false để LLTF và HT-LTF nối tiếp nhau
    //         -> info->len = 256 byte = 128 subcarrier (I+Q, mỗi cái 1 byte)
    //         đúng với N_SUBCARR = 128 trong Python.
    // FIX #5 (bonus): stbc_htltf2_en = false vì hệ thống không dùng STBC.
    // =========================================================================
    static wifi_csi_config_t csi_cfg = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = false, // FIX #5: tắt, ESP32-C3 không có STBC
        .ltf_merge_en      = true,  // Thay đổi: true -> 128 byte (64 subcarrier)
        .channel_filter_en = false,
        .manu_scale        = false,
        // rx_filter đã bị xóa khỏi wifi_csi_config_t trong ESP-IDF 5.x/6.x.
        // Việc lọc gói không mong muốn được xử lý thủ công trong wifi_csi_cb()
        // bằng cách kiểm tra info->len < 64 và mac_to_id(info->mac).
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main(void) {
    BOOT_LOG(">>> BOOT START\n");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    BOOT_LOG(">>> SYS INIT OK\n");

    // USB-Serial-JTAG: the physical interface to PC on ESP32-C3
    usb_serial_jtag_driver_config_t usj_cfg = {
        .tx_buffer_size = 1024 * 16,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_cfg));
    BOOT_LOG(">>> USJ OK\n");

    esp_log_level_set("*", ESP_LOG_NONE);

    csi_queue = xQueueCreate(256, sizeof(csi_record_t));
    if (csi_queue == NULL) {
        BOOT_LOG(">>> FATAL csi_queue NULL\n");
        return;
    }
    ack_queue = xQueueCreate(10, sizeof(uint16_t));
    if (ack_queue == NULL) {
        BOOT_LOG(">>> FATAL ack_queue NULL\n");
        return;
    }
    BOOT_LOG(">>> QUEUE OK\n");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(52));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    BOOT_LOG(">>> WIFI OK\n");

    uint8_t my_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    g_device_id = 0xFF;
    for (int i = 0; i < N_DEVICES; i++) {
        if (memcmp(my_mac, mac_table[i], 6) == 0) {
            g_device_id = i;
            break;
        }
    }
    BOOT_LOG(">>> MAC CHECK: device_id=%d  MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
               g_device_id,
               my_mac[0], my_mac[1], my_mac[2],
               my_mac[3], my_mac[4], my_mac[5]);

    if (g_device_id == 0xFF) {
        esp_rom_printf(">>> FATAL MAC NOT FOUND MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                       my_mac[0], my_mac[1], my_mac[2],
                       my_mac[3], my_mac[4], my_mac[5]);
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    BOOT_LOG(">>> ESPNOW OK\n");

    for (int i = 0; i < N_DEVICES; i++) {
        if (i == g_device_id) continue;
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, mac_table[i], 6);
        peer.channel = WIFI_CHANNEL;
        peer.ifidx   = WIFI_IF_STA;
        peer.encrypt = false;
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));

        esp_now_rate_config_t rate_cfg = {
            .phymode = WIFI_PHY_MODE_HT20,
            .rate    = ESPNOW_RATE,
        };
        ESP_ERROR_CHECK(esp_now_set_peer_rate_config(mac_table[i], &rate_cfg));
    }
    BOOT_LOG(">>> PEERS OK\n");

    wifi_csi_init();
    BOOT_LOG(">>> CSI INIT OK\n");

    xTaskCreatePinnedToCore(tx_task, "tx_task", 4096, NULL, 6, &tx_task_handle, 0);
    xTaskCreatePinnedToCore(csi_print_task, "csi_print", 4096, NULL, 5, NULL, 0);
    BOOT_LOG(">>> TASKS OK\n");

    esp_timer_create_args_t targs = {
        .callback = &on_slot_timer,
        .name     = "slot_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &slot_timer));
    BOOT_LOG(">>> TIMER CREATED\n");

    if (g_device_id == 0) {
        BOOT_LOG(">>> NODE 0: waiting 3s...\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_ERROR_CHECK(esp_timer_start_periodic(slot_timer, g_slot_us));
        BOOT_LOG(">>> NODE 0: TDMA STARTED\n");
    } else {
        BOOT_LOG(">>> NODE %d: waiting for master sync\n", g_device_id);
    }
}
