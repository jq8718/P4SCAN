/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "esp_check.h"
#if CONFIG_TINYUSB_RHPORT_HS
#include "soc/hp_sys_clkrst_reg.h"
#include "soc/hp_system_reg.h"
#endif
#include "esp_private/usb_phy.h"
#include "tusb.h"
#include "usb_device_uvc.h"

static const char *TAG = "usbd_uvc";

#if CONFIG_UVC_SUPPORT_TWO_CAM
#define UVC_CAM_NUM 2
#else
#define UVC_CAM_NUM 1
#endif

#define TUSB_EVENT_EXIT         (1<<0)
#define TUSB_EVENT_EXIT_DONE    (1<<1)
#define UVC1_EVENT_EXIT         (1<<2)
#define UVC1_EVENT_EXIT_DONE    (1<<3)
#define UVC_XFER_TIMEOUT_MS     1000
#if CONFIG_UVC_SUPPORT_TWO_CAM
#define UVC2_EVENT_EXIT         (1<<4)
#define UVC2_EVENT_EXIT_DONE    (1<<5)
#endif

typedef struct {
    usb_phy_handle_t phy_hdl;
    bool uvc_init[UVC_CAM_NUM];
    uvc_format_t format[UVC_CAM_NUM];
    uvc_device_config_t user_config[UVC_CAM_NUM];
    TaskHandle_t uvc_task_hdl[UVC_CAM_NUM];
    TaskHandle_t tusb_task_hdl;
    uint32_t interval_ms[UVC_CAM_NUM];
    bool streaming[UVC_CAM_NUM];
    EventGroupHandle_t event_group;
} uvc_device_t;

static uvc_device_t s_uvc_device;

static void usb_phy_init(void)
{
    // Configure USB PHY
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
#if CONFIG_TINYUSB_RHPORT_HS
        .otg_speed = USB_PHY_SPEED_HIGH,
#endif
    };
    usb_new_phy(&phy_conf, &s_uvc_device.phy_hdl);
}

static inline uint32_t get_time_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void tusb_device_task(void *arg)
{
    while (1) {
        EventBits_t uxBits = xEventGroupGetBits(s_uvc_device.event_group);
        if (uxBits & TUSB_EVENT_EXIT) {
            ESP_LOGI(TAG, "TUSB task exit");
            break;
        }
        tud_task();
    }
    xEventGroupSetBits(s_uvc_device.event_group, TUSB_EVENT_EXIT_DONE);
    vTaskDelete(NULL);
}

void tud_mount_cb(void)
{
    ESP_LOGI(TAG, "Mount");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    ESP_LOGI(TAG, "UN-Mount");
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;

    if (s_uvc_device.user_config[0].stop_cb) {
        s_uvc_device.streaming[0] = false;
        ESP_LOGI(TAG, "reset UVC1 video stream");
        tud_video_n_stream_reset(0, 0);
        s_uvc_device.user_config[0].stop_cb(s_uvc_device.user_config[0].cb_ctx);
    }
#if CONFIG_UVC_SUPPORT_TWO_CAM
    if (s_uvc_device.user_config[1].stop_cb) {
        s_uvc_device.streaming[1] = false;
        tud_video_n_stream_reset(1, 0);
        s_uvc_device.user_config[1].stop_cb(s_uvc_device.user_config[1].cb_ctx);
    }
#endif
    ESP_LOGI(TAG, "Suspend");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "Resume");
}

#if (CFG_TUD_VIDEO)
//--------------------------------------------------------------------+
// USB Video
//--------------------------------------------------------------------+
static void video_task(void *arg)
{
    uint32_t start_ms = 0;
    uint32_t frame_num = 0;
    uint32_t frame_len = 0;
    uint32_t already_start = 0;
    uint32_t tx_busy = 0;
    uint32_t tx_start_ms = 0;
    uint32_t state_log_count = 0;
    uint8_t *uvc_buffer = s_uvc_device.user_config[0].uvc_buffer;
    uint32_t uvc_buffer_size = s_uvc_device.user_config[0].uvc_buffer_size;
    uvc_fb_t *pic = NULL;

    while (1) {
        EventBits_t uxBits = xEventGroupGetBits(s_uvc_device.event_group);
        if (uxBits & UVC1_EVENT_EXIT) {
            ESP_LOGI(TAG, "UVC task exit");
            break;
        }

        if (!s_uvc_device.streaming[0] || !tud_video_n_streaming(0, 0)) {
            if (state_log_count < 8) {
                ESP_LOGI(TAG, "UVC1 task idle: app_streaming=%d tinyusb_streaming=%d frame=%" PRIu32,
                         s_uvc_device.streaming[0], tud_video_n_streaming(0, 0), frame_num);
                state_log_count++;
            }
            already_start = 0;
            frame_num = 0;
            tx_busy = 0;
            tx_start_ms = 0;
            ulTaskNotifyTake(pdTRUE, 0);
            vTaskDelay(1);
            continue;
        }

        if (!already_start) {
            already_start = 1;
            start_ms = get_time_millis() - s_uvc_device.interval_ms[0];
            ulTaskNotifyTake(pdTRUE, 0);
            ESP_LOGI(TAG, "UVC1 transfer notifications cleared");
        }

        uint32_t cur = get_time_millis();
        if (cur - start_ms < s_uvc_device.interval_ms[0]) {
            vTaskDelay(1);
            continue;
        }

        if (tx_busy) {
            if (tud_video_n_frame_xfer_busy(0, 0)) {
                if (tx_start_ms && cur - tx_start_ms > UVC_XFER_TIMEOUT_MS) {
                    ESP_LOGW(TAG, "frame %" PRIu32 " transfer timeout, reset UVC1 stream endpoint", frame_num);
                    tud_video_n_stream_reset(0, 0);
                    ulTaskNotifyTake(pdTRUE, 0);
                    tx_busy = 0;
                    tx_start_ms = 0;
                    start_ms = get_time_millis();
                }
                continue;
            }
            ++frame_num;
            tx_busy = 0;
            tx_start_ms = 0;
            ESP_LOGI(TAG, "UVC1 transfer idle, requesting next frame=%" PRIu32, frame_num);
        }

        start_ms += s_uvc_device.interval_ms[0];
        if (frame_num < 4) {
            ESP_LOGI(TAG, "UVC1 requesting frame=%" PRIu32, frame_num);
        }
        ESP_LOGD(TAG, "frame %" PRIu32 " taking picture...", frame_num);
        pic = s_uvc_device.user_config[0].fb_get_cb(s_uvc_device.user_config[0].cb_ctx);
        if (pic) {
            ESP_LOGD(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        } else {
            ESP_LOGE(TAG, "Failed to capture picture");
            continue;
        }

        if (pic->len > uvc_buffer_size) {
            ESP_LOGW(TAG, "frame size is too big, dropping frame");
            s_uvc_device.user_config[0].fb_return_cb(pic, s_uvc_device.user_config[0].cb_ctx);
            continue;
        }
        frame_len = pic->len;
        memcpy(uvc_buffer, pic->buf, frame_len);
        ESP_ERROR_CHECK(esp_cache_msync(uvc_buffer, frame_len,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED));
        s_uvc_device.user_config[0].fb_return_cb(pic, s_uvc_device.user_config[0].cb_ctx);
        bool transfer_started = tud_video_n_frame_xfer(0, 0, (void *)uvc_buffer, frame_len);
        if (frame_num < 4) {
            ESP_LOGI(TAG, "UVC1 xfer request: frame=%" PRIu32 " addr=%p len=%" PRIu32
                     " head=%02x%02x tail=%02x%02x started=%d",
                     frame_num, uvc_buffer, frame_len,
                     frame_len >= 2 ? uvc_buffer[0] : 0,
                     frame_len >= 2 ? uvc_buffer[1] : 0,
                     frame_len >= 2 ? uvc_buffer[frame_len - 2] : 0,
                     frame_len >= 2 ? uvc_buffer[frame_len - 1] : 0,
                     transfer_started);
        }
        if (transfer_started) {
            tx_busy = 1;
            tx_start_ms = get_time_millis();
            ESP_LOGD(TAG, "frame %" PRIu32 " transfer start, size %" PRIu32, frame_num, frame_len);
        } else {
            tx_busy = 0;
            tx_start_ms = 0;
            ESP_LOGW(TAG, "frame %" PRIu32 " transfer busy, dropping frame", frame_num);
        }
    }

    xEventGroupSetBits(s_uvc_device.event_group, UVC1_EVENT_EXIT_DONE);
    vTaskDelete(NULL);
}

#if CONFIG_UVC_SUPPORT_TWO_CAM
static void video_task2(void *arg)
{
    uint32_t start_ms = 0;
    uint32_t frame_num = 0;
    uint32_t frame_len = 0;
    uint32_t already_start = 0;
    uint32_t tx_busy = 0;
    uint32_t tx_start_ms = 0;
    uint8_t *uvc_buffer = s_uvc_device.user_config[1].uvc_buffer;
    uint32_t uvc_buffer_size = s_uvc_device.user_config[1].uvc_buffer_size;
    uvc_fb_t *pic = NULL;

    while (1) {
        EventBits_t uxBits = xEventGroupGetBits(s_uvc_device.event_group);
        if (uxBits & UVC2_EVENT_EXIT) {
            ESP_LOGI(TAG, "UVC2 task exit");
            break;
        }

        if (!s_uvc_device.streaming[1] || !tud_video_n_streaming(1, 0)) {
            already_start = 0;
            frame_num = 0;
            tx_busy = 0;
            tx_start_ms = 0;
            ulTaskNotifyTake(pdTRUE, 0);
            vTaskDelay(1);
            continue;
        }

        if (!already_start) {
            already_start = 1;
            start_ms = get_time_millis();
            ulTaskNotifyTake(pdTRUE, 0);
            ESP_LOGI(TAG, "UVC2 transfer notifications cleared");
        }

        uint32_t cur = get_time_millis();
        if (cur - start_ms < s_uvc_device.interval_ms[1]) {
            vTaskDelay(1);
            continue;
        }

        if (tx_busy) {
            if (tud_video_n_frame_xfer_busy(1, 0)) {
                if (tx_start_ms && cur - tx_start_ms > UVC_XFER_TIMEOUT_MS) {
                    ESP_LOGW(TAG, "frame %" PRIu32 " transfer timeout, reset UVC2 stream endpoint", frame_num);
                    tud_video_n_stream_reset(1, 0);
                    ulTaskNotifyTake(pdTRUE, 0);
                    tx_busy = 0;
                    tx_start_ms = 0;
                    start_ms = get_time_millis();
                }
                continue;
            }
            ++frame_num;
            tx_busy = 0;
            tx_start_ms = 0;
        }

        start_ms += s_uvc_device.interval_ms[1];
        ESP_LOGD(TAG, "frame %" PRIu32 " taking picture...", frame_num);
        pic = s_uvc_device.user_config[1].fb_get_cb(s_uvc_device.user_config[1].cb_ctx);
        if (pic) {
            ESP_LOGD(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        } else {
            ESP_LOGE(TAG, "Failed to capture picture");
            continue;
        }

        if (pic->len > uvc_buffer_size) {
            ESP_LOGW(TAG, "frame size is too big, dropping frame");
            s_uvc_device.user_config[1].fb_return_cb(pic, s_uvc_device.user_config[1].cb_ctx);
            continue;
        }
        frame_len = pic->len;
        memcpy(uvc_buffer, pic->buf, frame_len);
        s_uvc_device.user_config[1].fb_return_cb(pic, s_uvc_device.user_config[1].cb_ctx);
        if (tud_video_n_frame_xfer(1, 0, (void *)uvc_buffer, frame_len)) {
            tx_busy = 1;
            tx_start_ms = get_time_millis();
            ESP_LOGD(TAG, "frame %" PRIu32 " transfer start, size %" PRIu32, frame_num, frame_len);
        } else {
            tx_busy = 0;
            tx_start_ms = 0;
            ESP_LOGW(TAG, "frame %" PRIu32 " transfer busy, dropping frame", frame_num);
        }
    }

    xEventGroupSetBits(s_uvc_device.event_group, UVC2_EVENT_EXIT_DONE);
    vTaskDelete(NULL);
}
#endif

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
    (void)ctl_idx;
    (void)stm_idx;
    ESP_LOGI(TAG, "UVC%u transfer complete", (unsigned int)ctl_idx + 1);
    xTaskNotifyGive(s_uvc_device.uvc_task_hdl[ctl_idx]);
}

int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
                        video_probe_and_commit_control_t const *parameters)
{
    (void)ctl_idx;
    (void)stm_idx;
    /* convert unit to ms from 100 ns */
    ESP_LOGI(TAG, "bFrameIndex: %u", parameters->bFrameIndex);
    ESP_LOGI(TAG, "dwFrameInterval: %" PRIu32 "", parameters->dwFrameInterval);
    if (parameters->bFrameIndex > UVC_FRAME_NUM) {
        return VIDEO_ERROR_OUT_OF_RANGE;
    }
    s_uvc_device.streaming[ctl_idx] = false;
    ESP_LOGI(TAG, "reset UVC%d video stream before commit", (int)ctl_idx + 1);
    tud_video_n_stream_reset(ctl_idx, stm_idx);
    s_uvc_device.interval_ms[ctl_idx] = parameters->dwFrameInterval / 10000;
    int frame_index = parameters->bFrameIndex - 1;
    esp_err_t ret = s_uvc_device.user_config[ctl_idx].start_cb(s_uvc_device.format[ctl_idx], UVC_FRAMES_INFO[ctl_idx][frame_index].width,
                                                               UVC_FRAMES_INFO[ctl_idx][frame_index].height, UVC_FRAMES_INFO[ctl_idx][frame_index].rate, s_uvc_device.user_config[ctl_idx].cb_ctx);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed");
        return VIDEO_ERROR_OUT_OF_RANGE;
    }
    s_uvc_device.streaming[ctl_idx] = true;
    return VIDEO_ERROR_NONE;
}
#endif

esp_err_t uvc_device_config(int index, uvc_device_config_t *config)
{
    ESP_RETURN_ON_FALSE(index < UVC_CAM_NUM, ESP_ERR_INVALID_ARG, TAG, "index is invalid");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(config->start_cb != NULL, ESP_ERR_INVALID_ARG, TAG, "start_cb is NULL");
    ESP_RETURN_ON_FALSE(config->fb_get_cb != NULL, ESP_ERR_INVALID_ARG, TAG, "fb_get_cb is NULL");
    ESP_RETURN_ON_FALSE(config->fb_return_cb != NULL, ESP_ERR_INVALID_ARG, TAG, "fb_return_cb is NULL");
    ESP_RETURN_ON_FALSE(config->stop_cb != NULL, ESP_ERR_INVALID_ARG, TAG, "stop_cb is NULL");
    ESP_RETURN_ON_FALSE(config->uvc_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "uvc_buffer is NULL");
    ESP_RETURN_ON_FALSE(config->uvc_buffer_size > 0, ESP_ERR_INVALID_ARG, TAG, "uvc_buffer_size is 0");

    s_uvc_device.user_config[index] = *config;
    s_uvc_device.interval_ms[index] = 1000 / (index == 0 ? UVC_CAM1_FRAME_RATE : UVC_CAM2_FRAME_RATE);
    s_uvc_device.uvc_init[index] = true;
    return ESP_OK;
}

esp_err_t uvc_device_init(void)
{
    ESP_RETURN_ON_FALSE(s_uvc_device.uvc_init[0], ESP_ERR_INVALID_STATE, TAG, "uvc device 0 not init");
#if CONFIG_UVC_SUPPORT_TWO_CAM
    ESP_RETURN_ON_FALSE(s_uvc_device.uvc_init[1], ESP_ERR_INVALID_STATE, TAG, "uvc device 1 not init, if not use, please disable CONFIG_UVC_SUPPORT_TWO_CAM");
#endif

#if CONFIG_FORMAT_MJPEG_CAM1
    s_uvc_device.format[0] = UVC_FORMAT_JPEG;
#elif CONFIG_FORMAT_H264_CAM1
    s_uvc_device.format[0] = UVC_FORMAT_H264;
#endif

#if CONFIG_UVC_SUPPORT_TWO_CAM
#if CONFIG_FORMAT_MJPEG_CAM2
    s_uvc_device.format[1] = UVC_FORMAT_JPEG;
#elif CONFIG_FORMAT_H264_CAM2
    s_uvc_device.format[1] = UVC_FORMAT_H264;
#endif
#endif

    s_uvc_device.event_group = xEventGroupCreate();
    if (s_uvc_device.event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    // init device stack on configured roothub port
    usb_phy_init();
    bool usb_init = tusb_init();
    if (!usb_init) {
        ESP_LOGE(TAG, "USB Device Stack Init Fail");
        vEventGroupDelete(s_uvc_device.event_group);
        s_uvc_device.event_group = NULL;
        return ESP_FAIL;
    }

    BaseType_t core_id = (CONFIG_UVC_TINYUSB_TASK_CORE < 0) ? tskNO_AFFINITY : CONFIG_UVC_TINYUSB_TASK_CORE;
    xTaskCreatePinnedToCore(tusb_device_task, "TinyUSB", 4096, NULL, CONFIG_UVC_TINYUSB_TASK_PRIORITY, &s_uvc_device.tusb_task_hdl, core_id);
#if (CFG_TUD_VIDEO)
    core_id = (CONFIG_UVC_CAM1_TASK_CORE < 0) ? tskNO_AFFINITY : CONFIG_UVC_CAM1_TASK_CORE;
    xTaskCreatePinnedToCore(video_task, "UVC", 4096, NULL, CONFIG_UVC_CAM1_TASK_PRIORITY, &s_uvc_device.uvc_task_hdl[0], core_id);
#if CONFIG_UVC_SUPPORT_TWO_CAM
    core_id = (CONFIG_UVC_CAM2_TASK_CORE < 0) ? tskNO_AFFINITY : CONFIG_UVC_CAM2_TASK_CORE;
    xTaskCreatePinnedToCore(video_task2, "UVC2", 4096, NULL, CONFIG_UVC_CAM2_TASK_PRIORITY, &s_uvc_device.uvc_task_hdl[1], core_id);
#endif
#endif
    ESP_LOGI(TAG, "UVC Device Start, Version: %d.%d.%d", USB_DEVICE_UVC_VER_MAJOR, USB_DEVICE_UVC_VER_MINOR, USB_DEVICE_UVC_VER_PATCH);
    return ESP_OK;
}

esp_err_t uvc_device_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_uvc_device.uvc_init[0], ESP_ERR_INVALID_STATE, TAG, "uvc device 0 not init");
#if CONFIG_UVC_SUPPORT_TWO_CAM
    ESP_RETURN_ON_FALSE(s_uvc_device.uvc_init[1], ESP_ERR_INVALID_STATE, TAG, "uvc device 1 not init, if not use, please disable CONFIG_UVC_SUPPORT_TWO_CAM");
#endif
    ESP_RETURN_ON_FALSE(s_uvc_device.event_group != NULL, ESP_ERR_INVALID_STATE, TAG, "event group is NULL");

    // Stop UVC tasks first
    xEventGroupSetBits(s_uvc_device.event_group, UVC1_EVENT_EXIT);
    xEventGroupWaitBits(s_uvc_device.event_group, UVC1_EVENT_EXIT_DONE, pdTRUE, pdTRUE, portMAX_DELAY);
#if CONFIG_UVC_SUPPORT_TWO_CAM
    xEventGroupSetBits(s_uvc_device.event_group, UVC2_EVENT_EXIT);
    xEventGroupWaitBits(s_uvc_device.event_group, UVC2_EVENT_EXIT_DONE, pdTRUE, pdTRUE, portMAX_DELAY);
#endif

    // Call user stop callbacks
    if (s_uvc_device.user_config[0].stop_cb) {
        s_uvc_device.user_config[0].stop_cb(s_uvc_device.user_config[0].cb_ctx);
    }
#if CONFIG_UVC_SUPPORT_TWO_CAM
    if (s_uvc_device.user_config[1].stop_cb) {
        s_uvc_device.user_config[1].stop_cb(s_uvc_device.user_config[1].cb_ctx);
    }
#endif

    // Stop TinyUSB task
    xEventGroupSetBits(s_uvc_device.event_group, TUSB_EVENT_EXIT);
    EventBits_t bits = xEventGroupWaitBits(s_uvc_device.event_group, TUSB_EVENT_EXIT_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
    if (!(bits & TUSB_EVENT_EXIT_DONE)) {
        ESP_LOGW(TAG, "TinyUSB task exit timeout (5s), force delete");
        if (s_uvc_device.tusb_task_hdl) {
            vTaskDelete(s_uvc_device.tusb_task_hdl);
            s_uvc_device.tusb_task_hdl = NULL;
        }
    }

    // Clean up event group
    vEventGroupDelete(s_uvc_device.event_group);
    s_uvc_device.event_group = NULL;

    // Teardown USB stack
    tusb_teardown();
    if (s_uvc_device.phy_hdl) {
        usb_del_phy(s_uvc_device.phy_hdl);
        s_uvc_device.phy_hdl = NULL;
    }

    // Reset initialization flags
    memset(s_uvc_device.uvc_init, 0, sizeof(s_uvc_device.uvc_init));

    ESP_LOGI(TAG, "UVC Device Deinit");
    return ESP_OK;
}
