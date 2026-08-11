#include "usb_audio.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "usb_audio";

esp_err_t usb_audio_init(void)
{
    ESP_LOGI(TAG, "USB Audio Class (UAC) entry point initialized");
    ESP_LOGI(TAG, "The HUSB path is prepared for a TinyUSB-based UAC implementation");
    ESP_LOGI(TAG, "Replace this placeholder with the final UAC descriptors and PCM pipeline once the board-specific USB routing is validated.");
    return ESP_OK;
}

void usb_audio_task(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGD(TAG, "USB audio task tick");
}
