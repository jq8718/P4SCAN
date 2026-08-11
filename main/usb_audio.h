#pragma once

#include "esp_err.h"

esp_err_t usb_audio_init(void);
void usb_audio_task(void);
