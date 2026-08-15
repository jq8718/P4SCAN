#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "soc/mipi_csi_bridge_struct.h"
#include "ov01c1b.h"
#include "ov01c1b_settings.h"

typedef struct {
    uint32_t hmirror_en : 1;
    uint32_t vflip_en : 1;
} ov01c1b_para_t;

typedef struct {
    ov01c1b_para_t para;
} ov01c1b_cam_t;

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define OV01C1B_MCLK (24 * 1000 * 1000)

#if CONFIG_CAMERA_OV01C1B_COLORBAR_FULL_1032X1032
#define OV01C1B_FORMAT_WIDTH  1032
#define OV01C1B_FORMAT_HEIGHT 1032
#define OV01C1B_FORMAT_NAME   "MIPI_1lane_24Minput_RAW10_1032x1032_colorbar"
#else
#define OV01C1B_FORMAT_WIDTH  1024
#define OV01C1B_FORMAT_HEIGHT 1024
#define OV01C1B_FORMAT_NAME   "MIPI_1lane_24Minput_RAW10_1024x1024_50fps"
#endif

static const char *TAG = "ov01c1b";

static esp_cam_sensor_bayer_pattern_t ov01c1b_get_bayer_pattern(void)
{
#if CONFIG_CAMERA_OV01C1B_BAYER_RGGB
    return ESP_CAM_SENSOR_BAYER_RGGB;
#elif CONFIG_CAMERA_OV01C1B_BAYER_GRBG
    return ESP_CAM_SENSOR_BAYER_GRBG;
#elif CONFIG_CAMERA_OV01C1B_BAYER_GBRG
    return ESP_CAM_SENSOR_BAYER_GBRG;
#else
    return ESP_CAM_SENSOR_BAYER_BGGR;
#endif
}

static const esp_cam_sensor_isp_info_t ov01c1b_isp_info[] = {
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 89000000,
            .vts = 2500,
            .hts = 1024,
            .tline_ns = 4090,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        },
    },
};

static const esp_cam_sensor_format_t ov01c1b_format_info[] = {
#if CONFIG_CAMERA_OV01C1B_MIPI_RAW10_1024X1024_50FPS
    {
        .name = OV01C1B_FORMAT_NAME,
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = OV01C1B_MCLK,
        .width = OV01C1B_FORMAT_WIDTH,
        .height = OV01C1B_FORMAT_HEIGHT,
        .regs = ov01c1b_mipi_1lane_24Minput_1024x1024_raw10_50fps,
        .regs_size = ARRAY_SIZE(ov01c1b_mipi_1lane_24Minput_1024x1024_raw10_50fps),
        .fps = 50,
        .isp_info = &ov01c1b_isp_info[0],
        .mipi_info = {
            .mipi_clk = CONFIG_CAMERA_OV01C1B_MIPI_CLK_MHZ * 1000000,
            .lane_num = 1,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
#endif
};

static esp_err_t ov01c1b_read(esp_sccb_io_handle_t sccb_handle, uint8_t reg, uint8_t *read_buf)
{
    return esp_sccb_transmit_receive_reg_a8v8(sccb_handle, reg, read_buf);
}

static esp_err_t ov01c1b_write(esp_sccb_io_handle_t sccb_handle, uint8_t reg, uint8_t data)
{
    return esp_sccb_transmit_reg_a8v8(sccb_handle, reg, data);
}

static esp_err_t ov01c1b_write_array(esp_sccb_io_handle_t sccb_handle, const ov01c1b_reginfo_t *regarray)
{
    esp_err_t ret = ESP_OK;
    int i = 0;

    while (regarray[i].reg != OV01C1B_REG_END) {
        if (regarray[i].reg == OV01C1B_REG_DELAY) {
            delay_ms(regarray[i].val);
        } else {
            ret = ov01c1b_write(sccb_handle, (uint8_t)regarray[i].reg, regarray[i].val);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "write reg[0x%02x]=0x%02x failed at index %d",
                         (unsigned int)regarray[i].reg, regarray[i].val, i);
                return ret;
            }
        }
        i++;
    }

    ESP_LOGI(TAG, "wrote %d OV01C1B register entries", i);
    return ESP_OK;
}

static void ov01c1b_log_p1_debug_registers(esp_cam_sensor_device_t *dev, const char *stage)
{
    static const uint8_t registers[] = {0xbc, 0xbd, 0x35, 0x36, 0x05, 0x06};
    uint8_t values[sizeof(registers)] = {0};
    bool read_ok = true;

    if (ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x01) != ESP_OK) {
        ESP_LOGW(TAG, "%s: failed to select page 1", stage);
        return;
    }

    for (size_t i = 0; i < sizeof(registers); i++) {
        if (ov01c1b_read(dev->sccb_handle, registers[i], &values[i]) != ESP_OK) {
            ESP_LOGW(TAG, "%s: failed to read P1:0x%02x", stage, registers[i]);
            read_ok = false;
        }
    }

    if (ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00) != ESP_OK) {
        ESP_LOGW(TAG, "%s: failed to restore page 0", stage);
        return;
    }

    if (read_ok) {
        ESP_LOGI(TAG, "%s: P1:BC=0x%02x BD=0x%02x 35=0x%02x 36=0x%02x 05=0x%02x 06=0x%02x",
                 stage, values[0], values[1], values[2], values[3], values[4], values[5]);
    }
}

static void ov01c1b_log_mode_registers(esp_cam_sensor_device_t *dev)
{
    uint8_t value[5] = {0};

    if (ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00) != ESP_OK) {
        return;
    }
    for (uint8_t i = 0; i < 5; i++) {
        if (ov01c1b_read(dev->sccb_handle, (uint8_t)(0x8e + i), &value[i]) != ESP_OK) {
            return;
        }
    }
    ESP_LOGI(TAG, "mode P0:8e=0x%02x 8f=0x%02x 90=0x%02x 91=0x%02x 92=0x%02x",
             value[0], value[1], value[2], value[3], value[4]);

    if (ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x01) != ESP_OK) {
        return;
    }
    for (uint8_t i = 0; i < 5; i++) {
        if (ov01c1b_read(dev->sccb_handle, (uint8_t)(0x02 + i), &value[i]) != ESP_OK) {
            return;
        }
    }
    ESP_LOGI(TAG, "mode P1:02=0x%02x 03=0x%02x 04=0x%02x 05=0x%02x 06=0x%02x",
             value[0], value[1], value[2], value[3], value[4]);
    ov01c1b_log_p1_debug_registers(dev, "after-init");

    if (ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00) != ESP_OK) {
        return;
    }
    if (ov01c1b_read(dev->sccb_handle, 0x97, &value[0]) == ESP_OK) {
        ESP_LOGI(TAG, "mode P0:97 data_id=0x%02x", value[0]);
    }
}

static esp_err_t ov01c1b_set_test_pattern(esp_cam_sensor_device_t *dev, bool enable)
{
    uint8_t pattern_enable = 0;
    uint8_t pattern_output = 0;

    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x04), TAG,
                        "failed to select page 4 for ColorBar");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, 0xf3, enable ? 0x03 : 0x00), TAG,
                        "failed to configure P4:F3 ColorBar enable");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, 0x12, enable ? 0x01 : 0x00), TAG,
                        "failed to configure P4:12 ColorBar output");
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, 0xf3, &pattern_enable), TAG,
                        "failed to read P4:F3 ColorBar enable");
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, 0x12, &pattern_output), TAG,
                        "failed to read P4:12 ColorBar output");
    ESP_LOGW(TAG, "internal ColorBar %s: P4:F3=0x%02x P4:12=0x%02x; FSIN registers unchanged",
             enable ? "enabled" : "disabled", pattern_enable, pattern_output);
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00), TAG,
                        "failed to restore page 0 after test pattern");
    return ESP_OK;
}

static esp_err_t ov01c1b_set_colorbar_mipi_size(esp_cam_sensor_device_t *dev)
{
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00), TAG,
                        "failed to select page 0 for ColorBar MIPI size");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, 0x8e, 0x04), TAG,
                        "failed to set ColorBar MIPI width MSB");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, 0x8f, 0x08), TAG,
                        "failed to set ColorBar MIPI width LSB");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, 0x90, 0x04), TAG,
                        "failed to set ColorBar MIPI height MSB");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, 0x91, 0x08), TAG,
                        "failed to set ColorBar MIPI height LSB");
    ESP_LOGW(TAG, "ColorBar MIPI size override: P0:8e/8f=0x0408, P0:90/91=0x0408");
    return ESP_OK;
}

static esp_err_t ov01c1b_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    uint8_t chip_id[4] = {0};
    uint8_t eco_ver = 0;

    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00), TAG,
                        "failed to select page 0 for sensor ID");
    for (uint8_t reg = 0; reg < sizeof(chip_id); reg++) {
        ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, reg, &chip_id[reg]), TAG,
                            "failed to read chip ID byte");
    }
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, 0x04, &eco_ver), TAG,
                        "failed to read ECO version");

    uint32_t full_chip_id = ((uint32_t)chip_id[0] << 24) | ((uint32_t)chip_id[1] << 16) |
                            ((uint32_t)chip_id[2] << 8) | chip_id[3];
    ESP_RETURN_ON_FALSE(full_chip_id == OV01C1B_CHIP_ID, ESP_ERR_NOT_FOUND, TAG,
                        "unexpected OV01C1B chip_id=0x%08lx", (unsigned long)full_chip_id);

    id->pid = OV01C1B_PID;
    ESP_LOGI(TAG, "OV01C1B chip_id=%02x %02x %02x %02x, eco=0x%02x, pid=0x%04x",
             chip_id[0], chip_id[1], chip_id[2], chip_id[3], eco_ver, id->pid);
    return ESP_OK;
}

static esp_err_t ov01c1b_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    uint8_t soft_reset = 0;
    uint8_t mipi_output = 0;
    uint8_t output_gate = 0;
    uint8_t mipi_lane = 0;
    uint8_t mipi_ctrl = 0;

#if CONFIG_CAMERA_OV01C1B_KEEP_STREAM_ON
    if (!enable) {
        ESP_LOGW(TAG, "stream off ignored; OV01C1B MIPI output remains enabled");
        return ESP_OK;
    }
#endif

    if (enable) {
        MIPI_CSI_BRIDGE.dma_req_cfg.dma_burst_len = 512;
        ESP_LOGI(TAG, "CSI bridge DMA burst length=512 cfg=0x%08x",
                 (unsigned int)MIPI_CSI_BRIDGE.dma_req_cfg.val);
#if CONFIG_CAMERA_OV01C1B_TEST_PATTERN
        /* The vendor sequence enables the pattern before MIPI output. */
        ESP_RETURN_ON_ERROR(ov01c1b_set_test_pattern(dev, true), TAG,
                            "failed to enable internal test pattern before stream on");
#if CONFIG_CAMERA_OV01C1B_COLORBAR_FULL_1032X1032
        ESP_RETURN_ON_ERROR(ov01c1b_set_colorbar_mipi_size(dev), TAG,
                            "failed to set ColorBar MIPI frame size");
#endif
#endif
    }

    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00), TAG,
                        "failed to select page 0 for stream %s", enable ? "on" : "off");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_MIPI_OUTPUT,
                                      enable ? 0x01 : 0x00), TAG,
                        "failed to set MIPI output for stream %s", enable ? "on" : "off");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x01), TAG,
                        "failed to select page 1 for stream %s", enable ? "on" : "off");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_OUTPUT_GATE, 0x31), TAG,
                        "failed to set sensor output gate for stream %s", enable ? "on" : "off");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00), TAG,
                        "failed to restore page 0 for stream %s", enable ? "on" : "off");
    if (enable) {
        ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_SOFT_RESET,
                                          OV01C1B_SOFT_STREAMING), TAG,
                            "stream on failed");
    }
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, OV01C1B_REG_MIPI_OUTPUT, &mipi_output), TAG,
                        "failed to read MIPI output gate");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x01), TAG,
                        "failed to select page 1 for output gate readback");
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, OV01C1B_REG_OUTPUT_GATE, &output_gate), TAG,
                        "failed to read sensor output gate");
    ESP_RETURN_ON_ERROR(ov01c1b_write(dev->sccb_handle, OV01C1B_REG_PAGE_SEL, 0x00), TAG,
                        "failed to restore page 0 after output gate readback");
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, OV01C1B_REG_SOFT_RESET, &soft_reset), TAG,
                        "failed to read stream state");
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, OV01C1B_REG_MIPI_LANE, &mipi_lane), TAG,
                        "failed to read MIPI lane state");
    ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, OV01C1B_REG_MIPI_CTRL, &mipi_ctrl), TAG,
                        "failed to read MIPI control state");

    ESP_LOGI(TAG, "stream %s: A0=0x%02x, P1:01=0x%02x, P0:20=0x%02x, C2=0x%02x, C4=0x%02x",
             enable ? "on" : "off", mipi_output, output_gate, soft_reset, mipi_lane, mipi_ctrl);
    dev->stream_status = enable;
    return ESP_OK;
}

static esp_err_t ov01c1b_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    (void)dev;

    switch (qdesc->id) {
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 1;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        return ESP_OK;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t ov01c1b_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    ov01c1b_cam_t *cam = (ov01c1b_cam_t *)dev->priv;

    ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_ARG, TAG, "parameter size error");

    switch (id) {
    case ESP_CAM_SENSOR_VFLIP:
        *(uint32_t *)arg = cam->para.vflip_en;
        return ESP_OK;
    case ESP_CAM_SENSOR_HMIRROR:
        *(uint32_t *)arg = cam->para.hmirror_en;
        return ESP_OK;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t ov01c1b_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    ov01c1b_cam_t *cam = (ov01c1b_cam_t *)dev->priv;

    ESP_RETURN_ON_FALSE(size == sizeof(uint32_t) || size == sizeof(int), ESP_ERR_INVALID_ARG, TAG, "parameter size error");
    uint32_t value = *(const uint32_t *)arg ? 1 : 0;

    switch (id) {
    case ESP_CAM_SENSOR_VFLIP:
        cam->para.vflip_en = value;
        ESP_LOGW(TAG, "vflip cached as %u; register bit is not mapped yet", (unsigned int)value);
        return ESP_OK;
    case ESP_CAM_SENSOR_HMIRROR:
        cam->para.hmirror_en = value;
        ESP_LOGW(TAG, "hmirror cached as %u; register bit is not mapped yet", (unsigned int)value);
        return ESP_OK;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t ov01c1b_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    (void)dev;
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(ov01c1b_format_info);
    formats->format_array = &ov01c1b_format_info[0];
    return ESP_OK;
}

static esp_err_t ov01c1b_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    (void)dev;
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t ov01c1b_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);

    if (format == NULL) {
        format = &ov01c1b_format_info[CONFIG_CAMERA_OV01C1B_MIPI_IF_FORMAT_INDEX_DEFAULT];
    }

    ESP_LOGI(TAG, "set format: %s, mipi=%uMbps/lane, lanes=%u, bayer=%d",
             format->name,
             (unsigned int)(format->mipi_info.mipi_clk / 1000000),
             (unsigned int)format->mipi_info.lane_num,
             ov01c1b_get_bayer_pattern());

    esp_err_t ret = ov01c1b_write_array(dev->sccb_handle, (const ov01c1b_reginfo_t *)format->regs);
    if (ret != ESP_OK) {
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }
    ov01c1b_log_mode_registers(dev);

    ret = ov01c1b_set_stream(dev, 0);
    if (ret != ESP_OK) {
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }

    dev->cur_format = format;
    return ESP_OK;
}

static esp_err_t ov01c1b_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);

    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }

    memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
    return ESP_OK;
}

static esp_err_t ov01c1b_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_cam_sensor_reg_val_t *sensor_reg;
    uint8_t regval;

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        return ov01c1b_get_sensor_id(dev, arg);
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        return ov01c1b_write(dev->sccb_handle, (uint8_t)sensor_reg->regaddr, (uint8_t)sensor_reg->value);
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ESP_RETURN_ON_ERROR(ov01c1b_read(dev->sccb_handle, (uint8_t)sensor_reg->regaddr, &regval), TAG,
                            "read register failed");
        sensor_reg->value = regval;
        return ESP_OK;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        return ov01c1b_set_stream(dev, *(int *)arg);
    case ESP_CAM_SENSOR_IOC_HW_RESET:
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        return ESP_OK;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t ov01c1b_power_on(esp_cam_sensor_device_t *dev)
{
    (void)dev;
    return ESP_OK;
}

static esp_err_t ov01c1b_power_off(esp_cam_sensor_device_t *dev)
{
    (void)dev;
    return ESP_OK;
}

static esp_err_t ov01c1b_delete(esp_cam_sensor_device_t *dev)
{
    if (dev) {
        free(dev->priv);
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t ov01c1b_ops = {
    .query_para_desc = ov01c1b_query_para_desc,
    .get_para_value = ov01c1b_get_para_value,
    .set_para_value = ov01c1b_set_para_value,
    .query_support_formats = ov01c1b_query_support_formats,
    .query_support_capability = ov01c1b_query_support_capability,
    .set_format = ov01c1b_set_format,
    .get_format = ov01c1b_get_format,
    .priv_ioctl = ov01c1b_priv_ioctl,
    .del = ov01c1b_delete,
};

esp_cam_sensor_device_t *ov01c1b_detect(esp_cam_sensor_config_t *config)
{
    ESP_LOGI(TAG, "probing OV01C1B at 7-bit SCCB addr=0x%02x (8-bit write addr=0x%02x)",
             OV01C1B_SCCB_ADDR, OV01C1B_8BIT_WRITE_ADDR);

    if (config == NULL) {
        return NULL;
    }

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "no memory for sensor device");
        return NULL;
    }

    ov01c1b_cam_t *cam = heap_caps_calloc(1, sizeof(ov01c1b_cam_t), MALLOC_CAP_DEFAULT);
    if (cam == NULL) {
        ESP_LOGE(TAG, "no memory for sensor private data");
        free(dev);
        return NULL;
    }

    dev->name = (char *)OV01C1B_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &ov01c1b_ops;
    dev->priv = cam;
    dev->cur_format = &ov01c1b_format_info[CONFIG_CAMERA_OV01C1B_MIPI_IF_FORMAT_INDEX_DEFAULT];

    if (ov01c1b_power_on(dev) != ESP_OK) {
        goto failed;
    }

    if (ov01c1b_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "OV01C1B probe failed at 7-bit addr=0x%02x; check SID1/SID2, XSHUTDN, ECLK, and power rails",
                 OV01C1B_SCCB_ADDR);
        goto failed;
    }

    ov01c1b_log_p1_debug_registers(dev, "power-up");
    ESP_LOGI(TAG, "Detected OV01C1B-compatible sensor, probe PID=0x%04x", dev->id.pid);
    return dev;

failed:
    ov01c1b_power_off(dev);
    free(dev->priv);
    free(dev);
    return NULL;
}

#if CONFIG_CAMERA_OV01C1B_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(ov01c1b_detect, ESP_CAM_SENSOR_MIPI_CSI, OV01C1B_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return ov01c1b_detect(config);
}
#endif
