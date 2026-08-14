#include "p4scan_board_power.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCA9538A_REG_OUTPUT 0x01
#define PCA9538A_REG_CONFIG 0x03

#define PCA9538A_P0_DOVDD       BIT0
#define PCA9538A_P1_AVDD        BIT1
#define PCA9538A_P2_DVDD        BIT2
#define PCA9538A_P3_AIM         BIT3
#define PCA9538A_P4_ILLUM       BIT4
#define PCA9538A_P5_XSHUTDN     BIT5
#define PCA9538A_P6_CLK24M      BIT6
#define PCA9538A_P7_ILLUM_3V3   BIT7

#define OV01C1B_SCCB_ADDR_7BIT 0x10
#define OV01C1B_8BIT_WRITE_ADDR (OV01C1B_SCCB_ADDR_7BIT << 1)

static const char *TAG = "p4scan_power";

static esp_err_t pca9538a_add_device(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_P4SCAN_PCA9538A_I2C_ADDR,
        .scl_speed_hz = 100000,
    };

    return i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle);
}

static esp_err_t pca9538a_write(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};
    return i2c_master_transmit(dev_handle, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t pca9538a_set_output(i2c_master_dev_handle_t dev_handle, uint8_t value)
{
    ESP_RETURN_ON_ERROR(pca9538a_write(dev_handle, PCA9538A_REG_OUTPUT, value), TAG,
                        "failed to write PCA9538A output=0x%02x", value);
    ESP_LOGI(TAG, "PCA9538A output=0x%02x", value);
    return ESP_OK;
}

static esp_err_t p4scan_read_a8v8(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

static void p4scan_probe_sensor_id(i2c_master_bus_handle_t bus_handle, uint8_t addr)
{
    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    uint8_t page = 0xfd;
    uint8_t page0 = 0x00;
    uint8_t chip_id[4] = {0};
    uint8_t eco_ver = 0;

    esp_err_t ret = i2c_master_probe(bus_handle, addr, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "chip ID probe no ACK at 7-bit addr=0x%02x: %s", addr, esp_err_to_name(ret));
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_reset(bus_handle));
        return;
    }

    if (i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle) != ESP_OK) {
        ESP_LOGW(TAG, "probe addr=0x%02x add device failed", addr);
        return;
    }

    ret = i2c_master_transmit(dev_handle, (uint8_t[]){page, page0}, 2, pdMS_TO_TICKS(100));
    for (uint8_t reg = 0; ret == ESP_OK && reg < sizeof(chip_id); reg++) {
        ret = p4scan_read_a8v8(dev_handle, reg, &chip_id[reg]);
    }
    if (ret == ESP_OK) {
        ret = p4scan_read_a8v8(dev_handle, 0x04, &eco_ver);
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "chip ID probe addr=0x%02x id=%02x %02x %02x %02x eco=0x%02x",
                 addr, chip_id[0], chip_id[1], chip_id[2], chip_id[3], eco_ver);
    } else {
        ESP_LOGW(TAG, "chip ID probe addr=0x%02x failed: %s", addr, esp_err_to_name(ret));
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_reset(bus_handle));
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(dev_handle));
}

esp_err_t p4scan_ov01c1b_power_on(i2c_master_bus_handle_t bus_handle)
{
#if CONFIG_P4SCAN_OV01C1B_POWER_ENABLE
    ESP_RETURN_ON_FALSE(bus_handle, ESP_ERR_INVALID_ARG, TAG, "I2C bus handle is NULL");

    esp_err_t ret = ESP_OK;
    i2c_master_dev_handle_t dev_handle = NULL;
    uint8_t output = 0;

    ESP_LOGI(TAG, "OV01C1B power on through PCA9538A addr=0x%02x", CONFIG_P4SCAN_PCA9538A_I2C_ADDR);
    ESP_RETURN_ON_ERROR(pca9538a_add_device(bus_handle, &dev_handle), TAG, "failed to add PCA9538A device");

    ESP_GOTO_ON_ERROR(pca9538a_write(dev_handle, PCA9538A_REG_OUTPUT, 0x00), failed, TAG,
                      "failed to clear PCA9538A outputs");
    ESP_GOTO_ON_ERROR(pca9538a_write(dev_handle, PCA9538A_REG_CONFIG, 0x00), failed, TAG,
                      "failed to configure PCA9538A outputs");
    ESP_LOGI(TAG, "PCA9538A initialized: output=0x00, config=0x00");

    output |= PCA9538A_P7_ILLUM_3V3;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to enable 3V3");

    output |= PCA9538A_P6_CLK24M;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to enable 24MHz clock");

    output |= PCA9538A_P0_DOVDD;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to enable DOVDD");

    output |= PCA9538A_P1_AVDD;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to enable AVDD");

    output |= PCA9538A_P2_DVDD;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to enable DVDD");

    vTaskDelay(pdMS_TO_TICKS(5));

    output |= PCA9538A_P5_XSHUTDN;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to release XSHUTDN");

    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "OV01C1B power on sequence done");

    ESP_LOGI(TAG, "OV01C1B uses 7-bit addr=0x%02x; 8-bit write addr=0x%02x",
             OV01C1B_SCCB_ADDR_7BIT, OV01C1B_8BIT_WRITE_ADDR);
    p4scan_probe_sensor_id(bus_handle, OV01C1B_SCCB_ADDR_7BIT);

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(dev_handle));
    return ESP_OK;

failed:
    if (dev_handle) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(dev_handle));
    }
    return ret;
#else
    (void)bus_handle;
    return ESP_OK;
#endif
}

esp_err_t p4scan_ov01c1b_power_off(i2c_master_bus_handle_t bus_handle)
{
#if CONFIG_P4SCAN_OV01C1B_POWER_ENABLE
    ESP_RETURN_ON_FALSE(bus_handle, ESP_ERR_INVALID_ARG, TAG, "I2C bus handle is NULL");

    esp_err_t ret = ESP_OK;
    i2c_master_dev_handle_t dev_handle = NULL;
    uint8_t output = PCA9538A_P0_DOVDD | PCA9538A_P1_AVDD | PCA9538A_P2_DVDD |
                     PCA9538A_P5_XSHUTDN | PCA9538A_P6_CLK24M | PCA9538A_P7_ILLUM_3V3;

    ESP_LOGI(TAG, "OV01C1B power off through PCA9538A");
    ESP_RETURN_ON_ERROR(pca9538a_add_device(bus_handle, &dev_handle), TAG, "failed to add PCA9538A device");

    output &= ~PCA9538A_P4_ILLUM;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to disable illumination");

    output &= ~PCA9538A_P3_AIM;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to disable aiming light");

    output &= ~PCA9538A_P6_CLK24M;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to disable 24MHz clock");

    output &= ~PCA9538A_P5_XSHUTDN;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to assert XSHUTDN");

    output &= ~PCA9538A_P2_DVDD;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to disable DVDD");

    output &= ~PCA9538A_P1_AVDD;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to disable AVDD");

    output &= ~PCA9538A_P0_DOVDD;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to disable DOVDD");

    output &= ~PCA9538A_P7_ILLUM_3V3;
    ESP_GOTO_ON_ERROR(pca9538a_set_output(dev_handle, output), failed, TAG, "failed to disable 3V3");

    ESP_LOGI(TAG, "OV01C1B power off sequence done");
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(dev_handle));
    return ESP_OK;

failed:
    if (dev_handle) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(dev_handle));
    }
    return ret;
#else
    (void)bus_handle;
    return ESP_OK;
#endif
}
