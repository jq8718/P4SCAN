/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <assert.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/param.h>

#include "esp_err.h"
#include "esp_check.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "example_video_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"
#include "soc/mipi_csi_bridge_struct.h"
#include "soc/mipi_csi_host_struct.h"

#if CONFIG_FORMAT_MJPEG_CAM1
#define ENCODE_DEV_PATH     ESP_VIDEO_JPEG_DEVICE_NAME
#define UVC_OUTPUT_FORMAT   V4L2_PIX_FMT_JPEG
#elif CONFIG_FORMAT_H264_CAM1
#if CONFIG_EXAMPLE_H264_MAX_QP <= CONFIG_EXAMPLE_H264_MIN_QP
#error "CONFIG_EXAMPLE_H264_MAX_QP should larger than CONFIG_EXAMPLE_H264_MIN_QP"
#endif

#define ENCODE_DEV_PATH     ESP_VIDEO_H264_DEVICE_NAME
#define UVC_OUTPUT_FORMAT   V4L2_PIX_FMT_H264
#else
#error "Select a UVC output format in menuconfig"
#endif

#define BUFFER_COUNT        2
#define P4SCAN_FW_MARK      "ov01c1b-raw10-synthetic-input-20260815"
#define P4SCAN_CACHE_LINE   64
#define P4SCAN_VERBOSE_RAW_DIAGNOSTICS 1
#define P4SCAN_RAW8_SYNTHETIC_TEST 0
#define P4SCAN_STOP_CSI_BEFORE_JPEG 0
#define P4SCAN_USE_ISP_RGB565_TEST 0
#define P4SCAN_LOG_BUFFER_LAYOUT 1
#define P4SCAN_AUTO_CSI_TEST 0

typedef struct {
    int cap_fd;
    uint32_t format;
    uint8_t *cap_buffer[BUFFER_COUNT];
    size_t cap_buffer_len[BUFFER_COUNT];
    uint32_t capture_width;
    uint32_t capture_height;
    uint32_t capture_bytesperline;
    uint32_t capture_format;

    int m2m_fd;
    uint8_t *m2m_cap_buffer;
    size_t m2m_cap_buffer_len;
    uint8_t *raw8_buffer;
    size_t raw8_buffer_len;
    uint8_t *raw_compare_buffer;
    size_t raw_compare_buffer_len;
    bool raw_compare_valid;
    bool capture_stream_stopped;
    uint8_t *raw565_buffer;
    size_t raw565_buffer_len;
    uint8_t *pattern_buffer;
    size_t pattern_len;
    uint32_t frame_count;
    bool streaming;
    bool csi_only;
    SemaphoreHandle_t stream_lock;

    uvc_fb_t fb;
} p4_uvc_t;

static const char *TAG = "p4scan_uvc";

static esp_err_t cache_msync_aligned(void *buffer, size_t length, uint32_t flags)
{
    uintptr_t start = (uintptr_t)buffer & ~(uintptr_t)(P4SCAN_CACHE_LINE - 1);
    uintptr_t end = ((uintptr_t)buffer + length + P4SCAN_CACHE_LINE - 1) &
                    ~(uintptr_t)(P4SCAN_CACHE_LINE - 1);

    return esp_cache_msync((void *)start, end - start, flags);
}

static const char *v4l2_format_name(uint32_t format)
{
    switch (format) {
    case V4L2_PIX_FMT_RGB565:
        return "RGB565";
    case V4L2_PIX_FMT_UYVY:
        return "UYVY";
    case V4L2_PIX_FMT_RGB24:
        return "RGB24";
    case V4L2_PIX_FMT_GREY:
        return "GREY";
    case V4L2_PIX_FMT_SBGGR8:
        return "RAW8";
    case V4L2_PIX_FMT_SBGGR10:
        return "RAW10";
    case V4L2_PIX_FMT_YUV420:
        return "YUV420";
    case V4L2_PIX_FMT_JPEG:
        return "JPEG";
    case V4L2_PIX_FMT_H264:
        return "H.264";
    default:
        return "UNKNOWN";
    }
}

static void log_v4l2_pix_format(const char *prefix, const struct v4l2_format *format)
{
    uint32_t pixelformat = format->fmt.pix.pixelformat;

    ESP_LOGI(TAG, "%s: %ux%u %s (%c%c%c%c), bytesperline=%u, sizeimage=%u",
             prefix,
             (unsigned int)format->fmt.pix.width,
             (unsigned int)format->fmt.pix.height,
             v4l2_format_name(pixelformat),
             (char)(pixelformat & 0xff),
             (char)((pixelformat >> 8) & 0xff),
             (char)((pixelformat >> 16) & 0xff),
             (char)((pixelformat >> 24) & 0xff),
             (unsigned int)format->fmt.pix.bytesperline,
             (unsigned int)format->fmt.pix.sizeimage);
}

static void log_buffer_layout(const p4_uvc_t *uvc)
{
#if P4SCAN_LOG_BUFFER_LAYOUT
    ESP_LOGI(TAG, "buffers: cap0=%p/%u cap1=%p/%u raw8=%p/%u rawcmp=%p/%u raw565=%p/%u jpeg=%p/%u",
             uvc->cap_buffer[0], (unsigned int)uvc->cap_buffer_len[0],
             uvc->cap_buffer[1], (unsigned int)uvc->cap_buffer_len[1],
             uvc->raw8_buffer, (unsigned int)uvc->raw8_buffer_len,
             uvc->raw_compare_buffer, (unsigned int)uvc->raw_compare_buffer_len,
             uvc->raw565_buffer, (unsigned int)uvc->raw565_buffer_len,
             uvc->m2m_cap_buffer, (unsigned int)uvc->m2m_cap_buffer_len);
#else
    (void)uvc;
#endif
}

static void log_capture_diagnostics(const p4_uvc_t *uvc, const uint8_t *src, size_t src_len)
{
    const size_t line_bytes = uvc->capture_width * 5 / 4;
    const size_t expected_len = line_bytes * uvc->capture_height;
    const size_t crc_len = MIN(src_len, expected_len);
    const size_t crc_head_len = MIN(crc_len, (size_t)65536);
    const size_t crc_tail_offset = crc_len > crc_head_len ? crc_len - crc_head_len : 0;
    const size_t second_line = line_bytes < src_len ? line_bytes : 0;
    const size_t candidate_strides[] = {line_bytes, 1536, 2048, 2560};

    uint32_t full_crc = esp_rom_crc32_le(0, src, crc_len);
    uint32_t head_crc = esp_rom_crc32_le(0, src, crc_head_len);
    uint32_t tail_crc = esp_rom_crc32_le(0, src + crc_tail_offset, crc_len - crc_tail_offset);

    ESP_LOGI(TAG, "RAW10 layout: bytesused=%u expected=%u line=%u driver_stride=%u width=%u height=%u",
             (unsigned int)src_len,
             (unsigned int)expected_len,
             (unsigned int)line_bytes,
             (unsigned int)uvc->capture_bytesperline,
             (unsigned int)uvc->capture_width,
             (unsigned int)uvc->capture_height);
    ESP_LOGI(TAG, "RAW10 bytes: row0=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
             src_len > 0 ? src[0] : 0, src_len > 1 ? src[1] : 0,
             src_len > 2 ? src[2] : 0, src_len > 3 ? src[3] : 0,
             src_len > 4 ? src[4] : 0, src_len > 5 ? src[5] : 0,
             src_len > 6 ? src[6] : 0, src_len > 7 ? src[7] : 0,
             src_len > 8 ? src[8] : 0, src_len > 9 ? src[9] : 0);
    ESP_LOGI(TAG, "RAW10 bytes: row1=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
             src_len > second_line + 0 ? src[second_line + 0] : 0,
             src_len > second_line + 1 ? src[second_line + 1] : 0,
             src_len > second_line + 2 ? src[second_line + 2] : 0,
             src_len > second_line + 3 ? src[second_line + 3] : 0,
             src_len > second_line + 4 ? src[second_line + 4] : 0,
             src_len > second_line + 5 ? src[second_line + 5] : 0,
             src_len > second_line + 6 ? src[second_line + 6] : 0,
             src_len > second_line + 7 ? src[second_line + 7] : 0,
             src_len > second_line + 8 ? src[second_line + 8] : 0,
             src_len > second_line + 9 ? src[second_line + 9] : 0);

    for (size_t stride_index = 0; stride_index < ARRAY_SIZE(candidate_strides); stride_index++) {
        size_t stride = candidate_strides[stride_index];
        ESP_LOGI(TAG, "RAW10 stride=%u samples: y0=%02x y1=%02x y2=%02x y16=%02x y64=%02x y256=%02x",
                 (unsigned int)stride,
                 src_len > stride * 0 ? src[stride * 0] : 0,
                 src_len > stride * 1 ? src[stride * 1] : 0,
                 src_len > stride * 2 ? src[stride * 2] : 0,
                 src_len > stride * 16 ? src[stride * 16] : 0,
                 src_len > stride * 64 ? src[stride * 64] : 0,
                 src_len > stride * 256 ? src[stride * 256] : 0);
    }

    size_t nonzero_lines = 0;
    size_t last_nonzero_line = 0;
    size_t max_nonzero_bytes = 0;
    uint32_t row0_crc = 0;
    size_t rows_same_previous = 0;
    size_t rows_same_row0 = 0;
    size_t rows_compared = 0;
    size_t row_transition_count = 0;
    for (size_t y = 0; y < uvc->capture_height; y++) {
        size_t offset = y * line_bytes;
        size_t row_len = offset < src_len ? MIN(line_bytes, src_len - offset) : 0;
        size_t row_nonzero = 0;
        for (size_t x = 0; x < row_len; x++) {
            row_nonzero += src[offset + x] != 0;
        }
        uint32_t row_crc = row_len ? esp_rom_crc32_le(0, src + offset, row_len) : 0;
        if (y == 0) {
            row0_crc = row_crc;
        } else if (row_len == line_bytes && offset >= line_bytes) {
            rows_compared++;
            const uint8_t *previous_row = src + offset - line_bytes;
            bool same_previous = row_crc == esp_rom_crc32_le(0, previous_row, line_bytes) &&
                                 memcmp(src + offset, previous_row, line_bytes) == 0;
            if (same_previous) {
                rows_same_previous++;
            }
            if (row_crc == row0_crc && memcmp(src + offset, src, line_bytes) == 0) {
                rows_same_row0++;
            }
            if (!same_previous && row_transition_count < 16) {
                size_t diff_bytes = 0;
                for (size_t x = 0; x < line_bytes; x++) {
                    diff_bytes += src[offset + x] != previous_row[x];
                }
                ESP_LOGI(TAG, "RAW10 row transition: y=%u prev_crc=%08x crc=%08x diff_bytes=%u",
                         (unsigned int)y,
                         (unsigned int)esp_rom_crc32_le(0, previous_row, line_bytes),
                         (unsigned int)row_crc,
                         (unsigned int)diff_bytes);
                row_transition_count++;
            }
        }
        if (row_nonzero > 0) {
            nonzero_lines++;
            last_nonzero_line = y;
            max_nonzero_bytes = MAX(max_nonzero_bytes, row_nonzero);
        }
        if (y < 16) {
            ESP_LOGI(TAG, "RAW10 row detail: y=%u crc=%08x nonzero=%u",
                     (unsigned int)y, (unsigned int)row_crc,
                     (unsigned int)row_nonzero);
        }
        if (y == 0 || y == 64 || y == 128 || y == 192 || y == 256 || y == 512 || y == 768) {
            ESP_LOGI(TAG, "RAW10 row=%u nonzero_bytes=%u",
                     (unsigned int)y, (unsigned int)row_nonzero);
        }
    }
    ESP_LOGI(TAG, "RAW10 nonzero rows=%u last_row=%u max_row_nonzero=%u",
             (unsigned int)nonzero_lines, (unsigned int)last_nonzero_line,
             (unsigned int)max_nonzero_bytes);
    ESP_LOGI(TAG, "RAW10 row compare: compared=%u same_previous=%u same_row0=%u row0_crc=%08x",
             (unsigned int)rows_compared, (unsigned int)rows_same_previous,
             (unsigned int)rows_same_row0, (unsigned int)row0_crc);
    ESP_LOGI(TAG, "RAW10 fingerprint: frame=%u full_crc=%08x head_crc=%08x tail_crc=%08x crc_len=%u",
             (unsigned int)uvc->frame_count, (unsigned int)full_crc,
             (unsigned int)head_crc, (unsigned int)tail_crc, (unsigned int)crc_len);
}

static void compare_raw10_frame(p4_uvc_t *uvc, const uint8_t *src, size_t src_len)
{
    const size_t expected_len = (size_t)uvc->capture_width * uvc->capture_height * 5 / 4;
    const size_t compare_len = MIN(src_len, expected_len);
    const size_t no_diff = (size_t)-1;
    uint32_t crc = esp_rom_crc32_le(0, src, compare_len);

    if (!uvc->raw_compare_buffer || uvc->raw_compare_buffer_len < expected_len) {
        ESP_LOGW(TAG, "RAW10 compare unavailable: buffer=%p len=%u expected=%u",
                 uvc->raw_compare_buffer,
                 (unsigned int)uvc->raw_compare_buffer_len,
                 (unsigned int)expected_len);
        return;
    }

    if (uvc->raw_compare_valid) {
        size_t diff_bytes = 0;
        size_t first_diff = no_diff;

        for (size_t i = 0; i < expected_len; i++) {
            uint8_t previous = i < compare_len ? uvc->raw_compare_buffer[i] : 0;
            uint8_t current = i < src_len ? src[i] : 0;
            if (current != previous) {
                diff_bytes++;
                if (first_diff == no_diff) {
                    first_diff = i;
                }
            }
        }
        ESP_LOGI(TAG, "RAW10 compare: frame=%u crc=%08x same=%s diff_bytes=%u first_diff=%s%u bytes=%u",
                 (unsigned int)uvc->frame_count,
                 (unsigned int)crc,
                 diff_bytes == 0 ? "YES" : "NO",
                 (unsigned int)diff_bytes,
                 first_diff == no_diff ? "-" : "",
                 first_diff == no_diff ? 0 : (unsigned int)first_diff,
                 (unsigned int)src_len);
    } else {
        ESP_LOGI(TAG, "RAW10 compare: frame=%u crc=%08x baseline bytes=%u",
                 (unsigned int)uvc->frame_count,
                 (unsigned int)crc,
                 (unsigned int)src_len);
    }

    memcpy(uvc->raw_compare_buffer, src, compare_len);
    if (compare_len < expected_len) {
        memset(uvc->raw_compare_buffer + compare_len, 0, expected_len - compare_len);
    }
    uvc->raw_compare_valid = true;
}

static void log_csi_bridge_diagnostics(void)
{
    ESP_LOGI(TAG, "CSI bridge: frame=0x%08x type=0x%08x dma=0x%08x block=0x%08x flow=0x%08x interval=0x%08x int_raw=0x%08x int_st=0x%08x",
             (unsigned int)MIPI_CSI_BRIDGE.frame_cfg.val,
             (unsigned int)MIPI_CSI_BRIDGE.data_type_cfg.val,
             (unsigned int)MIPI_CSI_BRIDGE.dma_req_cfg.val,
             (unsigned int)MIPI_CSI_BRIDGE.dmablk_size.val,
             (unsigned int)MIPI_CSI_BRIDGE.buf_flow_ctl.val,
             (unsigned int)MIPI_CSI_BRIDGE.dma_req_interval.val,
             (unsigned int)MIPI_CSI_BRIDGE.int_raw.val,
             (unsigned int)MIPI_CSI_BRIDGE.int_st.val);
    ESP_LOGI(TAG, "CSI host: main=0x%08x phy=0x%08x pkt=0x%08x boundary=0x%08x seq=0x%08x crc=0x%08x pld_crc=0x%08x data_id=0x%08x ecc=0x%08x",
             (unsigned int)MIPI_CSI_HOST.int_st_main.val,
             (unsigned int)MIPI_CSI_HOST.int_st_phy_fatal.val,
             (unsigned int)MIPI_CSI_HOST.int_st_pkt_fatal.val,
             (unsigned int)MIPI_CSI_HOST.int_st_bndry_frame_fatal.val,
             (unsigned int)MIPI_CSI_HOST.int_st_seq_frame_fatal.val,
             (unsigned int)MIPI_CSI_HOST.int_st_crc_frame_fatal.val,
             (unsigned int)MIPI_CSI_HOST.int_st_pld_crc_fatal.val,
             (unsigned int)MIPI_CSI_HOST.int_st_data_id.val,
             (unsigned int)MIPI_CSI_HOST.int_st_ecc_corrected.val);
}

static void unpack_raw10_to_raw8(const uint8_t *src, uint8_t *dst, int width, int height)
{
    const size_t src_line_bytes = (size_t)width * 5 / 4;

    for (int y = 0; y < height; y++) {
        const uint8_t *src_line = src + (size_t)y * src_line_bytes;
        uint8_t *dst_line = dst + (size_t)y * width;
        for (int x = 0; x < width; x += 4) {
            const uint8_t *group = src_line + (size_t)x / 4 * 5;
            uint8_t low_bits = group[4];
            dst_line[x + 0] = (uint8_t)((group[0] << 2) | ((low_bits >> 0) & 0x03));
            dst_line[x + 1] = (uint8_t)((group[1] << 2) | ((low_bits >> 2) & 0x03));
            dst_line[x + 2] = (uint8_t)((group[2] << 2) | ((low_bits >> 4) & 0x03));
            dst_line[x + 3] = (uint8_t)((group[3] << 2) | ((low_bits >> 6) & 0x03));
        }
    }
}

static void convert_raw8_to_rgb565(const uint8_t *src, uint8_t *dst, int width, int height)
{
    const size_t pixel_count = (size_t)width * height;

    for (size_t i = 0; i < pixel_count; i++) {
        uint8_t value = src[i];
        uint16_t pixel = (uint16_t)(((value & 0xf8) << 8) |
                                    ((value & 0xfc) << 3) |
                                    (value >> 3));
        dst[i * 2 + 0] = (uint8_t)pixel;
        dst[i * 2 + 1] = (uint8_t)(pixel >> 8);
    }
}

static void log_raw8_diagnostics(const uint8_t *src, size_t len, int width, int height)
{
    size_t step = len / 4096;
    uint64_t sum = 0;
    uint8_t min_value = 0xff;
    uint8_t max_value = 0;

    if (step == 0) {
        step = 1;
    }
    for (size_t i = 0; i < len; i += step) {
        uint8_t value = src[i];
        sum += value;
        min_value = MIN(min_value, value);
        max_value = MAX(max_value, value);
    }
    uint32_t row_means[4] = {0};
    const int sample_rows[4] = {0, height / 4, height / 2, (height * 3) / 4};
    for (int row_index = 0; row_index < 4; row_index++) {
        const uint8_t *row = src + (size_t)sample_rows[row_index] * width;
        uint32_t row_sum = 0;
        for (int x = 0; x < width; x++) {
            row_sum += row[x];
        }
        row_means[row_index] = row_sum / (uint32_t)width;
    }
    ESP_LOGI(TAG, "RAW8 converted: row0=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x row1=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x sample_min=%u sample_max=%u sample_mean=%u",
             src[0], src[1], src[2], src[3], src[4], src[5], src[6], src[7], src[8], src[9],
             src[width], src[width + 1], src[width + 2], src[width + 3], src[width + 4],
             src[width + 5], src[width + 6], src[width + 7], src[width + 8], src[width + 9],
             min_value, max_value, (unsigned int)(sum / ((len + step - 1) / step)));
    ESP_LOGI(TAG, "RAW8 row means: y0=%u y=%u=%u y=%u=%u y=%u=%u",
             (unsigned int)row_means[0], sample_rows[1], (unsigned int)row_means[1],
             sample_rows[2], (unsigned int)row_means[2], sample_rows[3], (unsigned int)row_means[3]);
    (void)height;
}

static void set_capture_frame_rate(int fd, int rate)
{
    struct v4l2_streamparm parm = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };

    parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = rate;

    if (ioctl(fd, VIDIOC_S_PARM, &parm) != 0) {
        ESP_LOGW(TAG, "failed to set capture frame rate to %dfps", rate);
        return;
    }

    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &parm) == 0) {
        ESP_LOGI(TAG, "capture frame rate active: %u/%u sec",
                 (unsigned int)parm.parm.capture.timeperframe.numerator,
                 (unsigned int)parm.parm.capture.timeperframe.denominator);
    }
}

#if CONFIG_P4SCAN_UVC_TEST_PATTERN
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3);
}

static esp_err_t fill_rgb565_test_pattern(p4_uvc_t *uvc, int width, int height)
{
    const size_t frame_len = (size_t)width * height * 2;
    const uint16_t colors[] = {
        rgb565(255, 255, 255),
        rgb565(255, 255, 0),
        rgb565(0, 255, 255),
        rgb565(0, 255, 0),
        rgb565(255, 0, 255),
        rgb565(255, 0, 0),
        rgb565(0, 0, 255),
        rgb565(0, 0, 0),
    };

    if (uvc->pattern_len != frame_len) {
        if (uvc->pattern_buffer) {
            heap_caps_free(uvc->pattern_buffer);
        }
        uvc->pattern_buffer = heap_caps_aligned_alloc(128, frame_len,
                                                      MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
        if (!uvc->pattern_buffer) {
            ESP_LOGE(TAG, "failed to allocate %u bytes for test pattern", (unsigned int)frame_len);
            return ESP_ERR_NO_MEM;
        }
        uvc->pattern_len = frame_len;
    }

    uint16_t *dst = (uint16_t *)uvc->pattern_buffer;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int bar = (x * (int)(sizeof(colors) / sizeof(colors[0]))) / width;
            uint16_t color = colors[bar];

            if (((x / 64) ^ (y / 64)) & 1) {
                uint8_t shade = (uint8_t)((y * 255) / MAX(height - 1, 1));
                color = rgb565(shade, shade, shade);
            }
            dst[(size_t)y * width + x] = color;
        }
    }

    ESP_LOGI(TAG, "generated RGB565 test pattern: %dx%d, %u bytes",
             width, height, (unsigned int)frame_len);
    return ESP_OK;
}

static esp_err_t fill_grey_test_pattern(p4_uvc_t *uvc, int width, int height)
{
    const size_t frame_len = (size_t)width * height;

    if (uvc->pattern_len != frame_len) {
        if (uvc->pattern_buffer) {
            heap_caps_free(uvc->pattern_buffer);
        }
        uvc->pattern_buffer = heap_caps_aligned_alloc(128, frame_len,
                                                      MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
        if (!uvc->pattern_buffer) {
            ESP_LOGE(TAG, "failed to allocate %u bytes for grey test pattern", (unsigned int)frame_len);
            return ESP_ERR_NO_MEM;
        }
        uvc->pattern_len = frame_len;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int bar = (x * 8) / width;
            uint8_t value = (uint8_t)(bar * 32);
            if (((x / 64) ^ (y / 64)) & 1) {
                value = (uint8_t)((y * 255) / MAX(height - 1, 1));
            }
            uvc->pattern_buffer[(size_t)y * width + x] = value;
        }
    }

    ESP_LOGI(TAG, "generated GREY test pattern: %dx%d, %u bytes",
             width, height, (unsigned int)frame_len);
    return ESP_OK;
}
#endif

static void print_video_device_info(const struct v4l2_capability *capability)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability->version >> 16),
             (uint8_t)(capability->version >> 8),
             (uint8_t)capability->version);
    ESP_LOGI(TAG, "driver:  %s", capability->driver);
    ESP_LOGI(TAG, "card:    %s", capability->card);
    ESP_LOGI(TAG, "bus:     %s", capability->bus_info);
    ESP_LOGI(TAG, "capabilities:");
    if (capability->capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        ESP_LOGI(TAG, "\tVIDEO_CAPTURE");
    }
    if (capability->capabilities & V4L2_CAP_READWRITE) {
        ESP_LOGI(TAG, "\tREADWRITE");
    }
    if (capability->capabilities & V4L2_CAP_ASYNCIO) {
        ESP_LOGI(TAG, "\tASYNCIO");
    }
    if (capability->capabilities & V4L2_CAP_STREAMING) {
        ESP_LOGI(TAG, "\tSTREAMING");
    }
    if (capability->capabilities & V4L2_CAP_META_OUTPUT) {
        ESP_LOGI(TAG, "\tMETA_OUTPUT");
    }
    if (capability->capabilities & V4L2_CAP_DEVICE_CAPS) {
        ESP_LOGI(TAG, "device capabilities:");
        if (capability->device_caps & V4L2_CAP_VIDEO_CAPTURE) {
            ESP_LOGI(TAG, "\tVIDEO_CAPTURE");
        }
        if (capability->device_caps & V4L2_CAP_READWRITE) {
            ESP_LOGI(TAG, "\tREADWRITE");
        }
        if (capability->device_caps & V4L2_CAP_ASYNCIO) {
            ESP_LOGI(TAG, "\tASYNCIO");
        }
        if (capability->device_caps & V4L2_CAP_STREAMING) {
            ESP_LOGI(TAG, "\tSTREAMING");
        }
        if (capability->device_caps & V4L2_CAP_META_OUTPUT) {
            ESP_LOGI(TAG, "\tMETA_OUTPUT");
        }
    }
}

static esp_err_t init_capture_video(p4_uvc_t *uvc)
{
    struct v4l2_capability capability;

    int fd = open(EXAMPLE_CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open capture video device %s, errno=%d", EXAMPLE_CAM_DEV_PATH, errno);
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(ioctl(fd, VIDIOC_QUERYCAP, &capability));
    print_video_device_info(&capability);

    uvc->cap_fd = fd;
    return ESP_OK;
}

static esp_err_t init_codec_video(p4_uvc_t *uvc)
{
    struct v4l2_capability capability;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    int fd = open(ENCODE_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open encoder video device %s, errno=%d", ENCODE_DEV_PATH, errno);
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(ioctl(fd, VIDIOC_QUERYCAP, &capability));
    print_video_device_info(&capability);

#if CONFIG_FORMAT_MJPEG_CAM1
    controls.ctrl_class = V4L2_CID_JPEG_CLASS;
    controls.count = 1;
    controls.controls = control;
    control[0].id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
    control[0].value = CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set JPEG compression quality");
    }
#elif CONFIG_FORMAT_H264_CAM1
    controls.ctrl_class = V4L2_CID_CODEC_CLASS;
    controls.count = 1;
    controls.controls = control;
    control[0].id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD;
    control[0].value = CONFIG_EXAMPLE_H264_I_PERIOD;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 intra frame period");
    }

    control[0].id = V4L2_CID_MPEG_VIDEO_BITRATE;
    control[0].value = CONFIG_EXAMPLE_H264_BITRATE;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 bitrate");
    }

    control[0].id = V4L2_CID_MPEG_VIDEO_H264_MIN_QP;
    control[0].value = CONFIG_EXAMPLE_H264_MIN_QP;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 minimum quality");
    }

    control[0].id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP;
    control[0].value = CONFIG_EXAMPLE_H264_MAX_QP;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 maximum quality");
    }
#endif

    uvc->format = UVC_OUTPUT_FORMAT;
    uvc->m2m_fd = fd;
    return ESP_OK;
}

static void video_stop_cb(void *cb_ctx);

static esp_err_t video_start_cb_internal(uvc_format_t uvc_format, int width, int height, int rate,
                                         void *cb_ctx, bool csi_only)
{
    (void)uvc_format;

    struct v4l2_buffer buf;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    p4_uvc_t *uvc = (p4_uvc_t *)cb_ctx;
    uint32_t capture_fmt = 0;
    uint32_t encoder_input_fmt = 0;

    ESP_LOGI(TAG, "UVC stream start: %dx%d@%dfps", width, height, rate);
    xSemaphoreTake(uvc->stream_lock, portMAX_DELAY);
    if (uvc->streaming) {
        ESP_LOGW(TAG, "UVC stream already active; stopping it before reconfiguration");
        xSemaphoreGive(uvc->stream_lock);
        video_stop_cb(uvc);
        xSemaphoreTake(uvc->stream_lock, portMAX_DELAY);
    }
    uvc->frame_count = 0;
    uvc->raw_compare_valid = false;
    uvc->capture_stream_stopped = false;
    uvc->csi_only = csi_only;

#if CONFIG_P4SCAN_UVC_TEST_PATTERN
    capture_fmt = V4L2_PIX_FMT_GREY;
    encoder_input_fmt = capture_fmt;
    ESP_RETURN_ON_ERROR(fill_grey_test_pattern(uvc, width, height), TAG, "failed to generate grey test pattern");
#else
    if (uvc->format == V4L2_PIX_FMT_JPEG) {
        int fmt_index = 0;
        const uint32_t jpeg_input_formats[] = {
            V4L2_PIX_FMT_RGB565,
            V4L2_PIX_FMT_UYVY,
            V4L2_PIX_FMT_RGB24,
            V4L2_PIX_FMT_GREY,
        };
        const int jpeg_input_formats_num = sizeof(jpeg_input_formats) / sizeof(jpeg_input_formats[0]);

        while (!capture_fmt) {
            struct v4l2_fmtdesc fmtdesc = {
                .index = fmt_index++,
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            };

            if (ioctl(uvc->cap_fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
                break;
            }

            ESP_LOGI(TAG, "camera supports capture format: %s (%c%c%c%c)",
                     v4l2_format_name(fmtdesc.pixelformat),
                     (char)(fmtdesc.pixelformat & 0xff),
                     (char)((fmtdesc.pixelformat >> 8) & 0xff),
                     (char)((fmtdesc.pixelformat >> 16) & 0xff),
                     (char)((fmtdesc.pixelformat >> 24) & 0xff));

            for (int i = 0; i < jpeg_input_formats_num; i++) {
                if (jpeg_input_formats[i] == fmtdesc.pixelformat) {
                    capture_fmt = jpeg_input_formats[i];
                    break;
                }
            }
        }

        if (!capture_fmt) {
            ESP_LOGE(TAG, "camera output pixel format is not supported by JPEG encoder");
            xSemaphoreGive(uvc->stream_lock);
            return ESP_ERR_NOT_SUPPORTED;
        }
    } else {
        capture_fmt = V4L2_PIX_FMT_YUV420;
    }

#if P4SCAN_USE_ISP_RGB565_TEST
    capture_fmt = V4L2_PIX_FMT_RGB565;
    encoder_input_fmt = V4L2_PIX_FMT_RGB565;
    ESP_LOGI(TAG, "ISP path test: CSI RAW10 -> ISP RGB565 -> JPEG");
#else
    capture_fmt = V4L2_PIX_FMT_SBGGR10;
    encoder_input_fmt = V4L2_PIX_FMT_GREY;
    ESP_LOGI(TAG, "mono path: CSI RAW10 -> RAW10 unpack -> GREY -> JPEG");
#endif

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_S_FMT, &format));
    log_v4l2_pix_format("capture format selected", &format);
    uvc->capture_width = format.fmt.pix.width;
    uvc->capture_height = format.fmt.pix.height;
    uvc->capture_bytesperline = format.fmt.pix.bytesperline;
    uvc->capture_format = format.fmt.pix.pixelformat;
    if (capture_fmt == V4L2_PIX_FMT_SBGGR10 || capture_fmt == V4L2_PIX_FMT_SBGGR8) {
        if (capture_fmt == V4L2_PIX_FMT_SBGGR10) {
            uvc->raw8_buffer_len = (size_t)uvc->capture_width * uvc->capture_height;
            uvc->raw8_buffer = heap_caps_aligned_alloc(128, uvc->raw8_buffer_len,
                                                       MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
            ESP_RETURN_ON_FALSE(uvc->raw8_buffer, ESP_ERR_NO_MEM, TAG, "failed to allocate RAW8 conversion buffer");
#if P4SCAN_RAW8_SYNTHETIC_TEST
            uvc->pattern_buffer = heap_caps_aligned_alloc(128, uvc->raw8_buffer_len,
                                                          MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
            ESP_RETURN_ON_FALSE(uvc->pattern_buffer, ESP_ERR_NO_MEM, TAG, "failed to allocate RAW8 test input buffer");
            uvc->pattern_len = uvc->raw8_buffer_len;
            for (size_t i = 0; i < uvc->pattern_len; i++) {
                uvc->pattern_buffer[i] = (uint8_t)((i % uvc->capture_width) * 255 /
                                                   MAX(uvc->capture_width - 1, 1));
            }
#endif

            uvc->raw_compare_buffer_len = (size_t)uvc->capture_width * uvc->capture_height * 5 / 4;
            uvc->raw_compare_buffer = heap_caps_aligned_alloc(128, uvc->raw_compare_buffer_len,
                                                               MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
            ESP_RETURN_ON_FALSE(uvc->raw_compare_buffer, ESP_ERR_NO_MEM, TAG, "failed to allocate RAW10 compare buffer");
        }
        uvc->raw565_buffer_len = (size_t)uvc->capture_width * uvc->capture_height * 2;
        uvc->raw565_buffer = heap_caps_aligned_alloc(128, uvc->raw565_buffer_len,
                                                     MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
        ESP_RETURN_ON_FALSE(uvc->raw565_buffer, ESP_ERR_NO_MEM, TAG, "failed to allocate RGB565 conversion buffer");
    }
    set_capture_frame_rate(uvc->cap_fd, rate);

    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_REQBUFS, &req));

    for (int i = 0; i < BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QUERYBUF, &buf));

        uvc->cap_buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                              MAP_SHARED, uvc->cap_fd, buf.m.offset);
        assert(uvc->cap_buffer[i]);
        uvc->cap_buffer_len[i] = buf.length;
        memset(uvc->cap_buffer[i], 0, buf.length);
        ESP_ERROR_CHECK(cache_msync_aligned(uvc->cap_buffer[i], buf.length,
                                            ESP_CACHE_MSYNC_FLAG_DIR_C2M));

        ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QBUF, &buf));
    }
#endif

    if (!csi_only) {
        memset(&format, 0, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = encoder_input_fmt;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_S_FMT, &format));
        log_v4l2_pix_format("encoder input format selected", &format);

        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        req.memory = V4L2_MEMORY_USERPTR;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_REQBUFS, &req));

        memset(&format, 0, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = uvc->format;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_S_FMT, &format));
        log_v4l2_pix_format("encoder output format selected", &format);

        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_REQBUFS, &req));

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QUERYBUF, &buf));

        uvc->m2m_cap_buffer = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                               MAP_SHARED, uvc->m2m_fd, buf.m.offset);
        assert(uvc->m2m_cap_buffer);
        uvc->m2m_cap_buffer_len = buf.length;

        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &buf));

        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_STREAMON, &type));
        type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_STREAMON, &type));
    } else {
        ESP_LOGI(TAG, "CSI-only test: JPEG encoder path disabled");
    }
#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_STREAMON, &type));
#endif

    log_buffer_layout(uvc);

    uvc->streaming = true;
    xSemaphoreGive(uvc->stream_lock);
    return ESP_OK;
}

static esp_err_t video_start_cb(uvc_format_t uvc_format, int width, int height, int rate, void *cb_ctx)
{
    return video_start_cb_internal(uvc_format, width, height, rate, cb_ctx, false);
}

static void video_stop_cb(void *cb_ctx)
{
    p4_uvc_t *uvc = (p4_uvc_t *)cb_ctx;
    struct v4l2_requestbuffers req;

    ESP_LOGI(TAG, "UVC stream stop");
    xSemaphoreTake(uvc->stream_lock, portMAX_DELAY);
    if (!uvc->streaming) {
        xSemaphoreGive(uvc->stream_lock);
        ESP_LOGI(TAG, "UVC stream already stopped");
        return;
    }
    uvc->streaming = false;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    ioctl(uvc->cap_fd, VIDIOC_STREAMOFF, &type);

    for (int i = 0; i < BUFFER_COUNT; i++) {
        if (uvc->cap_buffer[i]) {
            munmap(uvc->cap_buffer[i], uvc->cap_buffer_len[i]);
            uvc->cap_buffer[i] = NULL;
            uvc->cap_buffer_len[i] = 0;
        }
    }

    memset(&req, 0, sizeof(req));
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(uvc->cap_fd, VIDIOC_REQBUFS, &req);

#endif

    if (!uvc->csi_only) {
        type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ioctl(uvc->m2m_fd, VIDIOC_STREAMOFF, &type);
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(uvc->m2m_fd, VIDIOC_STREAMOFF, &type);
    }

    if (uvc->m2m_cap_buffer) {
        munmap(uvc->m2m_cap_buffer, uvc->m2m_cap_buffer_len);
        uvc->m2m_cap_buffer = NULL;
        uvc->m2m_cap_buffer_len = 0;
    }

    if (uvc->raw8_buffer) {
        heap_caps_free(uvc->raw8_buffer);
        uvc->raw8_buffer = NULL;
        uvc->raw8_buffer_len = 0;
    }
    if (uvc->raw_compare_buffer) {
        heap_caps_free(uvc->raw_compare_buffer);
        uvc->raw_compare_buffer = NULL;
        uvc->raw_compare_buffer_len = 0;
        uvc->raw_compare_valid = false;
    }
    if (uvc->raw565_buffer) {
        heap_caps_free(uvc->raw565_buffer);
        uvc->raw565_buffer = NULL;
        uvc->raw565_buffer_len = 0;
    }

    if (!uvc->csi_only) {
        memset(&req, 0, sizeof(req));
        req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        req.memory = V4L2_MEMORY_USERPTR;
        ioctl(uvc->m2m_fd, VIDIOC_REQBUFS, &req);

        memset(&req, 0, sizeof(req));
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        ioctl(uvc->m2m_fd, VIDIOC_REQBUFS, &req);
    }

    xSemaphoreGive(uvc->stream_lock);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static uvc_fb_t *video_fb_get_cb(void *cb_ctx)
{
    p4_uvc_t *uvc = (p4_uvc_t *)cb_ctx;
    struct v4l2_format format;
    struct v4l2_buffer cap_buf;
    struct v4l2_buffer m2m_out_buf;
    struct v4l2_buffer m2m_cap_buf;
    const uint8_t *capture_frame = NULL;

    xSemaphoreTake(uvc->stream_lock, portMAX_DELAY);
    if (!uvc->streaming) {
        xSemaphoreGive(uvc->stream_lock);
        return NULL;
    }

#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    memset(&cap_buf, 0, sizeof(cap_buf));
    cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cap_buf.memory = V4L2_MEMORY_MMAP;
    if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count < 8) {
        ESP_LOGI(TAG, "CSI DQBUF wait: requested_frame=%u", (unsigned int)uvc->frame_count);
    }
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_DQBUF, &cap_buf));
    if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count < 8) {
        ESP_LOGI(TAG, "CSI DQBUF done: index=%u bytesused=%u",
                 (unsigned int)cap_buf.index, (unsigned int)cap_buf.bytesused);
    }
    ESP_ERROR_CHECK(cache_msync_aligned(uvc->cap_buffer[cap_buf.index],
                                        uvc->cap_buffer_len[cap_buf.index],
                                        ESP_CACHE_MSYNC_FLAG_DIR_M2C));
    capture_frame = uvc->cap_buffer[cap_buf.index];
#if P4SCAN_STOP_CSI_BEFORE_JPEG
    {
        int capture_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_STREAMOFF, &capture_type));
        uvc->capture_stream_stopped = true;
        ESP_LOGW(TAG, "CSI stream stopped after frame capture for JPEG isolation");
    }
#endif
        if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count < 2) {
            log_csi_bridge_diagnostics();
        }
        if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count < 8) {
            ESP_LOGI(TAG, "camera frame: index=%u bytesused=%u",
                 (unsigned int)cap_buf.index,
                 (unsigned int)cap_buf.bytesused);
        if (uvc->frame_count < 8 && uvc->capture_format == V4L2_PIX_FMT_SBGGR10) {
            log_capture_diagnostics(uvc, capture_frame, cap_buf.bytesused);
            compare_raw10_frame(uvc, capture_frame, cap_buf.bytesused);
        }
    }
#endif

    memset(&m2m_out_buf, 0, sizeof(m2m_out_buf));
    m2m_out_buf.index = 0;
    m2m_out_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    m2m_out_buf.memory = V4L2_MEMORY_USERPTR;
#if CONFIG_P4SCAN_UVC_TEST_PATTERN
    ESP_ERROR_CHECK(esp_cache_msync(uvc->pattern_buffer, uvc->pattern_len,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED));
    m2m_out_buf.m.userptr = (unsigned long)uvc->pattern_buffer;
    m2m_out_buf.length = uvc->pattern_len;
#else
    if (uvc->capture_format == V4L2_PIX_FMT_SBGGR10) {
        unpack_raw10_to_raw8(capture_frame, uvc->raw8_buffer,
                             uvc->capture_width, uvc->capture_height);
#if P4SCAN_RAW8_SYNTHETIC_TEST
        for (size_t i = 0; i < uvc->raw8_buffer_len; i++) {
            uvc->raw8_buffer[i] = (uint8_t)((i % uvc->capture_width) * 255 /
                                            MAX(uvc->capture_width - 1, 1));
        }
        ESP_LOGW(TAG, "RAW8 synthetic gradient substituted after RAW10 unpack");
#endif
        if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count < 2) {
            log_raw8_diagnostics(uvc->raw8_buffer, uvc->raw8_buffer_len,
                                 uvc->capture_width, uvc->capture_height);
        }
        ESP_ERROR_CHECK(esp_cache_msync(uvc->raw8_buffer, uvc->raw8_buffer_len,
                                         ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED));
#if P4SCAN_RAW8_SYNTHETIC_TEST
        ESP_ERROR_CHECK(esp_cache_msync(uvc->pattern_buffer, uvc->pattern_len,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED));
        m2m_out_buf.m.userptr = (unsigned long)uvc->pattern_buffer;
        m2m_out_buf.length = uvc->pattern_len;
#else
        m2m_out_buf.m.userptr = (unsigned long)uvc->raw8_buffer;
        m2m_out_buf.length = uvc->raw8_buffer_len;
#endif
    } else if (uvc->capture_format == V4L2_PIX_FMT_SBGGR8) {
        if (uvc->frame_count < 2) {
            log_raw8_diagnostics(uvc->cap_buffer[cap_buf.index], cap_buf.bytesused,
                                 uvc->capture_width, uvc->capture_height);
        }
        convert_raw8_to_rgb565(uvc->cap_buffer[cap_buf.index], uvc->raw565_buffer,
                               uvc->capture_width, uvc->capture_height);
        ESP_ERROR_CHECK(esp_cache_msync(uvc->raw565_buffer, uvc->raw565_buffer_len,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED));
        m2m_out_buf.m.userptr = (unsigned long)uvc->raw565_buffer;
        m2m_out_buf.length = uvc->raw565_buffer_len;
    } else {
        m2m_out_buf.m.userptr = (unsigned long)uvc->cap_buffer[cap_buf.index];
        m2m_out_buf.length = cap_buf.bytesused;
    }
#endif
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &m2m_out_buf));

    memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
    m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_DQBUF, &m2m_cap_buf));

    uintptr_t m2m_cache_start = (uintptr_t)uvc->m2m_cap_buffer & ~(uintptr_t)0x3f;
    size_t m2m_cache_offset = (uintptr_t)uvc->m2m_cap_buffer - m2m_cache_start;
    size_t m2m_cache_len = (m2m_cache_offset + m2m_cap_buf.bytesused + 0x3f) & ~(size_t)0x3f;
    ESP_ERROR_CHECK(esp_cache_msync((void *)m2m_cache_start, m2m_cache_len,
                                    ESP_CACHE_MSYNC_FLAG_DIR_M2C));

    if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count < 2) {
        ESP_LOGI(TAG, "JPEG prefix: %02x %02x %02x %02x %02x %02x %02x %02x "
                 "%02x %02x %02x %02x %02x %02x %02x %02x "
                 "%02x %02x %02x %02x %02x %02x %02x %02x "
                 "%02x %02x %02x %02x %02x %02x %02x %02x",
                 uvc->m2m_cap_buffer[0], uvc->m2m_cap_buffer[1],
                 uvc->m2m_cap_buffer[2], uvc->m2m_cap_buffer[3],
                 uvc->m2m_cap_buffer[4], uvc->m2m_cap_buffer[5],
                 uvc->m2m_cap_buffer[6], uvc->m2m_cap_buffer[7],
                 uvc->m2m_cap_buffer[8], uvc->m2m_cap_buffer[9],
                 uvc->m2m_cap_buffer[10], uvc->m2m_cap_buffer[11],
                 uvc->m2m_cap_buffer[12], uvc->m2m_cap_buffer[13],
                 uvc->m2m_cap_buffer[14], uvc->m2m_cap_buffer[15],
                 uvc->m2m_cap_buffer[16], uvc->m2m_cap_buffer[17],
                 uvc->m2m_cap_buffer[18], uvc->m2m_cap_buffer[19],
                 uvc->m2m_cap_buffer[20], uvc->m2m_cap_buffer[21],
                 uvc->m2m_cap_buffer[22], uvc->m2m_cap_buffer[23],
                 uvc->m2m_cap_buffer[24], uvc->m2m_cap_buffer[25],
                 uvc->m2m_cap_buffer[26], uvc->m2m_cap_buffer[27],
                 uvc->m2m_cap_buffer[28], uvc->m2m_cap_buffer[29],
                 uvc->m2m_cap_buffer[30], uvc->m2m_cap_buffer[31]);
    }

#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    if (!uvc->capture_stream_stopped) {
        ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QBUF, &cap_buf));
        if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count < 8) {
            ESP_LOGI(TAG, "CSI QBUF returned: index=%u", (unsigned int)cap_buf.index);
        }
    }
#endif
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_DQBUF, &m2m_out_buf));

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_G_FMT, &format));

    uvc->fb.buf = uvc->m2m_cap_buffer;
    uvc->fb.len = m2m_cap_buf.bytesused;
    uvc->fb.width = format.fmt.pix.width;
    uvc->fb.height = format.fmt.pix.height;
    uvc->fb.format = format.fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG ? UVC_FORMAT_JPEG : UVC_FORMAT_H264;

    int64_t us = esp_timer_get_time();
    uvc->fb.timestamp.tv_sec = us / 1000000UL;
    uvc->fb.timestamp.tv_usec = us % 1000000UL;

    uvc->frame_count++;
    if (P4SCAN_VERBOSE_RAW_DIAGNOSTICS && uvc->frame_count <= 8) {
        ESP_LOGI(TAG, "JPEG fingerprint: frame=%u crc=%08x len=%u",
                 (unsigned int)(uvc->frame_count - 1),
                 (unsigned int)esp_rom_crc32_le(0, uvc->fb.buf, uvc->fb.len),
                 (unsigned int)uvc->fb.len);
    }
    if ((uvc->frame_count % 1) == 0) {
        ESP_LOGI(TAG, "UVC frame ready: %ux%u %s, %u bytes, head=%02x%02x tail=%02x%02x",
                 uvc->fb.width,
                 uvc->fb.height,
                 uvc->fb.format == UVC_FORMAT_JPEG ? "JPEG" : "H.264",
                 (unsigned int)uvc->fb.len,
                 uvc->fb.len >= 2 ? uvc->fb.buf[0] : 0,
                 uvc->fb.len >= 2 ? uvc->fb.buf[1] : 0,
                 uvc->fb.len >= 2 ? uvc->fb.buf[uvc->fb.len - 2] : 0,
                 uvc->fb.len >= 2 ? uvc->fb.buf[uvc->fb.len - 1] : 0);
    }

    return &uvc->fb;
}

static void video_fb_return_cb(uvc_fb_t *fb, void *cb_ctx)
{
    (void)fb;
    p4_uvc_t *uvc = (p4_uvc_t *)cb_ctx;
    struct v4l2_buffer m2m_cap_buf;

    if (uvc->streaming && uvc->m2m_cap_buffer) {
        memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
        m2m_cap_buf.index = 0;
        m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
        ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &m2m_cap_buf));
    }
    xSemaphoreGive(uvc->stream_lock);
}

static esp_err_t run_csi_continuous_test(p4_uvc_t *uvc)
{
    esp_err_t ret = video_start_cb_internal(UVC_FORMAT_JPEG, 1024, 1024, 2, uvc, true);
    if (ret != ESP_OK) {
        return ret;
    }

    for (uint32_t frame = 0; frame < 5; frame++) {
        struct v4l2_buffer cap_buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(uvc->cap_fd, VIDIOC_DQBUF, &cap_buf) != 0) {
            ret = ESP_FAIL;
            ESP_LOGE(TAG, "continuous-frame test: frame=%u DQBUF failed",
                     (unsigned int)frame);
            log_csi_bridge_diagnostics();
            break;
        }

        ESP_ERROR_CHECK(cache_msync_aligned(uvc->cap_buffer[cap_buf.index],
                                            uvc->cap_buffer_len[cap_buf.index],
                                            ESP_CACHE_MSYNC_FLAG_DIR_M2C));
        uint32_t crc = esp_rom_crc32_le(0, uvc->cap_buffer[cap_buf.index], cap_buf.bytesused);
        ESP_LOGI(TAG, "continuous-frame test: frame=%u index=%u bytesused=%u crc=%08x flags=0x%08x",
                 (unsigned int)frame, (unsigned int)cap_buf.index,
                 (unsigned int)cap_buf.bytesused, (unsigned int)crc,
                 (unsigned int)cap_buf.flags);
        ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QBUF, &cap_buf));
    }

    video_stop_cb(uvc);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "continuous-frame test passed: 5 complete frames");
    }
    return ret;
}

static esp_err_t init_uvc(p4_uvc_t *uvc)
{
    const int index = 0;
    uvc_device_config_t config = {
        .start_cb = video_start_cb,
        .fb_get_cb = video_fb_get_cb,
        .fb_return_cb = video_fb_return_cb,
        .stop_cb = video_stop_cb,
        .cb_ctx = (void *)uvc,
    };

    config.uvc_buffer_size = UVC_FRAMES_INFO[index][0].width * UVC_FRAMES_INFO[index][0].height;
    config.uvc_buffer = malloc(config.uvc_buffer_size);
    assert(config.uvc_buffer);
    ESP_LOGI(TAG, "UVC buffer: addr=%p len=%u", config.uvc_buffer,
             (unsigned int)config.uvc_buffer_size);

    ESP_LOGI(TAG, "UVC format: %s", uvc->format == V4L2_PIX_FMT_JPEG ? "MJPEG" : "H.264");
    ESP_LOGI(TAG, "UVC frame: %d * %d @%dfps", UVC_FRAMES_INFO[index][0].width,
             UVC_FRAMES_INFO[index][0].height, UVC_FRAMES_INFO[index][0].rate);

    ESP_ERROR_CHECK(uvc_device_config(index, &config));
    ESP_ERROR_CHECK(uvc_device_init());

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "ESP32-P4 OV01C1B USB camera starting, fw=%s", P4SCAN_FW_MARK);

    p4_uvc_t *uvc = calloc(1, sizeof(p4_uvc_t));
    assert(uvc);
    uvc->stream_lock = xSemaphoreCreateMutex();
    assert(uvc->stream_lock);

    ESP_ERROR_CHECK(example_video_init());
#if CONFIG_P4SCAN_UVC_TEST_PATTERN
    ESP_LOGW(TAG, "UVC test pattern mode enabled; camera frames are not used");
#endif
    ret = init_capture_video(uvc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "capture video init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = init_codec_video(uvc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "codec video init failed: %s", esp_err_to_name(ret));
        return;
    }

#if P4SCAN_AUTO_CSI_TEST
    ESP_LOGW(TAG, "automatic CSI continuous-frame test starting");
    ret = run_csi_continuous_test(uvc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "automatic CSI continuous-frame test failed: %s", esp_err_to_name(ret));
        return;
    }
#endif

    ret = init_uvc(uvc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UVC init failed: %s", esp_err_to_name(ret));
        return;
    }
}
