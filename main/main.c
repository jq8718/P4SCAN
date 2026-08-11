/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <assert.h>
#include <fcntl.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>

#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "example_video_common.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"

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

typedef struct {
    int cap_fd;
    uint32_t format;
    uint8_t *cap_buffer[BUFFER_COUNT];

    int m2m_fd;
    uint8_t *m2m_cap_buffer;
    uint8_t *pattern_buffer;
    size_t pattern_len;

    uvc_fb_t fb;
} p4_uvc_t;

static const char *TAG = "p4scan_uvc";

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
    assert(fd >= 0);

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
    assert(fd >= 0);

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

static esp_err_t video_start_cb(uvc_format_t uvc_format, int width, int height, int rate, void *cb_ctx)
{
    (void)uvc_format;
    (void)rate;

    struct v4l2_buffer buf;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    p4_uvc_t *uvc = (p4_uvc_t *)cb_ctx;
    uint32_t capture_fmt = 0;

    ESP_LOGI(TAG, "UVC stream start: %dx%d@%dfps", width, height, rate);

#if CONFIG_P4SCAN_UVC_TEST_PATTERN
    capture_fmt = V4L2_PIX_FMT_RGB565;
    ESP_RETURN_ON_ERROR(fill_rgb565_test_pattern(uvc, width, height), TAG, "failed to generate test pattern");
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
            return ESP_ERR_NOT_SUPPORTED;
        }
    } else {
        capture_fmt = V4L2_PIX_FMT_YUV420;
    }

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_S_FMT, &format));
    log_v4l2_pix_format("capture format selected", &format);

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

        ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QBUF, &buf));
    }
#endif

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
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

    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &buf));

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_STREAMON, &type));
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_STREAMON, &type));
#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_STREAMON, &type));
#endif

    return ESP_OK;
}

static void video_stop_cb(void *cb_ctx)
{
    p4_uvc_t *uvc = (p4_uvc_t *)cb_ctx;

    ESP_LOGI(TAG, "UVC stream stop");

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    ioctl(uvc->cap_fd, VIDIOC_STREAMOFF, &type);
#endif

    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(uvc->m2m_fd, VIDIOC_STREAMOFF, &type);
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(uvc->m2m_fd, VIDIOC_STREAMOFF, &type);
}

static uvc_fb_t *video_fb_get_cb(void *cb_ctx)
{
    p4_uvc_t *uvc = (p4_uvc_t *)cb_ctx;
    static uint32_t frame_count;
    struct v4l2_format format;
    struct v4l2_buffer cap_buf;
    struct v4l2_buffer m2m_out_buf;
    struct v4l2_buffer m2m_cap_buf;

#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    memset(&cap_buf, 0, sizeof(cap_buf));
    cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_DQBUF, &cap_buf));
#endif

    memset(&m2m_out_buf, 0, sizeof(m2m_out_buf));
    m2m_out_buf.index = 0;
    m2m_out_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    m2m_out_buf.memory = V4L2_MEMORY_USERPTR;
#if CONFIG_P4SCAN_UVC_TEST_PATTERN
    m2m_out_buf.m.userptr = (unsigned long)uvc->pattern_buffer;
    m2m_out_buf.length = uvc->pattern_len;
#else
    m2m_out_buf.m.userptr = (unsigned long)uvc->cap_buffer[cap_buf.index];
    m2m_out_buf.length = cap_buf.bytesused;
#endif
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &m2m_out_buf));

    memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
    m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_DQBUF, &m2m_cap_buf));

#if !CONFIG_P4SCAN_UVC_TEST_PATTERN
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QBUF, &cap_buf));
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

    frame_count++;
    if ((frame_count % 1) == 0) {
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

    memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
    m2m_cap_buf.index = 0;
    m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &m2m_cap_buf));
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

    ESP_LOGI(TAG, "UVC format: %s", uvc->format == V4L2_PIX_FMT_JPEG ? "MJPEG" : "H.264");
    ESP_LOGI(TAG, "UVC frame: %d * %d @%dfps", UVC_FRAMES_INFO[index][0].width,
             UVC_FRAMES_INFO[index][0].height, UVC_FRAMES_INFO[index][0].rate);

    ESP_ERROR_CHECK(uvc_device_config(index, &config));
    ESP_ERROR_CHECK(uvc_device_init());

    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 SC2336 USB camera starting");

    p4_uvc_t *uvc = calloc(1, sizeof(p4_uvc_t));
    assert(uvc);

    ESP_ERROR_CHECK(example_video_init());
#if CONFIG_P4SCAN_UVC_TEST_PATTERN
    ESP_LOGW(TAG, "UVC test pattern mode enabled; camera frames are not used");
#endif
    ESP_ERROR_CHECK(init_capture_video(uvc));
    ESP_ERROR_CHECK(init_codec_video(uvc));
    ESP_ERROR_CHECK(init_uvc(uvc));
}
