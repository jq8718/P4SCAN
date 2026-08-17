#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define OV01C1B_REG_END      0xffff
#define OV01C1B_REG_DELAY    0xfffe
#define OV01C1B_REG_PAGE_SEL 0xfd
#define OV01C1B_REG_SOFT_RESET 0x20
#define OV01C1B_REG_MIPI_OUTPUT 0xa0
#define OV01C1B_REG_OUTPUT_GATE 0x01
#define OV01C1B_SOFT_STANDBY   0x0b
#define OV01C1B_SOFT_STREAMING 0x1f
#define OV01C1B_REG_MIPI_LANE  0xc2
#define OV01C1B_REG_MIPI_CTRL  0xc4

#define OV01C1B_REG_EXP_H      0x03
#define OV01C1B_REG_EXP_L      0x04
#define OV01C1B_REG_VBLANK_H   0x05
#define OV01C1B_REG_VBLANK_L   0x06
#define OV01C1B_REG_CTRL       0x01
#define OV01C1B_REG_DIG_GAIN   0x20
#define OV01C1B_REG_DIG_GAIN2  0x21
#define OV01C1B_REG_DIG_GAIN_M 0x22

#define OV01C1B_EXP_MIN        1
#define OV01C1B_EXP_MAX        2480
#define OV01C1B_EXP_DEFAULT    0x0206
#define OV01C1B_GAIN_MIN       0x040
#define OV01C1B_GAIN_MAX       0x7ff
#define OV01C1B_GAIN_DEFAULT   0x040

#ifdef __cplusplus
}
#endif
