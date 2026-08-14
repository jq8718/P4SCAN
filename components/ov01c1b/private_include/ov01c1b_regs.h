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

#ifdef __cplusplus
}
#endif
