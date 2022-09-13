#ifndef _FLASH_LED_IOCTL_H_
#define _FLASH_LED_IOCTL_H_

#include <linux/ioctl.h>

#define FLASH_LED_IOCTL_MAGIC 'f'

#define FLASH_LED_IOCTL_ENABLE_CH           _IOWR(FLASH_LED_IOCTL_MAGIC, 1, int)
#define FLASH_LED_IOCTL_DISABLE_CH          _IOWR(FLASH_LED_IOCTL_MAGIC, 2, int)
#define FLASH_LED_IOCTL_SET_MODE            _IOWR(FLASH_LED_IOCTL_MAGIC, 3, int)
#define FLASH_LED_IOCTL_SET_FLASH_BRIGHT    _IOWR(FLASH_LED_IOCTL_MAGIC, 4, int)
#define FLASH_LED_IOCTL_SET_TORCH_BRIGHT    _IOWR(FLASH_LED_IOCTL_MAGIC, 5, int)
#define FLASH_LED_IOCTL_WRITE_REG           _IOWR(FLASH_LED_IOCTL_MAGIC, 6, int)
#define FLASH_LED_IOCTL_READ_REG            _IOWR(FLASH_LED_IOCTL_MAGIC, 7, int)
#define FLASH_LED_IOCTL_ENABLE              _IOWR(FLASH_LED_IOCTL_MAGIC, 8, int)
#define FLASH_LED_IOCTL_DISABLE             _IOWR(FLASH_LED_IOCTL_MAGIC, 9, int)
#define FLASH_LED_IOCTL_GET_FRAME_MASK_INFO_ADDR             _IOWR(FLASH_LED_IOCTL_MAGIC, 10, int)

struct flash_led_sccb_cfg_s {
	unsigned char slave_addr;
	unsigned char addr_byte;
	unsigned char data_byte;
};

typedef enum {
    FLASH_LED_STANDBY,
    FLASH_LED_EXT_TORCH,
    FLASH_LED_EXT_FLASH,
    FLASH_LED_INT_TORCH,
    FLASH_LED_INT_FLASH,
    FLASH_LED_IR_STANDBY,
    FLASH_LED_IR_ENABLE,
} flash_led_mode_t;

typedef enum {
    FLOODLIGHT,
    PROJECTION,
} flash_led_type_t;

typedef enum {
    FLOODLIGHT_EN = 1,
    PROJECTION_EN = 2,
    FLOODLIGHT_PROJECTION_EN = 3,
} flash_led_enable_mask_t;

typedef struct {
    flash_led_type_t type;
    unsigned int offset;
    unsigned int value;
} flash_led_reg_t;

typedef struct {
    flash_led_type_t type;
    int channel;
    unsigned int value;
} flash_bright_cfg_t;

typedef struct {
    flash_led_type_t type;
    int channel;
} flash_ch_t;

typedef struct {
    flash_led_type_t type;
    flash_led_mode_t mode;
} flash_mode_t;

typedef struct {
    volatile uint64_t frame_irq_cnt;
    volatile uint64_t frame_time_us;
    int floodlight_temperature;
    int projection_temperature;
} frame_mark_t;

#endif
