#ifndef _AW36413_DRIVER_H_
#define _AW36413_DRIVER_H_

/* define registers */
#define AW36413_ADDR 0x6b

#define AW36413_REG_ENABLE           (0x01)
#define AW36413_MASK_ENABLE_LED1     (0x01)
#define AW36413_MASK_ENABLE_LED2     (0x02)
#define AW36413_DISABLE              (0x00)
#define AW36413_ENABLE_LED1          (0x01)
#define AW36413_ENABLE_LED1_TORCH    (0x09)
#define AW36413_ENABLE_LED1_FLASH    (0x0D)
#define AW36413_ENABLE_LED2          (0x02)
#define AW36413_ENABLE_LED2_TORCH    (0x0A)
#define AW36413_ENABLE_LED2_FLASH    (0x0E)

#define AW36413_REG_TORCH_LEVEL_LED1 (0x05)
#define AW36413_REG_FLASH_LEVEL_LED1 (0x03)
#define AW36413_REG_TORCH_LEVEL_LED2 (0x06)
#define AW36413_REG_FLASH_LEVEL_LED2 (0x04)

#define AW36413_REG_TIMING_CONF      (0x08)
#define AW36413_TORCH_RAMP_TIME      (0x10)
#define AW36413_FLASH_TIMEOUT        (0x0F)

/* define channel, level */
#define AW36413_CHANNEL_NUM          2
#define AW36413_CHANNEL_CH1          0
#define AW36413_CHANNEL_CH2          1
#define AW36413_LEVEL_NUM            26
#define AW36413_LEVEL_TORCH          26 //always in torch mode

extern struct flash_led_function_s aw36413_function;

#endif
