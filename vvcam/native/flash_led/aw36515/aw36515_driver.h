#ifndef _AW36515_DRIVER_H_
#define _AW36515_DRIVER_H_

/* define registers */
#define AW36515_ADDR 0x63

#define AW36515_REG_ENABLE           (0x01)
#define AW36515_MASK_ENABLE_LED1     (0x01)
#define AW36515_MASK_ENABLE_LED2     (0x02)
#define AW36515_DISABLE              (0x00)
#define AW36515_ENABLE_LED1          (0x01)
#define AW36515_ENABLE_LED1_TORCH    (0x09)
#define AW36515_ENABLE_LED1_FLASH    (0x0D)
#define AW36515_ENABLE_LED2          (0x02)
#define AW36515_ENABLE_LED2_TORCH    (0x0A)
#define AW36515_ENABLE_LED2_FLASH    (0x0E)

#define AW36515_REG_TORCH_LEVEL_LED1 (0x05)
#define AW36515_REG_FLASH_LEVEL_LED1 (0x03)
#define AW36515_REG_TORCH_LEVEL_LED2 (0x06)
#define AW36515_REG_FLASH_LEVEL_LED2 (0x04)

#define AW36515_REG_TIMING_CONF      (0x08)
#define AW36515_TORCH_RAMP_TIME      (0x10)
#define AW36515_FLASH_TIMEOUT        (0x0F)

/* define channel, level */
#define AW36515_CHANNEL_NUM          2
#define AW36515_CHANNEL_CH1          0
#define AW36515_CHANNEL_CH2          1
#define AW36515_LEVEL_NUM            26
#define AW36515_LEVEL_TORCH          26 //always in torch mode

extern struct flash_led_function_s aw36515_function;

#endif
