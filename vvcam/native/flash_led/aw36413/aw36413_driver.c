#include <linux/module.h>
#include <linux/uaccess.h>
#include "flash_led_driver.h"
#include "flash_led_ioctl.h"
#include "aw36413_driver.h"

/* flashlight enable function */
static volatile uint32_t aw_reg1_val = 0x0;
static int aw36413_enable(void *ctx, int channel)
{
    int ret;
    struct flash_led_dev *dev = ctx;
    unsigned char reg = 0;
    uint32_t mask = 0;

    if (channel == 1) {
        mask = AW36413_ENABLE_LED1;
    } else if (channel == 2) {
        mask = AW36413_ENABLE_LED2;
    } else if (channel == 3) {
        mask = AW36413_ENABLE_LED1 | AW36413_ENABLE_LED2;
    }

    aw_reg1_val |= mask;

    reg = AW36413_REG_ENABLE;

    return flash_led_i2c_write(dev, reg, aw_reg1_val);
}

/* flashlight disable function */
static int aw36413_disable(void *ctx, int channel)
{
    int ret;
    struct flash_led_dev *dev = ctx;
    unsigned char reg = 0;
    uint32_t mask = 0;

    if (channel == 1) {
        mask = AW36413_ENABLE_LED1;
    } else if (channel == 2) {
        mask = AW36413_ENABLE_LED2;
    } else if (channel == 3) {
        mask = AW36413_ENABLE_LED1 | AW36413_ENABLE_LED2;
    }

    aw_reg1_val &= ~mask;

    reg = AW36413_REG_ENABLE;

    return flash_led_i2c_write(dev, reg, aw_reg1_val);
}

/* set flashlight level */
static int aw36413_set_ch1_flash_brightness(void *ctx, int level)
{
    int ret;
    uint32_t val = 0;
    unsigned char reg = 0;
    struct flash_led_dev *dev = ctx;

    /* set flash brightness level */
    reg = AW36413_REG_FLASH_LEVEL_LED1;
    val = level & 0xff;
    ret = flash_led_i2c_write(dev, reg, val);

    return ret;
}

static int aw36413_set_ch1_torch_brightness(void *ctx, int level)
{
    int ret;
    uint32_t val = 0;
    unsigned char reg = 0;
    struct flash_led_dev *dev = ctx;

    /* set torch brightness level */
    reg = AW36413_REG_TORCH_LEVEL_LED1;
    val = level & 0xff;
    ret = flash_led_i2c_write(dev, reg, val);

    return ret;
}

static int aw36413_set_ch2_flash_brightness(void *ctx, int level)
{
    int ret;
    uint32_t val = 0;
    unsigned char reg = 0;
    struct flash_led_dev *dev = ctx;

    reg = AW36413_REG_FLASH_LEVEL_LED2;
    val = level & 0xff;
    ret = flash_led_i2c_write(dev, reg, val);

    return ret;
}

static int aw36413_set_ch2_torch_brightness(void *ctx, int level)
{
    int ret;
    uint32_t val = 0;
    unsigned char reg = 0;
    struct flash_led_dev *dev = ctx;

    reg = AW36413_REG_TORCH_LEVEL_LED2;
    val = level & 0xff;
    ret = flash_led_i2c_write(dev, reg, val);

    return ret;
}

static int aw36413_set_flash_brightness(void *ctx, int channel, uint32_t level)
{
    int ret = 0;
    struct flash_led_dev *dev = ctx;

    if (channel == 1) {
        ret = aw36413_set_ch1_flash_brightness(dev, level);
    } else if (channel == 2) {
        ret = aw36413_set_ch2_flash_brightness(dev, level);
    } else if (channel == 3) {
        ret = aw36413_set_ch1_flash_brightness(dev, level);
        if (ret != 0) {
            return -1;
        }
        ret = aw36413_set_ch2_flash_brightness(dev, level);
    } else {
        return -1;
    }

    return ret;
}

static int aw36413_set_torch_brightness(void *ctx, int channel, uint32_t level)
{
    int ret = 0;
    struct flash_led_dev *dev = ctx;

    if (channel == 1) {
        ret = aw36413_set_ch1_torch_brightness(dev, level);
    } else if (channel == 2) {
        ret = aw36413_set_ch2_torch_brightness(dev, level);
    } else if (channel == 3) {
        ret = aw36413_set_ch1_torch_brightness(dev, level);
        if (ret != 0) {
            return -1;
        }
        ret = aw36413_set_ch2_torch_brightness(dev, level);
    } else {
        return -1;
    }

    return ret;
}

#define AW36413_STANDBY_MODE      0x0
#define AW36413_EXT_TORCH_MODE    (0x1 << 4)
#define AW36413_EXT_FLASH_MODE    (0x1 << 5)
#define AW36413_INT_TORCH_MODE    (0x2 << 2)
#define AW36413_INT_FLASH_MODE    (0x3 << 2)
#define AW36413_IR_STANDBY_MODE   (0x1 << 2)
#define AW36413_IR_ENABLE_MODE    ((0x1 << 2) | (1 << 5))

static int aw36413_set_mode(void *ctx, flash_led_mode_t mode)
{
    int ret = 0;
    unsigned char reg = 0;
    struct flash_led_dev *dev = ctx;

    reg = AW36413_REG_ENABLE;
    aw_reg1_val &= 0x3;

    switch(mode) {
        case FLASH_LED_STANDBY:
            aw_reg1_val |= AW36413_STANDBY_MODE;
            ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
            break;
        case FLASH_LED_EXT_TORCH:
            aw_reg1_val |= AW36413_EXT_TORCH_MODE;
            ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
            break;
        case FLASH_LED_EXT_FLASH:
            aw_reg1_val |= AW36413_EXT_FLASH_MODE;
            ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
            break;
        case FLASH_LED_INT_TORCH:
            aw_reg1_val |= AW36413_INT_TORCH_MODE;
            ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
            break;
        case FLASH_LED_INT_FLASH:
            aw_reg1_val |= AW36413_INT_FLASH_MODE;
            ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
            break;
        case FLASH_LED_IR_STANDBY:
            aw_reg1_val |= AW36413_IR_STANDBY_MODE;
            ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
            break;
        case FLASH_LED_IR_ENABLE:
            aw_reg1_val |= AW36413_IR_ENABLE_MODE;
            ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
            break;
        default:
            return -1;
    }

    return ret;
}

/* flashlight init */
static int aw36413_init(void *ctx);

/* flashlight uninit */
static int aw36413_uninit(void *ctx)
{
    struct flash_led_dev *dev = ctx;
    aw36413_disable(dev, 3);

    return 0;
}

struct flash_led_function_s aw36413_function =
{
	.flash_led_name         = "aw36413",
	.reserve_id             = 0x36,
	.init                   = aw36413_init,
	.uninit                 = aw36413_uninit,
	.enable_channel         = aw36413_enable,
	.disable_channel        = aw36413_disable,
	.set_mode               = aw36413_set_mode,
	.set_flash_brightness   = aw36413_set_flash_brightness,
	.set_torch_brightness   = aw36413_set_torch_brightness,
};

static int aw36413_init(void *ctx)
{
    struct flash_led_dev *dev = ctx;
    int ret;
    uint32_t val = 0;
    unsigned char reg = 0;

    usleep_range(2000, 2500);
    dev->flash_led_func = &aw36413_function;

    dev->flash_led_sccb_cfg.addr_byte  = 1;
    dev->flash_led_sccb_cfg.data_byte  = 1;
    dev->flash_led_sccb_cfg.slave_addr = AW36413_ADDR;

    /* clear enable register */
    reg = AW36413_REG_ENABLE;
    aw_reg1_val = AW36413_DISABLE;
    ret = flash_led_i2c_write(dev, reg, aw_reg1_val);
    if (ret != 0) {
        return -1;
    }

    /* set torch current ramp time and flash timeout */
    reg = AW36413_REG_TIMING_CONF;
    val = AW36413_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT;
    ret = flash_led_i2c_write(dev, reg, val);
    if (ret != 0) {
        return -1;
    }

    ret = aw36413_set_mode(dev, FLASH_LED_IR_ENABLE);
    return ret;
}

