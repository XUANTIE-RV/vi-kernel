#ifndef _FLASH_LED_DRIVER_H_
#define _FLASH_LED_DRIVER_H_

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include "flash_led_ioctl.h"

#define UNDEFINED_IN_DTS	0xFF
#define FLASH_LED_MAX_REGULATORS     10

struct flash_led_driver_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct mutex vvmutex;
	void *private;
};

struct flash_led_regulators {
    struct regulator *supply[FLASH_LED_MAX_REGULATORS];
    const char *name[FLASH_LED_MAX_REGULATORS];
    int num;
};

struct flash_led_function_s {
	uint8_t flash_led_name[100];
	uint32_t reserve_id;
	uint32_t time_out_ms;
	int (*init)             (void *cxt);
	int (*uninit)           (void *cxt);
	int (*enable_channel)   (void *cxt, int channel);
	int (*disable_channel)  (void *cxt, int channel);
	int (*set_mode)         (void *cxt, flash_led_mode_t mode);
	int (*set_flash_brightness) (void *cxt, int channel, uint32_t val);
	int (*set_torch_brightness) (void *cxt, int channel, uint32_t val);
};

struct flash_led_dev {
    struct flash_led_function_s *flash_led_func;
    uint8_t i2c_bus;
    void *i2c_client;
    struct flash_led_sccb_cfg_s flash_led_sccb_cfg;
    struct flash_led_regulators regulators;
};

struct flash_led_ctrl {
    struct work_struct flash_led_work;
    const char *flash_led_name;
    long reg_size;
    void __iomem *base;
    int32_t device_idx;
    int touch_pin;
    int floodlight_en_pin;
    int projection_en_pin;
    int enable;
    struct iio_channel *floodlight_adc;
    struct iio_channel *projection_adc;
    struct flash_led_dev floodlight;
    struct flash_led_dev projection;
    dma_addr_t frame_mark_info_addr;
    volatile frame_mark_t *frame_mark;
};

int flash_led_init(struct flash_led_ctrl *dev);
int flash_led_deinit(struct flash_led_ctrl *dev);
long flash_led_priv_ioctl(struct flash_led_ctrl *dev, unsigned int cmd, void __user *args);
int32_t flash_led_i2c_write(struct flash_led_dev *dev, uint32_t address, uint32_t data);
int32_t flash_led_i2c_read(struct flash_led_dev *dev, uint32_t address, uint32_t *pdata);

#endif
