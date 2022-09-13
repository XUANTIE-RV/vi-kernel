
#include "flash_led_driver.h"
#include "flash_led_ioctl.h"

extern uint64_t touch_pin_int_num(struct flash_led_ctrl *pflash_led_dev);
extern uint64_t touch_pin_high_time_us(void);
extern uint64_t touch_pin_preiod_time_us(void);
extern uint64_t touch_pin_falling_int(void);
extern uint64_t touch_pin_rising_int(void);

static ssize_t flash_led_falling_int_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
   	char buffer[100];
    printk("<0>""flash_led falling interrupt num: %d\n", touch_pin_falling_int());
	snprintf(buffer, sizeof(buffer), "falling num :%llu\n", touch_pin_falling_int());

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t flash_led_rising_int_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
   	char buffer[100];

    printk("<0>""flash_led rising interrupt num: %d\n", touch_pin_rising_int());
	snprintf(buffer, sizeof(buffer), "rising interupt num :%llu\n", touch_pin_rising_int());

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t flash_led_info_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
    struct flash_led_ctrl *pflash_led_dev;
    struct flash_led_dev *floodlight;
    struct flash_led_dev *projection;
   	char buffer[200];
    char name1[30] = {0};
    char name2[30] = {0};

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;

    floodlight = &pflash_led_dev->floodlight;
    projection = &pflash_led_dev->projection;

    if (floodlight->flash_led_func != NULL) {
        strcpy(name1, floodlight->flash_led_func->flash_led_name);
    }

    if (projection->flash_led_func != NULL) {
        strcpy(name2, projection->flash_led_func->flash_led_name);
    }

	snprintf(buffer, sizeof(buffer), "f_led_%s i2c is :%d, p_led_%s i2c is %d\n",
                                      name1,
                                      floodlight->i2c_bus,
                                      name2,
                                      projection->i2c_bus);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t flash_led_int_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
    struct flash_led_ctrl *pflash_led_dev;
    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;

   	char buffer[100];

    printk("<0>""flash_led interrupt num: %d\n", touch_pin_int_num(pflash_led_dev));
	snprintf(buffer, sizeof(buffer), "interupt num :%llu\n", touch_pin_int_num(pflash_led_dev));

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t flash_led_pin_high_time_us_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
   	char buffer[100];
	snprintf(buffer, sizeof(buffer), "h time us is :%llu\n", touch_pin_high_time_us());

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t flash_led_pin_period_us_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
   	char buffer[100];
	snprintf(buffer, sizeof(buffer), "preiod us is :%llu\n", touch_pin_preiod_time_us());

	return strlcpy(buf, buffer, PAGE_SIZE);
}

int32_t flash_led_i2c_write(struct flash_led_dev *dev, uint32_t address, uint32_t data);
int32_t flash_led_i2c_read(struct flash_led_dev *dev, uint32_t address, uint32_t *pdata);

static ssize_t floodlight_led_reg_read_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
    struct flash_led_dev *floodlight;
    unsigned long addr;
    uint32_t val;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;
    floodlight = &pflash_led_dev->floodlight;

    if(floodlight->i2c_bus == UNDEFINED_IN_DTS) {
        return count;
    }

    ret = kstrtoul(buf, 16, &addr);
    if (ret < 0) {
        return ret;
    }

    ret = flash_led_i2c_read(floodlight, addr, &val);
    if (ret < 0) {
        return ret;
    }
    printk("read reg[0x%x]: 0x%x\n", addr, val);

	return count;
}

static ssize_t floodlight_led_reg_write_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
    struct flash_led_dev *floodlight;
   	char buffer[100];
    unsigned long addr;
    uint32_t val;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;
    floodlight = &pflash_led_dev->floodlight;

    if(floodlight->i2c_bus == UNDEFINED_IN_DTS) {
        return count;
    }

    ret = kstrtoul(buf, 16, &addr);
    if (ret < 0) {
        return ret;
    }

    val = addr & 0xff;
    addr = addr >> 8;

    ret = flash_led_i2c_write(floodlight, addr, val);
    if (ret < 0) {
        return ret;
    }

	printk("write reg[0x%x] 0x%x\n", addr, val);

	return count;
}

static ssize_t projection_led_reg_read_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
    struct flash_led_dev *projection;
    unsigned long addr;
    uint32_t val;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;
    projection = &pflash_led_dev->projection;

    if(projection->i2c_bus == UNDEFINED_IN_DTS) {
        return count;
    }

    ret = kstrtoul(buf, 16, &addr);
    if (ret < 0) {
        return ret;
    }

    ret = flash_led_i2c_read(projection, addr, &val);
    if (ret < 0) {
        return ret;
    }
    printk("read reg[0x%x]: 0x%x\n", addr, val);

	return count;
}

static ssize_t projection_led_reg_write_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
    struct flash_led_dev *projection;
   	char buffer[100];
    unsigned long addr;
    uint32_t val;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;
    projection = &pflash_led_dev->projection;

    if(projection->i2c_bus == UNDEFINED_IN_DTS) {
        return count;
    }

    ret = kstrtoul(buf, 16, &addr);
    if (ret < 0) {
        return ret;
    }

    val = addr & 0xff;
    addr = addr >> 8;

    ret = flash_led_i2c_write(projection, addr, val);
    if (ret < 0) {
        return ret;
    }

	printk("write reg[0x%x] 0x%x\n", addr, val);

	return count;
}

static ssize_t flash_led_enable_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
   	char buffer[100];
    unsigned long en;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl *)pdriver_dev->private;

    ret = kstrtoul(buf, 10, &en);
    if (ret < 0) {
        return ret;
    }

    pflash_led_dev->enable = en;

	return count;
}

static ssize_t flash_led_enable_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
   	char buffer[100];
    unsigned long en;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl *)pdriver_dev->private;
    snprintf(buffer, sizeof(buffer), "flash_led enable mask is 0x%x\n", pflash_led_dev->enable);

    return strlcpy(buf, buffer, PAGE_SIZE);
}

uint64_t get_us_time(void);

static ssize_t projection_enable_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
    struct flash_led_dev *projection;
   	char buffer[100];
    unsigned long ch;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;
    projection = &pflash_led_dev->projection;

    ret = kstrtoul(buf, 10, &ch);
    if (ret < 0) {
        return ret;
    }

    uint64_t us1 = get_us_time();

    if (projection->flash_led_func != NULL) {
        projection->flash_led_func->enable_channel(projection, ch);
    }
    uint64_t us2 = get_us_time();

    printk("used time %llu us\n", us2 - us1);

	return count;
}

static ssize_t floodlight_enable_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;
    struct flash_led_dev *floodlight;
    unsigned long ch;

    pdriver_dev = platform_get_drvdata(pdev);
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;
    floodlight = &pflash_led_dev->floodlight;

    ret = kstrtoul(buf, 10, &ch);
    if (ret < 0) {
        return ret;
    }

    uint64_t us1 = get_us_time();
    if (floodlight->flash_led_func != NULL) {
        floodlight->flash_led_func->enable_channel(floodlight, ch);
    }
    uint64_t us2 = get_us_time();

    printk("used time %llu us\n", us2 - us1);
	return count;
}

static DEVICE_ATTR_RO(flash_led_info);
static DEVICE_ATTR_RO(flash_led_int);
static DEVICE_ATTR_RO(flash_led_pin_high_time_us);
static DEVICE_ATTR_RO(flash_led_pin_period_us);
static DEVICE_ATTR_RO(flash_led_falling_int);
static DEVICE_ATTR_RO(flash_led_rising_int);
static DEVICE_ATTR_WO(projection_led_reg_write);
static DEVICE_ATTR_WO(projection_led_reg_read);
static DEVICE_ATTR_WO(floodlight_led_reg_write);
static DEVICE_ATTR_WO(floodlight_led_reg_read);
static DEVICE_ATTR_WO(projection_enable);
static DEVICE_ATTR_WO(floodlight_enable);
static DEVICE_ATTR_RW(flash_led_enable);

int flash_led_create_capabilities_sysfs(struct platform_device *pdev)
{
	device_create_file(&pdev->dev, &dev_attr_flash_led_info);
	device_create_file(&pdev->dev, &dev_attr_flash_led_int);
	device_create_file(&pdev->dev, &dev_attr_flash_led_pin_high_time_us);
	device_create_file(&pdev->dev, &dev_attr_flash_led_pin_period_us);
	device_create_file(&pdev->dev, &dev_attr_flash_led_falling_int);
	device_create_file(&pdev->dev, &dev_attr_flash_led_rising_int);
	device_create_file(&pdev->dev, &dev_attr_projection_led_reg_write);
	device_create_file(&pdev->dev, &dev_attr_projection_led_reg_read);
	device_create_file(&pdev->dev, &dev_attr_floodlight_led_reg_write);
	device_create_file(&pdev->dev, &dev_attr_floodlight_led_reg_read);
	device_create_file(&pdev->dev, &dev_attr_projection_enable);
	device_create_file(&pdev->dev, &dev_attr_floodlight_enable);
	device_create_file(&pdev->dev, &dev_attr_flash_led_enable);

	return 0;
}

int flash_led_remove_capabilities_sysfs(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_flash_led_info);
	device_remove_file(&pdev->dev, &dev_attr_flash_led_int);
	device_remove_file(&pdev->dev, &dev_attr_flash_led_pin_high_time_us);
	device_remove_file(&pdev->dev, &dev_attr_flash_led_pin_period_us);
	device_remove_file(&pdev->dev, &dev_attr_flash_led_falling_int);
	device_remove_file(&pdev->dev, &dev_attr_flash_led_rising_int);
	device_remove_file(&pdev->dev, &dev_attr_projection_led_reg_write);
	device_remove_file(&pdev->dev, &dev_attr_projection_led_reg_read);
	device_remove_file(&pdev->dev, &dev_attr_floodlight_led_reg_write);
	device_remove_file(&pdev->dev, &dev_attr_floodlight_led_reg_read);
	device_remove_file(&pdev->dev, &dev_attr_projection_enable);
	device_remove_file(&pdev->dev, &dev_attr_floodlight_enable);
	device_remove_file(&pdev->dev, &dev_attr_flash_led_enable);

	return 0;
}

