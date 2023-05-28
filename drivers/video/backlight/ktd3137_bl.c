#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>
#include <linux/pwm.h>
#include <linux/leds.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/bitops.h>
#include <linux/mfd/ktd3137.h>
#include <linux/device.h>
#include <linux/platform_device.h>

struct ktd3137_chip *bkl_chip;

static int ktd3137_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int ktd3137_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	*val = ret;
	return ret;
}

static int ktd3137_masked_write(struct i2c_client *client,
					int reg, u8 mask, u8 val)
{
	int rc;
	u8 temp = 0;

	rc = ktd3137_read_reg(client, reg, &temp);
	if (rc < 0) {
		dev_err(&client->dev, "failed to read reg\n");
	} else {
		temp &= ~mask;
		temp |= val & mask;
		rc = ktd3137_write_reg(client, reg, temp);
		if (rc < 0)
			dev_err(&client->dev, "failed to write masked data\n");
	}

	ktd3137_read_reg(client, reg, &temp);
	return rc;
}

static int ktd3137_find_bit(int x)
{
	int i = 0;

	while ((x = x >> 1))
		i++;

	return i+1;
}

static int ktd3137_parse_dt(struct device *dev, struct ktd3137_chip *chip)
{
	struct device_node *np = dev->of_node;
	struct ktd3137_bl_pdata *pdata = chip->pdata;
	int rc = 0;
	u32 bl_channel, temp;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->hwen_gpio = of_get_named_gpio(np, "ktd,hwen-gpio", 0);
	pdata->pwm_mode = of_property_read_bool(np, "ktd,pwm-mode");
	pdata->using_lsb = of_property_read_bool(np, "ktd,using-lsb");

	if (pdata->using_lsb) {
		pdata->default_brightness = 0x7ff;
		pdata->max_brightness = 2047;
	} else {
		pdata->default_brightness = 0xff;
		pdata->max_brightness = 255;
	}

	rc = of_property_read_u32(np, "ktd,pwm-frequency", &temp);
	if (!rc) {
		pdata->pwm_period = temp;
	} else {
		pr_err("Invalid pwm-frequency!\n");
	}

	rc = of_property_read_u32(np, "ktd,bl-fscal-led", &temp);
	if (!rc) {
		pdata->full_scale_led = temp;
	} else {
		pr_err("Invalid backlight full-scale led current!\n");
	}

	rc = of_property_read_u32(np, "ktd,turn-on-ramp", &temp);
	if (!rc) {
		pdata->ramp_on_time = temp;
	} else {
		pr_err("Invalid ramp timing ,,turnon!\n");
	}

	rc = of_property_read_u32(np, "ktd,turn-off-ramp", &temp);
	if (!rc) {
		pdata->ramp_off_time = temp;
	} else {
		pr_err("Invalid ramp timing ,,turnoff!\n");
	}

	rc = of_property_read_u32(np, "ktd,pwm-trans-dim", &temp);
	if (!rc) {
		pdata->pwm_trans_dim = temp;
	} else {
		pr_err("Invalid pwm-tarns-dim value!\n");
	}

	rc = of_property_read_u32(np, "ktd,i2c-trans-dim", &temp);
	if (!rc) {
		pdata->i2c_trans_dim = temp;
	} else {
		pr_err("Invalid i2c-trans-dim value !\n");
	}

	rc = of_property_read_u32(np, "ktd,bl-channel", &bl_channel);
	if (!rc) {
		pr_err("Invalid channel setup\n");
	} else {
		pdata->channel = bl_channel;
	}

	rc = of_property_read_u32(np, "ktd,ovp-level", &temp);
	if (!rc) {
		pdata->ovp_level = temp;
	} else
		pr_err("Invalid OVP level!\n");

	rc = of_property_read_u32(np, "ktd,switching-frequency", &temp);
	if (!rc) {
		pdata->frequency = temp;
	} else {
		pr_err("Invalid Frequency value!\n");
	}

	rc = of_property_read_u32(np, "ktd,inductor-current", &temp);
	if (!rc) {
		pdata->induct_current = temp;
	} else
		pr_err("invalid induct_current limit\n");

	rc = of_property_read_u32(np, "ktd,flash-timeout", &temp);
	if (!rc) {
		pdata->flash_timeout = temp;
	} else {
		pr_err("invalid flash-time value!\n");
	}

	rc = of_property_read_u32(np, "ktd,linear_ramp", &temp);
	if (!rc) {
		pdata->linear_ramp = temp;
	} else {
		pr_err("invalid linear_ramp value!\n");
	}

	rc = of_property_read_u32(np, "ktd,linear_backlight", &temp);
	if (!rc) {
		pdata->linear_backlight = temp;
	} else {
		pr_err("invalid linear_backlight value!\n");
	}

	rc = of_property_read_u32(np, "ktd,flash-current", &temp);
	if (!rc) {
		pdata->flash_current = temp;
	} else {
		pr_err("invalid flash current value!\n");
	}

	dev->platform_data = pdata;
	return 0;
}

static int ktd3137_bl_enable_channel(struct ktd3137_chip *chip)
{
	int ret;
	struct ktd3137_bl_pdata *pdata = chip->pdata;

	switch (pdata->channel) {
	case 0:
	case 1:
		ret = ktd3137_write_reg(chip->client, REG_PWM, 0x18);
		break;
	case 2:
		ret = ktd3137_masked_write(chip->client, REG_PWM, 0x9B, 0x1B);
		break;
	case 3:
		ret = ktd3137_masked_write(chip->client, REG_PWM, 0x9F, 0x9F);
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

static void ktd3137_pwm_mode_enable(struct ktd3137_chip *chip, bool en)
{
	struct ktd3137_bl_pdata *pdata = chip->pdata;
	u8 value;

	if (en) {
		if (!pdata->pwm_mode)
			pdata->pwm_mode = en;
		ktd3137_masked_write(chip->client, REG_PWM, 0x80, 0x80);
	} else {
		if (pdata->pwm_mode)
			pdata->pwm_mode = en;
		ktd3137_masked_write(chip->client, REG_PWM, 0x9B, 0x1B);
	}

	ktd3137_read_reg(chip->client, REG_PWM, &value);
}

static void ktd3137_ramp_setting(struct ktd3137_chip *chip)
{
	struct ktd3137_bl_pdata *pdata = chip->pdata;
	unsigned int max_time = 16384;
	int temp = 0;

	if (pdata->ramp_on_time == 0) {//512us
		ktd3137_masked_write(chip->client, REG_RAMP_ON, 0xf0, 0x00);
	} else if (pdata->ramp_on_time > max_time) {
		ktd3137_masked_write(chip->client, REG_RAMP_ON, 0xf0, 0xf0);
	} else {
		temp = ktd3137_find_bit(pdata->ramp_on_time);
		ktd3137_masked_write(chip->client, REG_RAMP_ON, 0xf0, temp<<4);
	}

	if (pdata->ramp_off_time == 0) {//512us
		ktd3137_masked_write(chip->client, REG_RAMP_ON, 0x0f, 0x00);
	} else if (pdata->ramp_off_time > max_time) {
		ktd3137_masked_write(chip->client, REG_RAMP_ON, 0x0f, 0x0f);
	} else {
		temp = ktd3137_find_bit(pdata->ramp_off_time);
		ktd3137_masked_write(chip->client, REG_RAMP_ON, 0x0f, temp);
	}
}

static void ktd3137_transition_ramp(struct ktd3137_chip *chip)
{
	struct ktd3137_bl_pdata *pdata = chip->pdata;
	int reg_i2c, reg_pwm, temp;

	if (pdata->i2c_trans_dim >= 1024) {
		reg_i2c = 0xf;
	} else if (pdata->i2c_trans_dim < 128) {
		reg_i2c = 0x0;
	} else {
		temp = pdata->i2c_trans_dim/64;
		reg_i2c = temp-1;
	}

	if (pdata->pwm_trans_dim >= 256) {
		reg_pwm = 0x7;
	} else if (pdata->pwm_trans_dim < 4) {
		reg_pwm = 0x0;
	} else {
		temp = ktd3137_find_bit(pdata->pwm_trans_dim);
		reg_pwm = temp - 2;
	}

	ktd3137_masked_write(chip->client, REG_TRANS_RAMP, 0x70, reg_pwm);
	ktd3137_masked_write(chip->client, REG_TRANS_RAMP, 0x0f, reg_i2c);
}

static void ktd3137_flash_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct ktd3137_chip *chip = container_of(cdev, struct ktd3137_chip, cdev_flash);
	u8 reg;

	cancel_delayed_work_sync(&chip->work);
	if (!brightness) // flash off
		return;
	else if (brightness > 15)
		brightness = 0x0f;

	if (chip->pdata->flash_timeout < 100)
		reg = 0x00;
	else if (chip->pdata->flash_timeout > 1500)
		reg = 0x0f;
	else
		reg = (chip->pdata->flash_timeout/100);

	reg = (reg << 4) | brightness;
	ktd3137_write_reg(chip->client, REG_FLASH_SETTING, reg);
	ktd3137_masked_write(chip->client, REG_MODE, 0x02, 0x02);
	schedule_delayed_work(&chip->work, chip->pdata->flash_timeout);
}

static int ktd3137_flashled_init(struct i2c_client *client,
				struct ktd3137_chip *chip)
{
	int ret;

	chip->cdev_flash.name = "ktd3137_flash";
	chip->cdev_flash.max_brightness = 16;
	chip->cdev_flash.brightness_set = ktd3137_flash_brightness_set;

	ret = led_classdev_register((struct device *) &client->dev,
						&chip->cdev_flash);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register ktd3137_flash\n");
		return ret;
	}

	return 0;
}

static void ktd3137_backlight_init(struct ktd3137_chip *chip)
{
	struct ktd3137_bl_pdata *pdata = chip->pdata;
	u8 value;
	u8 update_value;
	update_value = (pdata->ovp_level == 32) ? 0x20 : 0x00;
	(pdata->induct_current == 2600) ? update_value |= 0x08 : update_value;
	(pdata->frequency == 1000) ? update_value |= 0x40 : update_value;
	(pdata->linear_ramp == 1) ? update_value |= 0x04 : update_value;
	(pdata->linear_backlight == 1) ? update_value |= 0x02 : update_value;
	ktd3137_write_reg(chip->client, REG_CONTROL, update_value);
	ktd3137_bl_enable_channel(chip);

	if (pdata->linear_backlight == 1) {
		ktd3137_masked_write(chip->client, REG_CONTROL, 0x02, 0x02);
	}

	if (pdata->pwm_mode) {
		ktd3137_pwm_mode_enable(chip, true);
	} else {
		ktd3137_pwm_mode_enable(chip, false);
	}

	ktd3137_ramp_setting(chip);
	ktd3137_transition_ramp(chip);
	ktd3137_read_reg(chip->client, REG_CONTROL, &value);
	ktd3137_masked_write(chip->client, REG_MODE, 0xf8,
					pdata->full_scale_led);
}

static int ktd3137_gpio_init(struct ktd3137_chip *chip)
{
	struct ktd3137_bl_pdata *pdata = chip->pdata;
	int ret;

	if (gpio_is_valid(pdata->hwen_gpio)) {
		ret = gpio_request(pdata->hwen_gpio, "ktd_hwen_gpio");
		if (ret < 0) {
			pr_err("failed to request gpio\n");
			return  -ENOMEM;
		}
		ret = gpio_direction_output(pdata->hwen_gpio, 0);
		if (ret < 0) {
			pr_err("failed to set output");
			gpio_free(pdata->hwen_gpio);
			return ret;
		}
		gpio_set_value(pdata->hwen_gpio, true);
	}

	return 0;
}

static void ktd3137_pwm_control(struct ktd3137_chip *chip, int brightness)
{
	struct pwm_device *pwm = NULL;
	unsigned int duty, period;

	if (!chip->pwm) {
		pwm = devm_pwm_get(chip->dev, DEFAULT_PWM_NAME);

		if (IS_ERR(pwm)) {
			dev_err(chip->dev, "can't get pwm device\n");
			return;
		}
	}

	if (brightness > chip->pdata->max_brightness)
		brightness = chip->pdata->max_brightness;

	chip->pwm = pwm;
	period = chip->pdata->pwm_period;
	duty = brightness * period / chip->pdata->max_brightness;
	pwm_config(chip->pwm, duty, period);

	if (duty)
		pwm_enable(chip->pwm);
	else
		pwm_disable(chip->pwm);
}

void ktd3137_brightness_set_workfunc(struct ktd3137_chip *chip, int brightness)
{
	struct ktd3137_bl_pdata *pdata = chip->pdata;

	if (brightness == 0) {
		ktd3137_write_reg(chip->client, 0x07, 0x44);
		ktd3137_write_reg(chip->client, REG_MODE, 0x98);
		mdelay(10);
	} else {
		ktd3137_write_reg(chip->client, REG_MODE, 0x99);
	}

	if (brightness > pdata->max_brightness)
			brightness = pdata->max_brightness;

	if (pdata->pwm_mode) {
		ktd3137_pwm_control(chip, brightness);
	} else {
		if (pdata->using_lsb) {
			ktd3137_masked_write(chip->client, REG_RATIO_LSB,
							0x07, brightness);
			ktd3137_masked_write(chip->client, REG_RATIO_MSB,
							0xff, brightness>>3);
		} else {
			ktd3137_masked_write(chip->client, REG_RATIO_LSB,
							0x07, brightness<<3);
			ktd3137_masked_write(chip->client, REG_RATIO_MSB,
							0xff, brightness);
		}
	}
}

#define LOWEST_BRIGHTNESS          8
int ktd3137_brightness_set(int brightness)
{
	if ((brightness > 0) && (brightness <= LOWEST_BRIGHTNESS))
		brightness = LOWEST_BRIGHTNESS - 1;

	ktd3137_brightness_set_workfunc(bkl_chip, brightness);
	return brightness;
}

static int ktd3137_update_status(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;
	struct ktd3137_chip *chip = bl_get_data(bd);

	cancel_delayed_work_sync(&chip->work);
	if (bd->props.power != FB_BLANK_UNBLANK ||
	   bd->props.fb_blank != FB_BLANK_UNBLANK ||
	   bd->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (brightness)
		ktd3137_masked_write(chip->client, REG_MODE, 0x01, 0x01);
	else
		ktd3137_masked_write(chip->client, REG_MODE, 0x01, 0x00);

	ktd3137_brightness_set_workfunc(chip, brightness);
	schedule_delayed_work(&chip->work, 100);

	return 0;
}

static const struct backlight_ops ktd3137_backlight_ops = {
	.options    = BL_CORE_SUSPENDRESUME,
	.update_status = ktd3137_update_status,
};

static struct class *ktd3137_class;
static atomic_t ktd_dev;
static struct device *ktd3137_dev;

struct device *ktd3137_device_create(void *drvdata, const char *fmt)
{
	struct device *dev;
	if (IS_ERR(ktd3137_class)) {
		pr_err("Failed to create class %ld\n", PTR_ERR(ktd3137_class));
	}

	dev = device_create(ktd3137_class, NULL, atomic_inc_return(&ktd_dev), drvdata, fmt);
	if (IS_ERR(dev)) {
		pr_err("Failed to create device %s %ld\n", fmt, PTR_ERR(dev));
	} else {
		pr_debug("%s : %s : %d\n", __func__, fmt, dev->devt);
	}

	return dev;
}

static void ktd3137_sync_backlight_work(struct work_struct *work)
{
	struct ktd3137_chip *chip = container_of(work, struct ktd3137_chip, work.work);
	u8 value = 0;
	ktd3137_read_reg(chip->client, REG_STATUS, &value);
	if (value) {
		pr_debug("status bit has been change! <%x>", value);
		if (value & RESET_CONDITION_BITS) {
			gpio_set_value(chip->pdata->hwen_gpio, false);
			gpio_set_value(chip->pdata->hwen_gpio, true);
			ktd3137_backlight_init(chip);
		}
	}
}

static int ktd3137_probe(struct i2c_client *client,
			const struct i2c_device_id *id) 
{
	struct ktd3137_bl_pdata *pdata = dev_get_drvdata(&client->dev);
	struct ktd3137_chip *chip;
	int ret = 0;

#ifdef CONFIG_MACH_XIAOMI_SDM439
	extern char *saved_command_line;
	int bkl_id = 0;
	char *bkl_ptr = (char *)strnstr(saved_command_line, ":bklic=", strlen(saved_command_line));
	bkl_ptr += strlen(":bklic=");
	bkl_id = simple_strtol(bkl_ptr, NULL, 10);
	if (bkl_id != 24) {
		return -ENODEV;
	}
#endif

	client->addr = 0x36;
	if (!pdata) {
		ret = ktd3137_parse_dt(&client->dev, chip);
		if (ret)
			return ret;
		pdata = dev_get_platdata(&client->dev);
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check_functionality failed.\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = pdata;
	chip->dev = &client->dev;

	ktd3137_dev = ktd3137_device_create(chip, "ktd");
	if (IS_ERR(ktd3137_dev)) {
		dev_err(&client->dev, "failed_to create device for ktd");
	}

	i2c_set_clientdata(client, chip);

	ktd3137_gpio_init(chip);
	ktd3137_backlight_init(chip);
	INIT_DELAYED_WORK(&chip->work, ktd3137_sync_backlight_work);
	ktd3137_flashled_init(client, chip);
	bkl_chip = chip;

	return 0;
}

static int ktd3137_remove(struct i2c_client *client)
{
	struct ktd3137_chip *chip = i2c_get_clientdata(client);

	chip->bl->props.brightness = 0;

	backlight_update_status(chip->bl);
	cancel_delayed_work_sync(&chip->work);
	gpio_set_value(chip->pdata->hwen_gpio, false);
	gpio_free(chip->pdata->hwen_gpio);

	return 0;
}

static const struct i2c_device_id ktd3137_id[] = {
	{KTD_I2C_NAME, 0},
	{ }
};

static const struct of_device_id ktd3137_match_table[] = {
	{ .compatible = "ktd,ktd3137",},
	{ },
};

static struct i2c_driver ktd3137_driver = {
	.driver = {
		.name	= KTD_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktd3137_match_table,
	},
	.probe = ktd3137_probe,
	.remove = ktd3137_remove,
	.id_table = ktd3137_id,
};

static int __init ktd3137_init(void)
{
	int err;

	ktd3137_class = class_create(THIS_MODULE, "ktd3137");
	if (IS_ERR(ktd3137_class)) {
		pr_err("unable to create ktd3137 class; errno = %ld\n", PTR_ERR(ktd3137_class));
		ktd3137_class = NULL;
	}

	err = i2c_add_driver(&ktd3137_driver);
	if (err)
		pr_err("ktd3137 driver failed,(errno = %d)\n", err);

	return err;
}

static void __exit ktd3137_exit(void)
{
	i2c_del_driver(&ktd3137_driver);
}

module_init(ktd3137_init);
module_exit(ktd3137_exit);

MODULE_AUTHOR("kinet-ic.com");
MODULE_LICENSE("GPL");