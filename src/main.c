#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <errno.h>
#include <strings.h>
/* 定义日志模块 */
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
/* 定义LED引脚 */
#define LED1	DT_NODELABEL(led0)
#define LED2	DT_NODELABEL(led1)
#define LED3	DT_NODELABEL(led2)
#define LED4	DT_NODELABEL(led3)
static struct  gpio_dt_spec LED1_struct=GPIO_DT_SPEC_GET(LED1, gpios);
static struct  gpio_dt_spec LED2_struct=GPIO_DT_SPEC_GET(LED2, gpios);
static struct  gpio_dt_spec LED3_struct=GPIO_DT_SPEC_GET(LED3, gpios);
static struct  gpio_dt_spec LED4_struct=GPIO_DT_SPEC_GET(LED4, gpios);
/* 定义按键引脚 */
#define botton1	DT_NODELABEL(button0)
#define botton2	DT_NODELABEL(button1)
#define botton3	DT_NODELABEL(button2)
#define botton4	DT_NODELABEL(button3)
static struct  gpio_dt_spec botton1_struct=GPIO_DT_SPEC_GET(botton1, gpios);
static struct  gpio_dt_spec botton2_struct=GPIO_DT_SPEC_GET(botton2, gpios);
static struct  gpio_dt_spec botton3_struct=GPIO_DT_SPEC_GET(botton3, gpios);
static struct  gpio_dt_spec botton4_struct=GPIO_DT_SPEC_GET(botton4, gpios);
static struct gpio_callback button_callback_struct;

/* MPU6050 设备句柄与 I2C 配置 */
#define MPU6050_NODE DT_NODELABEL(mpu6050)
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_CHIP_ID      0x68
#define MPU6050_REG_PWR_MGMT1 0x6B

static const struct device *mpu6050_dev = DEVICE_DT_GET(MPU6050_NODE);
static const struct i2c_dt_spec mpu6050_i2c = I2C_DT_SPEC_GET(MPU6050_NODE);

/* 扫描 I2C0 总线上所有 7bit 地址 */
static void i2c0_scan(void)
{
	int ret;
	struct i2c_msg msg;
	uint8_t addr;
	uint8_t dummy=0x00;
	msg.buf = &dummy;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	if (!device_is_ready(mpu6050_i2c.bus))
	{
		LOG_ERR("I2C0 bus device is not ready");
		return;
	}

	LOG_INF("Start I2C0 bus scan");

	for (addr = 0x03; addr < 0xff; addr++)
	{
		//ret = i2c_read(mpu6050_i2c.bus, &dummy, 1, addr);
		 ret = i2c_transfer(mpu6050_i2c.bus, &msg, 1, addr);
		if (ret == 0)
		{
			LOG_INF("I2C device found at 0x%02x", addr);
		}
	}

	LOG_INF("I2C0 bus scan done");
}

static int mpu6050_init_chip(void)
{
	int ret=0;
	uint8_t chip_id=0;

	if (!device_is_ready(mpu6050_i2c.bus))
	{
		LOG_ERR("MPU6050 I2C bus is not ready");
		return -EIO;
	}

	/* 读取芯片 ID */
	ret = i2c_reg_read_byte_dt(&mpu6050_i2c, MPU6050_REG_WHO_AM_I, &chip_id);
	if (ret != 0)
	{
		LOG_ERR("Failed to read MPU6050 WHO_AM_I (err %d)", ret);
		return ret;
	}
	LOG_INF("MPU6050 WHO_AM_I=0x%02x", chip_id);
	if (chip_id != MPU6050_CHIP_ID)
	{
		LOG_ERR("Unexpected MPU6050 chip ID: 0x%02x (expected 0x%02x)", chip_id, MPU6050_CHIP_ID);
		return -EIO;
	}

	LOG_INF("MPU6050 WHO_AM_I=0x%02x", chip_id);

	/* 退出休眠模式，使用内部时钟 */
	ret = i2c_reg_write_byte_dt(&mpu6050_i2c, MPU6050_REG_PWR_MGMT1, 0x00);
	if (ret != 0)
	{
		LOG_ERR("Failed to write PWR_MGMT1 (err %d)", ret);
		return ret;
	}

	return 0;
}
/* 按键回调函数 */
void gpio_callback(const struct device *port,struct gpio_callback *cb,gpio_port_pins_t pins)
{
        if(pins==BIT(botton1_struct.pin))
        {
                LOG_DBG("Button pressed");
                gpio_pin_toggle_dt(&LED1_struct);
        }
        if(pins==BIT(botton2_struct.pin))
        {
                LOG_DBG("Button pressed");
                gpio_pin_toggle_dt(&LED2_struct);
        }
        if(pins==BIT(botton3_struct.pin))
        {
                LOG_DBG("Button pressed");
                gpio_pin_toggle_dt(&LED3_struct);
        }
        if(pins==BIT(botton4_struct.pin))
        {
                LOG_DBG("Button pressed");
                gpio_pin_toggle_dt(&LED4_struct);
        }
}
/* 主函数 */
int main(void)
{
	int ret=0;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	LOG_INF("Hello World! %s\n", CONFIG_BOARD_TARGET);
	uint8_t write_buf[3] = {0x75,0x6b,0x00};
	uint8_t read_buf[1] = { 0x00};
	ret=i2c_write_dt(&mpu6050_i2c, write_buf, 1);
	//ret=i2c_write(mpu6050_i2c.bus, write_buf, 1, 0x68);
	if (ret != 0)
	{
		LOG_ERR("Failed to write read (%d)", ret);
	}
	ret=i2c_read_dt(&mpu6050_i2c, read_buf, 1);
	//ret=i2c_read(mpu6050_i2c.bus, read_buf, 1, 0x68);
	if (ret != 0)
	{
		LOG_ERR("Failed to read (%d)", ret);
	}
	LOG_INF("read_buf = %02x", read_buf[0]);
	i2c_write_dt(&mpu6050_i2c, &write_buf[1], 2);
	k_sleep(K_MSEC(1000));
	uint8_t write_buf2[1] = {0x3B};
	uint8_t read_buf2[14] = {0};
	while (1)
	{
		ret = i2c_write_read_dt(&mpu6050_i2c, write_buf2, 1, read_buf2, 14);
		if (ret != 0)
		{
			LOG_ERR("Failed to write_read (%d)", ret);
		}
		else
		{
			LOG_INF("read_buf2 = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				read_buf2[0], read_buf2[1], read_buf2[2], read_buf2[3], read_buf2[4], read_buf2[5], read_buf2[6], read_buf2[7], read_buf2[8], read_buf2[9], read_buf2[10], read_buf2[11], read_buf2[12], read_buf2[13]);
		}
		k_sleep(K_MSEC(1000));
	}
	// while (1)
	// {
	// 	ret=i2c_write_dt(&mpu6050_i2c, write_buf2, 1);
	// 	if (ret != 0)
	// 	{
	// 		LOG_ERR("Failed to write (%d)", ret);
	// 	}
	// 	k_sleep(K_MSEC(1000));
	// 	ret=i2c_read_dt(&mpu6050_i2c, read_buf2, 14);
	// 	if (ret != 0)
	// 	{
	// 		LOG_ERR("Failed to read (%d)", ret);
	// 	}
	// 	LOG_INF("read_buf2 = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
	// 		read_buf2[0], read_buf2[1], read_buf2[2], read_buf2[3], read_buf2[4], read_buf2[5], read_buf2[6], read_buf2[7], read_buf2[8], read_buf2[9], read_buf2[10], read_buf2[11], read_buf2[12], read_buf2[13]);
	// 	k_sleep(K_MSEC(1000));
	// }
	//device_init(mpu6050_dev);
	/* 启动前先扫描 I2C0 总线 */
	//i2c0_scan();
	ret = mpu6050_init_chip();
	/* 检查 MPU6050 设备是否就绪并初始化芯片 */
	if (!device_is_ready(mpu6050_dev))
	{
		LOG_ERR("MPU6050 device is not ready");
	}

	//ret = mpu6050_init_chip();
	if (ret != 0)
	{
		LOG_ERR("MPU6050 init failed (%d)", ret);
	}
	if (gpio_is_ready_dt(&LED1_struct)==false)
	{
		LOG_ERR("LED1 is not ready");
	}
	if (gpio_is_ready_dt(&LED2_struct)==false)
	{
		LOG_ERR("LED2 is not ready");
	}
	if (gpio_is_ready_dt(&LED3_struct)==false)
	{
		LOG_ERR("LED3 is not ready");	
	}
	if (gpio_is_ready_dt(&LED4_struct)==false)
	{
		LOG_ERR("LED4 is not ready");
	}
	if (gpio_is_ready_dt(&botton1_struct)==false)
	{
		LOG_ERR("botton1 is not ready");
	}
	if (gpio_is_ready_dt(&botton2_struct)==false)
	{
		LOG_ERR("botton2 is not ready");
	}
	if (gpio_is_ready_dt(&botton3_struct)==false)
	{
		LOG_ERR("botton3 is not ready");
	}
	if (gpio_is_ready_dt(&botton4_struct)==false)
	{
		LOG_ERR("botton4 is not ready");
	}
	//配置LED引脚为输出
	gpio_pin_configure_dt(&LED1_struct, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&LED2_struct, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&LED3_struct, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&LED4_struct, GPIO_OUTPUT_ACTIVE);
	//配置按键引脚为输入
	gpio_pin_configure_dt(&botton1_struct, GPIO_INPUT);
	gpio_pin_configure_dt(&botton2_struct, GPIO_INPUT);
	gpio_pin_configure_dt(&botton3_struct, GPIO_INPUT);
	gpio_pin_configure_dt(&botton4_struct, GPIO_INPUT);
	//初始化按键回调函数
	ret=gpio_pin_interrupt_configure_dt(&botton1_struct, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret!=0)
	{
		LOG_ERR("Failed to configure botton1");
	}
	ret=gpio_pin_interrupt_configure_dt(&botton2_struct, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret!=0)
	{
		LOG_ERR("Failed to configure botton2");
	}
	ret=gpio_pin_interrupt_configure_dt(&botton3_struct, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret!=0)
	{
		LOG_ERR("Failed to configure botton3");
	}
	ret=gpio_pin_interrupt_configure_dt(&botton4_struct, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret!=0)
	{
		LOG_ERR("Failed to configure botton4");
	}
	gpio_init_callback(&button_callback_struct, gpio_callback, BIT(botton1_struct.pin) | BIT(botton2_struct.pin) | BIT(botton3_struct.pin) | BIT(botton4_struct.pin));
	ret=gpio_add_callback_dt(&botton1_struct, &button_callback_struct);
	if (ret!=0)
	{
		LOG_ERR("Failed to add callback to botton1");
	}
	ret=gpio_add_callback_dt(&botton2_struct, &button_callback_struct);
	if (ret!=0)
	{
		LOG_ERR("Failed to add callback to botton2");
	}
	ret=gpio_add_callback_dt(&botton3_struct, &button_callback_struct);
	if (ret!=0)
	{
		LOG_ERR("Failed to add callback to botton3");
	}
	ret=gpio_add_callback_dt(&botton4_struct, &button_callback_struct);
	if (ret!=0)
	{
		LOG_ERR("Failed to add callback to botton4");
	}


	while (1) 
	{
		ret = sensor_sample_fetch(mpu6050_dev);
		// if (ret != 0) 
		// {
		// 	LOG_ERR("MPU6050 sample fetch failed (%d)", ret);
		// 	k_sleep(K_MSEC(500));
		// 	continue;
		// }

		ret = sensor_channel_get(mpu6050_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		// if (ret != 0)
		// {
		// 	LOG_ERR("MPU6050 accel channel get failed (%d)", ret);
		// 	k_sleep(K_MSEC(500));
		// 	continue;
		// }

		ret = sensor_channel_get(mpu6050_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		// if (ret != 0)
		// {
		// 	LOG_ERR("MPU6050 gyro channel get failed (%d)", ret);
		// 	k_sleep(K_MSEC(500));
		// 	continue;
		// }

		LOG_INF("Accel x=%d.%06d y=%d.%06d z=%d.%06d",
			accel[0].val1, accel[0].val2,
			accel[1].val1, accel[1].val2,
			accel[2].val1, accel[2].val2);
		LOG_INF("Gyro  x=%d.%06d y=%d.%06d z=%d.%06d",
			gyro[0].val1, gyro[0].val2,
			gyro[1].val1, gyro[1].val2,
			gyro[2].val1, gyro[2].val2);

		k_sleep(K_MSEC(500));
	}
	return 0;
}
