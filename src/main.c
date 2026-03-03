#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>
#include <errno.h>
#include <strings.h>
/* 定义日志模块 */
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
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

#define LCD_NODE DT_NODELABEL(st7735r)

/* LCD 相关全局结构体，放在 main 之外 */
static const struct device *lcd_dev = DEVICE_DT_GET(LCD_NODE);
static struct display_capabilities lcd_caps;
static struct display_buffer_descriptor lcd_desc;
static uint8_t lcd_line_buf[240 * 2];
static size_t lcd_color_idx;

/* 简单 5x7 字符集，只实现常用字符：空格、0-9、A-Z */
#define LCD_CHAR_W 6
#define LCD_CHAR_H 8

static const uint8_t lcd_font5x7[][5] = {
	/* ' ' 0x20 */
	{ 0x00, 0x00, 0x00, 0x00, 0x00 },
	/* '0' 0x30 */
	{ 0x3E, 0x51, 0x49, 0x45, 0x3E },
	/* '1' */
	{ 0x00, 0x42, 0x7F, 0x40, 0x00 },
	/* '2' */
	{ 0x42, 0x61, 0x51, 0x49, 0x46 },
	/* '3' */
	{ 0x21, 0x41, 0x45, 0x4B, 0x31 },
	/* '4' */
	{ 0x18, 0x14, 0x12, 0x7F, 0x10 },
	/* '5' */
	{ 0x27, 0x45, 0x45, 0x45, 0x39 },
	/* '6' */
	{ 0x3C, 0x4A, 0x49, 0x49, 0x30 },
	/* '7' */
	{ 0x01, 0x71, 0x09, 0x05, 0x03 },
	/* '8' */
	{ 0x36, 0x49, 0x49, 0x49, 0x36 },
	/* '9' */
	{ 0x06, 0x49, 0x49, 0x29, 0x1E },
	/* 'A' 0x41 */
	{ 0x7E, 0x11, 0x11, 0x11, 0x7E },
	/* 'B' */
	{ 0x7F, 0x49, 0x49, 0x49, 0x36 },
	/* 'C' */
	{ 0x3E, 0x41, 0x41, 0x41, 0x22 },
	/* 'D' */
	{ 0x7F, 0x41, 0x41, 0x22, 0x1C },
	/* 'E' */
	{ 0x7F, 0x49, 0x49, 0x49, 0x41 },
	/* 'F' */
	{ 0x7F, 0x09, 0x09, 0x09, 0x01 },
	/* 'G' */
	{ 0x3E, 0x41, 0x49, 0x49, 0x7A },
	/* 'H' */
	{ 0x7F, 0x08, 0x08, 0x08, 0x7F },
	/* 'I' */
	{ 0x00, 0x41, 0x7F, 0x41, 0x00 },
	/* 'J' */
	{ 0x20, 0x40, 0x41, 0x3F, 0x01 },
	/* 'K' */
	{ 0x7F, 0x08, 0x14, 0x22, 0x41 },
	/* 'L' */
	{ 0x7F, 0x40, 0x40, 0x40, 0x40 },
	/* 'M' */
	{ 0x7F, 0x02, 0x0C, 0x02, 0x7F },
	/* 'N' */
	{ 0x7F, 0x04, 0x08, 0x10, 0x7F },
	/* 'O' */
	{ 0x3E, 0x41, 0x41, 0x41, 0x3E },
	/* 'P' */
	{ 0x7F, 0x09, 0x09, 0x09, 0x06 },
	/* 'Q' */
	{ 0x3E, 0x41, 0x51, 0x21, 0x5E },
	/* 'R' */
	{ 0x7F, 0x09, 0x19, 0x29, 0x46 },
	/* 'S' */
	{ 0x46, 0x49, 0x49, 0x49, 0x31 },
	/* 'T' */
	{ 0x01, 0x01, 0x7F, 0x01, 0x01 },
	/* 'U' */
	{ 0x3F, 0x40, 0x40, 0x40, 0x3F },
	/* 'V' */
	{ 0x1F, 0x20, 0x40, 0x20, 0x1F },
	/* 'W' */
	{ 0x7F, 0x20, 0x18, 0x20, 0x7F },
	/* 'X' */
	{ 0x63, 0x14, 0x08, 0x14, 0x63 },
	/* 'Y' */
	{ 0x03, 0x04, 0x78, 0x04, 0x03 },
	/* 'Z' */
	{ 0x61, 0x51, 0x49, 0x45, 0x43 },
};

static const uint16_t lcd_colors[] = {
	0xF800, /* red */
	0x07E0, /* green */
	0x001F, /* blue */
	0xFFFF, /* white */
	0x0000, /* black */
};

static int lcd_init(void)
{
	if (!device_is_ready(lcd_dev)) {
		LOG_ERR("LCD device not ready");
		return -ENODEV;
	}

	int ret = display_set_pixel_format(lcd_dev, PIXEL_FORMAT_RGB_565);
	if (ret != 0) {
		LOG_WRN("display_set_pixel_format failed (%d)", ret);
	}

	ret = display_blanking_off(lcd_dev);
	if (ret != 0) {
		LOG_WRN("display_blanking_off failed (%d)", ret);
	}

	display_get_capabilities(lcd_dev, &lcd_caps);

	const uint16_t w = lcd_caps.x_resolution;

	if (w == 0 || w > 240) {
		LOG_ERR("Unexpected LCD width %u", w);
		return -EINVAL;
	}

	lcd_desc.width = w;
	lcd_desc.height = 1;
	lcd_desc.pitch = w;
	lcd_desc.buf_size = (size_t)w * 2;

	lcd_color_idx = 0;

	return 0;
}

static int lcd_fill_color(uint16_t color)
{
	const uint16_t w = lcd_caps.x_resolution;
	const uint16_t h = lcd_caps.y_resolution;

	if (w == 0 || h == 0) {
		return -EINVAL;
	}

	for (uint16_t x = 0; x < w; x++) {
		sys_put_be16(color, &lcd_line_buf[x * 2]);
	}

	for (uint16_t y = 0; y < h; y++) {
		int ret = display_write(lcd_dev, 0, y, &lcd_desc, lcd_line_buf);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static const uint8_t *lcd_get_glyph(char c)
{
	if (c == ' ') {
		return lcd_font5x7[0];
	}
	if (c >= '0' && c <= '9') {
		return lcd_font5x7[1 + (c - '0')];
	}
	if (c >= 'A' && c <= 'Z') {
		return lcd_font5x7[11 + (c - 'A')];
	}
	if (c >= 'a' && c <= 'z') {
		return lcd_font5x7[11 + (c - 'a')]; /* 小写按大写显示 */
	}
	return lcd_font5x7[0];
}

/* 在 LCD 上绘制单个字符，左上角坐标 (x, y)，前景色/背景色为 fg/bg */
static int lcd_draw_char(uint16_t x, uint16_t y, char c,
			 uint16_t fg, uint16_t bg)
{
	const uint16_t w = lcd_caps.x_resolution;
	const uint16_t h = lcd_caps.y_resolution;

	if (x + LCD_CHAR_W > w || y + LCD_CHAR_H > h) {
		return -EINVAL;
	}

	const uint8_t *glyph = lcd_get_glyph(c);
	uint8_t buf[LCD_CHAR_W * LCD_CHAR_H * 2];

	for (uint16_t row = 0; row < LCD_CHAR_H; row++) {
		for (uint16_t col = 0; col < 5; col++) {
			/* 字库按列存储，每列低位在上方 */
			uint8_t col_bits = glyph[col];
			bool on = (row < 7) && ((col_bits >> row) & 0x01);
			uint16_t color = on ? fg : bg;
			sys_put_be16(color,
				     &buf[2 * (row * LCD_CHAR_W + col)]);
		}
		/* 第 6 列做 1 像素间隔 */
		sys_put_be16(bg,
			     &buf[2 * (row * LCD_CHAR_W + 5)]);
	}

	struct display_buffer_descriptor desc = {
		.width = LCD_CHAR_W,
		.height = LCD_CHAR_H,
		.pitch = LCD_CHAR_W,
		.buf_size = sizeof(buf),
	};

	return display_write(lcd_dev, x, y, &desc, buf);
}

/* 在 LCD 上绘制字符串，单行显示 */
static int lcd_draw_string(uint16_t x, uint16_t y,
			   const char *s,
			   uint16_t fg, uint16_t bg)
{
	if (s == NULL) {
		return -EINVAL;
	}

	while (*s) {
		int ret = lcd_draw_char(x, y, *s, fg, bg);
		if (ret != 0) {
			return ret;
		}
		x += LCD_CHAR_W;
		s++;
	}

	return 0;
}

#if 0
/* MPU6050 设备句柄与 I2C 配置 */
#define MPU6050_NODE DT_NODELABEL(mpu6050)
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_CHIP_ID      0x68
#define MPU6050_REG_PWR_MGMT1 0x6B

static const struct device *mpu6050_dev = DEVICE_DT_GET(MPU6050_NODE);
static const struct i2c_dt_spec mpu6050_i2c = I2C_DT_SPEC_GET(MPU6050_NODE);
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
//mpu6050数据读取
void mpu6050_data_read(void)
{
	int ret=0;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	ret = sensor_sample_fetch(mpu6050_dev);
	if (ret != 0)
	{
		LOG_ERR("MPU6050 sample fetch failed (%d)", ret);
		return;
	}
	ret = sensor_channel_get(mpu6050_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret != 0)
	{
		LOG_ERR("MPU6050 accel channel get failed (%d)", ret);
		return;
	}
	ret = sensor_channel_get(mpu6050_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (ret != 0)
	{
		LOG_ERR("MPU6050 gyro channel get failed (%d)", ret);
		return;
	}
	LOG_INF("Accel x=%d.%06d y=%d.%06d z=%d.%06d",
		accel[0].val1, accel[0].val2,
		accel[1].val1, accel[1].val2,
		accel[2].val1, accel[2].val2);
	LOG_INF("Gyro  x=%d.%06d y=%d.%06d z=%d.%06d",
		gyro[0].val1, gyro[0].val2,
		gyro[1].val1, gyro[1].val2,
		gyro[2].val1, gyro[2].val2);
}
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

	for (addr = 0x03; addr < 0x7f; addr++)
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
#endif

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
	LOG_INF("Hello World! %s\n", CONFIG_BOARD_TARGET);
	uint16_t color = lcd_colors[lcd_color_idx];
	char msg[16];
	ret = lcd_init();
	if (ret != 0) {
		LOG_ERR("LCD init failed (%d)", ret);
	}

	/* 检查 MPU6050 设备是否就绪并初始化芯片 */
	// device_init(mpu6050_dev);
	// i2c0_scan();
	// ret = mpu6050_init_chip();
	// if (ret != 0)
	// {
	// 	LOG_ERR("MPU6050 init failed (%d)", ret);
	// }
	// if (!device_is_ready(mpu6050_dev))
	// {
	// 	LOG_ERR("MPU6050 device is not ready");
	// }
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
		lcd_color_idx = (lcd_color_idx + 1) % ARRAY_SIZE(lcd_colors);
		color = lcd_colors[lcd_color_idx];

		/* 刷整屏颜色 */
		lcd_fill_color(color);

		/* 在左上角显示当前颜色索引字符串 */
		snprintk(msg, sizeof(msg), "COLOR %u",
			 (unsigned)lcd_color_idx);
		lcd_draw_string(0, 0, msg, 0xFFFF, 0x0000);

		k_sleep(K_MSEC(500));
	}
	return 0;
}
