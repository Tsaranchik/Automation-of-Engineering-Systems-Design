#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <driver/i2c.h>
#include <driver/rmt_tx.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_MASTER_NUM			I2C_NUM_0
#define I2C_MASTER_FREQ_HZ		100000
#define I2C_MASTER_SDA_IO		5
#define I2C_MASTER_SCL_IO		3
#define I2C_MASTER_TX_BUF_DISABLE	0
#define I2C_MASTER_RX_BUF_DISABLE	0

#define MPU6050_I2C_ADDR		0x68

#define MPU6050_REG_PWR_MGMT_1		0x6B
#define MPU6050_REG_ACCEL_XOUT_H	0x3B
#define MPU6050_REG_ACCEL_CONFIG	0x1C
#define MPU6050_REG_SMPLRT_DIV		0x19
#define MPU6050_REG_CONFIG		0x1A

#define LED_STRIP_GPIO			4
#define LED_MATRIX_SIZE			16
#define TOTAL_LEDS			(LED_MATRIX_SIZE * LED_MATRIX_SIZE)

#define WS2812_T0H_NS			350
#define WS2812_T0L_NS			900
#define WS2812_T1H_NS			900
#define WS2812_T1L_NS			350
#define WS2812_RESET_US			280

#define ACCEL_SCALE			16384.0f
#define ACCEL_THRESHOLD			0.1f
#define MAX_TILT_ANGLE			45.0f

typedef struct {
	float x;
	float y;
	float z;
} accel_data_t;

typedef struct {
	uint8_t matrix[LED_MATRIX_SIZE][LED_MATRIX_SIZE][3];
	rmt_channel_handle_t rmt_channel;
	rmt_encoder_handle_t encoder;
} led_matrix_t;

static const char *TAG = "LAB3";

static esp_err_t i2c_master_init(void)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
	};

	esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
	if (err != ESP_OK) return err;

	return i2c_driver_install(
		I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
		I2C_MASTER_TX_BUF_DISABLE, 0
	);
}

static esp_err_t mpu6050_register_write(uint8_t reg_addr, uint8_t data)
{
	uint8_t write_buf[2] = {reg_addr, data};

	return i2c_master_write_to_device(
		I2C_MASTER_NUM, MPU6050_I2C_ADDR,
		write_buf, sizeof(write_buf),
		pdMS_TO_TICKS(1000)
	);
}

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
	return i2c_master_write_read_device(
		I2C_MASTER_NUM, MPU6050_I2C_ADDR,
		&reg_addr, 1, data, len,
		pdMS_TO_TICKS(1000)
	);
}

static esp_err_t mpu6050_init(void)
{
	esp_err_t ret = mpu6050_register_write(MPU6050_REG_PWR_MGMT_1, 0x00);
	
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Не удалось вывести MPU6050 из спящего режима");
		return ret;
	}

	vTaskDelay(pdMS_TO_TICKS(100));

	ret = mpu6050_register_write(MPU6050_REG_ACCEL_CONFIG, 0x00);
	
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Не удалось настроить акселерометр");
		return ret;
	}

	ret = mpu6050_register_write(MPU6050_REG_CONFIG, 0x06);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Не удалось настроить фильтр");
		return ret;
	}

	ret = mpu6050_register_write(MPU6050_REG_SMPLRT_DIV, 0x07);

	ESP_LOGI(TAG, "MPU6050 инициализирован успешно");

	return ret;
}

static esp_err_t mpu6050_read_accelerometer(accel_data_t *accel)
{
	uint8_t data[6];
	esp_err_t ret = mpu6050_register_read(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data));

	if (ret == ESP_OK) {
		int16_t raw_x = (data[0] << 8) | data[1];
		int16_t raw_y = (data[2] << 8) | data[3];
		int16_t raw_z = (data[4] << 8) | data[5];

		accel->x = raw_x / ACCEL_SCALE;
		accel->y = raw_y / ACCEL_SCALE;
		accel->z = raw_z / ACCEL_SCALE;

		if (fabsf(accel->x) < ACCEL_THRESHOLD) accel->x = 0;
		if (fabsf(accel->y) < ACCEL_THRESHOLD) accel->y = 0;
	}

	return ret;
}

static esp_err_t led_matrix_init(led_matrix_t *matrix)
{
	esp_err_t ret;

	rmt_tx_channel_config_t tx_channel_cfg = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.gpio_num =LED_STRIP_GPIO,
		.mem_block_symbols = 64,
		.resolution_hz = 10 * 1000 * 1000,
		.trans_queue_depth = 4,
		.flags.with_dma = false

	};

	ret = rmt_new_tx_channel(&tx_channel_cfg, &matrix->rmt_channel);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Не удалось создать RMT канал: %s", esp_err_to_name(ret));
		return ret;
	}

	rmt_bytes_encoder_config_t bytes_encoder_cfg = {
		.bit0 = {
			.duration0 = WS2812_T0H_NS / (1000000000 / tx_channel_cfg.resolution_hz),
			.level0 = 1,
			.duration1 = WS2812_T0L_NS / (1000000000 / tx_channel_cfg.resolution_hz),
			.level1 = 0
		},
		.bit1 = {
			.duration0 = WS2812_T1H_NS / (1000000000 / tx_channel_cfg.resolution_hz),
			.level0 = 1,
			.duration1 = WS2812_T1L_NS / (1000000000 / tx_channel_cfg.resolution_hz),
			.level1 = 0
		},
		.flags.msb_first = 1
	};

	ret = rmt_new_bytes_encoder(&bytes_encoder_cfg, &matrix->encoder);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Не удалось создать RMT энкодер: %s", esp_err_to_name(ret));
		rmt_del_channel(matrix->rmt_channel);
		return ret;
	}

	ret = rmt_enable(matrix->rmt_channel);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Не удалось включить RMT канал: %s", esp_err_to_name(ret));
		rmt_del_encoder(matrix->encoder);
		rmt_del_channel(matrix->rmt_channel);
		return ret;
	}

	for (int y = 0; y < LED_MATRIX_SIZE; ++y) {
		for (int x = 0; x < LED_MATRIX_SIZE; ++x) {
			matrix->matrix[y][x][0] = 0;
			matrix->matrix[y][x][1] = 0;
			matrix->matrix[y][x][2] = 0;
		}
	}

	ESP_LOGI(TAG, "Светодиодная матрица инициализирована");
	return ESP_OK;
}

static void led_matrix_clear(led_matrix_t *matrix)
{
	for (int y = 0; y < LED_MATRIX_SIZE; ++y) {
		for (int x = 0; x < LED_MATRIX_SIZE; ++x) {
			matrix->matrix[y][x][0] = 0;
			matrix->matrix[y][x][1] = 0;
			matrix->matrix[y][x][2] = 0;
		}
	}
}

static void led_matrix_set_pixel(
	led_matrix_t *matrix, int x, int y,
	uint8_t r, uint8_t g, uint8_t b
)
{
	if (x >= 0 && x < LED_MATRIX_SIZE && y >= 0 && y < LED_MATRIX_SIZE) {
		matrix->matrix[y][x][0] = r;
		matrix->matrix[y][x][1] = g;
		matrix->matrix[y][x][2] = b;
	}
}

static esp_err_t led_matrix_update(led_matrix_t *matrix)
{
	esp_err_t ret;

	uint8_t led_data[TOTAL_LEDS * 3];

	for (int i = 0; i < TOTAL_LEDS; ++i) {
		int y = i / LED_MATRIX_SIZE;
		int x = i % LED_MATRIX_SIZE;
		led_data[i * 3] = matrix->matrix[y][x][1];
		led_data[i * 3 + 1] = matrix->matrix[y][x][0];
		led_data[i * 3 + 2] = matrix->matrix[y][x][2];
	}

	rmt_transmit_config_t tx_config = {
		.loop_count = 0
	};

	ret = rmt_transmit(
		matrix->rmt_channel, matrix->encoder,
		led_data, sizeof(led_data), &tx_config
	);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Ошибка передачи RMT: %s", esp_err_to_name(ret));
		return ret;
	}

	ret = rmt_tx_wait_all_done(matrix->rmt_channel, pdMS_TO_TICKS(100));
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Ошибка ожидания RMT: %s", esp_err_to_name(ret));
		rmt_disable(matrix->rmt_channel);
		vTaskDelay(pdMS_TO_TICKS(500));
		rmt_enable(matrix->rmt_channel);
	}

	return ret;
}

static void calculate_tilt_angels(const accel_data_t *accel, float *pitch, float *roll)
{
	*roll = atan2f(accel->y, sqrtf(accel->x * accel->x + accel->z * accel->z));
	*pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));

	*roll = fmaxf(fminf(*roll * 180.0f / M_PI, MAX_TILT_ANGLE), -MAX_TILT_ANGLE);
	*pitch = fmaxf(fminf(*pitch * 180.0f / M_PI, MAX_TILT_ANGLE), -MAX_TILT_ANGLE);
}

static float calculate_tilt_intensity(const accel_data_t *accel)
{
	float intensity = sqrtf(accel->x * accel->x + accel->y * accel->y);

	return fminf(intensity, 1.0f);
}

static void draw_line(led_matrix_t *matrix, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
	int dx = abs(x1 - x0);
	int dy = abs(y1 - y0);
	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;
	int err = dx - dy;
	
	while (1) {
		if (x0 >= 0 && x0 < LED_MATRIX_SIZE && y0 >= 0 && y0 < LED_MATRIX_SIZE)
			led_matrix_set_pixel(matrix, x0, y0, r, g, b);
		
		if (x0 == x1 && y0 == y1) break;
		int e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y0 += sy;
		}
	}
}

static inline int clamp(int v, int min, int max) {
	if (v < min) return min;
	if (v > max) return max;
	return v;
}

static void draw_square_with_fixed_corner(
	led_matrix_t *matrix,
	float pitch, float roll,
	float intensity
) {
	led_matrix_clear(matrix);

	int cx = LED_MATRIX_SIZE / 2;
	int cy = LED_MATRIX_SIZE / 2;

	float nx = roll  / MAX_TILT_ANGLE;
	float ny = pitch / MAX_TILT_ANGLE;

	if (nx > 1) nx = 1;
	if (nx < -1) nx = -1;
	if (ny > 1) ny = 1;
	if (ny < -1) ny = -1;

	if (fabs(nx) < 0.05f && fabs(ny) < 0.05f) {
		led_matrix_clear(matrix);
		led_matrix_set_pixel(matrix, cx, cy, 255, 255, 255);
		return;
	}

	int side = 2 + (int)(intensity * 6.0f);

	float len = sqrtf(nx * nx + ny * ny);
	if (len < 0.001f) len = 0.001f;

	float vx = nx / len * side;
	float vy = ny / len * side;

	float px = -vy;
	float py =  vx;

	int xA = cx;
	int yA = cy;

	int xB = clamp(cx + (int)vx, 0, LED_MATRIX_SIZE - 1);
	int yB = clamp(cy + (int)vy, 0, LED_MATRIX_SIZE - 1);

	int xC = clamp(cx + (int)px, 0, LED_MATRIX_SIZE - 1);
	int yC = clamp(cy + (int)py, 0, LED_MATRIX_SIZE - 1);

	int xD = clamp(xB + (int)px, 0, LED_MATRIX_SIZE - 1);
	int yD = clamp(yB + (int)py, 0, LED_MATRIX_SIZE - 1);

	uint8_t brightness = (uint8_t)(30 + intensity * 225);
	uint8_t r = brightness, g = brightness, b = brightness;

	draw_line(matrix, xA, yA, xB, yB, r, g, b);
	draw_line(matrix, xA, yA, xC, yC, r, g, b);
	draw_line(matrix, xB, yB, xD, yD, r, g, b);
	draw_line(matrix, xC, yC, xD, yD, r, g, b);

	led_matrix_set_pixel(matrix, xA, yA, 0, 0, 255);
	led_matrix_set_pixel(matrix, xD, yD, 255, 0, 0);
}

static void main_task(void *pvParameters)
{
	led_matrix_t led_matrix;
	accel_data_t accel_data;
	float pitch, roll, intensity;

	if (led_matrix_init(&led_matrix) != ESP_OK) {
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(TAG, "Система инициализирована");

	while (1) {
		if (mpu6050_read_accelerometer(&accel_data) == ESP_OK) {
			calculate_tilt_angels(&accel_data, &pitch, &roll);
			intensity = calculate_tilt_intensity(&accel_data);

			ESP_LOGI(
				TAG, "Углы: pitch=%.2f, roll=%.2f, intensity=%.2f",
				pitch, roll, intensity
			);
			ESP_LOGI(
				TAG, "Ускорение: X=%.3f, Y=%.3f, Z=%.3f",
				accel_data.x, accel_data.y, accel_data.z
			);

			draw_square_with_fixed_corner(&led_matrix, pitch, roll, intensity);
			vTaskDelay(pdMS_TO_TICKS(100));
			esp_err_t update_err = led_matrix_update(&led_matrix);
			if (update_err != ESP_OK) {
				ESP_LOGE(TAG, "Ошибка обновления матрицы: %s", esp_err_to_name(update_err));
				vTaskDelay(pdMS_TO_TICKS(50));
			}
		} else {
			ESP_LOGE(TAG, "Ошибка чтения данных с акселерометра");
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void app_main(void)
{
	esp_err_t ret = i2c_master_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Ошибка инициализации I2C: %s", esp_err_to_name(ret));
		return;
	}

	ret = mpu6050_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Ошибка инициализации MPU6050: %s", esp_err_to_name(ret));
		return;
	}

	xTaskCreate(main_task, "main_task", 4096 * 2, NULL, 5, NULL);
}