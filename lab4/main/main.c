#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define ENCODER_CLK_GPIO	GPIO_NUM_2
#define ENCODER_DT_GPIO		GPIO_NUM_3
#define ENCODER_SW_GPIO		GPIO_NUM_13

#define STEPPER_A_PLUS		GPIO_NUM_11
#define STEPPER_A_MINUS		GPIO_NUM_10
#define STEPPER_B_PLUS		GPIO_NUM_25
#define STEPPER_B_MINUS		GPIO_NUM_12

#define PCNT_UNIT		PCNT_UNIT_0
#define NUM_PROFILES		3
#define LONG_PRESS_TIME_MS	3000
#define DEBOUNCE_TIME_US	1000

#define INVERT_ENCODER		1

static const char *TAG = "lab4";

typedef struct {
	float speed;
	int16_t min_speed;
	int16_t max_speed;
	char name[20];
} motor_profile_t;

static motor_profile_t profiles[NUM_PROFILES] = {
	{20.0, -100, 100, "Profile 1"},
	{50.0, -150, 150, "Profile 2"},
	{-30.0, -80, 80, "Profile 3"}
};

static int current_profile = 0;
static bool motor_enabled = true;
static volatile bool button_state = false;
static volatile uint64_t button_press_time = 0;
static volatile bool long_press_detected = false;
static volatile bool profile_changed = false;

static float current_speed = 0;
static int current_step = 0;
static uint64_t last_step_time = 0;

static int16_t encoder_accumulator = 0;
static uint64_t last_encoder_time = 0;

static const uint8_t step_sequence[4][4] = {
	{1, 0, 0, 1},
	{1, 0, 1, 0},
	{0, 1, 1, 0},
	{0, 1, 0, 1}
};

static void init_encoder_pcnt(void) {
	pcnt_config_t pcnt_config_a = {
		.pulse_gpio_num = ENCODER_CLK_GPIO,
		.ctrl_gpio_num = ENCODER_DT_GPIO,
		.lctrl_mode = PCNT_MODE_REVERSE,
		.hctrl_mode = PCNT_MODE_KEEP,
		.pos_mode = PCNT_COUNT_INC,
		.neg_mode = PCNT_COUNT_DEC,
		.counter_h_lim = 100,
		.counter_l_lim = -100,
		.unit = PCNT_UNIT,
		.channel = PCNT_CHANNEL_0,
	};
	
	pcnt_config_t pcnt_config_b = {
		.pulse_gpio_num = ENCODER_DT_GPIO,
		.ctrl_gpio_num = ENCODER_CLK_GPIO,
		.lctrl_mode = PCNT_MODE_REVERSE,
		.hctrl_mode = PCNT_MODE_KEEP,
		.pos_mode = PCNT_COUNT_DEC,
		.neg_mode = PCNT_COUNT_INC,
		.counter_h_lim = 100,
		.counter_l_lim = -100,
		.unit = PCNT_UNIT,
		.channel = PCNT_CHANNEL_1,
	};
	
	ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config_a));
	ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config_b));
	
	pcnt_set_filter_value(PCNT_UNIT, 2000);
	pcnt_filter_enable(PCNT_UNIT);
	
	pcnt_counter_pause(PCNT_UNIT);
	pcnt_counter_clear(PCNT_UNIT);
	pcnt_counter_resume(PCNT_UNIT);
	
	ESP_LOGI(TAG, "PCNT энкодера инициализирован с фильтром 2000 тактов");
	ESP_LOGI(TAG, "Инверсия энкодера: %s", INVERT_ENCODER ? "ВКЛЮЧЕНА" : "ВЫКЛЮЧЕНА");
}

static void process_encoder(void) {
	static int16_t last_count = 0;
	int16_t current_count;
	pcnt_get_counter_value(PCNT_UNIT, &current_count);
	
	int16_t diff = current_count - last_count;
	
	if (diff != 0) {
		uint64_t now = esp_timer_get_time();
		
		if (now - last_encoder_time > DEBOUNCE_TIME_US) {
			#if INVERT_ENCODER
				encoder_accumulator -= diff;
			#else
				encoder_accumulator += diff;
			#endif
			
			last_encoder_time = now;
			
			ESP_LOGD(TAG, "Энкодер: diff=%d, accumulator=%d", diff, encoder_accumulator);
		}
		
		last_count = current_count;
		pcnt_counter_clear(PCNT_UNIT);
	}
}

static void IRAM_ATTR button_isr_handler(void *arg) {
	static uint64_t last_isr_time = 0;
	uint64_t now = esp_timer_get_time();
	
	if (now - last_isr_time > 50000) {
		bool current_button_state = (gpio_get_level(ENCODER_SW_GPIO) == 0);
		
		if (current_button_state && !button_state) {
			button_state = true;
			button_press_time = now;
			long_press_detected = false;
			ESP_LOGD(TAG, "Кнопка нажата (ISR)");
		} else if (!current_button_state && button_state) {
			button_state = false;
			uint64_t press_duration = (now - button_press_time) / 1000;
			
			if (!long_press_detected && press_duration < LONG_PRESS_TIME_MS) {
				profile_changed = true;
				ESP_LOGD(TAG, "Короткое нажатие, duration=%llu мс", press_duration);
			}
			ESP_LOGD(TAG, "Кнопка отпущена");
		}
		
		last_isr_time = now;
	}
}

static void init_gpio(void) {
	gpio_config_t stepper_config = {
		.pin_bit_mask = (1ULL << STEPPER_A_PLUS) |
				(1ULL << STEPPER_A_MINUS) |
				(1ULL << STEPPER_B_PLUS) |
				(1ULL << STEPPER_B_MINUS),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&stepper_config));
	
	gpio_config_t button_config = {
		.pin_bit_mask = (1ULL << ENCODER_SW_GPIO),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_ANYEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&button_config));
	
	ESP_ERROR_CHECK(gpio_install_isr_service(0));
	ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_SW_GPIO, button_isr_handler, NULL));
	
	ESP_LOGI(TAG, "GPIO инициализированы");
}

static void set_motor_step(int step) {
	gpio_set_level(STEPPER_A_PLUS, step_sequence[step][0]);
	gpio_set_level(STEPPER_A_MINUS, step_sequence[step][1]);
	gpio_set_level(STEPPER_B_PLUS, step_sequence[step][2]);
	gpio_set_level(STEPPER_B_MINUS, step_sequence[step][3]);
}

static void perform_step(void) {
	if (!motor_enabled || fabsf(current_speed) < 0.1f) {
		return;
	}
	
	uint64_t current_time = esp_timer_get_time();
	float step_interval = 1000000.0f / fabsf(current_speed);
	
	if (current_time - last_step_time >= (uint64_t)step_interval) {
		if (current_speed > 0) {
			current_step = (current_step + 1) % 4;
		} else {
			current_step = (current_step - 1 + 4) % 4;
		}
		
		set_motor_step(current_step);
		last_step_time = current_time;
	}
}

static void update_speed_from_encoder(void) {
	process_encoder();
	
	if (abs(encoder_accumulator) >= 4) {
		int clicks = encoder_accumulator / 4;
		
		float speed_change = clicks * 5.0f;
		float new_speed = profiles[current_profile].speed + speed_change;
		
		if (new_speed > profiles[current_profile].max_speed) {
			new_speed = profiles[current_profile].max_speed;
		}
		if (new_speed < profiles[current_profile].min_speed) {
			new_speed = profiles[current_profile].min_speed;
		}
		
		if (fabsf(new_speed - profiles[current_profile].speed) > 0.1f) {
			profiles[current_profile].speed = new_speed;
			current_speed = profiles[current_profile].speed;
			
			ESP_LOGI(TAG, "Энкодер: %d щелчков, скорость профиля %d: %.1f шаг/сек", 
					clicks, current_profile + 1, current_speed);
		}
		
		encoder_accumulator = 0;
	}
}

static void switch_profile(void) {
	current_profile = (current_profile + 1) % NUM_PROFILES;
	current_speed = profiles[current_profile].speed;
	
	ESP_LOGI(TAG, "=== Переключен профиль на %s ===", profiles[current_profile].name);
	ESP_LOGI(TAG, "Скорость: %.1f шаг/сек", current_speed);
	ESP_LOGI(TAG, "Диапазон: от %d до %d шаг/сек", 
			profiles[current_profile].min_speed,
			profiles[current_profile].max_speed);
}

static void toggle_motor(void) {
	motor_enabled = !motor_enabled;
	
	if (!motor_enabled) {
		gpio_set_level(STEPPER_A_PLUS, 0);
		gpio_set_level(STEPPER_A_MINUS, 0);
		gpio_set_level(STEPPER_B_PLUS, 0);
		gpio_set_level(STEPPER_B_MINUS, 0);
		ESP_LOGI(TAG, "Двигатель ВЫКЛЮЧЕН");
	} else {
		ESP_LOGI(TAG, "Двигатель ВКЛЮЧЕН");
	}
}

static void control_task(void *arg) {
	uint64_t last_button_check_time = 0;
	
	while (1) {
		uint64_t current_time = esp_timer_get_time() / 1000;
		
		if (current_time - last_button_check_time >= 100) {
			if (button_state && !long_press_detected) {
				uint64_t press_duration = current_time - (button_press_time / 1000);
				
				if (press_duration >= LONG_PRESS_TIME_MS) {
					long_press_detected = true;
					toggle_motor();
					ESP_LOGI(TAG, "Длинное нажатие обработано (%llu мс)", press_duration);
				}
			}
			last_button_check_time = current_time;
		}
		
		if (profile_changed) {
			profile_changed = false;
			switch_profile();
			
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		
		update_speed_from_encoder();
		
		perform_step();

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void app_main(void) {
	init_gpio();
	init_encoder_pcnt();
	
	current_speed = profiles[current_profile].speed;
	
	ESP_LOGI(TAG, "Текущий профиль: %s", profiles[current_profile].name);
	ESP_LOGI(TAG, "Начальная скорость: %.1f шаг/сек", current_speed);
	ESP_LOGI(TAG, "Диапазон скорости: [%d, %d]", 
		 profiles[current_profile].min_speed,
		 profiles[current_profile].max_speed);
	
	xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
}