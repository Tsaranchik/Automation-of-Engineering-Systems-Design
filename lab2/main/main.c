#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/soc_caps.h"

#define MIC_ADC_CH		ADC_CHANNEL_6
#define LED_GPIO		GPIO_NUM_2
#define SAMPLE_RATE		2000
#define BUFFER_SIZE		512
#define AMP_THRESHOLD		210
#define CALIBRATION_SAMPLES	2000
#define FILTER_ORDER		2

static const char *TAG = "lab2";

static int32_t dc_offset = 0;
static adc_oneshot_unit_handle_t adc_handle = NULL;

static int16_t buffer1[BUFFER_SIZE];
static int16_t buffer2[BUFFER_SIZE];
static int16_t *current_buffer = buffer1;
static volatile int16_t *ready_buffer = NULL;
static uint32_t buffer_index = 0;
static uint32_t buffer_sequence = 0;

static SemaphoreHandle_t buffer_semaphore = NULL;
static SemaphoreHandle_t data_mutex = NULL;

static uint32_t below_threshold_count = 0;
static uint32_t below_threshold_max = 0;
static uint32_t total_samples_processed = 0;

static const float hpf_b[] = {0.5690, -1.1380, 0.5690};
static const float hpf_a[] = {1.0, -0.9428, 0.3333};

static float x_buf[FILTER_ORDER + 1] = {0};
static float y_buf[FILTER_ORDER + 1] = {0};

static void init_gpio(void) {
	gpio_reset_pin(LED_GPIO);
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_GPIO, 0);
}

static void init_adc_oneshot(void) {
	adc_oneshot_unit_init_cfg_t init_config = {
		.unit_id = ADC_UNIT_1,
		.clk_src = ADC_RTC_CLK_SRC_DEFAULT,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};

	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

	adc_oneshot_chan_cfg_t config = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_12,
	};

	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MIC_ADC_CH, &config));
}

static void calibrate_dc_offset(void) {
	ESP_LOGI(TAG, "Calibrating DC offset... (%d samples)", CALIBRATION_SAMPLES);
	
	int64_t sum = 0;
	for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
		int raw;
		adc_oneshot_read(adc_handle, MIC_ADC_CH, &raw);
		sum += raw;
		
		if (i % 100 == 0) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}
	
	dc_offset = sum / CALIBRATION_SAMPLES;
	ESP_LOGI(TAG, "DC offset = %d (0x%03X)", dc_offset, dc_offset);
	
	memset(x_buf, 0, sizeof(x_buf));
	memset(y_buf, 0, sizeof(y_buf));
}

static float highpass_filter(float input) {
	for (int i = FILTER_ORDER; i > 0; i--) {
		x_buf[i] = x_buf[i - 1];
		y_buf[i] = y_buf[i - 1];
	}
	
	x_buf[0] = input;
	
	float output = hpf_b[0] * x_buf[0];
	for (int i = 1; i <= FILTER_ORDER; i++) {
		output += hpf_b[i] * x_buf[i];
		output -= hpf_a[i] * y_buf[i];
	}
	
	y_buf[0] = output;
	
	return output;
}

static float detect_amplitude(const int16_t *buffer, uint32_t size) {
	float max_amplitude = 0;
	
	for (uint32_t i = 0; i < size; i++) {
		float filtered = highpass_filter((float)(buffer[i] - dc_offset));
		float abs_val = fabsf(filtered);
		
		if (abs_val > max_amplitude) {
			max_amplitude = abs_val;
		}
	}
	
	return max_amplitude;
}

static bool IRAM_ATTR timer_callback(
	gptimer_handle_t timer,
	const gptimer_alarm_event_data_t *edata,
	void *user_data
) {
	int raw;
	adc_oneshot_read(adc_handle, MIC_ADC_CH, &raw);
	
	current_buffer[buffer_index] = (int16_t)(raw - dc_offset);
	buffer_index++;
	
	if (buffer_index >= BUFFER_SIZE) {
		if (current_buffer == buffer1) {
			ready_buffer = buffer1;
			current_buffer = buffer2;
		} else {
			ready_buffer = buffer2;
			current_buffer = buffer1;
		}
		
		buffer_index = 0;
		buffer_sequence++;
		
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(buffer_semaphore, &xHigherPriorityTaskWoken);
		
		if (xHigherPriorityTaskWoken) {
			portYIELD_FROM_ISR();
		}
	}
	
	return pdTRUE;
}

static void processing_task(void *arg) {
	ESP_LOGI(TAG, "Processing task started");
	
	uint32_t local_below_count = 0;
	
	while (1) {
		if (xSemaphoreTake(buffer_semaphore, portMAX_DELAY) == pdTRUE) {
			if (ready_buffer != NULL) {
				int16_t *buffer_to_process = (int16_t *)ready_buffer;
				ready_buffer = NULL;
				
				float amplitude = detect_amplitude(buffer_to_process, BUFFER_SIZE);
				
				xSemaphoreTake(data_mutex, portMAX_DELAY);
				
				if (amplitude < AMP_THRESHOLD) {
					local_below_count += BUFFER_SIZE;
				} else {
					if (local_below_count > below_threshold_max) {
						below_threshold_max = local_below_count;
					}
					local_below_count = 0;
				}
				
				below_threshold_count = local_below_count;
				total_samples_processed += BUFFER_SIZE;
				
				gpio_set_level(LED_GPIO, (amplitude > AMP_THRESHOLD) ? 1 : 0);
				
				xSemaphoreGive(data_mutex);
				
				if (buffer_sequence % 10 == 0) {
					ESP_LOGI(TAG, "Buffer %lu: Amplitude=%.2f, Below threshold=%lu samples (%.2f sec)",
						 buffer_sequence, amplitude, below_threshold_count,
						 below_threshold_count / (float)SAMPLE_RATE);
				}
			}
		}
	}
}

static void init_gptimer(void) {
	gptimer_handle_t gptimer = NULL;
	gptimer_config_t timer_config = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1000000,
	};

	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

	gptimer_alarm_config_t alarm_config = {
		.alarm_count = 1000000 / SAMPLE_RATE,
		.reload_count = 0,
		.flags.auto_reload_on_alarm = true,
	};

	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

	gptimer_event_callbacks_t cbs = {
		.on_alarm = timer_callback,
	};

	ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(gptimer));
	ESP_ERROR_CHECK(gptimer_start(gptimer));

	ESP_LOGI(TAG, "GPTimer started at %d Hz", SAMPLE_RATE);
}

static void stats_task(void *arg) {
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(5000));
		
		xSemaphoreTake(data_mutex, portMAX_DELAY);
		
		uint32_t current_below = below_threshold_count;
		uint32_t current_max = below_threshold_max;
		uint32_t current_total = total_samples_processed;
		
		xSemaphoreGive(data_mutex);
		
		float time_below_sec = current_below / (float)SAMPLE_RATE;
		float time_max_sec = current_max / (float)SAMPLE_RATE;
		
		printf("\n");
		ESP_LOGI(TAG, "=== STATISTICS ===");
		ESP_LOGI(TAG, "Total samples processed: %lu", current_total);
		ESP_LOGI(TAG, "Current time below threshold: %.2f sec", time_below_sec);
		ESP_LOGI(TAG, "Maximum time below threshold: %.2f sec", time_max_sec);
		ESP_LOGI(TAG, "Sampling rate: %d Hz", SAMPLE_RATE);
		ESP_LOGI(TAG, "Buffer size: %d samples", BUFFER_SIZE);
		ESP_LOGI(TAG, "========================\n");
	}
}

void app_main(void) {
	ESP_LOGI(TAG, "=== Starting HPF system with double buffering ===");
	
	init_gpio();
	init_adc_oneshot();
	calibrate_dc_offset();
	
	buffer_semaphore = xSemaphoreCreateBinary();
	data_mutex = xSemaphoreCreateMutex();
	
	if (buffer_semaphore == NULL || data_mutex == NULL) {
		ESP_LOGE(TAG, "Failed to create synchronization objects!");
		return;
	}
	
	xTaskCreate(processing_task, "processing_task", 8192, NULL, 3, NULL);
	xTaskCreate(stats_task, "stats_task", 8192, NULL, 1, NULL);
	init_gptimer();
	
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}