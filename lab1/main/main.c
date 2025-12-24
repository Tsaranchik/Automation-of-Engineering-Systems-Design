#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED_PIN			GPIO_NUM_2
#define BUTTON_PIN_1		GPIO_NUM_4
#define BUTTON_PIN_2		GPIO_NUM_5

#define DEBOUNCE_TIME_MS 	50

static QueueHandle_t gpio_evt_queue = NULL;
static volatile uint32_t last_press_time1 = 0;
static volatile uint32_t last_press_time2 = 0;
static bool led_state = false;

typedef struct {
	uint32_t pin;
} button_event_t;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t)arg;
	uint32_t now = esp_timer_get_time() / 1000;

	if (gpio_num == BUTTON_PIN_1) {
		if (now - last_press_time1 < DEBOUNCE_TIME_MS) return;

		last_press_time1 = now;

		button_event_t evt = {
			.pin = gpio_num
		};
		xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
	} else if (gpio_num == BUTTON_PIN_2) {
		if (now - last_press_time2 < DEBOUNCE_TIME_MS) return;

		last_press_time2 = now;

		button_event_t evt = {
			.pin = gpio_num
		};
		xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
	}
}

static void button_task(void* arg)
{
	button_event_t evt;
	
	while(1) {
		if (xQueueReceive(gpio_evt_queue, &evt, portMAX_DELAY)) {
			if (evt.pin == BUTTON_PIN_1) {
				printf("BUTTON 1\n");
				led_state = true;
				gpio_set_level(LED_PIN, led_state);
			} else if (evt.pin == BUTTON_PIN_2) {
				printf("BUTTON 2\n");
				led_state = false;
				gpio_set_level(LED_PIN, led_state);
			}
		}
	}
}

void app_main(void)
{
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_PIN, led_state);

	gpio_set_direction(BUTTON_PIN_1, GPIO_MODE_INPUT);
	gpio_set_direction(BUTTON_PIN_2, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_PIN_1, GPIO_FLOATING);
	gpio_set_pull_mode(BUTTON_PIN_2, GPIO_FLOATING);

	gpio_evt_queue = xQueueCreate(20, sizeof(button_event_t));

	gpio_set_intr_type(BUTTON_PIN_1, GPIO_INTR_NEGEDGE);
	gpio_set_intr_type(BUTTON_PIN_2, GPIO_INTR_NEGEDGE);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(BUTTON_PIN_1, gpio_isr_handler, (void*)BUTTON_PIN_1);
	gpio_isr_handler_add(BUTTON_PIN_2, gpio_isr_handler, (void*)BUTTON_PIN_2);

	xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}