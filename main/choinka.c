/* MCPWM basic config example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use each submodule of MCPWM unit.
 * The example can't be used without modifying the code first.
 * Edit the macros at the top of mcpwm_example_basic_config.c to enable/disable the submodules which are used in the example.
 */

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define DR_REG_RNG_BASE			0x3ff75144

#define LEDS_NUMBER 12


#define LED_TOP				14   //Set GPIO 14 as PWM2B
#define LED_TOP_UNIT		MCPWM_UNIT_0
#define LED_TOP_CHANNEL		MCPWM0A
#define LED_TOP_TIMER		MCPWM_TIMER_0
#define LED_TOP_OPERATOR	MCPWM_OPR_A

#define LED_HIGH_0			15   //Set GPIO 14 as PWM2B
#define LED_HIGH_0_UNIT		MCPWM_UNIT_0
#define LED_HIGH_0_CHANNEL	MCPWM0B
#define LED_HIGH_0_TIMER	MCPWM_TIMER_0
#define LED_HIGH_0_OPERATOR	MCPWM_OPR_B
            
#define LED_HIGH_1			16   //Set GPIO 14 as PWM2B
#define LED_HIGH_1_UNIT		MCPWM_UNIT_0
#define LED_HIGH_1_CHANNEL	MCPWM1A
#define LED_HIGH_1_TIMER	MCPWM_TIMER_1
#define LED_HIGH_1_OPERATOR	MCPWM_OPR_A

#define LED_MID_0			17   //Set GPIO 14 as PWM2B
#define LED_MID_0_UNIT		MCPWM_UNIT_0
#define LED_MID_0_CHANNEL	MCPWM1B
#define LED_MID_0_TIMER		MCPWM_TIMER_1
#define LED_MID_0_OPERATOR	MCPWM_OPR_B
            
#define LED_MID_1			18   //Set GPIO 14 as PWM2B
#define LED_MID_1_UNIT		MCPWM_UNIT_0
#define LED_MID_1_CHANNEL	MCPWM2A
#define LED_MID_1_TIMER		MCPWM_TIMER_2
#define LED_MID_1_OPERATOR	MCPWM_OPR_A
            
#define LED_MID_2			19   //Set GPIO 14 as PWM2B
#define LED_MID_2_UNIT		MCPWM_UNIT_0
#define LED_MID_2_CHANNEL	MCPWM2B
#define LED_MID_2_TIMER		MCPWM_TIMER_2
#define LED_MID_2_OPERATOR	MCPWM_OPR_B
            
            
#define LED_MID_3			21   //Set GPIO 14 as PWM2B
#define LED_MID_3_UNIT		MCPWM_UNIT_1
#define LED_MID_3_CHANNEL	MCPWM0A
#define LED_MID_3_TIMER		MCPWM_TIMER_0
#define LED_MID_3_OPERATOR	MCPWM_OPR_A

#define LED_LOW_0			22   //Set GPIO 14 as PWM2B
#define LED_LOW_0_UNIT		MCPWM_UNIT_1
#define LED_LOW_0_CHANNEL	MCPWM0B
#define LED_LOW_0_TIMER		MCPWM_TIMER_0
#define LED_LOW_0_OPERATOR	MCPWM_OPR_B
            
#define LED_LOW_1			23   //Set GPIO 14 as PWM2B
#define LED_LOW_1_UNIT		MCPWM_UNIT_1
#define LED_LOW_1_CHANNEL	MCPWM1A
#define LED_LOW_1_TIMER		MCPWM_TIMER_1
#define LED_LOW_1_OPERATOR	MCPWM_OPR_A
            
#define LED_LOW_2			25   //Set GPIO 14 as PWM2B
#define LED_LOW_2_UNIT		MCPWM_UNIT_1
#define LED_LOW_2_CHANNEL	MCPWM1B
#define LED_LOW_2_TIMER		MCPWM_TIMER_1
#define LED_LOW_2_OPERATOR	MCPWM_OPR_B
            
#define LED_LOW_3			26   //Set GPIO 14 as PWM2B
#define LED_LOW_3_UNIT		MCPWM_UNIT_1
#define LED_LOW_3_CHANNEL	MCPWM2A
#define LED_LOW_3_TIMER		MCPWM_TIMER_2
#define LED_LOW_3_OPERATOR	MCPWM_OPR_A
            
#define LED_LOW_4			27   //Set GPIO 14 as PWM2B
#define LED_LOW_4_UNIT		MCPWM_UNIT_1
#define LED_LOW_4_CHANNEL	MCPWM2B
#define LED_LOW_4_TIMER		MCPWM_TIMER_2
#define LED_LOW_4_OPERATOR	MCPWM_OPR_B

typedef struct PwmConfig
{
	uint8_t				gpio;
	mcpwm_unit_t 		unit;
	mcpwm_io_signals_t 	channel;
	mcpwm_timer_t		timer;
	mcpwm_operator_t 	operator;
} PwmConfig;

PwmConfig LedsArray[LEDS_NUMBER] = 
{
	{LED_TOP, LED_TOP_UNIT, LED_TOP_CHANNEL, LED_TOP_TIMER, LED_TOP_OPERATOR},
	{LED_HIGH_0, LED_HIGH_0_UNIT, LED_HIGH_0_CHANNEL, LED_HIGH_0_TIMER, LED_HIGH_0_OPERATOR},
	{LED_HIGH_1, LED_HIGH_1_UNIT, LED_HIGH_1_CHANNEL, LED_HIGH_1_TIMER, LED_HIGH_1_OPERATOR},
	{LED_MID_0, LED_MID_0_UNIT, LED_MID_0_CHANNEL, LED_MID_0_TIMER, LED_MID_0_OPERATOR},
	{LED_MID_1, LED_MID_1_UNIT, LED_MID_1_CHANNEL, LED_MID_1_TIMER, LED_MID_1_OPERATOR},
	{LED_MID_2, LED_MID_2_UNIT, LED_MID_2_CHANNEL, LED_MID_2_TIMER, LED_MID_2_OPERATOR},
	{LED_MID_3, LED_MID_3_UNIT, LED_MID_3_CHANNEL, LED_MID_3_TIMER, LED_MID_3_OPERATOR},
	{LED_LOW_0, LED_LOW_0_UNIT, LED_LOW_0_CHANNEL, LED_LOW_0_TIMER, LED_LOW_0_OPERATOR},
	{LED_LOW_1, LED_LOW_1_UNIT, LED_LOW_1_CHANNEL, LED_LOW_1_TIMER, LED_LOW_1_OPERATOR},
	{LED_LOW_2, LED_LOW_2_UNIT, LED_LOW_2_CHANNEL, LED_LOW_2_TIMER, LED_LOW_2_OPERATOR},
	{LED_LOW_3, LED_LOW_3_UNIT, LED_LOW_3_CHANNEL, LED_LOW_3_TIMER, LED_LOW_3_OPERATOR},
	{LED_LOW_4, LED_LOW_4_UNIT, LED_LOW_4_CHANNEL, LED_LOW_4_TIMER, LED_LOW_4_OPERATOR}
};

static void gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
	
	for(int i = 0; i < LEDS_NUMBER; i++)
	{
		mcpwm_gpio_init(LedsArray[i].unit, LedsArray[i].channel, LedsArray[i].gpio);
	}
	
    printf("initializing mcpwm gpio COMPLETED\n");
}

void pwmInitialize()
{
    printf("initializing mcpwm pwm...\n");
	mcpwm_config_t pwm_config;
	
    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA = 0%
    pwm_config.cmpr_b = 0.0;       //duty cycle of PWMxb = 0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    printf("initializing mcpwm pwm COMPLETED\n");
}

double getRandomNumber(double limit)
{
	double number = READ_PERI_REG(DR_REG_RNG_BASE);
	return (number * limit) / UINT32_MAX;
}

void task_alive(void *arg)
{
	printf("Initializing alive signal...");
	gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = GPIO_NUM_12;
    gpio_config(&gp);
    while (1) {
        //here the period of test signal is 20ms
        gpio_set_level(GPIO_NUM_12, 1); //Set high
        vTaskDelay(1000);             //delay of 10ms
        gpio_set_level(GPIO_NUM_12, 0); //Set low
        vTaskDelay(1000);         //delay of 10ms		
    }
}

void task_LED(void *arg)
{
	PwmConfig myCfg = *(PwmConfig *) arg;
	
	float duty = 0;
	float step = 0.2 + getRandomNumber(2);
	
	while(1)
	{
		if (duty <=0)
		{
			duty = 100.0;
			//step = 0.01 + getRandomNumber(0.1);
			step = 0.2 + getRandomNumber(2);
			printf("Loop %2.2f %2.2f \n", duty, step);
		}
		else
		{
			duty -= step;
		}
		mcpwm_set_duty(myCfg.unit, myCfg.timer, myCfg.operator, duty);
		vTaskDelay(20);
	}
	
    vTaskDelete(NULL);	
}

void createLedsTasks()
{	

	for(int i = 0; i < LEDS_NUMBER; i++)
	{
		xTaskCreate(task_LED, "LED TASK", 4096, &LedsArray[i], 5, NULL);
	}	
}

void app_main()
{
    printf("CHOINKA INDZYNIERSKA :D");
	gpio_initialize();
	pwmInitialize();
	xTaskCreate(task_alive, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module
	createLedsTasks();
}
