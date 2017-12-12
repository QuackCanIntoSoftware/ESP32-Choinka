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
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

//#include "datatypes.h"

#define DR_REG_RNG_BASE			0x3ff75144
//#define UINT32_MAX 				4294967296 

//#define LED_TOP			14   //Set GPIO 14 as PWM2B
//#define LED_HIGH_0 		15   //Set GPIO 15 as PWM2A
//#define LED_HIGH_1 		16   //Set GPIO 16 as PWM1B
//#define LED_MID_0		17   //Set GPIO 17 as PWM1A
//#define LED_MID_1		18   //Set GPIO 18 as PWM0B
//#define LED_MID_2		19   //Set GPIO 19 as PWM0A
//#define LED_MID_3		21   //Set GPIO 19 as PWM0A
//#define LED_LOW_0		22   //Set GPIO 18 as PWM0B
//#define LED_LOW_1		23   //Set GPIO 17 as PWM1A
//#define LED_LOW_2		25   //Set GPIO 16 as PWM1B
//#define LED_LOW_3		26   //Set GPIO 15 as PWM2A
//#define LED_LOW_4		27   //Set GPIO 14 as PWM2B


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
	mcpwm_unit_t 		unit;
	mcpwm_io_signals_t 	channel;
	mcpwm_timer_t		timer;
	mcpwm_operator_t 	operator;
} PwmConfig;


static void gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(LED_TOP_UNIT, LED_TOP_CHANNEL, LED_TOP);
	
    mcpwm_gpio_init(LED_HIGH_0_UNIT, LED_HIGH_0_CHANNEL, LED_HIGH_0);
    mcpwm_gpio_init(LED_HIGH_1_UNIT, LED_HIGH_1_CHANNEL, LED_HIGH_1);
	
    mcpwm_gpio_init(LED_MID_0_UNIT, LED_MID_0_CHANNEL, LED_MID_0);
    mcpwm_gpio_init(LED_MID_1_UNIT, LED_MID_1_CHANNEL, LED_MID_1);
    mcpwm_gpio_init(LED_MID_2_UNIT, LED_MID_2_CHANNEL, LED_MID_2);
    mcpwm_gpio_init(LED_MID_3_UNIT, LED_MID_3_CHANNEL, LED_MID_3);
	
    mcpwm_gpio_init(LED_LOW_0_UNIT, LED_LOW_0_CHANNEL, LED_LOW_0);
    mcpwm_gpio_init(LED_LOW_1_UNIT, LED_LOW_1_CHANNEL, LED_LOW_1);
    mcpwm_gpio_init(LED_LOW_2_UNIT, LED_LOW_2_CHANNEL, LED_LOW_2);
    mcpwm_gpio_init(LED_LOW_3_UNIT, LED_LOW_3_CHANNEL, LED_LOW_3);
    mcpwm_gpio_init(LED_LOW_4_UNIT, LED_LOW_4_CHANNEL, LED_LOW_4);
	
    printf("initializing mcpwm gpio COMPLETED\n");
/*	
	
	
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
	//
    //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, GPIO_PWM20A_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, GPIO_PWM20B_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_PWM21A_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, GPIO_PWM21B_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, GPIO_PWM22A_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2B, GPIO_PWM22B_OUT);
	
	
	//ooooold
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM3A, GPIO_PWM3A_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM3B, GPIO_PWM3B_OUT);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_0, GPIO_SYNC0_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_1, GPIO_SYNC1_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_2, GPIO_SYNC2_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_0, GPIO_FAULT0_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_1, GPIO_FAULT1_IN);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_2, GPIO_FAULT2_IN);
	*/
}

void pwmInitialize()
{
    printf("initializing mcpwm pwm...\n");
	mcpwm_config_t pwm_config;
	
    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 50.0;       //duty cycle of PWMxA = 0%
    pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 0%
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
    gp.pin_bit_mask = GPIO_SEL_12;
    gpio_config(&gp);
    while (1) {
        //here the period of test signal is 20ms
        gpio_set_level(GPIO_NUM_12, 1); //Set high
        vTaskDelay(1000);             //delay of 10ms
        gpio_set_level(GPIO_NUM_12, 0); //Set low
        vTaskDelay(1000);         //delay of 10ms
		
		uint32_t randomNumber = getRandomNumber(100);
		printf("Random number %zu\n", randomNumber);
		
    }
}

void task_blinker(void *arg)
{
	/*
	mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 70.0;       //duty cycle of PWMxA = 60.0%
    pwm_config.cmpr_b = 40.0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
	
    pwm_config.frequency = 10;    //frequency = 1000Hz
    pwm_config.cmpr_a = 70.0;       //duty cycle of PWMxA = 60.0%
    pwm_config.cmpr_b = 40.0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
	
	
	while (1)
	{
		
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 20.0);
        vTaskDelay(2000);         //delay of 10ms
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 80.0);
        vTaskDelay(2000);         //delay of 10ms
		
	}
	*/
    vTaskDelete(NULL);
}

void task_LED(void *arg)
{
	PwmConfig myCfg = *(PwmConfig *) arg;
	printf("TASKUS %d", myCfg.operator);
	
	vTaskDelete(NULL);
	
}

void createLedsTasks()
{	
	PwmConfig cfg;
	
	cfg.unit = LED_TOP_UNIT;
	cfg.channel = LED_TOP_CHANNEL;
	cfg.timer = LED_TOP_TIMER;
	cfg.operator = LED_TOP_OPERATOR;
	xTaskCreate(task_LED, "Test task", 4096, &cfg, 5, NULL);
	
	cfg.unit = LED_HIGH_0_UNIT;
	cfg.channel = LED_HIGH_0_CHANNEL;
	cfg.timer = LED_HIGH_0_TIMER;
	cfg.operator = LED_HIGH_0_OPERATOR;
	xTaskCreate(task_LED, "Test task 2", 4096, &cfg, 5, NULL);
	
}

void task_LED_Top(void *arg)
{
	//mcpwm_config_t pwm_config;
    //pwm_config.frequency = 10000;    //frequency = 1000Hz
    //pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA = 60.0%
    //pwm_config.cmpr_b = 0.0;       //duty cycle of PWMxb = 50.0%
    //pwm_config.counter_mode = MCPWM_UP_COUNTER;
    //pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    //mcpwm_init(LED_TOP_UNIT, LED_TOP_TIMER, &pwm_config);   //Configure PWM2A & PWM2B with above settings
	
	float duty = 0;
	float step = 0.01 + getRandomNumber(0.1);
	
	while(1)
	{
		if (duty <=0)
		{
			duty = 100.0;
			step = 0.01 + getRandomNumber(0.1);
			//if (step > 0.09)
			//{
			//	step = 0.01;
			//}
			//else
			//{
			//	step += 0.01;
			//}
			printf("Loop %2.2f %2.2f \n", duty, step);
		}
		else
		{
			duty -= step;
		}
		mcpwm_set_duty(LED_TOP_UNIT, LED_TOP_TIMER, LED_TOP_UNIT, duty);
		vTaskDelay(1);
	}
	
    vTaskDelete(NULL);
}

void app_main()
{
	
    printf("CHOINKA INZYNIERSKA :D");
	
	gpio_initialize();
	pwmInitialize();
	
    //cap_queue = xQueueCreate(1, sizeof(capture)); //comment if you don't want to use capture module
    //xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL);  //comment if you don't want to use capture module
	createLedsTasks();
    xTaskCreate(task_alive, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module
	xTaskCreate(task_blinker, "task blinker", 4096, NULL, 5, NULL);
	xTaskCreate(task_LED_Top, "task blinker", 4096, NULL, 5, NULL);
    //xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
}
