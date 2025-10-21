#ifndef MOTOR_CONTROL_FILE_H
#define MOTOR_CONTROL_FILE_H
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "lib_ee152.h"


int enable_gpio_output_motor(GPIO_TypeDef *GPIOX,int pin);
int set_pin(GPIO_TypeDef *GPIOX, int pin, int high);
int get_GPIO_input(GPIO_TypeDef *GPIOX, int pin);
int enable_pwm();
int change_duty(int channel,int duty);
#endif