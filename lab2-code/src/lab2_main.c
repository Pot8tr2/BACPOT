
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "lib_ee152.h"
#include "stm32l4xx_hal.h"
#include "interupts_file.h"

int speed=0;
char command=0;


volatile uint32_t system_ticks = 0;

// config the system time 
void SysTick_Init(uint32_t tick_hz)
{
    uint32_t reload = (SystemCoreClock / tick_hz) - 1;
    SysTick->LOAD  = reload;
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}



void SysTick_Handler(void)
{
    system_ticks++;
}

uint64_t cur_time_ms(void){
    return system_ticks;
}


int enable_gpio_output_motor(GPIO_TypeDef *GPIOX,int pin){
    RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN|RCC_AHB2ENR_GPIOCEN;

    unsigned int b0 = 1<<(pin<<1);	// bit 0 of this pin's two-bit field
    GPIOX->MODER &= ~(b0<<1);		// Clear the field's bit 1
    GPIOX->MODER |= b0;	

    GPIOX->OSPEEDR |=   0x2<<(2*pin);

    // Both PA2 and PA15 are in pullup/down mode 01, which means pull-up only.
    // This is arguably not needed. During normal operation, we're doing push-
    // pull drive and so don't need a pullup or pulldown. Some people like a
    // pullup to stop the data line from bouncing during reset before any MCU
    // drives it -- but our pullup won't turn on until this code runs, anyway!
    GPIOX->PUPDR   &= ~((0x3<<(2*pin)));	// Clear bits
    GPIOX->PUPDR   |=   (0x1<<(2*pin));	// Set each to 01.
    // Both PA2 and PA15 are push-pull (which is the reset default, anyway).
    GPIOX->OTYPER  &= ~((0x1<<(pin)));	// Clear bits
    // set it to default high
    GPIOX->ODR|= 1<<pin;
    return 0;
}

int set_pin(GPIO_TypeDef *GPIOX, int pin, int high){
    if(high)GPIOX->ODR|= 1<<pin;
    else{
        GPIOX->ODR&=~(1<<pin);
    }
    return 0;
}

int get_GPIO_input(GPIO_TypeDef *GPIOX, int pin){
    return ((GPIOX->IDR)&(1<<pin))>>pin;
}



int enable_pwm(){
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN);
    // enable the tim clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    // set up the pwm pin mode.
    
    // enable the pwm mode
    set_gpio_alt_func(GPIOA,9,1);
    set_gpio_alt_func(GPIOA,10,1);
    set_gpio_alt_func(GPIOA,11,1);
    set_gpio_alt_func(GPIOA,8,1);

    //config to high speed
    GPIOA->OSPEEDR |=   0x3<<(GPIO_OSPEEDR_OSPEED9_Pos)|0x3<<(GPIO_OSPEEDR_OSPEED10_Pos)|0x3<<(GPIO_OSPEEDR_OSPEED11_Pos)|0x3<<(GPIO_OSPEEDR_OSPEED8_Pos);
    // Set PA9 in pullup/down mode 01, which means pull-up only.
    GPIOA->PUPDR   &= ~(0x3<<(GPIO_PUPDR_PUPD9_Pos)|0x3<<(GPIO_PUPDR_PUPD10_Pos)|0x3<<(GPIO_PUPDR_PUPD8_Pos)|0x3<<(GPIO_PUPDR_PUPD11_Pos));	
    // Set PA9 as push-pull (which is the reset default, anyway).
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);


    TIM1->PSC = 7999;     
    TIM1->ARR = 999; 
    TIM1->CR1 &= ~TIM_CR1_DIR;

    // clear this
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M|TIM_CCMR1_OC1M);
    TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M|TIM_CCMR2_OC4M);

    // enable this
    TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos)|(0x6 << TIM_CCMR1_OC1M_Pos);
    TIM1->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos)|(0x6 << TIM_CCMR2_OC4M_Pos);

    // enable preload for channel
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE | TIM_CCMR2_OC3PE;

    TIM1->CCR2 = 500;
    TIM1->CCR1 =500;
    TIM1->CCR3 =500;
    TIM1->CCR4=500;

    TIM1->CCER |= TIM_CCER_CC2E|TIM_CCER_CC1E|TIM_CCER_CC3E|TIM_CCER_CC4E;

    TIM1->BDTR |= TIM_BDTR_MOE;

    TIM1->CR1 |= TIM_CR1_CEN;  
    return 0;
}

int change_duty(int channel,int duty){
    if(channel==1)TIM1->CCR1=duty;
    else if(channel==2)TIM1->CCR2=duty;
    else if(channel==3)TIM1->CCR3=duty;
    else if(channel==4)TIM1->CCR4=duty;
    return 0;
}


int control_motors(char command, int speed){
    speed=500;
    if(command=='w'){
        // change the pwm duty cycle of the motors
        set_pin(GPIOB, 1, 1);
        set_pin(GPIOB, 0, 1);
        change_duty(1,0);
        change_duty(2,speed);
        change_duty(3,speed);
        change_duty(4,0);
    }else if (command=='s'){
        // change the pwm duty cycle of the motors
        set_pin(GPIOB, 1, 0);
        set_pin(GPIOB, 0, 0);
        change_duty(1,0);
        change_duty(2,speed);
        change_duty(3,speed);
        change_duty(4,0);
    }
    else if (command=='d'){
        set_pin(GPIOB, 5, 1);
        set_pin(GPIOB, 4, 1);
        change_duty(1,speed);
        change_duty(2,0);
        change_duty(3,0);
        change_duty(4,speed);
    }
    else if (command=='a'){
        set_pin(GPIOB, 5, 0);
        set_pin(GPIOB, 4,0);
        change_duty(1,speed);
        change_duty(2,0);
        change_duty(3,0);
        change_duty(4,speed);
    }
    else{
        change_duty(1,0);
        change_duty(2,0);
        change_duty(3,0);
        change_duty(4,0);
    }
    return 0;
}

int main()
{

    // The default clock is 4MHz, which is more than fast enough for LEDs.
    // clock_setup_16MHz();		// 16 MHz
    clock_setup_80MHz(); // 80 MHz
    serial_begin(USART2);
    // init the hal skip this?
    enable_pwm();
    // init the gpio for the motors
    enable_gpio_output_motor(GPIOB,1);
    enable_gpio_output_motor(GPIOB,0);
    enable_gpio_output_motor(GPIOB,4);
    enable_gpio_output_motor(GPIOB,5);

    SysTick_Init(1000);
    enable_input_and_intetupts();

    // Create tasks.
    char prompt[] = "R=red, G=green, B=both, N=neither: \n\0";
    

    serial_write(USART2, prompt);
    change_duty(2,500);

    for(;;){
        speed=RXBUFFER2[1];
        command=RXBUFFER2[0];
        // 
        // control_motors(command,speed<<1);
        // control_motors(command, 500);
        for(int i=0; i<10000;i++){
            int t=0;
        }
        // memcpy(prompt,(int*)(TIM1->CNT),strlen(prompt));
        // serial_write(USART2, prompt);
        
    }

}
