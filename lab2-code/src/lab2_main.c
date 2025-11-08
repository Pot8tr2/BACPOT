
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "lib_ee152.h"
#include "stm32l4xx_hal.h"
#include "interupts_file.h"
#include "motor_control_file.h"
#include <stdio.h>


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
// increment the amount of ticks to contubue incrementing time
void SysTick_Handler(void)
{
    system_ticks++;
}
// get system ticks to be changed
uint64_t cur_time_ms(void){
    return system_ticks;
}

// default motor control function to handle the motor controls.
// TODO: FIGURE OUT HOW TO DRIVE THE MOTORS
// IMPLEMENT PID CONTROL FOR TURNING x amount of Degrees with the bot so slows down to rotate correctly, 
//(in case need to rotate the bot a persise amount)
//Needs to also be able to drive forward, backward, sideways and capible of rotating a certain amount of degrees.
// should be able to do this with the econders themselves. 
// extern volatile int previous_state_enc1;
// extern volatile int enc_ticks1;
// // emcpder 2
// extern volatile int previous_state_enc2;
// extern volatile int enc_ticks2;
// // emcpder 3
// extern volatile int previous_state_enc3;
// extern volatile int enc_ticks3;
// // encoder 4
// extern volatile int previous_state_enc4;
// extern volatile int enc_ticks4;
int control_motors(char command, int speed, int ticks){
    printf("function called\n");
    if(command == 'f'){ //forward
        //enable speed
        change_duty(1, 0); //top
        change_duty(2, speed); //left
        change_duty(3, 0); //bottom
        change_duty(4, speed); //right

        //phase direction
        set_pin(GPIOB, 0, 1); //J1 left forward
        set_pin(GPIOB, 1, 0); //J2 bottom not moving
        set_pin(GPIOB, 4, 0); //J3 top not moving
        set_pin(GPIOB, 5, 1); //J4 right forward
    }else if(command == 'b'){ //backward
        //speed
        change_duty(1, 0); //top
        change_duty(2, speed); //left
        change_duty(3, 0); //bottom
        change_duty(4, speed); //right
        //directions
        set_pin(GPIOB, 0, 0); //J1 left backward
        set_pin(GPIOB, 1, 1); //J2 bottom not moving
        set_pin(GPIOB, 4, 1); //J3 top not moving
        set_pin(GPIOB, 5, 0); //J4 right backward
    }else if(command == 'r'){ //right
        //speed
        change_duty(1, speed); //top
        change_duty(2, 0); //left
        change_duty(3, speed); //bottom
        change_duty(4, 0); //right
        //direction
        set_pin(GPIOB, 0, 0); //J1 left not moving
        set_pin(GPIOB, 1, 1); //J2 bottom forward
        set_pin(GPIOB, 4, 1); //J3 top forward
        set_pin(GPIOB, 5, 0); //J4 right not moving
    }else if(command = 'l'){ //left
        //speed
        change_duty(1, speed); //top
        change_duty(2, 0); //left
        change_duty(3, speed); //bottom
        change_duty(4, 0); //right
        //direction
        set_pin(GPIOB, 0, 1); // J1 left not moving
        set_pin(GPIOB, 1, 0); //J2 bottom backward
        set_pin(GPIOB, 4, 0); //J3 top backward
        set_pin(GPIOB, 5, 1); //J4 right not moving
    }
}

int main()
{
    printf("in main\n");
    //set the clock to 80 mhz
    clock_setup_80MHz(); 
    //set the uart to start
    serial_begin(USART2);
    //enable pwm
    enable_pwm();
    // init the gpio for the motors
    enable_gpio_output_motor(GPIOB,1); //Phase A1 - bottom
    enable_gpio_output_motor(GPIOB,0); //Phase B1 - left
    enable_gpio_output_motor(GPIOB,4); //Phase B2 - top
    enable_gpio_output_motor(GPIOB,5); //Phase A2 - right
    //enable the clock so can check how long actions have taken
    SysTick_Init(1000);
    //enable interupts
    enable_input_and_intetupts();

    // set up prompt
    char prompt[] = "TEST \n\0";
    

    serial_write(USART2, prompt);
    change_duty(2,500);
    //main loop
    for(;;){
        speed=RXBUFFER2[1];
        command=RXBUFFER2[0];
        // TODO: CALL THIS FUNCTION WITH HOWEVER YOU WANT TO MODIFY IT
        //call control motors here based on the input from the buffer.
        char command = 'f';
        int speed = 300;
        int ticks = 1;
        printf("first for loop\n");
        control_motors(command, speed, ticks);
        for(int i=0; i<10000;i++){
            int t=0;
        }
        // memcpy(prompt,(int*)(TIM1->CNT),strlen(prompt));
        // serial_write(USART2, prompt);
        
    }
}
