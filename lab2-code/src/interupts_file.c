#include "interupts_file.h"

// emcpder 1
volatile int previous_state_enc1=0;
volatile int enc_ticks1=0;
// emcpder 2
volatile int previous_state_enc2=0;
volatile int enc_ticks2=0;
// emcpder 3
volatile int previous_state_enc3=0;
volatile int enc_ticks3=0;
// encoder 4
volatile int previous_state_enc4=0;
volatile int enc_ticks4=0;



int enable_input_pin(GPIO_TypeDef *GPIOX,int pin){
    RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN|RCC_AHB2ENR_GPIOCEN;
    // enable the input mode of the pin
    GPIOX->MODER &= ~(0x3<<(pin<<1));
    // enable the open drain of the pin NOT ANYMORE DO NOT DO THIS

    GPIOX->OTYPER &= ~(0x1<<(pin));
    // GPIOX->OTYPER |= (0x1<<pin);
    // enable a pull up for the pin, may not be nessary. check tho if the encoders do need it
    GPIOX->PUPDR &= ~(0x3<<(pin<<1));
    // GPIOX->PUPDR |= 0x1<<(pin<<1);
    // set high speed
    GPIOX->OSPEEDR &= ~(0x3<<(pin<<1));
    GPIOX->OSPEEDR &= 0x2<<(pin<<1);

    int var=(GPIOX->IDR & GPIO_IDR_IDR_0)>>GPIO_IDR_ID0_Pos;
    var=var+1;
    return 0;
}



int enable_input_and_intetupts(){
    enable_input_pin(GPIOA,7);
    enable_input_pin(GPIOA,6);
    enable_input_pin(GPIOA,5);
    enable_input_pin(GPIOA,4);
    enable_input_pin(GPIOA,3);
    enable_input_pin(GPIOA,1);
    enable_input_pin(GPIOA,0);
    enable_input_pin(GPIOA,12);
    // enable the interupts on the pins
    // enable interupt for pa0
    // enable the clock oops
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
    // enable the interupt for pa1
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    // enable interupt for pa3
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA;
    // enable the interupt for pa4
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA;
     // enable the interupt for pa5
     SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5;
     SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PA;
    // enable the interupt for pa6
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI6;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PA;
    // enable the interupts for pa7
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI7;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA;
    // enable the interupts for pa12
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI12;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PA;

    // enable the rising edge for the interupts
    EXTI->RTSR1 |= EXTI_RTSR1_RT0; 
    EXTI->RTSR1 |= EXTI_RTSR1_RT1; 
    EXTI->RTSR1 |= EXTI_RTSR1_RT3; 
    EXTI->RTSR1 |= EXTI_RTSR1_RT4; 
    EXTI->RTSR1 |= EXTI_RTSR1_RT5; 
    EXTI->RTSR1 |= EXTI_RTSR1_RT6; 
    EXTI->RTSR1 |= EXTI_RTSR1_RT7; 
    EXTI->RTSR1 |= EXTI_RTSR1_RT12; 

    // enable the falling edge for the interupts

    EXTI->FTSR1 |= EXTI_FTSR1_FT0;
    EXTI->FTSR1 |= EXTI_FTSR1_FT1;
    EXTI->FTSR1 |= EXTI_FTSR1_FT2;
    EXTI->FTSR1 |= EXTI_FTSR1_FT3;
    EXTI->FTSR1 |= EXTI_FTSR1_FT4;
    EXTI->FTSR1 |= EXTI_FTSR1_FT5;
    EXTI->FTSR1 |= EXTI_FTSR1_FT6;
    EXTI->FTSR1 |= EXTI_FTSR1_FT7;
    EXTI->FTSR1 |= EXTI_FTSR1_FT12;

    EXTI->IMR1 |=EXTI_IMR1_IM0_Msk|EXTI_IMR1_IM1_Msk|EXTI_IMR1_IM3_Msk|EXTI_IMR1_IM4_Msk|EXTI_IMR1_IM5_Msk|EXTI_IMR1_IM6_Msk|EXTI_IMR1_IM7_Msk|EXTI_IMR1_IM12_Msk;

    // enable the irqs
    NVIC_EnableIRQ(EXTI0_IRQn); 
    NVIC_EnableIRQ(EXTI1_IRQn); 
    NVIC_EnableIRQ(EXTI3_IRQn); 
    NVIC_EnableIRQ(EXTI4_IRQn);  
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // clear any existing interupts
    // encoder a
    EXTI->PR1 |= EXTI_PR1_PIF0;
    EXTI->PR1 |= EXTI_PR1_PIF1;
    // encoder b
    EXTI->PR1 |= EXTI_PR1_PIF2;
    EXTI->PR1 |= EXTI_PR1_PIF4;
    // encoder c
    EXTI->PR1 |= EXTI_PR1_PIF5;
    EXTI->PR1 |= EXTI_PR1_PIF6;
    // set to be encoder d
    EXTI->PR1 |= EXTI_PR1_PIF7;
    EXTI->PR1 |= EXTI_PR1_PIF12;

    return 0;
}

// how many ticks in a rotation????? iapparently 7 full cycles per rotation. 
// this means that a single cycle is 1/28*360 degrees. 
//or 0.224399475 rad. so like it could pause the pwm until aligned but that doesn't really work well. 
// so if it's off by. also we can get the time with systick.
//may also want to reset the ticks to be 0 when changing directions expecially for the pid. 

void EXTI0_IRQHandler(void){

    if (EXTI->PR1 & EXTI_PR1_PIF0) // Check if the interupt was triggered
    {
        // variable for pulses per rotation variable

        // return the state right

        // get the current state  of the encoder

        uint32_t input1=(GPIOA->IDR & GPIO_IDR_IDR_0)>>GPIO_IDR_ID0_Pos;
        uint32_t input2=(GPIOA->IDR & GPIO_IDR_IDR_1)>>GPIO_IDR_ID1_Pos;

        // get current state less lines in assembly and no branches so better
        int cur_state= (input1^input2)+(input2<<1);

        int diff_state= (cur_state-previous_state_enc1)&0x3;
        enc_ticks1+=(diff_state-2)*(diff_state&0x1);

        previous_state_enc1=cur_state;
        EXTI->PR1 |= EXTI_PR1_PIF0; //clear the interupt

    }
}

// 
void EXTI1_IRQHandler(void){

    if (EXTI->PR1 & EXTI_PR1_PIF1) // Check if the interupt was triggered
    {
        uint32_t input1=(GPIOA->IDR & GPIO_IDR_IDR_0)>>GPIO_IDR_ID0_Pos;
        uint32_t input2=(GPIOA->IDR & GPIO_IDR_IDR_1)>>GPIO_IDR_ID1_Pos;

        // get current state less lines in assembly and no branches so better
        int cur_state= (input1^input2)+(input2<<1);

        int diff_state= (cur_state-previous_state_enc1)&0x3;
        enc_ticks1+=(diff_state-2)*(diff_state&0x1);
        
        previous_state_enc1=cur_state;
        EXTI->PR1 |= EXTI_PR1_PIF1; //clear the interupt
        // need to set the previous state arrgh

        // if(enc_ticks1==20 || enc_ticks1==-20){
        //     volatile int x=0;
        //     x=x+1;
        // }
    }
}

void EXTI3_IRQHandler(void){

    if (EXTI->PR1 & EXTI_PR1_PIF3) // Check if the interupt was triggered
    {
        uint32_t input1=(GPIOA->IDR & GPIO_IDR_IDR_3)>>GPIO_IDR_ID3_Pos;
        uint32_t input2=(GPIOA->IDR & GPIO_IDR_IDR_4)>>GPIO_IDR_ID4_Pos;

        // get current state less lines in assembly and no branches so better
        int cur_state= (input1^input2)+(input2<<1);

        int diff_state= (cur_state-previous_state_enc2)&0x3;
        enc_ticks2+=(diff_state-2)*(diff_state&0x1);
        
        EXTI->PR1 |= EXTI_PR1_PIF2; //clear the interupt
        previous_state_enc2=cur_state;
    }
}

void EXTI4_IRQHandler(void){

    if (EXTI->PR1 & EXTI_PR1_PIF4) // Check if the interupt was triggered
    {
        uint32_t input1=(GPIOA->IDR & GPIO_IDR_IDR_3)>>GPIO_IDR_ID3_Pos;
        uint32_t input2=(GPIOA->IDR & GPIO_IDR_IDR_4)>>GPIO_IDR_ID4_Pos;

        // get current state less lines in assembly and no branches so better
        int cur_state= (input1^input2)+(input2<<1);

        int diff_state= (cur_state-previous_state_enc2)&0x3;
        enc_ticks2+=(diff_state-2)*(diff_state&0x1);
        
        previous_state_enc2=cur_state;
        EXTI->PR1 |= EXTI_PR1_PIF3; //clear the interupt

    }
}


void EXTI9_5_IRQHandler(void){
    // check encoder 3 for interupts
    if ((EXTI->PR1 & EXTI_PR1_PIF5) | (EXTI->PR1 & EXTI_PR1_PIF6)) // Check if the interupt was triggered
    {
        uint32_t input1=(GPIOA->IDR & GPIO_IDR_IDR_5)>>GPIO_IDR_ID5_Pos;
        uint32_t input2=(GPIOA->IDR & GPIO_IDR_IDR_6)>>GPIO_IDR_ID6_Pos;

        // get current state less lines in assembly and no branches so better
        int cur_state= (input1^input2)+(input2<<1);

        int diff_state= (cur_state-previous_state_enc2)&0x3;
        enc_ticks3+=(diff_state-2)*(diff_state&0x1);
        
        previous_state_enc3=cur_state;

        EXTI->PR1 |= EXTI_PR1_PIF5; //clear the interupts
        EXTI->PR1 |= EXTI_PR1_PIF6;
        // check encoder 4 for interupts
    }else if(EXTI->PR1 & EXTI_PR1_PIF7){
        uint32_t input1=(GPIOA->IDR & GPIO_IDR_IDR_7)>>GPIO_IDR_ID7_Pos;
        uint32_t input2=(GPIOA->IDR & GPIO_IDR_IDR_12)>>GPIO_IDR_ID12_Pos;

        // get current state less lines in assembly and no branches so better
        int cur_state= (input1^input2)+(input2<<1);

        int diff_state= (cur_state-previous_state_enc2)&0x3;
        enc_ticks4+=(diff_state-2)*(diff_state&0x1);
        
        previous_state_enc4=cur_state;
        EXTI->PR1 |= EXTI_PR1_PIF7; //clear the interupt
    }
}


void EXTI15_10_IRQHandler(void){
    if (EXTI->PR1 & EXTI_PR1_PIF12) // Check if the interupt was triggered
    {
        uint32_t input1=(GPIOA->IDR & GPIO_IDR_IDR_7)>>GPIO_IDR_ID5_Pos;
        uint32_t input2=(GPIOA->IDR & GPIO_IDR_IDR_12)>>GPIO_IDR_ID12_Pos;

        // get current state less lines in assembly and no branches so better
        int cur_state= (input1^input2)+(input2<<1);

        int diff_state= (cur_state-previous_state_enc2)&0x3;
        enc_ticks3+=(diff_state-2)*(diff_state&0x1);
        previous_state_enc4=cur_state;
        EXTI->PR1 |= EXTI_PR1_PIF12; //clear the interupt
    }
}





