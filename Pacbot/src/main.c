#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_tim.h"

int SystemCoreClock=0;

int init_UART(void){

}

    // A0=PA0,A1=PA1,A2=PA3,A3=PA4
    // A4=PA5,A5=PA6,A6=PA7,A7=PA2
    // D0=PA10,D1=PA9,D2=PA12,D3=PB0
    // D4=PB7,D5=PB6,D6=PB1,D7=PC14
    // D8=PC15,D9=PA8,D10=PA11,D11=PB5
    // D12=PB4,D13=PB3.


int init_GPIO(void){

    //SO PWM PINS ARE PA6(A5),PA7(A6),PB0(D3),PB1(D6)for tim1

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // d2(pa12),d3(pb0),d4(pb7),d5(pb6)
    // init the gpio pin(D2-D9),
    GPIO_InitTypeDef MotorStruct[8] = {0};

    // probawbly should be functions but whatev.
    // init motor pins
    // init GPIO Pin d2(on pa12) enable m1
    MotorStruct[0].Pin = GPIO_PIN_12;
    MotorStruct[0].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    MotorStruct[0].Pull = GPIO_NOPULL;          // No pull-up or pull-down
    MotorStruct[0].Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA,&MotorStruct[0]);
    // init GPIO pin d3(on pb0) phase m1. NOTE: USED FOR PWM declare later
    // MotorStruct[1].Pin = GPIO_PIN_0;
    // MotorStruct[1].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    // MotorStruct[1].Pull = GPIO_NOPULL;          //np need for pull up/pull down
    // MotorStruct[1].Speed = GPIO_SPEED_FREQ_HIGH;
    // HAL_GPIO_Init(GPIOB,&MotorStruct[1]);
    // init gpio pin d4(pb7) enable m2
    MotorStruct[2].Pin = GPIO_PIN_7;
    MotorStruct[2].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    MotorStruct[2].Pull = GPIO_NOPULL;          //np need for pull up/pull down
    MotorStruct[2].Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&MotorStruct[2]);
    // init gpio pin d5(pb6) phase m2
    MotorStruct[3].Pin = GPIO_PIN_6;
    MotorStruct[3].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    MotorStruct[3].Pull = GPIO_NOPULL;          //np need for pull up/pull down
    MotorStruct[3].Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&MotorStruct[3]);
    // init gpio pin d6(pb1) enable m3 also used for a pwm pin
    // MotorStruct[4].Pin = GPIO_PIN_1;
    // MotorStruct[4].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    // MotorStruct[4].Pull = GPIO_NOPULL;          //np need for pull up/pull down
    // MotorStruct[4].Speed = GPIO_SPEED_FREQ_HIGH;
    // HAL_GPIO_Init(GPIOB,&MotorStruct[4]);
    // init gpio pin d7(pc14) phase m3
    MotorStruct[5].Pin = GPIO_PIN_14;
    MotorStruct[5].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    MotorStruct[5].Pull = GPIO_NOPULL;          //np need for pull up/pull down
    MotorStruct[5].Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC,&MotorStruct[5]);
    // init gpio pin d8(pc15) enable m4
    MotorStruct[6].Pin = GPIO_PIN_15;
    MotorStruct[6].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    MotorStruct[6].Pull = GPIO_NOPULL;          //np need for pull up/pull down
    MotorStruct[6].Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC,&MotorStruct[6]);
    // init gpio pin d9(pa8) phase m4
    MotorStruct[7].Pin = GPIO_PIN_8;
    MotorStruct[7].Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull mode
    MotorStruct[7].Pull = GPIO_NOPULL;          //np need for pull up/pull down
    MotorStruct[7].Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA,&MotorStruct[7]);
    return 0;

}

//SO PWM PINS ARE PA6(A5),PA7(A6),PB0(D3),PB1(D6)for tim1
int init_PWM_GPIO(uint16_t GPIO_PINno,GPIO_TypeDef *GPIO_X){
    GPIO_InitTypeDef PWM = {0};
    PWM.Pin = GPIO_PINno;
    PWM.Mode = GPIO_MODE_AF_PP;  // Push-pull mode
    PWM.Pull = GPIO_NOPULL;          //np need for pull up/pull down
    PWM.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_X,&PWM);
    return 0;
}

int init_PWM_motors(void){
    // init gpio pins that correspond to tim1 pwm
    initPWM_GPIO(GPIO_PIN_5,GPIOA);
    initPWM_GPIO(GPIO_PIN_7,GPIOA);
    initPWM_GPIO(GPIO_PIN_3,GPIOB);
    initPWM_GPIO(GPIO_PIN_6,GPIOB);
    
    // enable tim1 clock
    __HAL_RCC_TIM1_CLK_ENABLE();

    // setup tim1
    TIM_HandleTypeDef tim1;
    tim1.Instance = TIM1;
    tim1.Init.Prescaler= 0;
    tim1.Init.Period=1000-1; //want 1khz
    tim1.Init.CounterMode = TIM_COUNTERMODE_UP;//count upwards
    tim1.Init.ClockDivision =TIM_CLOCKDIVISION_DIV1;//no clock division
    tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // init tim1
    HAL_TIM_PWM_Init(&tim1);

    // config for the channels

    TIM_OC_InitTypeDef config = {0};
    config.OCMode = TIM_OCMODE_PWM1; 
    config.Pulse = 500;    //half on half off. need to play around with
    config.OCPolarity = TIM_OCPOLARITY_HIGH;  // High polarity for the PWM signal. this means high by default
    config.OCFastMode = TIM_OCFAST_DISABLE;

    // enable TIM1 channels 1-4.
    HAL_TIM_PWM_ConfigChannel(&tim1, &config, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&tim1, &config, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&tim1, &config, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&tim1, &config, TIM_CHANNEL_4);

    // start pwm on these channels
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_4);

    // disable the channels
    HAL_TIM_SET_COMPARE(tim1, TIM_CHANNEL_1, 0);
    HAL_TIM_SET_COMPARE(tim1, TIM_CHANNEL_2, 0);
    HAL_TIM_SET_COMPARE(tim1, TIM_CHANNEL_3, 0);
    HAL_TIM_SET_COMPARE(tim1, TIM_CHANNEL_4, 0);
    // alloc heap space for tim1 and return it, nah don't want to deal with leaks whatsoever, just redeclare where need it in that
    
}

// yoinked function from joel class
void clock_setup_80MHz(void){
    uint32_t HSITrim;

    // To correctly read data from flash memory, the number of wait states
    // must be set based on the frequency of the CPU clock (HCLK) and the
    // supply voltage. According to RM0394 3.3.3, we need 4 wait states at 80MHz
    // (and note that we don't need any wait states at 16MHz).
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;
		
    // Clock Control Register (CR), bit 8
    // Enable High Speed Internal Clock (HSI = 16 MHz). This just turns it on,
    // without changing any mux selects.
    RCC->CR |= RCC_CR_HSION;

    // Wait until HSI has settled down after turnon. We know this by checking
    // another bit in RCC->CR.
    while((RCC->CR & RCC_CR_HSIRDY) == 0);

    // Internal Clock Sources Calibration Register (ICSCR).
    // This 32-bit register has 16 bits each for the HSI and MSI clocks. For
    // each of those, the 16 bits is 8 bits of read-only manufacturing fuses,
    // and 8 bits of user-selected trim values. The manufacturing fuses are
    // pretty good, so we just pick a nice midrange user-trim here.
    HSITrim = 16; // user-programmable trim, added to ICSCR.HSICAL[7:0].
    RCC->ICSCR &= ~RCC_ICSCR_HSITRIM;	// Zero out user trim.
    RCC->ICSCR |= HSITrim << 24;	// And set a midrange value.

    // Turn off the main PLL. It's off at reset anyway, but just be sure in case
    // this function has been called before, or whatever.
    RCC->CR    &= ~RCC_CR_PLLON; 

    // Wait for it to lock. Not sure why this is needed, since it's off!
    while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);

    // Now that the PLL is off, let's configure it.
    // First, select clock source to PLL. The reset value is no clock at all.
    // Now we switch it to HSI (i.e., 16 MHz).
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;	// 00=No clock, 01=MSI,
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;	// 10=HSI, 11 = HSE

    // Set the PLL dividers to get 80 MHz output. The final output is
    // 16 MHz * N / (M*R). We'll set N=20, M=2, R=2 to get 16 * 20/4 = 80 MHz.
    //	000: PLLM = 1, 001: PLLM = 2, 010: PLLM = 3, 011: PLLM = 4,
    // Note that N must be in [8,86].
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 20U << 8; // N=20.
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 1U << 4;  // M=2.

    // Set R=2. This is the default, anyway, with bits[26:25]=00.
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;

    // The PLL is configured; now turn it on. Note that there are two bits to
    // poke, not just one.
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable Main PLL PLLCLK output 
    RCC->CR   |= RCC_CR_PLLON; 

    // And wait for it to lock.
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Select line on the main SYSCLK mux, choosing between HSE, MSI, HSI16 and
    // a frequency-multiplied PLL to drive SYSCLK. We haven't touched it so far,
    // and the reset-default value is to use the MSI oscillator. Now we set it
    // to use the PLL that we just configured/locked.
    RCC->CFGR &= ~RCC_CFGR_SW;	  // Turn off all bits, so that we can...
    RCC->CFGR |= RCC_CFGR_SW_PLL; // ...set to the PLL.

    // Wait until the mux has switched
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	
    // Set the AHB, APB1 and APB2 clock prescalers all to x1 (which is their
    // reset default anyway).
    // AHB controls some peripherals itself, as well as driving APB1 (the
    // low-speed peripheral clock) and APB2 (high-speed peripheral clock) via
    // prescalers.
    RCC->CFGR &= ~RCC_CFGR_HPRE;  // HCLK = SYSCLK x 1
    RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB1 = HCLK x 1
    RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 = HCLK x 1

    // Turn off the SA1 PLL
    RCC->CR &= ~RCC_CR_PLLSAI1ON;
    // Then wait for it to actually be off.
    while ( (RCC->CR & RCC_CR_PLLSAI1ON) == RCC_CR_PLLSAI1ON );

    // SA1 VCO freq = PLL-in-freq * N/M = 16 MHz * 24/2 = 192 MHz
    // We already set the PLL-in-freq=16Mh and M=2 above, for the main PLL.
    // Now set divider SA1N=24.
    RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1N;	// bits[14:8]=0
    RCC->PLLSAI1CFGR |= 24U<<8;				// bits[14:8]=0x18

    // SA1 divider P = 17
    // This sets the SAI1 PLL P output = 192 MHz / 17 = 11.294MHz
    RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1P;	// bit[17]=1 -> P=17.

    // Enable the SA1 PLL clock output.
    RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;	// bit[16]

    // SAI1PLL division factor for PLL48M2CLK (48 MHz clock)
    // RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1Q;
    // RCC->PLLSAI1CFGR |= U<<21;
    // RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;

    // PLLSAI1 division factor for PLLADC1CLK (ADC clock)
    // 00: PLLSAI1R = 2, 01: PLLSAI1R = 4, 10: PLLSAI1R = 6, 11: PLLSAI1R = 8
    // RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1R; 
    // RCC->PLLSAI1CFGR |= U<<25;
    // RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;

    // Turn on the SA1 PLL
    RCC->CR |= RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
    // Then wait for it to actually be on.
    while ( (RCC->CR & RCC_CR_PLLSAI1ON) == 0);

    // Swing the final mux to drive the SA1 clock. It can come from
    // the SAI1 PLL P output, the SAI2 PLL P output, the main-PLL P output
    // or an external clock. We choose the SAI1-PLL P output.
    RCC->CCIPR &= ~RCC_CCIPR_SAI1SEL;

    // Final clock enable for the SA1 clock.
    RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;

    // Update the software global variable to talk with other API funcs.
    SystemCoreClock = 80000000;
}

int init(void){
    // init the hal
    HAL_Init();
    // init the system clock
    clock_setup_80MHz();
    // init gpio pins for motors
    init_GPIO();
    // init pwm on the pins set it up to be 0 immediately, modify D2-5
    // init tim1
    // init the motors
    init_PWM_motors();
    // d2(pa12),d3(pb0),d4(pb7),d5(pb6),d7(pc14)motor dir pins
    //SO PWM PINS ARE PA6(A5),PA7(A6),PB0(D3),PB1(D6)for tim1
    // drive the gpio pins of the motors to low should set dir to forward
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);


    // init pins for the quadrature encoder

    
}

int main (void)
{
    init();

}





