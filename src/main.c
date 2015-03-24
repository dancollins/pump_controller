/*
 * Pump Controller
 *
 * Used to maintain the water level in the C.1.12 lab water cylinder by
 * controlling the rate water is pumped in. The water level is measured
 * using an ultrasonic sensor and the level is moved by changing the rate
 * the water enters. Changing the rate water leaves the tank will result
 * in the controller varying the rate water enters using a PID control
 * loop.
 *
 * Dan Collins 2015
 * ENEL517-15A
 * 1183446
 */
#include <stdio.h>

#include <stm32f0xx.h>

/* Used for the debug capabilities and all the nice newlibc porting */
#include "libnarm.h"


#define LED1_PIN GPIO_Pin_3
#define LED2_PIN GPIO_Pin_4
#define LED_PORT GPIOB


void init_sonar(void) {
  GPIO_InitTypeDef gpio_cfg;
  TIM_TimeBaseInitTypeDef tim_base_cfg;
  TIM_OCInitTypeDef oc_cfg;
  TIM_BDTRInitTypeDef bdtr_cfg;

  uint16_t TIM1_Period, Channel1Pulse;

  /* Enable clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* Configure TIM1_CH1 and TIM1_CH1N for the ultrasonic transmitter */
  gpio_cfg.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AF;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio_cfg);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);

  /* Configure a timer to generate a complimentary PWM signal with a 50%
   * duty cycle.
   *
   * The timer input clock is set to APB2 clock (PCLK2) which is set to
   * HCLK telling us that TIM1 clock = SystemCoreClock (48MHz)
   *
   * We want to generate a PWM signal at 40KHz:
   * - TIM1_Period = (SystemCoreClock / 40000) - 1
   *
   * The timer pulse is calculated as:
   * - Channel1Pulse = 50 * (TIM1_Period - 1) / 100
   *
   * We insert a short dead time so that there's no overlap with the
   * transistors of 11/SystemCoreClock ns (selected to copy the example)
   */
  TIM1_Period = (SystemCoreClock / 40000) - 1;
  Channel1Pulse = (uint16_t)(((uint32_t) 5 * (TIM1_Period - 1)) / 10);

  /* Configure the time base */
  tim_base_cfg.TIM_Prescaler = 0;
  tim_base_cfg.TIM_CounterMode = TIM_CounterMode_Up;
  tim_base_cfg.TIM_Period = TIM1_Period;
  tim_base_cfg.TIM_ClockDivision = 0;
  tim_base_cfg.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &tim_base_cfg);

  /* Configure CH1 to be in PWM mode */
  oc_cfg.TIM_OCMode = TIM_OCMode_PWM2;
  oc_cfg.TIM_OutputState = TIM_OutputState_Enable;
  oc_cfg.TIM_OutputNState = TIM_OutputNState_Enable;
  oc_cfg.TIM_Pulse = Channel1Pulse;
  oc_cfg.TIM_OCPolarity = TIM_OCPolarity_High;
  oc_cfg.TIM_OCNPolarity = TIM_OCNPolarity_High;
  oc_cfg.TIM_OCIdleState = TIM_OCIdleState_Reset;
  oc_cfg.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  TIM_OC1Init(TIM1, &oc_cfg);

  /* Dead time, lock configuration and automatic output enable */
  bdtr_cfg.TIM_OSSRState = TIM_OSSRState_Enable;
  bdtr_cfg.TIM_OSSIState = TIM_OSSIState_Enable;
  bdtr_cfg.TIM_LOCKLevel = TIM_LOCKLevel_1;
  bdtr_cfg.TIM_DeadTime = 11;
  bdtr_cfg.TIM_Break = TIM_Break_Disable;
  bdtr_cfg.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM1, &bdtr_cfg);

  TIM_Cmd(TIM1, ENABLE);
}


int main(void) {
  GPIO_InitTypeDef gpio_cfg;

  /* Start libnarm */
  nm_systick_init();
  nm_debug_init();

  /* Start the sonar */
  init_sonar();

  /* Prepare the status LEDs */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  gpio_cfg.GPIO_Pin = LED1_PIN | LED2_PIN;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_PORT, &gpio_cfg);
  GPIO_ResetBits(LED_PORT, LED1_PIN | LED2_PIN);

  printf("Pump Controller v0.1a\n");

  while(1) {
	GPIO_WriteBit(LED_PORT, LED1_PIN, Bit_SET);
	nm_systick_delay(250);
	GPIO_WriteBit(LED_PORT, LED1_PIN, Bit_RESET);
	nm_systick_delay(250);
  }

  return 0;
}
