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

int main(void) {
  GPIO_InitTypeDef gpio_cfg;

  nm_systick_init();
  nm_debug_init();

  gpio_cfg.GPIO_Pin = GPIO_Pin_1;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio_cfg);

  while(1) {
	printf("Hello, world!\n");

	GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
	nm_systick_delay(250);
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
	nm_systick_delay(250);
  }

  return 0;
}
