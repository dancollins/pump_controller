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
#include <stdint.h>
#include <inttypes.h>

#include <stm32f0xx.h>

/* Used for the debug capabilities and all the nice newlibc porting */
#include "libnarm.h"


#define VERSION "0.1b"

#define MAX16 ((1<<16)-1)
#define constrain(x) \
  if (x < 0)\
	x = 0; \
  if (x > MAX16) \
	x = MAX16;


/* Convenience macros for the LEDs */
#define LED1_PIN GPIO_Pin_3
#define LED2_PIN GPIO_Pin_4
#define LED_PORT GPIOB

/* Number of crests to sound out the sonar */
#define N_CRESTS (3)

/* Number of samples needed to capture the waveform. Keep in mind that each
 * sample is two octets.
 *
 * Time needed to capture a reflection 30cm away. 30cm is about as far
 * as we can send a pulse before the captured waveform is in the noise
 * t_reflect = d/v => t_reflect = 0.6m / 340ms^-1 = 0.0017s
 *
 * Time needed per ADC sample
 * T_s = 28.5 cycles, T_sar = 12.5 cycles
 * T_conv = 41 cycles
 * F_adc = 14MHz
 * t_conv = 1/F_adc * T_conv = 2.9us
 *
 * Samples needed to capture 60cm of data
 * n_samp = t_reflect / t_conv = 608.519
 */
#define N_SAMPLES (1000)
#define T_SAMPLE (2.9e-6)

#define V_SOUND (340.0)

/* Point at which we consider the receiver output to be showing data.
 * This value was calibrated through trial and error. It's set such
 * that the reflected pulse at index N_SAMPLES would never be big
 * enough to get measured. If not, we get a problem where the returned
 * index wraps around... */
#define THRESHOLD (2600)

/* PID values */
#define Kp (50)
#define Ki (15)
#define Kd (5)


/* Buffer to store captured waveform in. We waste a little memory by
 * using 16bit values, but it saves a lot of time both inserting and
 * processing the data as we don't need to pack/unpack. */
static uint16_t waveform_buffer[N_SAMPLES];
static volatile uint32_t waveform_done;

/* We keep this value around to make it easy to change the ADC
 * configuration */
ADC_InitTypeDef adc_cfg;


void sn_init(void) {
  GPIO_InitTypeDef gpio_cfg;
  TIM_TimeBaseInitTypeDef tim_base_cfg;
  TIM_OCInitTypeDef oc_cfg;
  TIM_BDTRInitTypeDef bdtr_cfg;
  DMA_InitTypeDef dma_cfg;
  NVIC_InitTypeDef nvic_cfg;

  uint16_t TIM1_Period, Channel1Pulse;


  /* Enable clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);


  /* Configure TIM1_CH1 and TIM1_CH1N for the ultrasonic transmitter */
  gpio_cfg.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AF;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio_cfg);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);

  /* Configure AN3 for the ADC */
  gpio_cfg.GPIO_Pin = GPIO_Pin_3;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &gpio_cfg);


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
   *
   * We also set the repitition value to generate an interrupt after a
   * certain time.
   */
  TIM1_Period = (SystemCoreClock / 40000) - 1;
  Channel1Pulse = (uint16_t)(((uint32_t) 5 * (TIM1_Period - 1)) / 10);

  /* Configure the time base */
  tim_base_cfg.TIM_Prescaler = 0;
  tim_base_cfg.TIM_CounterMode = TIM_CounterMode_Up;
  tim_base_cfg.TIM_Period = TIM1_Period;
  tim_base_cfg.TIM_ClockDivision = 0;
  tim_base_cfg.TIM_RepetitionCounter = N_CRESTS - 1;
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

  TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Single);

  /* Dead time, lock configuration and automatic output enable */
  bdtr_cfg.TIM_OSSRState = TIM_OSSRState_Enable;
  bdtr_cfg.TIM_OSSIState = TIM_OSSIState_Enable;
  bdtr_cfg.TIM_LOCKLevel = TIM_LOCKLevel_1;
  bdtr_cfg.TIM_DeadTime = 11;
  bdtr_cfg.TIM_Break = TIM_Break_Disable;
  bdtr_cfg.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM1, &bdtr_cfg);

  /* Allow the timer to generate an interrupt on the update event. In this
   * case, that will happen when the repetition counter gets to zero */
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);


  /* Configure the ADC to sample continuously with 12bits of resolution.
   * The ADC will be set up such that after each conversion it tells the
   * DMA controller. The DMA controller will copy a fixed number of bytes
   * and then generate an interrupt. This lets us start the ADC and get
   * an interrupt when the entire wave is sampled. */
  ADC_DeInit(ADC1);
  ADC_StructInit(&adc_cfg);
  adc_cfg.ADC_Resolution = ADC_Resolution_12b;
  adc_cfg.ADC_ContinuousConvMode = ENABLE;
  adc_cfg.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_cfg.ADC_DataAlign = ADC_DataAlign_Right;
  adc_cfg.ADC_ScanDirection = ADC_ScanDirection_Backward;
  ADC_Init(ADC1, &adc_cfg);

  /* Convert the ADC1 Channel3 with a 28.5 cycle sampling time (341KHz) */
  ADC_ChannelConfig(ADC1, ADC_Channel_3, ADC_SampleTime_28_5Cycles);

  /* ADC Calibration. We need to do this before we configure DMA. */
  ADC_GetCalibrationFactor(ADC1);

  /* We want the DMA controller to copy a specified number of bytes once
   * started, so set it into one-shot mode */
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_OneShot);

  /* Enable the DMA notification and the ADC. This won't start the
   * conversion yet */
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);


  /* Configure DMA1 Channel1 to copy from the ADC to the waveform buffer */
  DMA_DeInit(DMA1_Channel1);
  dma_cfg.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  dma_cfg.DMA_MemoryBaseAddr = (uint32_t)waveform_buffer;
  dma_cfg.DMA_DIR = DMA_DIR_PeripheralSRC;
  dma_cfg.DMA_BufferSize = N_SAMPLES;
  dma_cfg.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_cfg.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma_cfg.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  dma_cfg.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  dma_cfg.DMA_Mode = DMA_Mode_Circular;
  /* Priority isn't important when we've only got one channel in use */
  dma_cfg.DMA_Priority = DMA_Priority_High;
  /* We're copying P2M, so disable the M2M functionality */
  dma_cfg.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &dma_cfg);

  /* Generate an interrupt each time the DMA transfer is completed */
  DMA_ClearITPendingBit(DMA1_IT_TC1);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  /* Enable the DMA so when the ADC starts giving requests it will copy
   * the samples for us */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Prepare the NVIC to generate interrupts */
  /* Set to be higher than the debug serial port. This is also shared with
   * the DMA, as we wouldn't expect both interrupts to happen at the same
   * time. */
  nvic_cfg.NVIC_IRQChannelPriority = 1;
  nvic_cfg.NVIC_IRQChannelCmd = ENABLE;

  /* Configure the NVIC to generate an interrupt when the PWM pulses have
   * been sent */
  nvic_cfg.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
  NVIC_Init(&nvic_cfg);

  /* Configure the NVIC to generate an interrupt when the DMA transfer is
   * complete */
  nvic_cfg.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_Init(&nvic_cfg);
}

void sn_send_pulse(void) {
  /* Start the PWM. The outputs will automatically start */
  TIM_Cmd(TIM1, ENABLE);
}

int sn_get_index(void) {
  int i;

  for (i = 0; i < N_SAMPLES; i++) {
	if (waveform_buffer[i] > THRESHOLD)
	  break;
  }

  if (i == N_SAMPLES)
	i = -1;

  return i;
}


void motor_init(void) {
  GPIO_InitTypeDef gpio_cfg;
  TIM_TimeBaseInitTypeDef tim_base_cfg;
  TIM_OCInitTypeDef oc_cfg;

  uint16_t TIM2_Period;


  /* Enable clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


  /* Configure TIM2_CH2 for the motor */
  gpio_cfg.GPIO_Pin = GPIO_Pin_1;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AF;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio_cfg);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);


  /* Configure a timer to generate a PWM signal at 1KHz. We'll vary
   * the duty cycle depending on the water level reading.
   *
   * The timer input clock is set to APB2 clock (PCLK2) which is set to
   * HCLK telling us that the TIM2 clock = SystemCoreClock (48MHz)
   *
   * We want to generate a PWM signal at 10kHz:
   * - TIM2_Period = (SystemCoreClock / 1000) - 1
   * Notice this fits in a 16bit value up until the core clock is
   * 65MHz. This chip wont go that fast, but it's important to keep
   * it in mind!
   *
   * The timer pulse, initially, will be set to 0%.
   */
  TIM2_Period = (SystemCoreClock / 10000) - 1;

  /* Configure the time base */
  tim_base_cfg.TIM_Prescaler = 0;
  tim_base_cfg.TIM_CounterMode = TIM_CounterMode_Up;
  tim_base_cfg.TIM_Period = TIM2_Period;
  tim_base_cfg.TIM_ClockDivision = 0;
  TIM_TimeBaseInit(TIM2, &tim_base_cfg);

  /* Configure CH2 to be in PWM mode */
  oc_cfg.TIM_OCMode = TIM_OCMode_PWM1;
  oc_cfg.TIM_OutputState = TIM_OutputState_Enable;
  oc_cfg.TIM_OutputNState = TIM_OutputNState_Enable;
  oc_cfg.TIM_Pulse = (uint16_t)((80 * ((uint32_t)TIM2_Period - 1)) / 100);
  oc_cfg.TIM_OCPolarity = TIM_OCPolarity_High;
  oc_cfg.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OC2Init(TIM2, &oc_cfg);

  TIM_Cmd(TIM2, ENABLE);
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void motor_set_pulse(uint16_t pulse) {
  /* The timer pulse is calculated as:
   * - Channel1Pulse = duty * (TIM2_Period - 1) / 100 */
  TIM_SetCompare2(TIM2, pulse);
}


/**
 * Does an ADC read of the potentiometer to find the set point.
 * We need to take the ADC from DMA, take a reading, and then give
 * it back to DMA.
 */
uint16_t get_set_point(void) {
  uint16_t ret;

  /* Stop DMA from running */
  ADC_DMACmd(ADC1, DISABLE);
  ADC_Cmd(ADC1, DISABLE);
  DMA_Cmd(DMA1_Channel1, DISABLE);

  /* Select the potentiometer channel. We sample for a long time so
   * the value is a bit more stable */
  ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_239_5Cycles);
  ADC_GetCalibrationFactor(ADC1);
  ADC_Cmd(ADC1, ENABLE);

  /* Perform an ADC conversion */
  ADC_StartOfConversion(ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  ret = ADC_GetConversionValue(ADC1);
  /* This is a bit of a sneaky hack. We know the ADC value will be
   * between 0 - 4096, and we know the maximum index is 1000 (so
   * the biggest set point is 1000). Dividing by 4 means the pot
   * range is roughly that of our set point. Also, the compiler
   * will optimise the divide by four into a right shift by 2. */
  ret /= 4;

  /* Stop any more conversions from happening */
  ADC_StopOfConversion(ADC1);

  /* Set the ADC up for the sonar */
  ADC_Cmd(ADC1, DISABLE);

  adc_cfg.ADC_ContinuousConvMode = ENABLE;
  ADC_Init(ADC1, &adc_cfg);

  ADC_ChannelConfig(ADC1, ADC_Channel_3, ADC_SampleTime_28_5Cycles);
  ADC_GetCalibrationFactor(ADC1);
  ADC_Cmd(ADC1, ENABLE);

  ADC_DMACmd(ADC1, ENABLE);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  return ret;
}


int main(void) {
  GPIO_InitTypeDef gpio_cfg;
  TIM_TimeBaseInitTypeDef tim_base_cfg;

  int index;
  int set_point = 400;
  int error = 0, old_error = 0;

  int inte = 0;
  int diff = 0;

  int output_value;

  /* Start libnarm */
  nm_systick_init();
  nm_debug_init();

  printf("Pump Controller " VERSION "\n");

  /* Prepare the sonar */
  sn_init();

  /* Prepare the motor driver */
  motor_init();

  /* Prepare the status LEDs */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  gpio_cfg.GPIO_Pin = LED1_PIN | LED2_PIN;
  gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_PORT, &gpio_cfg);
  GPIO_ResetBits(LED_PORT, LED1_PIN | LED2_PIN);

  /* Prepare set point pin */
  gpio_cfg.GPIO_Pin = GPIO_Pin_4;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &gpio_cfg);

  /* Clear the terminal */
  printf("\x1B[2J");

  while(1) {
	/* Get the current set point */
	set_point = get_set_point();

	/* Send a pulse and measure the reflection */
    waveform_done = 0;
    sn_send_pulse();
	while (!waveform_done)
	  ;

	/* Find the ADC sample index crossing the threshold */
	index = sn_get_index();

	/* Skip samples if we can't see the water */
	if (index == -1)
	  continue;

	/* Calculate the error. A positive error indicates we have too
	 * much water in the tank. */
	old_error = error;
	error = index - set_point;

	/* Integral */
	inte += error;
	constrain(inte);

	/* Differential */
	diff = error - old_error;
	constrain(diff);

	/* Now we have all the terms we need, let's calculate the output
	 * value */
	output_value = (Kp * error) + (inte / Ki) + (diff / Kd);
	constrain(output_value);

	motor_set_pulse(output_value);

	/* Display all of the variables on the terminal */
	printf("\x1B[H");
	printf("input: %d\x1B[K\n", index);
	printf("set point: %d\x1B[K\n", set_point);
	printf("error: %d\x1B[K\n", error);
	printf("inte: %d\x1B[K\n", inte);
	printf("diff: %d\x1B[K\n", diff);
	printf("output: %d\x1B[K\n", output_value);

	nm_systick_delay(20);
  }

  return 0;
}


void HardFault_Handler(void) {
  fprintf(stderr, "HardFault Occured.\n");
  while(1)
	;
}


/* Run when the PWM has sent N_CRESTS */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

  /* Start the ADC */
  ADC_StartOfConversion(ADC1);

  /* Turn on the LED to let us know the sample has begun */
  GPIO_WriteBit(LED_PORT, LED2_PIN, Bit_SET);
}

/* Run when the DMA has copied N_SAMPLES from the ADC */
void DMA1_Channel1_IRQHandler(void) {
  DMA_ClearITPendingBit(DMA1_IT_TC1);

  /* Stop the ADC */
  ADC_StopOfConversion(ADC1);

  waveform_done = 1;

  /* Turn off the LED to say we're done! */
  GPIO_WriteBit(LED_PORT, LED2_PIN, Bit_RESET);
}


