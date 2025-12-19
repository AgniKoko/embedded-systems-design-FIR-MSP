#include <stdbool.h>
#include <stdio.h>

#include "msp430_version.h"
#include "intrinsics.h"

/* ----- FIR filter dimensions ----- */
#define SIGNAL_LEN  16u
#define KERNEL_LEN  5u
#define RESULT_LEN  (SIGNAL_LEN + KERNEL_LEN - 1u)

/* ----- Port 1 pins for Enable input and LED output ----- */
#define ENABLE_BIT   BIT0      /* PORT1/PIN0 used as Enable input (button/switch) */
#define LED_BIT      BIT7      /* PORT1/PIN7 used as status LED (Enabled or Disabled)*/



/* ----- Circular buffer for ADC input samples ----- */
volatile float signal_buffer[SIGNAL_LEN];   /* circular buffer of ADC samples */
volatile unsigned int write_index = 0;      /* next position to write in buffer */


/* ----- Linearised signal and convolution result ----- */
float signal_linear[SIGNAL_LEN];            /* unwrapped buffer for convolution */
float result[RESULT_LEN];                   /* convolution result array */

/* FIR kernel: simple length-5 moving average */
const float kernel[KERNEL_LEN] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};


/* ----- Status flags ----- */
volatile bool system_enabled = false;       /* enabled/disabled status flag */
volatile bool adc_new_sample = false;       /* new ADC sample arrival */


/* ----- Discrete-Time Linear Convolution Function -----*/
void convolve(const float Signal[], unsigned int SignalLen,
              const float Kernel[], unsigned int KernelLen,
              float Result[]);


int main(void)
{
  
  /* ======================= Configurations ======================= */
  
  /* Stop watchdog timer */
  WDTCTL = WDTPW + WDTHOLD;

  /* -------- Port 6 configuration for ADC12 (channel A1 on PORT6/PIN1) -------- */
  P6DIR &= ~BIT1;   /* PORT6/PIN1 as input for ADC12 */
  P6SEL |=  BIT1;   /* PORT6/PIN1 => ADC12 Channel A1  */

  
  /* -------- Port 1 configuration for Enable input and LED output -------- */
  
  /* LED on PORT1/PIN7: output, initially OFF */
  P1OUT &= ~LED_BIT;     /* clear LED output (LED OFF) */
  P1DIR |=  LED_BIT;     /* PORT1/PIN7 as output (LED) */

  /* Enable input on PORT1/PIN0: input with interrupt on rising edge (L -> H) */
  P1DIR &= ~ENABLE_BIT;  /* PORT1/PIN0 as input */
  P1IE  |=  ENABLE_BIT;  /* enable interrupt for PORT1/PIN0 */
  P1IES &= ~ENABLE_BIT;  /* interrupt on low-to-high edge */
  P1IFG &= ~ENABLE_BIT;  /* clear any pending interrupt flags on PORT1/PIN0 */

  
  /* -------- 12-bit Analog to Digital Converter (ADC12) configuration  -------- */
  
  ADC12CTL0 &= ~ENC;         /* disable conversion for configuration */

  ADC12CTL0 |= ADC12ON       /* turn on ADC12 */
             + SHT0_4;       /* Number of sampling cycles: 64 */

  ADC12CTL1 |= ADC12SSEL_2   /* Clock source: MCLK */
             + ADC12DIV_3    /* ADC12 clock divider: /4 */
             + CSTARTADD_1   /* Conversion channel address: ADC12MEM1 */
             + CONSEQ_0      /* single-channel, single-conversion */
             + SHS_0         /* Conversion trigger: ADC12SC bit */
             + SHP;          /* SAMPCON source: sampling timer */

  ADC12IE    |= BIT1;        /* Enable interrupts on Channel A1 */
  ADC12MCTL1  = SREF_0       /* Voltage range: VR+ = AVCC to VR- = AVSS */
              + INCH_1;      /* Input channel: A1 (PORT6/PIN1) */

  ADC12CTL0 |= ENC;          /* enable conversions to operate */

  
  /* -------- Timer_A configuration for periodic sampling -------- */

  TACCR0 = 500;              /* Period: 500 */
  TACTL  = TASSEL_1          /* Clock source: ACLK */
         + MC_1              /* Counting Mode: Up mode (count up to TACCR0) */
         + TAIE;             /* enable interrupts triggered by TAIFG */

  
  __enable_interrupt();      /* Redundant: Set GIE=1 (LPM0 does this) */
  
  /* Enter LPM with GIE enabled */
  __low_power_mode_0();

  
  
  /* ==================== Convolution Calculation ==================== */
  
  while (1) {

    __disable_interrupt(); /* Disable interrupts during convolution calculation  */

    if (system_enabled && adc_new_sample) {

      unsigned int i;
      unsigned int idx;

      adc_new_sample = false;

      /*
       * Unwrap the circular buffer into a linear array:
       * - Oldest sample becomes signal_linear[0]
       * - Newest sample becomes signal_linear[SIGNAL_LEN - 1]
       *
       * When buffer is full, the oldest sample is at write_index,
       * because write_index points to the next position that will be
       * overwritten by the next sample.
       */
      idx = write_index;
      for (i = 0; i < SIGNAL_LEN; i++) {
        if (idx >= SIGNAL_LEN) {
          idx = 0;
        }
        signal_linear[i] = signal_buffer[idx];
        idx++;
      }

      /* Compute convolution between unwrapped digital Signal and Kernel */
      convolve(signal_linear, SIGNAL_LEN,
               kernel, KERNEL_LEN,
               result);

    }

    __enable_interrupt(); /* Redundant: Set GIE=1 (LPM0 does this) */

    /* Go back to LPM0 until next interrupt occurs */
    __low_power_mode_0();
  }

}



/* ==================== Key Functions and ISRs ==================== */


/**
 * @brief           : performs Discrete-time linear convolution between a signal and a kernel
 * @param Signal    : array that contains the values of the input signal
 * @param SignalLen : length N of input signal (array)
 * @param Kernel    : array with the values of kernel
 * @param KernelLen : length M of input signal (array)
 * @param Result    : array of N+M-1 size to which the result of the convolution is stored 
 * @return          : N/A
 */
void convolve(const float Signal[], unsigned int SignalLen,
              const float Kernel[], unsigned int KernelLen,
              float Result[])
{
  unsigned int n;

  for (n = 0; n < SignalLen + KernelLen - 1u; n++) {

    float acc = 0.0f;
    unsigned int kmin, kmax, k;
    
    kmin = (n >= (KernelLen - 1u)) ? n - (KernelLen - 1u) : 0u;
    kmax = (n < (SignalLen - 1u)) ? n : SignalLen - 1u ; 

    for (k = kmin; k <= kmax; k++) {
      acc += Signal[k] * Kernel[n - k];
    }
    
    Result[n] = acc;
  
  }
}


/**
 * @brief           : PORT1/PIN0 interrupt handler. After PORT1/PIN0 sees a low-to-high
 *                    edge and thus bit 0 of P1IFG is set, the enabled/disabled 
 *                    state is to toggled and LED of PORT1/PIN7 is turned on/off 
 *                    accordingly.
 * @return          : N/A
 */
#pragma vector = PORT1_VECTOR
__interrupt void Port1_ISR(void)
{

  system_enabled = !system_enabled; /* Toggle Enabled/Disabled state */

  if (system_enabled) {
    P1OUT |= LED_BIT;   /* LED ON when Enabled */
  } else {
    P1OUT &= ~LED_BIT;  /* LED OFF when Disabled */
  }

  P1IFG &= ~ENABLE_BIT; /* Clear interrupt flag of PORT1/PIN0 */

}


/**
 * @brief           : TimerA TACTL interrupt handler. After TAR counts up to TACCR0
 *                    and thus TAIFG is set, initiates Analog-to-Digital Conversion,
 *                    but only if system is enabled (PORT1/PIN7 LED ON).
 * @return          : N/A
 */
#pragma vector = TIMERA1_VECTOR
__interrupt void Timer_A_ISR_TAIFG(void)
{
  
  if (system_enabled) {
    ADC12CTL0 |= ADC12SC; /* Start Conversion */
  }
  
  TACTL &= ~TAIFG;        /* Reset TACTL interrupt flag*/

}


/**
 * @brief           : ADC12 ADC12MEM1 interrupt handler. After single conversion
 *                    value is stored to ADC12MEM1, and thus ADC12IFG1 is set, the 
 *                    digital value is stored on the circular buffer accordingly
 *                    and CPU exits LPM0 to perform convolution, only if system is
 *                    enabled (PORT1/PIN7 LED ON).
 * @return          : N/A
 */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
  unsigned int digital_value;

  digital_value = ADC12MEM1;  /* Read ADC12MEM1 and automatically reset ADC12IFG1 flag */

  if (system_enabled) {

    signal_buffer[write_index] = (float)digital_value;

    /* Update write_index so the next sample is stored correctly */
    write_index++;
    if (write_index >= SIGNAL_LEN) {
      write_index = 0;
    }

    adc_new_sample = true;
  }

  /* Exit LPM0 to begin convolution calculation */
  __low_power_mode_off_on_exit();
  
}

