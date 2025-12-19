# FIR (ADC12 + Timer_A) on MSP430x1xx

This repository contains an MSP430x1xx application that performs **periodic ADC sampling** (ADC12 on **P6.1 / A1**) and applies a simple **FIR filter** (discrete-time linear convolution) on a sliding window of samples.

Sampling is paced by **Timer_A** (ACLK, up mode). The system can be **enabled/disabled** through an external signal on **P1.0**, while **P1.7** drives an LED that indicates the current state.

![System overview](docs/system_overview.svg)

## File overview

* `src/FIR_ADC_main_19.c`  
  Single-file implementation of the complete pipeline:
  * ADC12 configuration for single-channel, single-conversion on A1 (results in `ADC12MEM1`)
  * Timer_A periodic interrupt to trigger conversions (`ADC12SC`)
  * GPIO interrupt on P1.0 to toggle *Enable/Disable* and drive the LED on P1.7
  * circular buffer (`signal_buffer[]`) + linear unwrap (`signal_linear[]`)
  * FIR kernel (length 5 moving average) + `convolve()` implementation
  * main loop: **new sample → convolution → back to LPM0**

* `docs/part2_report.pdf`  
  Full course report (Greek): theoretical background (ADC12 + Timer_A), ISR design, convolution implementation and timing constraints.

* `docs/index.md`  
  GitHub Pages / HTML documentation summary (mirrors the report at a higher level).

## Target platform & dependencies

* MCU family: **MSP430x1xx**
* Peripherals used: **ADC12**, **Timer_A**, GPIO Port 1 / Port 6
* Assumes a vendor device header is available in the include path (e.g. `msp430x14x.h`) so that memory-mapped registers (`ADC12CTLx`, `ADC12MEM1`, `TACTL`, `TACCR0`, …) are defined.
* Typical toolchains: **IAR Embedded Workbench**, **Code Composer Studio**, or any MSP430-compatible compiler.

## Quick usage example

### Changing the sampling rate

Sampling frequency is controlled by `TACCR0` (Timer_A up-mode period) and the selected clock:

```c
/* Timer_A configuration for periodic sampling */
TACCR0 = 500;             /* Period ticks (ACLK) */
TACTL  = TASSEL_1         /* ACLK */
       + MC_1             /* up mode: 0..TACCR0 */
       + TAIE;            /* overflow interrupt */
```

> Tip: keep the sampling period comfortably larger than the worst-case time where interrupts are disabled during convolution (see `docs/part2_report.pdf`).

### FIR kernel and convolution call

The example uses a simple 5-tap moving-average FIR:

```c
#define SIGNAL_LEN  16u
#define KERNEL_LEN  5u
#define RESULT_LEN  (SIGNAL_LEN + KERNEL_LEN - 1u)

const float kernel[KERNEL_LEN] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};

/* ...unwrap circular buffer into signal_linear[]... */

convolve(signal_linear, SIGNAL_LEN, kernel, KERNEL_LEN, result);
```

## Documentation

* Full report: `docs/part2_report.pdf`
* GitHub Pages summary: `docs/index.md`

## About

Project page (GitHub Pages): https://agnikoko.github.io/embedded-systems-design-FIR-MSP/
