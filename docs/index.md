# FIR (ADC12 + Timer_A) – Documentation

This page documents an MSP430x1xx application that samples an analog signal through **ADC12** and applies a simple **FIR filter** (discrete-time linear convolution) on a sliding window of samples. Sampling is paced by **Timer_A**, and the CPU stays mostly in **LPM0** to reduce power.

[<- Back to GitHub repository](https://github.com/AgniKoko/embedded-systems-design-FIR-MSP)

---

## 1. Repository structure

| File / path | Description |
|---|---|
| `src/FIR_ADC_main_19.c` | Complete implementation: ADC12 + Timer_A setup, ISR logic, circular buffer management, FIR kernel and `convolve()` function. |
| `docs/part2_report.pdf` | Full report (Greek) with detailed analysis and references to the MSP430x1xx User’s Guide. |
| `docs/index.md` | This page (GitHub Pages). |
| `docs/system_overview.svg` | Small block diagram of the signal path. |

---

## 2. Implementation walkthrough

1. **Study of requirements & peripherals**  
   The design is based on the MSP430x1xx User’s Guide (ADC12 and Timer_A chapters) and the Part #02 project requirements.

2. **ADC12 configuration (channel A1 on P6.1)**  
   * Input: `INCH_1` → **A1 / P6.1**  
   * References: `SREF_0` → **AVCC/AVSS**  
   * Results stored in `ADC12MEM1` (`CSTARTADD_1`)  
   * Single-channel / single-conversion (`CONSEQ_0`) with internal sample timer (`SHP`)  
   * Interrupt enabled on `ADC12IFG1` (`ADC12IE |= BIT1`)

3. **Timer_A as a sampling clock**  
   Timer_A runs in **up mode** (`MC_1`) using **ACLK** (`TASSEL_1`). When the counter reaches `TACCR0`, the timer interrupt fires and the ISR triggers a conversion by setting `ADC12SC`.

4. **Enable / Disable input on P1.0 + status LED on P1.7**  
   A GPIO interrupt on **P1.0** toggles `system_enabled`. The LED on **P1.7** mirrors this state (ON = enabled, OFF = disabled).

5. **Circular buffer for ADC samples**  
   Each new ADC sample (when enabled) is stored in `signal_buffer[write_index]`. The index wraps around, keeping the latest **SIGNAL_LEN** samples.

6. **Main loop: new sample → convolution → sleep**  
   When `adc_new_sample` is set by the ADC ISR:
   * interrupts are temporarily disabled to avoid buffer corruption,
   * the circular buffer is “unwrapped” into `signal_linear[]` (oldest → newest),
   * `convolve()` computes the linear convolution into `result[]`,
   * interrupts are re-enabled and the CPU returns to **LPM0**.

---

## 3. FIR filter and convolution (summary)

The FIR output is computed with the standard discrete-time linear convolution:

`y[n] = Σ x[k] · h[n-k]`  for `n = 0 .. N+M-2`

In this project:
* `x[]` is the most recent window of ADC samples (`SIGNAL_LEN = 16`)
* `h[]` is a simple **moving-average kernel** (`KERNEL_LEN = 5`, coefficients = 0.2)

---

## 4. Timing note (interrupt safety)

During the convolution step, interrupts are disabled for a short time. To avoid losing interrupt requests, the sampling period should be larger than the worst-case “interrupts-off” time:

`T_sampling > T_disable`

The report includes a measurement example and a practical upper bound on the sampling frequency.

---

## 5. Full report

For the complete analysis, theoretical background, register configuration details and timing measurements, see:

* [part2_report.pdf](part2_report.pdf)
