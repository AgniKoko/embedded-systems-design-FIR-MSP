# Timer_A HAL – Documentation

This page documents the Timer_A Hardware Abstraction Layer (HAL) implemented
for the MSP430x1xx microcontroller family, as part of the course *Embedded
Systems Design*.

The aim of the HAL is to make the Timer_A peripheral easier and safer to use by
exposing a small, high-level API instead of raw register accesses. The design
follows the structure of chapter 11 of the MSP430x1xx User’s Guide and the
GPIO HAL developed in the lab sessions. 

[<- Back to GitHub repository](https://github.com/AgniKoko/embedded-systems-design-Timer_A)

---

## 1. Repository structure

| File / path        | Description |
|--------------------|-------------|
| `timerA_HAL_19.h`  | Public Timer_A HAL interface: enums, bitfields, configuration structs and API prototypes. |
| `timerA_HAL_19.c`  | Implementation of the HAL on top of the Timer_A control and capture/compare registers. |
| `timerA_main_19.c` | Example program demonstrating three typical use cases (counter, PWM generation, dynamic duty-cycle update). |
| `docs/timerA_report.pdf` | Full course report (Greek) with theoretical background and detailed design description. |

---

## 2. Implementation walkthrough 

This section summarises, step by step, how the Timer_A HAL and examples were
implemented.

1. **Study of Timer_A and requirements**  
   The work started from the MSP430x1xx User’s Guide (Timer_A chapter) and the
   course slides. From these, the main use cases were identified: simple
   counter, compare events and PWM generation.

2. **Definition of the HAL interface**  
   Before touching any registers, the high-level interface was sketched:
   configuration structs (`TimerA_Config`, `TimerA_ChannelConfig`,
   `TimerA_PWMConfig`) and a small set of API calls such as
   `TimerA_ApplyConfig`, `TimerA_ConfigureChannel`, `TimerA_ConfigPWM`,
   `TimerA_Start` and `TimerA_Stop`.

3. **Mapping enums and bitfields to control registers**  
   Enumerations were created for the key fields of `TACTL` and `TACCTLx`
   (clock source, divider, mode, output mode, capture settings). Bitfield
   types (`TimerA_TACTL_t`, `TimerA_TACCTL_t`) were added as an explicit
   mapping of the control registers, even though the implementation primarily
   uses bit masks.

4. **Global timer configuration (`TimerA_ApplyConfig`)**  
   The first core function configures `TACTL` and `TACCR0` from a
   `TimerA_Config`: it stops the timer, clears the counter and flags, sets the
   clock source/divider/mode and writes the period. This provides a safe way to
   switch configurations without leaving the timer running in an undefined
   state.

5. **Per-channel configuration (`TimerA_ConfigureChannel`)**  
   Helper functions select the correct `TACCTLx`/`TACCRx` register based on the
   requested channel (CCR0/1/2). The function then forces compare mode,
   configures `OUTMOD` and `CCIE` from `TimerA_ChannelConfig`, clears capture
   flags and writes the compare value.

6. **PWM helper (`TimerA_ConfigPWM`)**  
   A convenience function was added on top of the previous building blocks:
   it sets the period in `TACCR0`, configures the selected CCRx channel with
   a suitable output mode for PWM (e.g. reset/set) and the required duty
   value, and finally switches the timer to up mode.

7. **Utility functions (start/stop/read/write)**  
   Small helpers such as `TimerA_Start`, `TimerA_StartInMode`, `TimerA_Stop`,
   `TimerA_GetCounter`, `TimerA_ResetCounter`, `TimerA_SetPeriod`,
   `TimerA_SetDuty` and `TimerA_GetCaptureCompare` were implemented to hide
   direct access to `TAR` and `TACCRx` and provide a clean abstraction to the
   application.

8. **Example program (`timerA_main_19.c`)**  
   Finally, a simple `main()` was written to exercise the HAL in three
   scenarios: a basic counter, a fixed-duty PWM signal on CCR1 and run-time
   duty-cycle changes. These examples serve both as tests and as usage
   documentation.

---

## 3. The Timer_A peripheral (summary)

Timer_A is a 16-bit timer/counter with three capture/compare channels
(TACCR0–TACCR2). It supports four counting modes (stop, up, continuous,
up/down), PWM generation and multiple interrupt sources (overflow and
capture/compare events). The main control and data registers used by this HAL
are:  

- `TACTL` – Timer_A Control Register (clock source selection, divider, mode,
  clear, timer interrupt enable/flag).
- `TAR` – Timer_A counter value.
- `TACCTLx` – Capture/Compare Control Registers (x = 0,1,2).
- `TACCRx` – Capture/Compare Registers (x = 0,1,2), used here mainly for
  compare and PWM.

The HAL never re-defines these registers; they are provided by the vendor
device header (e.g. `msp430x14x.h`) and are manipulated via bit masks and
helper functions. 

---

## 4. HAL design

### 4.1 Enumerations and bitfields

The header file defines enums that mirror the fields of `TACTL` and `TACCTLx`
so that application code can use symbolic names instead of literal values:  

- `TimerA_ClockSource` → `TASSELx` (TACLK / ACLK / SMCLK / INCLK)
- `TimerA_ClockDivider` → `IDx` (÷1, ÷2, ÷4, ÷8)
- `TimerA_Mode` → `MCx` (stop, up, continuous, up/down)
- `TimerA_OutputMode` → `OUTMODx` (output/PWM mode)
- plus enums for capture mode and capture input selection.

Two bitfield types, `TimerA_TACTL_t` and `TimerA_TACCTL_t`, provide an
alternative, more explicit mapping of `TACTL` and `TACCTLx` bits to named
fields. The current implementation mainly uses bit masks, but the bitfields are
useful for debugging or future extensions. 

### 4.2 Configuration structs and API

To configure Timer_A at a higher level, the HAL uses three structs:  

- **`TimerA_Config`**  
  Global timer configuration (clock source, divider, mode, `TACCR0` period and
  Timer_A interrupt enable).

- **`TimerA_ChannelConfig`**  
  Per-channel configuration (which CCRx channel is used, output mode, compare
  value in `TACCRx`, and CC interrupt enable).

- **`TimerA_PWMConfig`**  
  Convenience struct for PWM (period in `TACCR0`, duty in `TACCRx`, output
  mode and channel).

The main API functions are:

- `TimerA_ApplyConfig(const TimerA_Config *cfg)`  
  Applies a global configuration: stops the timer, clears `TAR` and `TAIFG`,
  sets clock source/divider/mode and writes the period into `TACCR0`. 

- `TimerA_ConfigureChannel(const TimerA_ChannelConfig *chCfg)`  
  Selects the appropriate `TACCTLx`/`TACCRx`, forces compare mode, clears
  capture-related bits and flags, sets `OUTMOD` and `CCIE`, and writes the
  channel’s compare value. 

- `TimerA_ConfigPWM(const TimerA_PWMConfig *pwmCfg)`  
  High-level helper that sets `TACCR0` to the desired period, configures the
  selected channel’s `TACCTLx/TACCRx` for PWM using
  `TimerA_ConfigureChannel()`, stores `MODE_UP` as the last mode and updates
  the `MC` bits in `TACTL`. 

- Utility functions:  
  `TimerA_Start`, `TimerA_StartInMode`, `TimerA_Stop`, `TimerA_GetCounter`,
  `TimerA_ResetCounter`, `TimerA_SetPeriod`, `TimerA_SetDuty`,
  `TimerA_GetCaptureCompare`. These hide direct register access from the
  application and provide a small, coherent interface to the peripheral.

---

## 5. Usage examples (`timerA_main_19.c`)

The example `main()` demonstrates how to use the HAL in three scenarios:  

1. **Simple counter**  
   - Configure Timer_A in up mode with SMCLK and a prescaler of 8.  
   - Read `TAR` into `t1`, `t2`, `t3` with software delays and a reset in
     between to show how the counter evolves.  
   - Demonstrates `TimerA_ApplyConfig`, `TimerA_Start`,
     `TimerA_GetCounter`, `TimerA_ResetCounter`, `TimerA_Stop`.

2. **PWM generation on CCR1**  
   - Use `TimerA_PWMConfig` with `period = 1000`, `duty = 500` and
     `OUTMODE_RST_SET`.  
   - Call `TimerA_ConfigPWM()` to set up `TACCR0` and `TACCR1` and start the
     timer in up mode.  
   - The corresponding CCR1 output pin (configured as timer function in GPIO)
     will output a 50 % duty-cycle PWM signal.

3. **Dynamic duty-cycle update**  
   - After a delay, change the duty from 50 % to 75 % (`TACCR1 = 750`), then
   to 25 % (`TACCR1 = 250`).  
   - Read back the current CCR1 value with `TimerA_GetCaptureCompare()`.  
   - Demonstrates run-time PWM control using `TimerA_SetDuty()`.

---

## 6. Full report

For a more detailed explanation (in Greek) of the theoretical background and
design choices, including references to the MSP430x1xx User’s Guide and course
material, see:

- [`timerA_report.pdf`](docs/timerA_report.pdf)
