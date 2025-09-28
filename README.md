# 🚗 E-RALLY V2 – Sinusoidal PWM BLDC Motor Controller (STM32 Blue Pill)

This project presents a **Sinusoidal PWM (Interpolated)** control algorithm for a **3-phase BLDC motor**, implemented on the **STM32F103C6T6** microcontroller (commonly known as the "Blue Pill"). It is an evolution of the E-RALLY V1 Trapezoidal controller, focused on delivering **smoother torque**, **lower acoustic noise**, and **reduced cogging** using a lookup-table and runtime interpolation.

Designed for the **E-RALLY electric vehicle prototype**, the firmware integrates Hall-effect sensor feedback, high-resolution PWM generation, ADC-based throttle control (with filtering), and multiple safety mechanisms to enable reliable motor control in a compact MCU.

---

## 👨‍💻 Project Contributors

Developed by the Embedded Systems Team of the E-RALLY competition vehicle:

- **Adham Amr** – LUT generator, Interpolation logic, PWM mapping, system integration, Debugging, Testing and Validation

---

## 🧠 Technical Overview

- **Microcontroller:** STM32F103C6T6 (ARM Cortex-M3, 72 MHz)
- **Motor Type:** 3-phase Brushless DC (BLDC)
- **Control Strategy:** Sinusoidal PWM (lookup table + interpolation)
- **Sensor Feedback:** 3 digital Hall-effect sensors
- **PWM Timer:** TIM1 with complementary outputs and dead-time
- **Throttle Input:** ADC1 (moving average filter)
- **Safety:** Dead-time insertion, invalid-state protection, soft disable (fault handling)

---

## 🔍 Features

- ✅ Smooth, low-ripple sinusoidal PWM (reduced torque ripple vs trapezoidal)
- ✅ Offline LUT generator (`LookUpTableGenerate.py`) with export to C header
- ✅ Efficient runtime interpolation for high effective resolution without heavy math
- ✅ Hall sensor decoding to determine electrical angle / commutation sector
- ✅ ADC throttle filtering and mapping to PWM amplitude
- ✅ Safety features: invalid Hall state handling, dead-time, soft-disable
- ✅ Modular and well-documented C code for maintainability

---

## 📦 Directory Structure

```

Control-BLDC-Sinusoidal\_PWM/
├── Core/
│   ├── Inc/        # Header files (main.h, control.h, lut.h)
│   ├── Src/        # Source files (main.c, control.c, interpolation.c)
│   └── Startup/    # Startup code and vector table
├── Drivers/
│   ├── STM32F1xx\_HAL\_Driver/ # HAL sources
│   └── CMSIS/      # Cortex-M core support
├── Tools/
│   └── LookUpTableGenerate.py # Python LUT generator (exports C header)
├── Debug/          # Binaries and debug artifacts
├── E-RALLY\_V2.ioc   # STM32CubeMX configuration
├── README.md       # Project README
└── STM32F103C6TX\_FLASH.ld # Linker script

````

---

## 🧩 Pin Configuration (same as V1)

| Function       | STM32 Pin       | Description |
|----------------|------------------|-------------|
| Hall Sensor A  | PA15             | Rotor feedback (bit 0) |
| Hall Sensor B  | PA12             | Rotor feedback (bit 1) |
| Hall Sensor C  | PA11             | Rotor feedback (bit 2) |
| Phase A High   | PA8 (TIM1_CH1)   | High-side PWM for Phase A |
| Phase B High   | PA9 (TIM1_CH2)   | High-side PWM for Phase B |
| Phase C High   | PA10 (TIM1_CH3)  | High-side PWM for Phase C |
| Phase A Low    | PA7 (TIM1_CH1N)  | Low-side PWM for Phase A |
| Phase B Low    | PB0 (TIM1_CH2N)  | Low-side PWM for Phase B |
| Phase C Low    | PB1 (TIM1_CH3N)  | Low-side PWM for Phase C |
| Throttle Input | PA0 (ADC1 IN0)   | Analog input (0–3.3V or 0–5V with divider) mapped to PWM |

> **Note:** Ensure ADC reference and throttle scaling match your hardware (voltage divider if using 0–5V throttle).

---

## ⚙️ Control Logic Summary

### Throttle Mapping
- ADC samples throttle on `PA0`.
- Moving-average filter (configurable buffer length) smooths noisy input.
- Filtered value mapped to PWM amplitude (scaled to TIM1 ARR/CCR range).

### Lookup Table & Interpolation
1. `LookUpTableGenerate.py` computes a high-resolution sine LUT for one electrical cycle and exports a C header (`lut.h`).
2. At runtime, the controller tracks rotor electrical angle using Hall sensors (and optional interpolation within a commutation sector).
3. The code reads two adjacent LUT entries and performs linear interpolation to obtain a high-resolution amplitude value, reducing runtime trig operations.
4. The interpolated amplitude is converted to PWM compare values and applied to TIM1 channels with complementary outputs.

### Hall Sensor Handling
- Hall sensors produce a 3-bit code (0–7). Valid codes map to 6 commutation sectors.
- The current sector + intra-sector position is used to index the LUT and compute phase offsets for the three outputs.
- Invalid Hall state triggers a safe shutdown (all PWM outputs off) and fault flag.

### Safety & Faults
- **Dead-time insertion** configured in TIM1 to prevent shoot-through.
- **Invalid Hall State** disables outputs and requires soft reset or user confirm to resume.
- **Overcurrent/Overtemp** hooks available for user-defined protection (not implemented by default).

---

## 🛠️ Build & Flash Instructions

### Prerequisites
- **STM32CubeIDE** (or arm-none-eabi toolchain)
- **ST-Link V2** or compatible programmer
- BLDC motor with Hall sensors and a 3-phase MOSFET driver (e.g., IR2101)
- Stable power supply, current-limited for testing

### Flashing Steps
```bash
# clone the repo
git clone https://github.com/Adham-amr-1/Control-BLDC-Sinusoidal_PWM.git
cd Control-BLDC-Sinusoidal_PWM
# open E-RALLY_V2.ioc with STM32CubeIDE, adjust settings if needed
# build & flash using STM32CubeIDE or use makefile if provided
````

* Run `LookUpTableGenerate.py` on your PC to generate/adjust `lut.h` and place it into `Core/Inc/` before building (optional — a default LUT is included).
* Build the project (Project → Build All).
* Connect ST-Link and flash (Run → Debug / Run).
* Power the motor driver with a low-power/current-limited supply for initial testing.

**Safety first:** never power the motor at full voltage without verifying gate-driver dead-time, FET wiring, and Hall phasing.

---

> The full interpolation implementation and LUT-format are in `Core/Src/interpolation.c` and `Tools/LookUpTableGenerate.py`.

---

## 🔬 Testing Recommendations

* Test with motor shaft free (no load) and current-limited supply.
* Verify Hall sensor codes and mapping before enabling PWM outputs.
* Observe phase voltages with oscilloscope to ensure proper sinusoidal shapes.
* Gradually increase throttle and monitor current/temperature.

---

## 📚 References & Documentation

* STM32F103C6 Datasheet – STMicroelectronics: [https://www.st.com/resource/en/datasheet/stm32f103c6.pdf](https://www.st.com/resource/en/datasheet/stm32f103c6.pdf)
* STM32CubeIDE & HAL manuals

---

## 📜 License

This project is distributed under the **MIT License**. STM32Cube autogenerated code is governed by STMicroelectronics' license located in `/Drivers`.

---

```
```
