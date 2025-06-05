# STM32F4 Temperature Sensing with FreeRTOS

## Project Overview

This project demonstrates a temperature sensing application on the STM32F446RE microcontroller using the onboard ADC to read the internal temperature sensor. It uses FreeRTOS for multitasking with dedicated tasks for ADC sampling, LED status indication based on temperature thresholds, and UART logging. The temperature data is shared between tasks using a mutex for thread-safe access.

---

## Purpose

- To sample temperature using ADC with FreeRTOS multitasking  
- To signal temperature states via LED behavior (ON, blinking, OFF) based on thresholds  
- To log temperature data over UART for monitoring or debugging

---

## Technical Details

### Hardware

- **MCU:** STM32F446RE (Cortex-M4 @ 84 MHz)  
- **Peripherals Used:** ADC (internal temperature sensor), GPIO (LED), USART2 (UART)  
- **Board:** NUCLEO-F446RE  

### Software

- **RTOS:** FreeRTOS (tasks for ADC reading, LED control, UART logging)  
- **Development Environment:** STM32CubeIDE  
- **HAL:** STM32Cube HAL library for peripheral abstraction  

### Task Breakdown

| Name       | Priority        | Stack Size (bytes) | Notes              |
|------------|------------------|---------------------|---------------------|
| ADC_Task   | osPriorityNormal | 512                 | Handles ADC reading and temperature conversion |
| LED_Task   | osPriorityLow    | 512                 | Controls LED based on temperature thresholds    |
| UART_Task  | osPriorityLow    | 1024                | Logs temperature data over UART                |


### Synchronization

- **Mutex:** `gx_temp_mutex_handle` protects shared temperature variable (`gf_temperature`)

### Temperature Thresholds

- `TEMP_HIGH_THRESHOLD` (29.0 °C): LED blinks when temperature exceeds this value  
- `TEMP_LOW_THRESHOLD` (26.0 °C): LED turns ON when temperature is above this but below the high threshold  
- Below `TEMP_LOW_THRESHOLD`, LED remains OFF  

---

## Naming Conventions

- Variables use **snake_case**  
- Prefixes:  
  - `g_` for global variables (e.g., `g_temperature`)  
  - `lu32_` for local `uint32_t` variables  
  - `str_` for character arrays (strings)  
  - `f_` or `lf_` for floats (local floats)  
  - `gx_` for global FreeRTOS objects (e.g., `gx_temp_mutex_handle`)  
- Task functions follow `snake_case` with `_task` suffix (e.g., `adc_task`, `led_task`)  

---

## Key Functions

### `convert_adc_to_temperature(uint32_t raw_adc)`

Converts raw ADC value to temperature in Celsius using factory calibration values embedded in MCU memory.

### Task Functions

- `adc_task`: Periodically reads ADC, converts to temperature, updates shared variable under mutex protection  
- `led_task`: Reads temperature under mutex; toggles or sets LED according to temperature thresholds  
- `uart_task`: Reads temperature under mutex; sends temperature data via UART for monitoring  

---

## Usage

1. Clone this repository.  
2. Open the project in STM32CubeIDE.  
3. Build and flash the code to an STM32F446RE-based board.  
4. Connect UART2 pins to a serial terminal at 115200 baud to monitor temperature logs. e.g. using Tera Term:
![UART Output on Teraterm](images/uart_output.png) 
5. Observe LED behavior as temperature varies. 

---

## Future Possibilities
 
- Add timestamp in UART Logging
- Extend UART communication for remote monitoring or control via PC 
- Add button input or command parser for dynamic threshold adjustment

---

## License

This project is open-source and free to use for educational and personal projects.