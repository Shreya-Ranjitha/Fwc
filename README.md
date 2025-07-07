# ✨Internship work at Future Wireless Communication at IIT Hyderabad✨

Welcome to my internship documentation repository. This repository contains a detailed record of my work, assignments, and learning progress during my internship at IIT Hyderabad (IITH).

---

## Table of Contents

- [About](#about)
- [Setup Details](#setup-details)
- [PlatformIO](#platformio)
- [Assembly Programming](#assembly-programming)
- [Embedded C](#embedded-c)
- [Vaman Esp32](vaman-esp)
- [Vaman FPGA](vaman-fpga)
- [Vaman Arm](vaman-arm)
- [Acknowledgements](#acknowledgements)

---

## 🙋‍♀️About

I applied for the Summer Research Fellowship Programme (SRFP) offered by the Indian Academy of Sciences (IASc) during my third semester. After submitting my application with academic details and a statement of purpose, I was thrilled to receive my selection letter in March. I was assigned to work under the guidance of Dr. GVV Sharma at the Indian Institute of Technology (IIT) Hyderabad. This internship has been a great opportunity to dive deeper into research and practical problem-solving. This repository serves as a portfolio of my internship work, documenting my progress, solutions, and notes as I complete assignments and projects related to digital design.

---

## 📱Setup Details

This project follows the same setup used in Prof. G.V.V. Sharma’s [FWC repository](https://github.com/gadepall/fwc-1?tab=readme-ov-file). All required packages, toolchains, and instructions for Termux-based development are listed there.

You can also refer to the digital design book and supporting material using:

```bash
git clone https://github.com/gadepall/digital-design
```
---

## 📃PlatformIO

PlatformIO is an open-source ecosystem for embedded development. It simplifies compiling and uploading firmware to microcontrollers like the ATmega328P.


### 1. Create or Open a PlatformIO Project

Use PlatformIO CLI to create a new project:

```bash
pio project init --board atmega328p
```

> Replace `atmega328p` with your actual board ID. Use `pio boards` to list available options.


### 2. Write Your Firmware in `src/main.cpp`

Example:

```cpp
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    DDRB |= (1 << PB0); // Set PB0 as output

    while (1) {
        PORTB ^= (1 << PB0); // Toggle LED
        _delay_ms(500);
    }
}
```


### 3. Configure `platformio.ini`

Edit the `platformio.ini` file in the root directory of your project:

```ini
[env:atmega328p]
platform = atmelavr
board = atmega328p
framework = arduino
upload_protocol = custom
upload_command = avrdude -v -p m328p -c usbasp -U flash:w:.pio/build/atmega328p/firmware.hex:i
```


### 4. Build the Project

Use PlatformIO to compile the firmware:

```bash
pio run
```


### 5. Upload the Firmware

Make sure your board is connected to your android device and upload the code using Arduino Droid.



### ⛏ Assignment
▫ GATE EE-2017,36 - A 10½ digit timer counter possesses a base clock of frequency 100 MHz. When measuring a
particular input, the reading obtained is the same in: (i) Frequency mode of operation with a gating
time of one second and (ii) Period mode of operation (in the x 10 ns scale). The frequency of the
unknown input (reading obtained) in Hz is _________. 
Ans: 1 0 0 0 0 0 0 0 0

▫ Implementation: A digital timer counter based on a frequency of 100Hz is difficult to visualise. A 5Hz-based counter was implemented. Pin 8 is toggled, generating a pulse every 200ms for this purpose. The pulse was visualised using a led initially. Number of pulses were also counted using a 7 segment display later.

▫ Code:
```bash
#include <Arduino.h>

volatile uint16_t pulseCount = 0;
uint8_t displayValue = 0;
unsigned long lastUpdate = 0;
unsigned long lastPulseTime = 0;

void countPulse() {
    pulseCount++;
}

void setup() {
    pinMode(2, INPUT);                         // Pulse input
    attachInterrupt(digitalPinToInterrupt(2), countPulse, RISING);

    for (int i = 4; i <= 7; i++) {
        pinMode(i, OUTPUT);                    // Binary display output
    }

    pinMode(8, OUTPUT);                        // Pulse generation output
}

void loop() {
    // Generate a pulse every 200 ms on pin 8
    if (millis() - lastPulseTime >= 200) {
        digitalWrite(8, HIGH);
        delayMicroseconds(10);  // short HIGH pulse (~10 microseconds)
        digitalWrite(8, LOW);
        lastPulseTime = millis();
    }

    // Every 1 second, display pulse count % 10 on binary pins 4–7
    if (millis() - lastUpdate >= 1000) {
        noInterrupts();
        displayValue = pulseCount % 10;
        pulseCount = 0;
        interrupts();

        digitalWrite(4, displayValue & 0x01);
        digitalWrite(5, (displayValue >> 1) & 0x01);
        digitalWrite(6, (displayValue >> 2) & 0x01);
        digitalWrite(7, (displayValue >> 3) & 0x01);

        lastUpdate = millis();
    }
}

```
▫ Output Video:

[Using LED](videos/video_1.mp4)  
[Using 7 segment](videos/video_2.mp4)

---
## 🛠 Assembly Programming
Assembly programming lets you interact directly with the microcontroller hardware, offering maximum control and efficiency. Below are the steps to set up, write, assemble, and upload assembly code for the ATmega328P (the chip on Arduino Uno).

### 1. Install Required Tools
AVRA: Assembler for AVR microcontrollers.
avrdude: Tool to upload .hex files to Arduino.

### 2. Create Your Assembly Program
Write your code in a file, e.g., hello.asm. Example to turn on LED on pin 13 (PB5):

```
; hello.asm - Turns on LED on PB5 (Arduino digital pin 13)
.include "m328Pdef.inc"

ldi r16, 0b00100000  ; Set bit 5 (PB5) in r16
out DDRB, r16        ; Set PB5 as output
out PORTB, r16       ; Set PB5 high (LED ON)

loop:
  rjmp loop          ; Infinite loop
```
Note: Adjust the .include path as needed.

### 3. Assemble the Program
Run:
```
avra hello.asm
```
This generates hello.hex (machine code for the microcontroller).

### 4. Upload the Hex File to Arduino
Connect your Arduino via USB. Upload using Arduino Droid.

### 5. Verify Operation
The LED on pin 13 should turn on.
Modify your assembly code as needed (e.g., to blink the LED, add delays and toggle logic).

### ⛏ Assignment
▫ GATE EE-2017,47 - A finite state machine (FSM) is implemented using the D flip-flops A and B, and logic gates, as shown in the figure below. The four possible states of the FSM are  Q_A Q_B = 00, 01, 10, and 11.

Assume that X_{IN}  is held at a constant logic level throughout the operation of the FSM. When the FSM is initialized to the state  Q_A Q_B = 00  and clocked, after a few clock cycles, it starts cycling through

- (A) all of the four possible states if  X_{IN} = 1 
- (B) three of the four possible states if  X_{IN} = 0 
- (C) only two of the four possible states if X_{IN} = 1 
- (D) only two of the four possible states if X_{IN} = 0 


▫ Code:
```bash
; Registers:
; R16 - Q_A
; R17 - Q_B
; R18 - X_IN (0 or 1)
; R19 - D_A (temp)
; R20 - D_B (temp)

; Assume LED1 is connected to Arduino pin 8 (PORTB0)
; Assume LED2 is connected to Arduino pin 9 (PORTB1)

    ldi R16, 0      ; Q_A = 0
    ldi R17, 0      ; Q_B = 0
    ldi R18, 1      ; X_IN = 1 (set to 0 or 1 as needed)

    sbi DDRB, 0     ; Set PORTB0 (pin 8) as output
    sbi DDRB, 1     ; Set PORTB1 (pin 9) as output

main_loop:
    ; D_A = Q_A XOR Q_B
    mov R19, R16
    eor R19, R17

    ; D_B = Q_A AND X_IN
    mov R20, R16
    and R20, R18

    ; Output Q_A and Q_B to LEDs
    cpi R16, 0
    breq led1_off
    sbi PORTB, 0    ; Turn on LED1
    rjmp led2_check
led1_off:
    cbi PORTB, 0    ; Turn off LED1

led2_check:
    cpi R17, 0
    breq led2_off
    sbi PORTB, 1    ; Turn on LED2
    rjmp update_state
led2_off:
    cbi PORTB, 1    ; Turn off LED2

update_state:
    ; Update Q_A and Q_B
    mov R16, R19
    mov R17, R20

    rjmp main_loop


```
▫ Output Video:

[Demonstration](videos/video_3.mp4)  

---
## ⚙ Embedded C
This includes Embedded C programs for microcontroller development, focusing on AVR-GCC and the ATmega328P/ATmega32 series. 

### 1. Required Tools
To build and run these examples, you will need:
AVR-GCC toolchain (for compiling C code)
AVRDUDE (for uploading hex files to the microcontroller)
Supported hardware: ATmega328P, ATmega32, or compatible

### 2. Example Blink Code
```
//Turns LED on and off
#include <avr/io.h>
#include <util/delay.h>

 
int main (void)
{
	
	
  /* Arduino boards have a LED at PB5 */
 //set PB5, pin 13 of arduino as output
  DDRB    |= ((1 << DDB5));
  while (1) {
//turn led off    
    PORTB = ((0 <<  PB5));
	_delay_ms(500);
//turn led on
    PORTB = ((1 <<  PB5));
    _delay_ms(500);
  }

  /* . */
  return 0;

}
```

### 3. Assemble the Program
Run:
```
make
```
This generates hello.hex (machine code for the microcontroller).

### 4. Upload the Hex File to Arduino
Connect your Arduino via USB. Upload using Arduino Droid.

### 5. Verify Operation
The LED on pin 13 should turn on.
Modify your assembly code as needed (e.g., to blink the LED, add delays and toggle logic).

### ⛏ Assignment
▫ GATE EE-2017,44 - A 4-bit shift register circuit configured for right-shift operation, i.e.
Din → A, A → B, B → C, C → D, is shown.  
If the present state of the shift register is ABCD = 1101, the number of clock cycles required to reach the state ABCD = 1111 is __________.

▫ Code:
```bash
#include <avr/io.h>
#include <util/delay.h>

void setup_pins() {
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
}

void display_state(uint8_t state) {
    if (state & 0x08) PORTB |= (1 << PB0); else PORTB &= ~(1 << PB0);
    if (state & 0x04) PORTB |= (1 << PB1); else PORTB &= ~(1 << PB1);
    if (state & 0x02) PORTB |= (1 << PB2); else PORTB &= ~(1 << PB2);
    if (state & 0x01) PORTB |= (1 << PB3); else PORTB &= ~(1 << PB3);
}

int main(void) {
    setup_pins();
    uint8_t reg = 0b1101;
    display_state(reg);
    _delay_ms(1000);
    while (reg != 0b1111) {
        uint8_t A = (reg & 0x08) >> 3; // MSB
        uint8_t D = (reg & 0x01);     // LSB
        uint8_t din = A ^ D;          // XOR for feedback
        reg = (din << 3) | (reg >> 1);
        display_state(reg);
        _delay_ms(1000);
    }
    while (1);
}

```
▫ Output Video:

[Demonstration](videos/video_4.mp4)  

---
## 🔧Vaaman Esp32

### 1. Tools Required

**Hardware Requirements:**
Vaman board
USB cable for programming
Raspberry Pi (for ARM programming)
Android device (for mobile setup)
WiFi network access

**Software Requirements:**
F-Droid (Android)
Termux and Termux-API
PlatformIO
Python 3
Git

### 2. Flashing through UART and RaspberryPi
i. Ensure the Vaman board does not power any devices.
ii. Make the following UART connections:

   | VAMAN-ESP | UART PINS |
   |-----------|-----------|
   | 5V        | 5V        |
   | GND       | GND       |
   | TXD0      | TXD       |
   | RXD0      | RXD       |
   | 0         | GND       |

iii. Refer to the Vaman pin diagram for details.
iv. Connect the UART to the Raspberry Pi using USB.
v. Connect the pins between Vaman-ESP32 and Vaman-PYGMY as follows:

   | ESP32  | Vaman   |
   |--------|---------|
   | GPIO2  | GPIO18  |
   | GPIO4  | GPIO21  |
   | GPIO5  | GPIO22  |

vi. Run the following commands to build and upload the code:
```
cd vaman/esp32/codes/ide/blink
pio run
```

vii. Transfer the ini and bin files to the Raspberry Pi:
```
scp platformio.ini pi@192.168.50.252:~/hi/platformio.ini
scp .pio/build/esp32doit-devkit-v1/firmware.bin pi@192.168.50.252:~/hi/.pio/build/esp32doit-devkit-v1/firmware.bin
```

viii. On the Raspberry Pi, upload the firmware:
```
cd /home/pi/hi
pio run --target upload
```

ix. Confirm the onboard LED blinks.

### 3. OTA Setup

i. Flash the following code:
 ```
 vaman/esp32/codes/ide/ota/setup
 ```
 Enter your WiFi credentials:
 ```
 #define STASSID "your_ssid"
 #define STAPSK "your_password"
 ```

ii. In `src/main.cpp`, find the IP address of your Vaman-ESP:
 ```
 ifconfig
 nmap -sn 192.168.231.1/24
 ```

### 4. Flashing through OTA
i. Disconnect Pi and directly power your Vaaman using USB. Make sure the Vaaman is connected to your wifi.
ii. Assuming your computer's IP is `192.168.231.245`, flash the code wirelessly:
 ```
 pio run
 pio run --target upload --upload-port 192.168.231.245
 ```

iii. Flash the OTA blink code:
 ```
 vaman/esp32/codes/ide/ota/blink
 ```
iv. Flash any code in similar manner

### ⛏ Assignment
▫ GATE EC-2017,17 - Consider the D-Latch shown in the figure, which is transparent when its clock input CK is high and has zero propagation delay. The clock signal CLK1 has a 50% duty cycle, and CLK2 is a one-fifth period delayed version of CLK1. The duty cycle at the output of the latch in percentage is _______.

▫ Code:
```bash
#include <Arduino.h>

volatile uint16_t pulseCount = 0;
uint8_t displayValue = 0;
unsigned long lastUpdate = 0;

void countPulse() {
    pulseCount++;
}

void setup() {
    pinMode(2, INPUT);
    attachInterrupt(digitalPinToInterrupt(2), countPulse, RISING);

    for (int i = 4; i <= 7; i++) {
        pinMode(i, OUTPUT);
    }
}

void loop() {
    if (millis() - lastUpdate >= 1000) {
        noInterrupts();
        displayValue = pulseCount % 10;
        pulseCount = 0;
        interrupts();

        digitalWrite(4, displayValue & 0x01);
        digitalWrite(5, (displayValue >> 1) & 0x01);
        digitalWrite(6, (displayValue >> 2) & 0x01);
        digitalWrite(7, (displayValue >> 3) & 0x01);

        lastUpdate = millis();
    }
}

```
▫ Output Video:

[Demonstration](videos/video_5.mp4)

---
## 🔧Vaaman FPGA

### 1. **Follow the Setup Instructions**
   - Refer to the video instructions at:
     ```
     https://github.com/whyakari/TermuxDisableProcess?tab=readme-ov-file
     ```
   - This ensures Termux is not killed during installation.

### 2. **Install Required Scripts on Termux-Debian**
   - Run:
     ```
     wget https://raw.githubusercontent.com/gadepall/fwc-1/main/scripts/setup.sh
     bash setup.sh
     ```

### 3. **Login to Termux-Debian on Android and Prepare the Environment**
   - Execute:
     ```
     cd vaman/fpga/setup/codes/blink
     source ~/.vamenv/bin/activate
     ql_symbiflow --compile --src vaman/fpga/setup/codes/blink -d ql-eos-s3 -P PU64 -v helloworldfpga.v -t helloworldfpga -p pygmy.pcf --dump binary
     scp blink/helloworldfpga.bin pi@192.168.0.114:
     ```
   - Replace the IP address with that of your Raspberry Pi.

### 4. **Put the Vaman Board in Download Mode**
   - Press the button to the right of the USB port, then immediately press the button to the left. The green LED should flash, indicating download mode.

### 5. **On the Raspberry Pi, Set Up the Python Environment**
   - Run:
     ```
     python3 -m venv ~/.vamenv
     source ~/.vamenv/bin/activate
     git clone --recursive https://github.com/QuickLogic-Corp/TinyFPGA-Programmer-Application.git
     pip3 install tinyfpga
     deactivate
     sudo reboot
     source ~/.vamenv/bin/activate
     python3 TinyFPGA-Programmer-Application/tinyfpga-programmer-gui.py --port /dev/ttyACM0 --appfpga /home/pi/helloworldfpga.bin --mode fpga --reset
     ```
   - Ensure the correct USB port address is used.

### 6. **Verify LED Operation**
   - The LED will start blinking red if programmed correctly.

### 7. **Edit the Pin Constraints File**
   - In `codes/blink/pygmy.pcf`, ensure correct pin mappings for your code.

### 8. **Modify Verilog Code**
   - Update `helloworldfpga.v` as needed.

### 9. **Recompile and Upload**
    - Repeat the compilation and upload steps above with your updated code and constraints.
    
### ⛏ Assignment
▫ GATE EE-2018,14 - In the logic circuit shown in the figure, Y is given by

(A) Y = ABCD  
(B) Y = (A + B)(C + D)  
(C) Y = A + B + C + D  
(D) Y = AB + CD  



▫ Code:
```bash
module sevenseg(input a, input b, input c, input d, output y);
assign y= ~ (~(a&b) & ~(c&d))
endmodule

```
▫ Output Video:

[Demonstration](videos/video_6.mp4) 

---
## 🔧Vaaman Arm
These steps provide a concise workflow for programming an ARM Cortex-M4 on the Vaman board to control an LED, including building, transferring, and flashing the code, as well as modifying the source for custom behaviors.

### 1. **Check Your Path**
   - Open a terminal and navigate to the project directory:
     ```
     cd vaman/arm/setup/blink/GCC_Project
     nvim config.mk
     ```
   - Modify the configuration file as needed.

### 2. **Set Project Environment Variable**
   - Export your project root:
     ```
     export PROJ_ROOT=/data/data/com.termux/files/home/pygmy-dev/pygmy-sdk
     ```

### 3. **Build the Project**
   - Execute the following commands to build the project:
     ```
     cd vaman/arm/setup/blink/GCC_Project
     make -j4
     ```

### 4. **Transfer the Binary to Raspberry Pi**
   - Send the generated binary to your Raspberry Pi (replace with your Pi's IP address):
     ```
     scp output/bin/blink.bin pi@192.168.0.114:
     ```

### 5. **Program the FPGA via Raspberry Pi**
   - Log in to the Raspberry Pi and run:
     ```
     sudo python3 /home/pi/Vaman-dev/pyVaman-sdk/TinyFPGA-Programmer-Application/tinyfpga-programmer-gui.py --port /dev/ttyACM0 --m4app blink.bin --mode m4-fpga
     ```

### 6. **Connect USB Device and Execute**
   - Connect the appropriate USB device to the Raspberry Pi.
   - Press the button to the left of the USB port on the Vaman board to enter programming mode.
   - After running the command, the onboard LED should start blinking if the process is successful.

### 7. **Edit Source Code for Custom Behavior**
   - To change the blink frequency or pin, modify the C source file:
     ```
     codes/setup/blink/src/main.c
     ```
   - Adjust the delay or GPIO pin as required, then repeat the build and upload process.

### 8. **Pin Reference**
   - Refer to the Vaman board's pin definition table for correct GPIO assignments.

### ⛏ Assignment
▫ GATE EC-2017,15 - In the latch circuit shown, the NAND gates have non-zero, but unequal propagation delays. The present input condition is: P = Q = '0'. If the input condition is changed simultaneously to P = Q = '1', the outputs X and Y are

(A) X = '1', Y = '1'  
(B) either X = '1', Y = '0' or X = '0', Y = '1'  
(C) either X = '1', Y = '1' or X = '0', Y = '0'  
(D) X = '0', Y = '0'  


▫ Code:
```bash
/*==========================================================
Code by G V V Sharma
March 7, 2021,
Released under GNU/GPL
https://www.gnu.org/licenses/gpl-3.0.en.html
/*==========================================================
 *
 *    File   : main.c
 *    Purpose: main for Pygmy blink onboard led
 *                                                          
 *=========================================================*/

#include "Fw_global_config.h"   // This defines application specific charactersitics

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "RtosTask.h"

/*    Include the generic headers required for QORC */
#include "eoss3_hal_gpio.h"
#include "eoss3_hal_rtc.h"
#include "eoss3_hal_timer.h"
#include "eoss3_hal_fpga_usbserial.h"
#include "ql_time.h"
#include "s3x_clock_hal.h"
#include "s3x_clock.h"
#include "s3x_pi.h"
#include "dbg_uart.h"

#include "cli.h"


extern const struct cli_cmd_entry my_main_menu[];


const char *SOFTWARE_VERSION_STR;


/*
 * Global variable definition
 */


extern void qf_hardwareSetup();
static void nvic_init(void);

#define GPIO_OUTPUT_MODE (1)
#define GPIO_INPUT_MODE (0)
void PyHal_GPIO_SetDir(uint8_t gpionum,uint8_t iomode);
int PyHal_GPIO_GetDir(uint8_t gpionum);
int PyHal_GPIO_Set(uint8_t gpionum, uint8_t gpioval);
int PyHal_GPIO_Get(uint8_t gpionum);

int main(void)
{
    uint32_t i=0,j=0,k=0;
    SOFTWARE_VERSION_STR = "qorc-onion-apps/qf_hello-fpga-gpio-ctlr";
    
    qf_hardwareSetup();
    nvic_init();

    dbg_str("\n\n");
    dbg_str( "##########################\n");
    dbg_str( "Quicklogic QuickFeather FPGA GPIO CONTROLLER EXAMPLE\n");
    dbg_str( "SW Version: ");
    dbg_str( SOFTWARE_VERSION_STR );
    dbg_str( "\n" );
    dbg_str( __DATE__ " " __TIME__ "\n" );
    dbg_str( "##########################\n\n");

    dbg_str( "\n\nHello GPIO!!\n\n");	// <<<<<<<<<<<<<<<<<<<<<  Change me!

    CLI_start_task( my_main_menu );
	HAL_Delay_Init();
    
    //LED pins Output
    PyHal_GPIO_SetDir(2, 0);
    PyHal_GPIO_SetDir(3, 0);                              
    PyHal_GPIO_SetDir(4, 1);                              
    PyHal_GPIO_SetDir(5, 1);

    while(1)
    {
        //Test GPIO Code
        PyHal_GPIO_Set(4,1);                             
        PyHal_GPIO_Set(5,1);                             

        int X = 1;                                         
        int Y = 1;                                         
        int P = 0;                                         
        int Q = 0;                                         
        HAL_DelayUSec(2000000);     

        P = PyHal_GPIO_Get(2);                             

        X = ~(P & X);                                  
        Y = ~(Q & Y);                                  
        PyHal_GPIO_Set(4, X);                             
        PyHal_GPIO_Set(5, Y);                             

        HAL_DelayUSec(2000000);                          

        Q = PyHal_GPIO_Get(3);                            

        X = ~(P & X);                                 
        Y = ~(Q & Y);                                 
        PyHal_GPIO_Set(4, X);
        PyHal_GPIO_Set(5, Y);                             

        HAL_DelayUSec(2000000);
            }
    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    dbg_str("\n");

    while(1);
}

static void nvic_init(void)
 {
    // To initialize system, this interrupt should be triggered at main.
    // So, we will set its priority just before calling vTaskStartScheduler(), not the time of enabling each irq.
    NVIC_SetPriority(Ffe0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SpiMs_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(CfgDma_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(Uart_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(FbMsg_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
 }    

//needed for startup_EOSS3b.s asm file
void SystemInit(void)
{

}

//gpionum --> 0 --> 31 corresponding to the IO PADs
//gpioval --> 0 or 1
#define FGPIO_DIRECTION_REG (0x40024008)
#define FGPIO_OUTPUT_REG (0x40024004)
#define FGPIO_INPUT_REG (0x40024000)
//Set GPIO(=gpionum) Mode: Input(iomode = 0) or Output(iomode = 1)
//Before Set/Get GPIO value, the direction must be correctly set
void PyHal_GPIO_SetDir(uint8_t gpionum,uint8_t iomode)
{
    uint32_t tempscratch32;

    if (gpionum > 31)
        return;

    tempscratch32 = *(uint32_t*)(FGPIO_DIRECTION_REG);
    if (iomode)
        *(uint32_t*)(FGPIO_DIRECTION_REG) = tempscratch32 | (0x1 << gpionum);
    else
        *(uint32_t*)(FGPIO_DIRECTION_REG) = tempscratch32 & (~(0x1 << gpionum));

}


//Get current GPIO(=gpionum) Mode: Input(iomode = 0) or Output(iomode = 1)
int PyHal_GPIO_GetDir(uint8_t gpionum)
{
    uint32_t tempscratch32;
    int result = 0;

    if (gpionum > 31)
        return -1;

    tempscratch32 = *(uint32_t*)(FGPIO_DIRECTION_REG);

    result = ((tempscratch32 & (0x1 << gpionum)) ? GPIO_OUTPUT_MODE : GPIO_INPUT_MODE);

    return result;
}

//Set GPIO(=gpionum) to 0 or 1 (= gpioval)
//The direction must be set as Output for this GPIO already
//Return value = 0, success OR -1 if error.
int PyHal_GPIO_Set(uint8_t gpionum, uint8_t gpioval)
{
    uint32_t tempscratch32;

    if (gpionum > 31)
        return -1;

    tempscratch32 = *(uint32_t*)(FGPIO_DIRECTION_REG);

    //Setting Direction moved out as separate API, we will only check
    //*(uint32_t*)(FGPIO_DIRECTION_REG) = tempscratch32 | (0x1 << gpionum);
    if (!(tempscratch32 & (0x1 << gpionum)))
    {
        //Direction not Set to Output
        return -1;
    }
    
    tempscratch32 = *(uint32_t*)(FGPIO_OUTPUT_REG);

    if(gpioval > 0)
    {
        *(uint32_t*)(FGPIO_OUTPUT_REG) = tempscratch32 | (0x1 << gpionum);
    }
    else
    {
        *(uint32_t*)(FGPIO_OUTPUT_REG) = tempscratch32 & ~(0x1 << gpionum);
    }    

    return 0;
}
//Get GPIO(=gpionum): 0 or 1 returned (or in erros -1)
//The direction must be set as Input for this GPIO already
int PyHal_GPIO_Get(uint8_t gpionum)
{
    uint32_t tempscratch32;
    uint32_t gpioval_input;

    if (gpionum > 31)
        return -1;

    tempscratch32 = *(uint32_t*)(FGPIO_INPUT_REG);
    gpioval_input = (tempscratch32 >> gpionum) & 0x1;

    return ((int)gpioval_input);
}

```
▫ Output Video:

[Demonstration](videos/video_8.mp4) 
---


