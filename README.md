# ✨Internship work at Future Wireless Communication at IIT Hyderabad✨

Welcome to my internship documentation repository. This repository contains a detailed record of my work, assignments, and learning progress during my internship at IIT Hyderabad (IITH).

---

## Table of Contents

- [About](#about)
- [Setup Details](#setup-details)
- [PlatformIO](#platformio)
- [Assembly Programming](#assembly-programming)
- [Embedded C](#embedded-c)
- [Vaman](vaman)
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



