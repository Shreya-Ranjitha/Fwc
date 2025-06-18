# Internship work at Future Wireless Communication at IIT Hyderabad

Welcome to my internship documentation repository. This repository contains a detailed record of my work, assignments, and learning progress during my internship at IIT Hyderabad (IITH).

---

## Table of Contents

- [About](#about)
- [Setup Details](#setup-details)
- [Platformio](#platformio)
- [Assembly Programming](#assembly-programming)
- [Embedded C](#embedded-c)
- [Vaman](vaman)
- [Acknowledgements](#acknowledgements)

---

## About

I applied for the Summer Research Fellowship Programme (SRFP) offered by the Indian Academy of Sciences (IASc) during my third semester. After submitting my application with academic details and a statement of purpose, I was thrilled to receive my selection letter in March. I was assigned to work under the guidance of Dr. GVV Sharma at the Indian Institute of Technology (IIT) Hyderabad. This internship has been a great opportunity to dive deeper into research and practical problem-solving. This repository serves as a portfolio of my internship work, documenting my progress, solutions, and notes as I complete assignments and projects related to digital design.

---

## Setup Details

This project follows the same setup used in Prof. G.V.V. Sharma’s [FWC repository](https://github.com/gadepall/fwc-1?tab=readme-ov-file). All required packages, toolchains, and instructions for Termux-based development are listed there.

You can also refer to the digital design book and supporting material using:

```bash
git clone https://github.com/gadepall/digital-design
```
---
##Platformio
## PlatformIO

PlatformIO is an open-source ecosystem for embedded development. It simplifies compiling and uploading firmware to microcontrollers like the ATmega328P.

---

### 1. Create or Open a PlatformIO Project

Use PlatformIO CLI to create a new project:

```bash
pio project init --board atmega328p
```

> Replace `atmega328p` with your actual board ID. Use `pio boards` to list available options.

---

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

---

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

---

### 4. Build the Project

Use PlatformIO to compile the firmware:

```bash
pio run
```

---

### 5. Upload the Firmware

Make sure your board is connected to your android device and upload the code using Arduino Droid.

