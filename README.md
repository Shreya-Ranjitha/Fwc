# Internship work at Future Wireless Communication at IIT Hyderabad

Welcome to my internship documentation repository. This repository contains a detailed record of my work, assignments, and learning progress during my internship at IIT Hyderabad (IITH).

---

## Table of Contents

- [About](#about)
- [Setup Details](#setup-details)
- [Platformio](#platformio)
- [Assembly Programming](#assembly-programming)
- [Embedded C](#embedded-c)
- [Acknowledgements](#acknowledgements)

---

## About

I applied for the Summer Research Fellowship Programme (SRFP) offered by the Indian Academy of Sciences (IASc) during my third semester. After submitting my application with academic details and a statement of purpose, I was thrilled to receive my selection letter in March. I was assigned to work under the guidance of Dr. GVV Sharma at the Indian Institute of Technology (IIT) Hyderabad. This internship has been a great opportunity to dive deeper into research and practical problem-solving. This repository serves as a portfolio of my internship work, documenting my progress, solutions, and notes as I complete assignments and projects related to digital design.

---

## Setup Details

The entire workflow was carried out on an Android device using [Termux](https://f-droid.org/en/packages/com.termux/), enabling a portable and efficient development environment. Below is a step-by-step guide to replicate the setup:

### 1. Install Termux

Install the Termux app from [F-Droid](https://f-droid.org/packages/com.termux/) for access to all required packages.

### 2. Give Storage Permission

This allows Termux to access files on your device.

```bash
termux-setup-storage
```

### 3. Update Existing Packages

Make sure everything is up to date before installing new tools.

```bash
pkg update && pkg upgrade
```

### 4. Install Essential Packages

These are required for compiling LaTeX files, running code, and basic CLI utilities.

```bash
pkg install git wget clang make texlive-core texlive-latexextra termux-api python
```

### 5. Install Python Packages

Used for plotting and scientific computing tasks.

```bash
pip install numpy matplotlib scipy
```

### 6. Install AVR Toolchain

This is used to compile and flash code to AVR microcontrollers like ATmega328P.

```bash
pkg install avr-gcc avr-libc avrdude
```

### 7. (Optional) Install PlatformIO

PlatformIO CLI can be used for firmware development and advanced board management.

```bash
pip install platformio
```

### 8. Clone the Repository

Fetch your project files from GitHub.

```bash
git clone https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
```

### 9. Build the Code Using Make

Use `make` to compile source files (C, assembly, etc.).

```bash
make
```

### 10. Compile LaTeX Documents

Used to generate final PDF reports, plots, and documentation.

```bash
pdflatex filename.tex
```


