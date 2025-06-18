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

The entire workflow was carried out on an Android device using [Termux](https://f-droid.org/en/packages/com.termux/), enabling a portable and efficient development environment.

### 🔧 Step-by-Step Setup

```bash
# One-time storage permission
termux-setup-storage

# Update packages
pkg update && pkg upgrade

# Install essential packages
pkg install git wget clang make texlive-core texlive-latexextra termux-api python

# Install Python packages
pip install numpy matplotlib scipy

# Install AVR toolchain
pkg install avr-gcc avr-libc avrdude

# (Optional) Install PlatformIO CLI
pip install platformio

# Clone and build the project
git clone https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
make

# Compile LaTeX documents
pdflatex filename.tex



