# 🌱 Smart Irrigation System

Industrial automatic irrigation system based on **ESP32**, **OpenPLC**, **Raspberry Pi** and **Modbus TCP**.

This project demonstrates how a programmable logic controller (PLC) can be integrated with embedded hardware to create a reliable and modular irrigation system.

---

## 🌍 Live Demo

https://koval0ff.github.io/Smart_Irrigation_System/

---

## 📖 Project Overview

The goal of this project is to automate plant irrigation using industrial automation principles rather than a simple Arduino-only solution.

The ESP32 acts as a remote I/O controller responsible for reading sensors and controlling physical outputs.

OpenPLC executes the irrigation logic and makes all control decisions.

Communication between both devices is implemented via Modbus TCP over Wi-Fi.

The project was developed as a personal portfolio project to demonstrate practical skills in PLC programming, embedded systems and industrial communication protocols.

---

## 🚀 Features

- Soil moisture monitoring
- Water tank level detection
- Pump control
- Battery voltage monitoring
- Pump current monitoring
- Temperature monitoring
- Embedded diagnostic web interface
- OTA firmware updates
- Modbus TCP communication
- PLC-based irrigation logic

---

## 🛠 Technologies

- ESP32
- OpenPLC
- Raspberry Pi
- Arduino Framework
- Modbus TCP
- HTML / CSS / JavaScript
- C++
- IEC 61131-3 Structured Text

---

## 📂 Repository Structure

### 📁 docs

Contains the project landing page used by GitHub Pages.

### 📁 project/OpenPLC

Contains the PLC program written in Structured Text.

### 📁 project/ESP32

Contains the ESP32 firmware responsible for hardware interaction.

### 📁 project/Hardware

Hardware diagrams, wiring documentation and component information.

### 📁 project/Documentation

Additional project documentation.

---

## 👨‍💻 Author

**Danylo Kovalov**

Automation & PLC Developer

Germany, Hannover