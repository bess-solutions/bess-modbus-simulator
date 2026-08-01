# BESS Solutions — Modbus TCP Simulator

A standalone, lightweight Modbus TCP server simulator specifically designed to simulate battery energy storage systems (BESS) holding registers. It is used to test and validate edge integration logic (like `open-bess-edge`) without requiring connection to physical hardware (Huawei, SMA, BYD, etc.).

By separating the simulator into its own repository, we avoid dependency version clashes (e.g. `pymodbus` package conflicts) with the main gateway production environments.

---

## ⚡ Features

- Simulates active power, state of charge (SoC), voltage, and pack temperatures.
- Simulates realistic physical dynamics (dynamic battery discharge curves).
- Integrates a secondary thread simulating local droop controls (frequency response) and ramping guards.
- Compatible with PyModbus 3.x.

---

## 🚀 Quick Start

### 1. Installation

Clone this repository and install dependencies in a clean virtual environment:

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows use: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Run the Simulator

Start the simulator server:

```bash
python server.py
```

By default, the server will bind to `0.0.0.0` on port `5020` (to avoid requiring root/admin privileges on standard port 502).

---

## 🐳 Docker Deployment

To run the simulator inside a container:

```bash
# Build the image
docker build -t bess-modbus-simulator .

# Run the container
docker run -d -p 5020:5020 bess-modbus-simulator
```

---
*BESS Solutions SpA — open@bess-solutions.cl*
