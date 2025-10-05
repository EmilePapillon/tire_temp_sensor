# BLE Thermal Strip Visualization

## Setup and Usage

1. Activate the virtual environment:
   ```bash
   source venv/bin/activate
   ```

2. Run the script:
   ```bash
   python main.py
   ```

## Description

This script connects to an MLX90641 thermal sensor over BLE, receiving temperature data from 12 pixels and displaying them as a real-time horizontal thermal strip visualization. The visualization uses a color gradient to represent temperatures, updating continuously as new data arrives from the sensor.