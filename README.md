# Multimodal Vibration Detection System

Mixed signal vibration monitoring platform that integrates analog and digital motion sensors to detect and analyze vibration characteristics in real time.

The system compares a custom analog accelerometer signal chain with a digital IMU and evaluates their performance using a composite Figure of Merit (FoM) based on noise, response, and sensitivity.

---

# Overview

The project combines multiple sensing approaches to capture both continuous motion and sudden vibration events.

An analog accelerometer path is implemented using external signal conditioning and an SPI ADC, while a digital IMU provides calibrated motion measurements through I²C. Additional sensors are used for event detection and validation.

The firmware continuously collects sensor data, performs filtering and calibration, and periodically evaluates system performance using a quantitative metric.

---

# System Architecture

Analog sensing path  
ADXL335 → RC low pass filter → MCP6004 op amp buffer → MCP3008 ADC → Arduino Due (SPI)

Digital sensing path  
MPU6050 IMU → Arduino Due (I²C)

Event detection sensors  
Piezoelectric disk → analog inputs  
SW420 vibration sensor → digital trigger

This architecture enables direct comparison between analog and digital sensing approaches.

---

# Hardware

### Microcontroller
Arduino Due  
32 bit ARM Cortex M3 microcontroller operating at 3.3 V logic.

### Sensors

ADXL335  
3 axis analog accelerometer used to build the analog sensing path.

MPU6050  
6 axis IMU providing calibrated accelerometer data via I²C.

Piezoelectric disk  
High sensitivity vibration sensor capable of detecting transient impacts.

SW420 vibration sensor  
Digital vibration switch used for threshold based event detection.

### Signal conditioning

MCP6004 quad operational amplifier  
Used to buffer accelerometer outputs and prevent loading effects.

RC low pass filters  
Reduce high frequency noise before digitization.

MCP3008 ADC  
10 bit SPI analog to digital converter used to digitize ADXL335 outputs.

---

# Firmware Overview

The firmware performs the following tasks

- sensor acquisition  
- signal filtering and smoothing  
- sensor calibration  
- event detection  
- performance metric computation  

Sensor data is streamed continuously while vibration metrics are evaluated periodically.

---

# Important Code Components

## ADC Interface

The ADXL335 produces analog voltages which are digitized using the MCP3008 ADC via SPI.

Single ADC read:

`readMCP3008_once()` 


This function performs a single SPI transaction with the MCP3008 to retrieve a raw ADC value.

Median filtered reading:


`readMCP3008_median()`


Multiple ADC samples are collected and sorted, and the median value is returned to suppress noise spikes.

Oversampled reading:


`readMCP3008_os()`


This function averages several median filtered readings to improve measurement stability.

---

## IMU Interface

The MPU6050 communicates through the I²C bus.

Initialization function:


`imuInit()`


Configures sampling rate and filtering parameters for the IMU.

Accelerometer read function:


`readAccelRaw()`


Reads accelerometer registers from the MPU6050 and converts them into acceleration values.

---

## Signal Filtering

Several filtering techniques are applied to stabilize the signal.

Median filtering removes transient spikes from ADC readings.

Oversampling averages several measurements to reduce quantization noise.

IIR filtering applies exponential smoothing to stabilize accelerometer signals.

Moving average filtering maintains a sliding window of recent samples to reduce jitter.

---

# Calibration

The firmware performs calibration during startup.

Zero centering:


`calibrateZeroCenter()`


This function samples both accelerometers while the system is stationary and calculates baseline offsets so that readings are centered around zero.

Two point calibration:


`twoPointCalibrateADXL()`


Captures +1 g and −1 g orientations of the accelerometer to determine

- ADC midpoint  
- accelerometer counts per g  

This improves the accuracy of the analog sensing path.

---

# Event Detection

Two sensors are used to detect vibration events.

SW420 vibration sensor  
Produces a digital HIGH signal when vibration exceeds a preset threshold.

Piezoelectric disk  
Generates voltage spikes when mechanical disturbances occur.

An event is detected when either condition is true:


`piezo_env > threshold OR SW420 trigger`


During an event the firmware records peak acceleration values from both sensors.

---

# Performance Metrics

The firmware computes three primary vibration metrics.

Noise RMS


`NoiseRMS = sqrt(sum(x²) / N)`


Measures the baseline noise level of the sensor.

Response average  
Measures how quickly the sensor responds to changes in acceleration.

Sensitivity  
Compares peak acceleration detected during vibration events.

---

# Figure of Merit

Overall system performance is summarized using a weighted metric.

```
FoM = W_noise * NoiseScore
+ W_response * ResponseScore
+ W_sensitivity * SensitivityScore
```

Default weights used in the firmware:

```
W_noise = 0.45
W_response = 0.30
W_sensitivity = 0.25
```

Experimental testing produced FoM values typically between 68 and 75.

---

# Serial Commands

The firmware supports runtime commands through the serial interface.

| Command | Function |
|--------|--------|
| c | Run two point ADXL calibration |
| r | Re zero sensors |
| w | Print FoM weight parameters |

---

# Output

Sensor data is streamed in real time and can be visualized using the Arduino Serial Plotter.

Displayed signals include

- ADXL335 X Y Z acceleration  
- MPU6050 X Y Z acceleration  
- SW420 vibration trigger  
- Piezo sensor signals  

Periodic metric summaries including noise RMS, response score, sensitivity score and FoM are printed to the serial console.

---

# Key Learning Outcomes

mixed signal system design  
analog signal conditioning and ADC interfacing  
sensor fusion and vibration analysis  
noise filtering for embedded sensing systems  
comparative evaluation of analog and digital accelerometers

---

# Authors

Adit Goyal  
Nitigya Khaneja  
Stavyansh Mishra
