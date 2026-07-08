# PCB Designs
Parent repository for my PCB designs.

Larger projects have their own repositories with more detailed documentation. Links to these are provided below in their respective sections.

## Projects

- [Waveform Generator](#waveform-generator) - STM32-controlled power supply and analog waveform generator.
- [Subsystem A (RF Receiver Chain)](#subsystem-a-rf-receiver-chain) - Discrete BJT Quadrature Mixer and RX Filter board.
- [Drone Brushless DC Motor Controller](#drone-brushless-dc-motor-controller) - Three-phase sensorless BLDC motor controller.
- [LED Heart Keychain](#led-heart-keychain) - Compact heart-shaped LED array keychain.

---

## Waveform Generator

[GitHub Repository](https://github.com/wavius/Waveform-Generator)

Power supply board with a configurable analog output using the IP2721 IC and digital +12V, -12V, +5V, +3V3 outputs.
- 45 W USB-C input for power delivery.
- Switches, rotary encoder, and potentiometer to select waveform, frequency, phase, and output analog voltage.
- Integrated STM32 microcontroller with STLINK interface for waveform generation and system control.
- LCD interface header for waveform and parameter display.

<br>
<div align="left">
  <img src="https://raw.githubusercontent.com/wavius/Waveform-Generator/main/assets/pcb_lcd.jpg" width="800" alt="PCB LCD Display">
</div>
<br>

---

## Subsystem A (RF Receiver Chain)

[GitHub Repository](https://github.com/wavius/Subsystem-A-ECE295)

This subsystem implements the RX Filter and Quadrature Mixer as part of the receive chain of a Flexible Radio Transceiver (FLRTRX), designed as part of the ECE295 course.
- **Band-pass Filter**: 3rd-order passive Butterworth filter with 8 and 16 MHz cutoff frequencies.
- **Low Noise Amplifier (LNA)**: Discrete cascode (common-emitter to common-base) NPN BJT amplifier with 50 $\Omega$ input impedance.
- **Gilbert Cell Mixer**: Active Gilbert Cell mixer providing quadrature mixing with a 90° phase difference between I and Q channels.
- **Low-pass Filter**: Second-order Sallen-Key active Butterworth filter with a 92 kHz cutoff.
- **Amplifier**: Dedicated post-mixer non-inverting amplifier with $\ge 30\text{ dB}$ gain.
- **Discrete BJT Design**: LNA and mixer stages are custom-designed using discrete transistors.

<br>
<div align="left">
  <img src="https://raw.githubusercontent.com/wavius/Subsystem-A-ECE295/main/assets/images/sysA_pcb.jpg" alt="Subsystem A PCB" width="500px">
</div>
<br>

---

## Drone Brushless DC Motor Controller

Three-phase sensorless BLDC motor controller using the A4963GLPTR-T driver IC with external N-channel and P-channel MOSFETs. 
- 20 A per phase.
- Integrated back-EMF sensing and PWM speed control.

<br>
<div align="left">
  <img src="Drone%20Brushless%20DC%20Motor%20Controller/PCB%203D.png" alt="Drone Brushless DC Motor Controller" width="500px">
</div>
<br>

---

## LED Heart Keychain

Compact PCB featuring a heart-shaped LED array.
- 5 V USB-C input.
- Mounting hole for attachment to a keychain.

<br>
<div align="left">
  <img src="LED%20Heart%20Keychain/PCB%203D.png" alt="LED Heart Keychain 3D" height="500px">
  <img src="LED%20Heart%20Keychain/heart.jpg" alt="LED Heart Keychain Real" height="500px">
</div>
<br>

---

## Breakout Boards

These are smaller breakout boards created for prototyping and testing.

### DAC Breakout Board
Breakout board for the MCP4725 12-Bit DAC by Microchip Technology.

<br>
<div align="left">
  <img src="Breakout%20Boards/DAC%20Breakout%20Board/PCB%203D.png" alt="DAC Breakout Board 3D" width="500px">
  <img src="Breakout%20Boards/DAC%20Breakout%20Board/DAC.jpg" alt="DAC Breakout Board Real" width="500px">
</div>
<br>

### LCD Breakout Board
Breakout board for the NHD-C12832A1Z-FSW-FBW-3V3 LCD Display by Newhaven Display.

<br>
<div align="left">
  <img src="Breakout%20Boards/LCD%20Breakout%20Board/PCB%203D.png" alt="LCD Breakout Board 3D" width="500px">
  <img src="Breakout%20Boards/LCD%20Breakout%20Board/LCD.jpg" alt="LCD Breakout Board Real" width="500px">
</div>
<br>
