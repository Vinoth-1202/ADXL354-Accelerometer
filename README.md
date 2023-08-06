# **ADXL354-Accelerometer**
I've provided a code snippet written in C for an ESP32 microcontroller to interface with an ADXL345 accelerometer using the I2C protocol. The code initializes the I2C communication, configures the ADXL345 accelerometer, sets up interrupt handlers for activity/inactivity detection and single/double detection, and continuously reads the interrupt source register to determine whether activity/inactivity and single/double has been detected.
## **I2C Initialization**
I2C_Master_Initialize function initializes the I2C communication for the ESP32.It configures the I2C pins, sets the clock frequency, and installs the I2C driver.
## **I2C Write and Read Functions**
I2C_Master_Write_Slave_Reg and I2C_Master_Read_Slave_Reg these functions handle writing and reading data from the ADXL345 accelerometer using I2C. They construct the necessary I2C command sequences to communicate with the device.
## **ADXL345 Initialization**
ADXL345_Initialize function initializes the ADXL345 accelerometer by configuring various registers for power control, interrupts, activity/inactivity thresholds, tap threshold, duration reg, Latent reg, window reg and tap axes.
## **Interrupt Handler**
Functions (activity_isr_handler, inactivity_isr_handler, single_isr_handler and double_isr_handler): These functions handle the interrupt triggered when activity or inactivity and single or double tap is detected by the ADXL345 accelerometer. They set flags (activity_flag, inactivity_flag, single_flag and double tap) to indicate the detected events
## **GPIO Configuration and ISR Installation** 
def function configures GPIO pins for interrupt handling and installs the interrupt service routine (ISR) functions for the specified GPIO pins.
## **Main Function**
This function is the entry point of the program. It calls the initialization functions, sets up the interrupt handlers, and enters a loop to continuously monitor the interrupt source register for activity/inactivity and single/double tap detection.
