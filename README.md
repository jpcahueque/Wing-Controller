# Wing-Controller
Firmware that controls the STM NUCLEO F439 board, in charge of receiving data incoming from XSENS IMU, an XBEE device, and the gWind garmin sensor, sending then data though the CAN1 port.
## IMU DATA
The XSENS IMU Data is received through the UART6 port , configurated at 115200bps and linked to the DMA channel for the continuous reception of the data.

## Remote Control
Two Xbee devices were configured in order to control remotely the desired reference angle of the drone.
By connecting one of the Xbee devices to the UART7 port of the STM32 board, the communication is performed.

## Anemometer data
In order to change the reference angle in automatic mode of the drone, the incoming Apparent Wind Angle and Apparent Wind Speed is received incoming from the gWind garmin sensor via the USB-OTG port.

## Rope control
After receiving the data, the difference in rope to be rolled or unrolled is the calculated with the corresponding mathematical model for each of the 4 actuators in the base of the mast of the drone.

## CAN Communication
The calculated rope difference and the desired current for the actuators are then sent via the CAN1 port at a configurated speed of 250000 Mbps
