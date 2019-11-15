# Arduino
This repository contains the Arduino code for the autonomous scooter. Currently there are two boards on the scooter, one Uno and one Mega. The code for the Uno is in **Arduino_Uno** and the code for the Mega is in **Arduino_Mega**. In the near future these two codes will be combined so we can use just the Mega board. **IMU_test** and **Ultrasonic** contains test codes for the IMU and the ultrasonic sensors respectively, but at the current stage of the scooter neither of those codes ar being used right now.

# Pin Out List for electronic hardware on the scooter
### For left incremental encoder next to the left rear wheel:

Green wire (Output A): Pin 2 on the Arduino Mega 2560

Gray wire (Output B): Pin 3 on the Arduino Mega 2560

Yellow wire (Output Z): Pin 18 on the Arduino Mega 2560

Red wire to VCC

Black wire to GND


### For right incremental encoder next to the right rear wheel:

Green wire (Output A): Pin 19 on the Arduino Mega 2560

Gray wire (Output B): Pin 20 on the Arduino Mega 2560

Yellow wire (Output Z): Pin 21 on the Arduino Mega 2560

Red wire to VCC

Black wire to GND

All the numbered pins shown above are used for digital interrupts.


### For the Absolute Encoder:

Pin 1 (green wire) on EMS22A (Digital Input) connects to GND

Pin 2 (white wire) on EMS22A (Clock) connects to pin 26 on Arduino Mega 2560

Pin 3 (black wire) on EMS22A (GND) connects to GND

Pin 4 (purple wire) on EMS22A (Digital Output) connects to pin 28 on Arduino Mega 2560

Pin 5 (red wire) on EMS22A (VCC) connects to VCC

Pin 6 (yellow wire) on EMS22A (Chip Select) connects to pin 24 on Arduino Mega 2560

Arduino Mega pin 26 outputs the clocking signal that triggers the Digital Output pin on EMS22A to send out bits of data. One reading
corresponds to 10 bits of data, plus another 6 bits for checking purposes.
Arduino Mega pin 24 outputs a pulse signal to enable the absolute encoder to output a new reading every byte.


### For the front Motor Controller:

PWM pin goes to Pin 6 on the Arduino Mega

DIR pin goes the Pin 7 on the Arduino Mega


### For the rear Motor Controller:

PWM pin goes to Pin 9 on the Arduino Mega

DIR pin goes the Pin 8 on the Arduino Mega
