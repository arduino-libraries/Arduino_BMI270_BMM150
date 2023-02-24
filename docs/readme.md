# ArduinoBMI_270_BMM150 library

The ArduinoBMI270_BMM150 library allows you to use the inertial measurement unit (IMU) system available on the Arduino&reg; Nano 33 BLE Sense Rev2 board. The IMU system is made up up the 3-axis accelerometer and gyroscope [BMI270](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf), and the 3-axis magnetometer [BMM150](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf). The IMU is connected to the Nano 33 BLE Sense Rev2 board's microcontroller through I2C. The values returned are signed floats.

To use this library:

```
#include "Arduino_BMI270_BMM150.h"
```

The ArduinoBMI270_BMM150 library takes care of the sensor initialization and sets its values as follows:

- Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
- Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
- Magnetometer range is set at ±1300 uT with a resolution of 0.3 uT.
- Accelerometer and gyrospcope output data rate is fixed at 99.84 Hz.
- Magnetometer output data rate is fixed at 10 Hz.