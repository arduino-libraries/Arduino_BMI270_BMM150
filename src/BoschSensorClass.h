/*
  This file is part of the Arduino_LSM9DS1 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include <Wire.h>
#include "utilities/BMI270-Sensor-API/bmi270.h"
#include "utilities/BMM150-Sensor-API/bmm150.h"

typedef enum {
  BOSCH_ACCELEROMETER_ONLY,
  BOSCH_MAGNETOMETER_ONLY,
  BOSCH_ACCEL_AND_MAGN
} CfgBoshSensor_t;

typedef enum {
  BMI270_ACCELEROMETER,
  BMI270_GYROSCOPE,
} BoschSensorType_t;

struct dev_info {
  TwoWire* _wire;
  uint8_t dev_addr;
};

class ContinuousMode {
public:
  ContinuousMode(struct bmi2_dev * dev) : bmi2(dev) {}
  void begin() {
    fifoFrame.data = fifoData;
    fifoFrame.length = sizeof(fifoData);
    bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 1, bmi2);
    bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, bmi2);
    bmi2_map_data_int(BMI2_FWM_INT, BMI2_INT1, bmi2);
    bmi2_set_fifo_wm((sizeof(fifoData)), bmi2);
    continuousMode = true;
  }

  void end() {
    bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 0, bmi2);
    continuousMode = false;
  }

  int available(BoschSensorType_t sensor) {
    uint16_t status;
    bmi2_get_int_status(&status, bmi2);
    if ((status & BMI2_FWM_INT_STATUS_MASK) == 0) {
      return 0;
    }
    switch (sensor) {
      case BMI270_ACCELEROMETER:
        if (_availableA != 0) {
          return 1;
        }
        break;
      case BMI270_GYROSCOPE:
        if (_availableG != 0) {
          return 1;
        }
        break;
    }
    bmi2_get_fifo_length(&status, bmi2);
    auto ret = bmi2_read_fifo_data(&fifoFrame, bmi2);
    if (ret != 0) {
      return 0;
    }
    _available = min(status, sizeof(fifoData)) / (6 + 6); // 6 bytes per accel sample
    _availableG = _available;
    _availableA = _available;
    ret = bmi2_extract_accel(accel_data, &_available, &fifoFrame, bmi2);
    ret = bmi2_extract_gyro(gyro_data, &_available, &fifoFrame, bmi2);
    return _available;
  }

  int hasData(BoschSensorType_t sensor) {
    switch (sensor) {
      case BMI270_ACCELEROMETER:
        return _availableA > 0;
      case BMI270_GYROSCOPE:
        return _availableG > 0;
    }
    return 0;
  }

  void getGyroscopeData(struct bmi2_sens_axes_data* gyr) {
    gyr->x = gyro_data[samples_count - 1 - _availableG].x;
    gyr->y = gyro_data[samples_count - 1 - _availableG].y;
    gyr->z = gyro_data[samples_count - 1 - _availableG].z;
    _availableG--;
  }

  void getAccelerometerData(struct bmi2_sens_axes_data* acc) {
    acc->x = accel_data[samples_count - 1 - _availableA].x;
    acc->y = accel_data[samples_count - 1 - _availableA].y;
    acc->z = accel_data[samples_count - 1 - _availableA].z;
    _availableA--;
  }

  operator bool() const {
    return continuousMode == true;
  }

private:
  bool continuousMode = false;
  struct bmi2_dev * bmi2;
  static const size_t samples_count = 8;
  bmi2_fifo_frame fifoFrame;
  uint8_t fifoData[samples_count * (6+6)];
  uint16_t _available = 0;
  uint16_t _availableG = 0;
  uint16_t _availableA = 0;
  struct bmi2_sens_axes_data accel_data[samples_count];
  struct bmi2_sens_axes_data gyro_data[samples_count];
};

class BoschSensorClass {
  public:
    BoschSensorClass(TwoWire& wire = Wire);
    ~BoschSensorClass() {}

    void setContinuousMode();
    void oneShotMode();

    int begin(CfgBoshSensor_t cfg = BOSCH_ACCEL_AND_MAGN);
    void end();

    void debug(Stream&);
    #ifdef __MBED__
    void onInterrupt(mbed::Callback<void()>);
    void setInterruptPin(PinName irq_pin) {
      BMI270_INT1 = irq_pin;
    }
    void setInterruptPin(pin_size_t irq_pin) {
      BMI270_INT1 = digitalPinToPinName(irq_pin);
    }
    PinName BMI270_INT1 = NC;
    #endif
    // Accelerometer
    virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
    virtual int accelerationAvailable(); // Number of samples in the FIFO.
    virtual float accelerationSampleRate(); // Sampling rate of the sensor.

    // Gyroscope
    virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
    virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.

    // Magnetometer
    virtual int readMagneticField(float& x, float& y, float& z); // Results are in uT (micro Tesla).
    virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
    virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.

  protected:
    // can be modified by subclassing for finer configuration
    virtual int8_t configure_sensor(struct bmm150_dev *dev);
    virtual int8_t configure_sensor(struct bmi2_dev *dev);

  private:
    static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static void bmi2_delay_us(uint32_t period, void *intf_ptr);
    void interrupt_handler();
    void print_rslt(int8_t rslt);

  private:
    TwoWire* _wire;
    Stream* _debug = nullptr;
    #ifdef __MBED__
    mbed::Callback<void(void)> _cb;
    #endif
    bool _initialized = false;
    int _interrupts = 0;
    struct dev_info accel_gyro_dev_info;
    struct dev_info mag_dev_info;
    struct bmi2_dev bmi2;
    struct bmm150_dev bmm1;
    uint16_t _int_status;
  private:
    ContinuousMode continuousMode{&bmi2};
};

extern BoschSensorClass IMU_BMI270_BMM150;
#undef IMU
#define IMU IMU_BMI270_BMM150
