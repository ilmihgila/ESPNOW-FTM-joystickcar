# ESPNOW-FTM-joystickcar

This project use esp-now and ftm to control an rc car. Esp-now is used to transmit joystick data from the controller and imu data from the car, while the ftm is used to approximate the distance between the controller and the car.


# Schematic and Images

<div style="display: flex;">
  <img src="/images/car.jpeg" alt="car image" width="45%">
  <img src="/images/controller.jpeg" alt="controller image" width="45%">
</div>

<div style="display: flex;">
  <img src="/images/car_schematic.jpg" alt="car schematic" width="50%">
  <img src="/images/controller_schematic.jpg" alt="controller schematic" width="50%">
</div>

# Important

Esp-now and FTM could only be used simultanously when both are in the same channel. This prject initialize wifi in the FTM task while ESP-NOW task would be delayed by 1 second to hopefully make sure wifi were initialized. It would be nice to use dynamic timing for this asynchronous task in the future. 

For this example, every pin that are used located at the left side of the esp. This would make it easier to use since there could only be one side of pins if we use a normal bread board. If you consider to change the pins, always use adc channel 1 (pin 1-10) for the joystick, and L298N enA & enB. This ensure everything to work at any time because wifi uses adc channel 2.


# What you need

- esp32. (Ftm could only be used on supported devices see [ESP32  FTM github](https://github.com/espressif/esp-idf/blob/v5.5.1/examples/wifi/ftm))
- joystick module ky-023.
- L298N dual channel motor driver.
- 9V battery.
- 2 dc motor with wheels.
- imu mpu6050 (optional).
- mini oled 64 x 128 (optional).

# How It Work

## Car

## Controller

# What to Improve

- [ ] Change esp-now setup so it could dynamicly run only after wifi init.
- [ ] Add global movement based on the controller perspective
- [ ] idk







