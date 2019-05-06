# VESC PWM and UART combined code specific for blue pill STM32F103

## for foocars project

This is still a WIP
Uses VESC Uart library [found at](https://github.com/SolidGeek/VescUart)
This was testing to be working with the VESC hardware ver 4.12 and latest firmware.  This will take PWM in and send it back out unless there is a ADC input from the hall throttle sensor.  Will also get VESC telemetry.

After some debugging, this appears to give telemetry and serial requests are working now.

Modification to send data on serial 2 for upstream microcontroller.  ADC throttle imput is modifed to work with half range with E-brake from RC input