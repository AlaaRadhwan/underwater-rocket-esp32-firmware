#include "Servo_Module.h"

// Static definitions
Adafruit_PWMServoDriver Servo_Module::driver;
bool     Servo_Module::bus_inited    = false;
uint8_t  Servo_Module::cfg_addr      = 0x40;
int      Servo_Module::cfg_sda       = 21;
int      Servo_Module::cfg_scl       = 22;
uint32_t Servo_Module::cfg_i2c_hz    = 400000;
uint16_t Servo_Module::cfg_servo_freq= 50;
uint32_t Servo_Module::cfg_osc_hz    = 0; // set to 27000000 if your board needs it
