#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Servo_Module {
public:
  // Constructor matches your old usage: second arg is ignored (kept for compatibility)
  Servo_Module(uint8_t pcaChannel, uint8_t /*ignoredChannel*/ = 0,
               uint16_t minUs = 500, uint16_t maxUs = 2500, bool reverse = false)
  : ch(pcaChannel), min_us(minUs), max_us(maxUs), reversed(reverse) {}

  // Call once per instance (bus init is deduped)
  void begin() {
    ensureBusInit();
    // Center to 90° on begin for safety
    writeAngle(90.0f);
  }

  void writeAngle(float angleDeg) {
    angleDeg = constrain(angleDeg, 0.0f, 180.0f);
    if (reversed) angleDeg = 180.0f - angleDeg;
    uint16_t us = min_us + (uint16_t)((max_us - min_us) * (angleDeg / 180.0f));
    writeMicroseconds(us);
  }

  void writeMicroseconds(uint16_t us) {
    ensureBusInit();
    us = constrain(us, min_us, max_us);
    uint16_t ticks = usToTicks(us);
    driver.setPWM(ch, 0, ticks);
  }

  void setLimits(uint16_t minUs, uint16_t maxUs) {
    min_us = minUs; max_us = maxUs;
  }

  void setReverse(bool r) { reversed = r; }

  // Optional: configure I2C pins/address/frequency *before* any begin()
  static void configureBus(uint8_t addr = 0x40, int sda = 21, int scl = 22,
                           uint32_t i2c_hz = 400000, uint16_t servo_freq = 50,
                           uint32_t osc_hz = 0 /*0 = default*/) {
    cfg_addr = addr; cfg_sda = sda; cfg_scl = scl;
    cfg_i2c_hz = i2c_hz; cfg_servo_freq = servo_freq; cfg_osc_hz = osc_hz;
    // If already inited, update freq immediately
    if (bus_inited) {
      if (cfg_osc_hz) driver.setOscillatorFrequency(cfg_osc_hz);
      driver.setPWMFreq(cfg_servo_freq);
    }
  }

  static bool busReady() { return bus_inited; }

private:
  uint8_t  ch;
  uint16_t min_us, max_us;
  bool     reversed;

  // ---- Shared PCA bus ----
  static Adafruit_PWMServoDriver driver;
  static bool bus_inited;
  static uint8_t  cfg_addr;
  static int      cfg_sda, cfg_scl;
  static uint32_t cfg_i2c_hz;
  static uint16_t cfg_servo_freq;
  static uint32_t cfg_osc_hz;

  static void ensureBusInit() {
    if (bus_inited) return;
    Wire.begin(cfg_sda, cfg_scl, cfg_i2c_hz);
    driver = Adafruit_PWMServoDriver(cfg_addr);
    driver.begin();
    if (cfg_osc_hz) driver.setOscillatorFrequency(cfg_osc_hz); // many clones ~27MHz
    driver.setPWMFreq(cfg_servo_freq); // 50 Hz for servos
    bus_inited = true;
  }

  static uint16_t usToTicks(uint16_t us) {
    // ticks = us * freq * 4096 / 1e6
    uint32_t ticks = (uint32_t)us * cfg_servo_freq * 4096UL / 1000000UL;
    if (ticks > 4095) ticks = 4095;
    return (uint16_t)ticks;
  }
};
