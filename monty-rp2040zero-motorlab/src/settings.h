#ifndef SETTINGS_H
#define SETTINGS_H

#include "../config.h"
#include "utils.h"
#include <Arduino.h>
//#include <EEPROM.h>

/***
 * Settings is where we hold the working copies of many of the
 * parameters described in the config files
 */
struct Settings {
  struct Data {
    uint8_t control_flags;
    float degPerCount;
    float Km;
    float Tm;
    float zeta;
    float Td;
    float Kp;
    float Kd;
    float rKm;
    float rTm;
    float rzeta;
    float rTd;
    float rKp;
    float rKd;
    float biasFF;
    float speedFF;
    float accFF;
  };

  Data data;

  void init(Data defaults) {
    data = defaults;
  }

  /***
   * Read the working settings from EEPROM.
   * Undoes changes
   */
  void read() {
    Data d;
  //  EEPROM.get(0, d);
  //  data = d;
  };

  /***
   * Store the current working settings to EEPROM
   */
  void write() {
  //  EEPROM.put(0, data);
  };

  /***
   * Print the current working settings
   */
  /* clang-format off */
  void print() {
    SerialPort.print(F("    degPerCount = "));    SerialPort.println(data.degPerCount, 5);
    SerialPort.print(F("             Km = "));    SerialPort.println(data.Km, 2);
    SerialPort.print(F("             Tm = "));    SerialPort.println(data.Tm, 5);
    SerialPort.print(F("            rKm = "));    SerialPort.println(data.rKm, 2);
    SerialPort.print(F("            rTm = "));    SerialPort.println(data.rTm, 5);
    SerialPort.print(F("         biasFF = "));    SerialPort.println(data.biasFF, 5);
    SerialPort.print(F("        speedFF = "));    SerialPort.println(data.speedFF, 5);
    SerialPort.print(F("          accFF = "));    SerialPort.println(data.accFF, 5);
    SerialPort.print(F("           zeta = "));    SerialPort.println(data.zeta, 5);
    SerialPort.print(F("             Td = "));    SerialPort.println(data.Td, 5);
    SerialPort.print(F("             KP = "));    SerialPort.println(data.Kp, 5);
    SerialPort.print(F("             KD = "));    SerialPort.println(data.Kd, 5);
    SerialPort.print(F("          rZeta = "));    SerialPort.println(data.rzeta, 5);
    SerialPort.print(F("            rTd = "));    SerialPort.println(data.rTd, 5);
    SerialPort.print(F("            rKP = "));    SerialPort.println(data.rKp, 5);
    SerialPort.print(F("            rKD = "));    SerialPort.println(data.rKd, 5);
  };
};
/* clang-format on */

extern Settings settings;

/***
 * These are the as-compiled defaults. They are
 * used to initialise the working settings on-demand
 */
const Settings::Data defaults = {
  control_flags : 0,
  degPerCount : DEG_PER_COUNT,
  Km : FWD_KM,
  Tm : FWD_TM,
  zeta : FWD_ZETA,
  Td : FWD_TD,
  Kp : KP,
  Kd : KD,
  rKm : ROT_KM,
  rTm : ROT_TM,
  rzeta : ROT_ZETA,
  rTd : ROT_TD,
  rKp : RKP,
  rKd : RKD,
  biasFF : BIAS_FF,
  speedFF : SPEED_FF,
  accFF : ACC_FF,
};

#endif
