/*!
 * @file ATDev_INA220.h
 *
 * This is a library for the Adafruit INA220 breakout board
 * ----> https://www.adafruit.com/product/904
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Bryan Siepert and Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef _LIB_ATDev_INA220_
#define _LIB_ATDev_INA220_

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>

/** calculated I2C address: 0 = GND, 1 = V+ **/
/* The address is controlled by the A0 and A1 inputs on the INA220:
 *
 * Calculated address: b100ABCD
 * A0 controls C and D: GND = 00, V+ = 01, SDA = 10, SCL = 11
 * A1 controls A and B: GND = 00, V+ = 01, SDA = 10, SCL = 11
 *
 * E.g. if A0 is tied to ground and A1 is tied to V+,
 * the resulting address is b1000100 = 0x44
 *
 * SDA and SCL options aren't implemented.
 */
#define INA220_CALC_ADDRESS(INA_ADDR0, INA_ADDR1)                              \
  (0x40 | (INA_ADDR0 != 0 ? 0x01 : 0x00) | (INA_ADDR1 != 0 ? 0x04 : 0x00))

/** default I2C address **/
#define INA220_ADDRESS (0x41) // 1000001 (A0=V A1=GND)

/** read **/
#define INA220_READ (0x01)

/*=========================================================================
    CONFIG REGISTER (R/W)
**************************************************************************/

/** config register address **/
#define INA220_REG_CONFIG (0x00)

/** reset bit **/
#define INA220_CONFIG_RESET (0x8000) // Reset Bit

/** mask for bus voltage range **/
#define INA220_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask

/** bus voltage range values **/
enum {
  INA220_CONFIG_BVOLTAGERANGE_16V = (0x0000), // 0-16V Range
  INA220_CONFIG_BVOLTAGERANGE_32V = (0x2000), // 0-32V Range
};

/** mask for gain bits **/
#define INA220_CONFIG_GAIN_MASK (0x1800) // Gain Mask

/** values for gain bits **/
enum {
  INA220_CONFIG_GAIN_1_40MV = (0x0000),  // Gain 1, 40mV Range
  INA220_CONFIG_GAIN_2_80MV = (0x0800),  // Gain 2, 80mV Range
  INA220_CONFIG_GAIN_4_160MV = (0x1000), // Gain 4, 160mV Range
  INA220_CONFIG_GAIN_8_320MV = (0x1800), // Gain 8, 320mV Range
};

/** mask for bus ADC resolution bits **/
#define INA220_CONFIG_BADCRES_MASK (0x0780)

/** values for bus ADC resolution **/
enum {
  INA220_CONFIG_BADCRES_9BIT = (0x0000),  // 9-bit bus res = 0..511
  INA220_CONFIG_BADCRES_10BIT = (0x0080), // 10-bit bus res = 0..1023
  INA220_CONFIG_BADCRES_11BIT = (0x0100), // 11-bit bus res = 0..2047
  INA220_CONFIG_BADCRES_12BIT = (0x0180), // 12-bit bus res = 0..4097
  INA220_CONFIG_BADCRES_12BIT_2S_1060US =
      (0x0480), // 2 x 12-bit bus samples averaged together
  INA220_CONFIG_BADCRES_12BIT_4S_2130US =
      (0x0500), // 4 x 12-bit bus samples averaged together
  INA220_CONFIG_BADCRES_12BIT_8S_4260US =
      (0x0580), // 8 x 12-bit bus samples averaged together
  INA220_CONFIG_BADCRES_12BIT_16S_8510US =
      (0x0600), // 16 x 12-bit bus samples averaged together
  INA220_CONFIG_BADCRES_12BIT_32S_17MS =
      (0x0680), // 32 x 12-bit bus samples averaged together
  INA220_CONFIG_BADCRES_12BIT_64S_34MS =
      (0x0700), // 64 x 12-bit bus samples averaged together
  INA220_CONFIG_BADCRES_12BIT_128S_69MS =
      (0x0780), // 128 x 12-bit bus samples averaged together

};

/** mask for shunt ADC resolution bits **/
#define INA220_CONFIG_SADCRES_MASK                                             \
  (0x0078) // Shunt ADC Resolution and Averaging Mask

/** values for shunt ADC resolution **/
enum {
  INA220_CONFIG_SADCRES_9BIT_1S_84US = (0x0000),   // 1 x 9-bit shunt sample
  INA220_CONFIG_SADCRES_10BIT_1S_148US = (0x0008), // 1 x 10-bit shunt sample
  INA220_CONFIG_SADCRES_11BIT_1S_276US = (0x0010), // 1 x 11-bit shunt sample
  INA220_CONFIG_SADCRES_12BIT_1S_532US = (0x0018), // 1 x 12-bit shunt sample
  INA220_CONFIG_SADCRES_12BIT_2S_1060US =
      (0x0048), // 2 x 12-bit shunt samples averaged together
  INA220_CONFIG_SADCRES_12BIT_4S_2130US =
      (0x0050), // 4 x 12-bit shunt samples averaged together
  INA220_CONFIG_SADCRES_12BIT_8S_4260US =
      (0x0058), // 8 x 12-bit shunt samples averaged together
  INA220_CONFIG_SADCRES_12BIT_16S_8510US =
      (0x0060), // 16 x 12-bit shunt samples averaged together
  INA220_CONFIG_SADCRES_12BIT_32S_17MS =
      (0x0068), // 32 x 12-bit shunt samples averaged together
  INA220_CONFIG_SADCRES_12BIT_64S_34MS =
      (0x0070), // 64 x 12-bit shunt samples averaged together
  INA220_CONFIG_SADCRES_12BIT_128S_69MS =
      (0x0078), // 128 x 12-bit shunt samples averaged together
};

/** mask for operating mode bits **/
#define INA220_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask

/** values for operating mode **/
enum {
  INA220_CONFIG_MODE_POWERDOWN = 0x00,       /**< power down */
  INA220_CONFIG_MODE_SVOLT_TRIGGERED = 0x01, /**< shunt voltage triggered */
  INA220_CONFIG_MODE_BVOLT_TRIGGERED = 0x02, /**< bus voltage triggered */
  INA220_CONFIG_MODE_SANDBVOLT_TRIGGERED =
      0x03,                         /**< shunt and bus voltage triggered */
  INA220_CONFIG_MODE_ADCOFF = 0x04, /**< ADC off */
  INA220_CONFIG_MODE_SVOLT_CONTINUOUS = 0x05, /**< shunt voltage continuous */
  INA220_CONFIG_MODE_BVOLT_CONTINUOUS = 0x06, /**< bus voltage continuous */
  INA220_CONFIG_MODE_SANDBVOLT_CONTINUOUS =
      0x07, /**< shunt and bus voltage continuous */
};

/** shunt voltage register **/
#define INA220_REG_SHUNTVOLTAGE (0x01)

/** bus voltage register **/
#define INA220_REG_BUSVOLTAGE (0x02)

/** power register **/
#define INA220_REG_POWER (0x03)

/** current register **/
#define INA220_REG_CURRENT (0x04)

/** calibration register **/
#define INA220_REG_CALIBRATION (0x05)

/*!
 *   @brief  Class that stores state and functions for interacting with INA220
 *  current/power monitor IC
 */
class ATDev_INA220 {
public:
  ATDev_INA220(uint8_t addr = INA220_ADDRESS);
  ~ATDev_INA220();
  bool begin(TwoWire *theWire = &Wire);
  void setCalibration_ATDev_32V_2A();
  void setCalibration_32V_2A();
  void setCalibration_32V_1A();
  void setCalibration_16V_400mA();
  float getBusVoltage_V();
  float getShuntVoltage_mV();
  float getCurrent_mA();
  float getPower_mW();
  void powerSave(bool on);
  bool success();

private:
  Adafruit_I2CDevice *i2c_dev = NULL;

  bool _success;

  uint8_t INA220_i2caddr = -1;
  uint32_t INA220_calValue;
  // The following multipliers are used to convert raw current and power
  // values to mA and mW, taking into account the current config settings
  float INA220_currentDivider_mA;
  float INA220_powerMultiplier_mW;

  void init();
  int16_t getBusVoltage_raw();
  int16_t getShuntVoltage_raw();
  int16_t getCurrent_raw();
  int16_t getPower_raw();
};

#endif
