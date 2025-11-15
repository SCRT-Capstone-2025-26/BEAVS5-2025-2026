#ifndef BMP_H
#define BMP_H

#include <cstdint>
#include <stdint.h>

#include "wire.h"

#define BMP3_NO_OVERSAMPLING UINT8_C(0x00)
#define BMP3_OVERSAMPLING_2X UINT8_C(0x01)
#define BMP3_OVERSAMPLING_4X UINT8_C(0x02)
#define BMP3_OVERSAMPLING_8X UINT8_C(0x03)
#define BMP3_OVERSAMPLING_16X UINT8_C(0x04)
#define BMP3_OVERSAMPLING_32X UINT8_C(0x05)

class Sim_s;

// Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf
// Only some of the parts are implemented. Some non-implemented things being used it will cause an assert to fail
// There are other functions that are not used and the ODR seems to be unused given that it is read in FORCED mode
class Adafruit_BMP3XX {
  bool began = false;

  uint8_t temp_os = BMP3_NO_OVERSAMPLING;
  uint8_t pres_os = BMP3_NO_OVERSAMPLING;

  // IDK why this is a double and readPressure is not
  double pressure;

public:
  Adafruit_BMP3XX() {}

  float pressure_s = 0.0F;
  Sim_s *sim_s = nullptr;

  bool begin_I2C(uint8_t addr, TwoWire *theWire);
  float readPressure();

  bool setTemperatureOversampling(uint8_t os);
  bool setPressureOversampling(uint8_t os);
  bool setIIRFilterCoeff(uint8_t fs);
  bool setOutputDataRate(uint8_t odr);

  bool performReading();
};

extern float tempurature_s;
extern float pressure_s;
extern float altitude_s;

#endif
