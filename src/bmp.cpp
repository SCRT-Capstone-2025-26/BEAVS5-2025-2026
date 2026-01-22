#include "bmp.h"

#include "wire.h"
#include <cassert>
#include <stdexcept>

bool Adafruit_BMP3XX::begin_I2C(uint8_t addr, TwoWire *theWire) {
  if (began) { throw std::invalid_argument("Already began"); }
  if (addr != 0x77) { throw std::invalid_argument("Unsupported address"); }
  if (!theWire->began_s()) { throw std::invalid_argument("Wire hasn't began"); }

  began = true;
  return true;
}

float Adafruit_BMP3XX::readTemperature() {
  if (!began) { throw std::invalid_argument("Hasn't began"); }

  return tempurature_s;
}

float Adafruit_BMP3XX::readPressure(void) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }

  return pressure_s;
}

float Adafruit_BMP3XX::readAltitude(float seaLevel) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }
  if (seaLevel != 0) { throw std::invalid_argument("Unsupported seaLevel"); }

  return altitude_s;
}

bool Adafruit_BMP3XX::setTemperatureOversampling(uint8_t os) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }
  if (os != BMP3_OVERSAMPLING_8X) { throw std::invalid_argument("Unsupported os"); }

  return true;
}

bool Adafruit_BMP3XX::setPressureOversampling(uint8_t os) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }
  if (os != BMP3_OVERSAMPLING_16X) { throw std::invalid_argument("Unsupported os"); }

  return true;
}

bool Adafruit_BMP3XX::setIIRFilterCoeff(uint8_t fs) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }
  if (fs != BMP3_IIR_FILTER_COEFF_3) { throw std::invalid_argument("Unsupported fs"); }

  return true;
}

bool Adafruit_BMP3XX::setOutputDataRate(uint8_t odr) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }
  if (odr != BMP3_ODR_50_HZ) { throw std::invalid_argument("Unsupported odr"); }

  return true;
}

bool Adafruit_BMP3XX::performReading() {
  if (!began) { throw std::invalid_argument("Hasn't began"); }

  return true;
}
