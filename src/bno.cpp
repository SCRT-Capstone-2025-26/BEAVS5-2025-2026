#include "bno.h"

#include "wire.h"
#include <cassert>
#include <stdexcept>

Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address, TwoWire *theWire) {
  if (began) { throw std::invalid_argument("Already began"); }
  if (sensorID != 55) { throw std::invalid_argument("Unsupported sensorID"); }
  if (address != 0x28) { throw std::invalid_argument("Unsupported address"); }

  wire = theWire;
}

bool Adafruit_BNO055::begin() {
  if (!wire->began_s()) { throw std::invalid_argument("Wire hasn't began"); }

  began = true;

  return true;
}

void Adafruit_BNO055::setExtCrystalUse(bool usextal) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }

  extCrystal = usextal;
}

bool Adafruit_BNO055::getEvent(sensors_event_t *event,
                               adafruit_vector_type_t type) {
  if (!began) { throw std::invalid_argument("Hasn't began"); }
  if (!extCrystal) { throw std::invalid_argument("Ext crystal not set"); }

  switch (type) {
  case VECTOR_LINEARACCEL:
    event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
    event->acceleration = acc_state_s;
    break;
  case VECTOR_EULER:
    event->type = SENSOR_TYPE_ORIENTATION;
    event->orientation = gyro_state_s;
    break;
  }

  return true;
}
