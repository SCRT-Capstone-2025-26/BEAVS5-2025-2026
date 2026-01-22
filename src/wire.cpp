#include "wire.h"

#include <cassert>
#include <stdexcept>

void TwoWire::begin() {
  if (began) { throw std::invalid_argument("Already began"); }
  began = true;
}

bool TwoWire::began_s() const { return began; }
