#ifndef MS5607_H
#define MS5607_H

#include <cstdint>
#include <stdint.h>

#include "wire.h"

#define R_ADC 0X00
#define PROM_READ 0xA0
#define RESET 0x1E

class Sim_s;

// Datasheet: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5607-02BA03&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=MS560702BA03-50f
// Arudino Library: https://github.com/UravuLabs/MS5607
// Only some of the parts are implemented. Some non-implemented things being used it will cause an assert to fail
// Some assertations are more strict than the actual software such as begin to avoid bad code
class MS5607 {
  const short OSR_UNDEF = 0;

  bool began = false;
  short osr = OSR_UNDEF;

  float pressure_mbar;
  float temperature;
public:
  Sim_s *sim_s = nullptr;

  char begin();

  void setOSR(short OSR_U);

  float getTemperature(void);
  float getPressure(void);

  char readDigitalValue(void);
};

#endif
