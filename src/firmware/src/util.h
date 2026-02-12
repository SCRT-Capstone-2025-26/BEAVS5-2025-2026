#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

#define gravity_acc 981

typedef unsigned long Millis;

enum BoardMode {
  BOOTING,
  UNARMED,
  ARMED,
  FLYING,
  DONE,
  FAILURE
};

static String modeToName[] = {
  "Booting",
  "Unarmed",
  "Armed",
  "Flying",
  "Done",
  "Failure",
};

#endif

