#include "InterpolationLib.h"
#include "bmp.h"
#include "bno.h"
#include "misc.h"
#include "sdfat.h"
#include "wire.h"
#include <atomic>
#include <climits>
#include <cstdint>
#include <deque>
#include <mutex>

unsigned long millis();
unsigned long micros();
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
void delay(unsigned long value);
void delayMicroseconds(unsigned long value);

// TODO: This needs to be added to aditions.txt and this signature may be wrong
void analogWriteResolution(uint8_t);

HardwareSerial_s Serial;

// --------------------------------------------------------------------------

// The string addition feels wrong, but it seems right

// This should be more than enough
const int MAX_EVENTS = 64;

const uint8_t PIN_SD_CS = 17;
const SdSpiConfig SD_CONFIG =
    SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(50));

const unsigned long AUX_DELAY_MIL = 100;

const uint8_t BMP_ADDR = 0x77;

const uint8_t BNO_ID = 55;
const uint8_t BNO_ADDR = 0x28;

const uint8_t PIN_SERVO = 28;
const uint8_t PIN_SERVO_MOSFET = 27;

const unsigned long BMP_DELAY_MIC = 100;
const unsigned long BNO_DELAY_MIC = 100;
const unsigned long MISC_DELAY_MIC = 10000;

const float SERVO_PREP = 0;

typedef struct {
  unsigned long time;
  String message;
} Event;

// Shared
std::mutex events_lock;
std::deque<Event> events(MAX_EVENTS);

std::atomic<float> target_angle = 0;

// Core 1
bool inited = false;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR);

enum { BMP, BNO, MISC, TASK_COUNT };
unsigned long task_timers[TASK_COUNT] = {};
// TASK_COUNT will just be ignored it only does logic on the other enum values
int task = TASK_COUNT;

enum { IDLE, PREP, ARMED, FLYING, DONE };
int state = IDLE;

// Core 2
bool sd_inited = true;
SdFs sd;
FsFile log_file;
FsFile data_file;

void log(const Event &event) {
  String text = "[" + event.time + "] " + event.message;
  if (!sd_inited) {
    log_file.println(event.message);
  }

  Serial.println(text);
}

void log(const String &string) {
  log((Event){.time = millis(), .message = string});
}

// serial_now should only be true in core2
void push_event(String &&message) {
  std::scoped_lock<std::mutex> _lock(events_lock);

  if (events.size() == MAX_EVENTS) {
    // Maybe don't replace if there was already an overflow
    events.front() = (Event){.time = millis(), .message = "Event overflow"};
  } else {
    events.push_front((Event){.time = millis(), .message = message});
  }
}

bool pop_event(Event &event) {
  std::scoped_lock<std::mutex> _lock(events_lock);

  if (events.empty()) {
    return false;
  }

  event = events.back();
  events.pop_back();

  return true;
}

// Micros overflows about every 70 minutes, but because unsigneds
// implement modulo arithmetic it shouldn't matter
unsigned long get_delay(int &state) {
  // If there were more tasks a min heap should be used
  unsigned long time = micros();
  unsigned long delay = ULONG_LONG_MAX;
  for (int i = 0; i < TASK_COUNT; i++) {
    // Time has to be subtracted from each to make sure that overflows don't
    // affect the system in weird ways
    unsigned long curr = task_timers[i] - time;
    // This checks if the task_timers[i] is smaller than time (it will not
    // trigger on an overflow though)
    if (curr > ULONG_LONG_MAX / 2) {
      curr = 0;
    }
    if (curr < delay) {
      delay = curr;
      state = i;
    }
  }
}

void set_servo(float angle) {}

void setup() {
  Wire.begin();
  push_event("Wire inited");

  if (!bmp.begin_I2C(BMP_ADDR)) {
    push_event("BMP failed");
    return;
  }

  if (!bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X)) {
    push_event("BMP temp sampling failed");
    return;
  }

  if (!bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X)) {
    push_event("BMP pressure sampling failed");
    return;
  }
  if (!bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3)) {
    push_event("BMP IIR failed");
    return;
  }
  if (!bmp.setOutputDataRate(BMP3_ODR_50_HZ)) {
    push_event("BMP rate failed");
    return;
  }

  push_event("BMP inited");

  if (!bno.begin()) {
    push_event("BNO failed");
    return;
  }
  bno.setExtCrystalUse(true);

  push_event("BNO inited");

  analogWriteResolution(16);

  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_SERVO_MOSFET, OUTPUT);

  push_event("Pins inited");

  unsigned long time = micros();
  task_timers[0] = time;
  task_timers[1] = time;
  task_timers[2] = time;

  inited = true;
  push_event("Main inited");
}

void setup1() {
  Serial.begin(115200);
  log("Started serial");

  pinMode(LED_BUILTIN, OUTPUT);
  log("Init aux pins");

  if (sd.begin(SD_CONFIG)) {
    log("SD inited");

    sd.mkdir("Logs");
    sd.mkdir("Data");

    for (int i = 0; i < INT_MAX; i++) {
      String log_path = "Logs/log_" + String(i) + ".txt";
      String data_path = "Data/data_" + String(i) + ".csv";
      if (sd.exists(log_path) || sd.exists(data_path)) {
        continue;
      }

      sd_inited = true;
      log_file = sd.open(log_path, O_CREAT | O_WRITE | O_APPEND);
      data_file = sd.open(data_path, O_CREAT | O_WRITE | O_APPEND);
      data_file.println("time,altitude");

      log("Files " + String(i) + " created");
    }
  } else {
    log("SD init failed");
  }

  log("Aux inited");
}

void read_bmp() {}

void read_bno() {}

void update_loop() {}

void set_state(int next) {
  switch (next) {
  case IDLE:
    push_event("New state IDLE");
    break;
  case PREP:
    push_event("New state PREP");
    break;
  case ARMED:
    push_event("New state ARMED");
    break;
  case FLYING:
    push_event("New state FLYING");
    break;
  case DONE:
    push_event("New state DONE");
    break;
  }

  state = next;
}

void run_state() {
  switch (state) {
  case IDLE:
    break;
  case PREP:
    set_servo(SERVO_PREP);
    break;
  case ARMED:
    break;
  case FLYING:
    break;
  case DONE:
    break;
  }
}

void loop() {
  switch (task) {
  case BMP:
    read_bmp();
    if (state == FLYING) {
      update_loop();
      set_servo(target_angle);
    }

    task_timers[BMP] = micros() + BMP_DELAY_MIC;
    break;
  case BNO:
    read_bno();
    if (state == FLYING) {
      update_loop();
      set_servo(target_angle);
    }

    task_timers[BNO] = micros() + BNO_DELAY_MIC;
    break;
  case MISC:
    run_state();

    task_timers[MISC] = micros() + MISC_DELAY_MIC;
    break;
  }

  delayMicroseconds(get_delay(task));
}

void loop1() {
  if (sd_inited) {
    String entry = String(millis()) + ',' + 0;
    data_file.println(entry);
  }

  Event event;
  while (pop_event(event)) {
    log(event);
  }

  delay(AUX_DELAY_MIL);
}
