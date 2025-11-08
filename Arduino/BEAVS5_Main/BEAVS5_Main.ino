#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <atomic>
#include <deque>
#include <cmath>

// The string addition feels wrong, but it seems right
// This code will not handle rollovers for millis() (~49.7 days), but will for
// micros() (~71.5 minutes)

typedef struct {
  unsigned long time;
  String message;
} Event;

// Spinlocks are not very efficient and should be used sparingly on quick operations
typedef class {
private:
    std::atomic_flag flag = ATOMIC_FLAG_INIT;

public:
    void lock() {
        while (flag.test_and_set(std::memory_order_acquire));
    }

    void unlock() {
        flag.clear(std::memory_order_release);
    }
} SpinLock;

typedef unsigned long Millis;
typedef unsigned long Micros;

// This should be more than enough
const size_t MAX_EVENTS = 64;

const uint8_t PIN_SD_CS = 17;
const SdSpiConfig SD_CONFIG =
    SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(50));

const Millis AUX_DELAY = 25;

const uint8_t BMP_ADDR = 0x77;

const uint8_t BNO_ID = 55;
const uint8_t BNO_ADDR = 0x28;

const uint8_t PIN_SERVO = 28;
const uint8_t PIN_SERVO_MOSFET = 27;
const uint8_t PIN_ARM = 29;

const Micros BMP_DELAY = 1000;
const Micros BNO_DELAY = 1000;
// Running PID faster than sensors is kinda pointless
const Micros PID_DELAY = 500;
const Micros MISC_DELAY = 10000;

const float BMP_DELAY_SEC = (float)BMP_DELAY / 1000.0F / 1000.0F;
const float PID_DELAY_SEC = (float)PID_DELAY / 1000.0F / 1000.0F;

const float P = 0.1;
const float I = 0;
const float D = 0;

const float I_MAX = 0;

const int SERVO_RETRACTED = 0;
const int SERVO_FLUSH = 50;
const int SERVO_MAX = 100;

const float SEA_PRESSURE = 0;

const Millis ARM_TIME = 5 * 1000;

const float TARGET_HEIGHT = 1000;
const float G = 9.81;
const float DRAG = 0.5F;

// Shared
// AFAIK the atomics will cause lots of mfences and so could be optimized
// Espiecally given some are written to one one thread
// This code will used explicit references to atomic read and write operations
// There is seemingly no std::mutex so a spinlock will be used
SpinLock events_lock;
std::deque<Event> events = std::deque<Event>();

std::atomic<int> servo_pwm;

std::atomic<float> altitude;
std::atomic<float> velocity;

std::atomic<float> sense_alt;
std::atomic<sensors_event_t> sense_gyro;
std::atomic<sensors_event_t> sense_acc;

// Core 1
bool inited = false;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);

typedef enum { BMP, BNO, PID, MISC, TASK_COUNT } Task;
Micros task_timers[TASK_COUNT] = {};
// TASK_COUNT will just be ignored it only does logic on the other enum values
Task task = TASK_COUNT;

typedef enum { PREFLIGHT, ARMING, ARMED, BURN, FLYING, DONE } State;
State state;

Micros curr_state_time;

float integral_sum;
float prev_error;

// Core 2
bool sd_inited = false;
SdFs sd;
FsFile log_file;
FsFile data_file;

void write_log(const Event &event) {
  String text = "[" + String(event.time) + "] " + event.message;
  if (sd_inited) {
    log_file.println(event.message);
  }

  Serial.println(text);
}

void write_log(const String &string) {
  write_log((Event){.time = millis(), .message = string});
}

// serial_now should only be true in core2
void push_event(String &&message) {
  events_lock.lock();

  if (events.size() == MAX_EVENTS) {
    // Maybe don't replace if there was already an overflow
    events.front() = (Event){.time = millis(), .message = "Event overflow"};
  } else {
    events.push_front((Event){.time = millis(), .message = message});
  }

  events_lock.unlock();
}

bool pop_event(Event &event) {
  events_lock.lock();

  if (events.empty()) {
  events_lock.unlock();
    return false;
  }

  event = events.back();
  events.pop_back();

  events_lock.unlock();
  return true;
}

// Micros overflows about every 70 minutes, but because unsigneds
// implement modulo arithmetic it shouldn't matter
Micros get_delay(Task &task) {
  // If there were more tasks a min heap should be used
  Micros time = micros();
  Micros value = ULONG_LONG_MAX;
  for (int i = 0; i < TASK_COUNT; i++) {
    // Time has to be subtracted from each to make sure that overflows don't
    // affect the system in weird ways
    Micros curr = task_timers[i] - time;
    // This checks if the task_timers[i] is smaller than time (it will not
    // trigger on an overflow though)
    if (curr > ULONG_LONG_MAX / 2) {
      curr = 0;
    }

    if (curr < value) {
      value = curr;
      task = (Task)i;
    }
  }

  return value;
}

// Maps float to servo extension
// 0 is flush and 1 is full extension
int to_servo_pwm(float value) {
  if (value <= 0) {
    return SERVO_FLUSH;
  }

  if (value >= 1) {
    return SERVO_MAX;
  }

  // The + 0.5F makes it round to the nearest int on return
  return (value * (SERVO_MAX - SERVO_FLUSH)) + SERVO_FLUSH + 0.5F;
}

void set_servo(float value) { analogWrite(PIN_SERVO, value); }

void read_bmp() {
  if (!bmp.performReading()) {
    push_event("BMP failure");
  }

  sense_alt.store(bmp.readAltitude(SEA_PRESSURE));

  velocity.store((sense_alt.load() - altitude.load()) / BMP_DELAY_SEC);
  altitude.store(sense_alt.load());
}

void read_bno() {
  sensors_event_t sense;
  if (!bno.getEvent(&sense, Adafruit_BNO055::VECTOR_LINEARACCEL)) {
    push_event("BNO accelerometer failure");
  } else {
    sense_acc.store(sense);
  }

  if (!bno.getEvent(&sense, Adafruit_BNO055::VECTOR_EULER)) {
    push_event("BNO gyroscope failure");
  } else {
    sense_gyro.store(sense);
  }
}

// This is not code uses some pretty slow functions and should be reconsidered
float calc_error() {
  float c1 = -velocity.load() - (G / DRAG);
  float max_time = log(-c1 * DRAG / G) * DRAG;

  float max_alt = (((c1 * exp(-DRAG * max_time)) - (G * max_time) - c1) / DRAG) + altitude.load();
  return max_alt - TARGET_HEIGHT;
}

void set_state(State next) {
  curr_state_time = millis();

  switch (next) {
  case PREFLIGHT:
    push_event("New state PREFLIGHT");

    set_servo(SERVO_RETRACTED);
    break;
  case ARMING:
    push_event("New state ARMING");

    set_servo((SERVO_RETRACTED + SERVO_FLUSH) * 0.5F);
    break;
  case ARMED:
    push_event("New state ARMED");

    // Calling twice will init velocity properly
    read_bmp();
    read_bmp();

    read_bno();

    set_servo(SERVO_FLUSH);
    break;
  case BURN:
    break;
  case FLYING:
    push_event("New state FLYING");

    integral_sum = 0;
    prev_error = calc_error();

    break;
  case DONE:
    push_event("New state DONE");
    break;
  }

  state = next;
}

// This is not the only place where the state can change
void run_state() {
  switch (state) {
  case PREFLIGHT:
    if (digitalRead(PIN_ARM)) {
      set_state(ARMING);
    }

    break;
  case ARMING:
    if (digitalRead(PIN_ARM)) {
      if (curr_state_time + ARM_TIME <= millis())
        set_state(ARMED);
    } else {
      set_state(PREFLIGHT);
    }

    break;
  case ARMED:
    // TODO: Make this more robust
    if (altitude.load() > 10) {
      set_state(BURN);
    }
    break;
  case BURN:
    // TODO: Make this more robust
    if (sense_acc.load().acceleration.y < 1) {
      set_state(FLYING);
    }
    break;
  case FLYING:
    // TODO: Make this more robust
    if (velocity.load() < 5) {
      set_state(DONE);
    }
    break;
  case DONE:
    break;
  }
}

void setup() {
  Wire.begin();
  push_event("Wire inited");

  if (!bmp.begin_I2C(BMP_ADDR, &Wire)) {
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

  // This does nothing, but is for clarity
  pinMode(PIN_ARM, INPUT);

  // The other servo pin is analog and so no inited
  pinMode(PIN_SERVO_MOSFET, OUTPUT);

  push_event("Pins inited");

  Micros time = micros();
  task_timers[0] = time;
  task_timers[1] = time;
  task_timers[2] = time;

  set_state(PREFLIGHT);

  inited = true;
  push_event("Main inited");
}

void setup1() {
  Serial.begin(115200);
  write_log("Started serial");

  if (sd.begin(SD_CONFIG)) {
    write_log("SD inited");

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

      write_log("Files " + String(i) + " created");
      break;
    }
  } else {
    write_log("SD init failed");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  write_log("Init aux pins");

  write_log("Aux inited");
}

void update_loop() {
  float error = calc_error();

  integral_sum += error * PID_DELAY_SEC;
  if (integral_sum > I_MAX) {
    integral_sum = INT_MAX;
  } else if (integral_sum < -I_MAX) {
    integral_sum = -I_MAX;
  }

  float p_out = P * error;
  float i_out = I * integral_sum;
  float d_out = D * (error - prev_error) / PID_DELAY_SEC;

  prev_error = error;

  float out = p_out + i_out - d_out;
  servo_pwm.store(to_servo_pwm(out));
}

void loop() {
  switch (task) {
  case BMP:
    read_bmp();

    task_timers[BMP] += BMP_DELAY;
    break;
  case BNO:
    read_bno();

    task_timers[BNO] += BNO_DELAY;
    break;
  case PID:
    if (state == FLYING) {
      update_loop();
      set_servo(servo_pwm.load());
    }

    task_timers[PID] += PID_DELAY;
    break;
  case MISC:
    task_timers[MISC] += MISC_DELAY;
    break;
  }

  run_state();
  delayMicroseconds(get_delay(task));
}

void loop1() {
  if (sd_inited) {
    String entry = String(millis()) + ',' + servo_pwm.load();
    data_file.println(entry);
  }

  Event event;
  while (pop_event(event)) {
    write_log(event);
  }

  // This loop doesn't need to run on a strict time
  delay(AUX_DELAY);
}
