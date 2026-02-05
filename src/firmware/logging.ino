#include <atomic>
#include <SdFat.h>
#include <variant>
// TODO: Possibly add radio

#include "logging.h"
#include "defs.h"
#include "eventqueue.h"

std::atomic_bool log_booted = false;
std::atomic<bool> log_failure;

SdFs sd;
FsFile log_file;
FsFile data_file;

struct LogEvent {
  Millis timestamp;
  std::variant<String> value;
};

EventQueue<LogEvent, 16> events();

// Shared functions

void log_message(String &content) {
  LogEvent event;
  event.timestamp = millis();
  event.value = content;

  events.putQ(event);
}

bool wait_log_boot() {
  // There are fancier ways, but it doesn't matter since this should be a short wait
  while (!log_booted) { delay(1); }

  return log_failure;
}

// Core 1 functions

void setup1() {
  Serial.begin(115200);

  if (!sd.begin(SD_CONFIG)) {
    goto failure;
  }

  sd.mkdir("Logs");
  sd.mkdir("Data");

  bool failed = true;
  for (int i = 0; i < INT_MAX; i++) {
    String log_path = "Logs/log_" + String(i) + ".txt";
    String data_path = "Data/data_" + String(i) + ".csv";
    if (sd.exists(log_path) || sd.exists(data_path)) {
      continue;
    }

    log_file = sd.open(log_path, (oflag_t)(O_CREAT | O_WRITE | O_APPEND));
    data_file = sd.open(data_path, (oflag_t)(O_CREAT | O_WRITE | O_APPEND));
    data_file.println("time,altitude");

    write_log("Files " + String(i) + " created");
    failed = false;
    break;
  }

  if (failed) {
    goto failure;
  }

  log_failure = false;
  log_booted = true;
  return;

failure:
  log_failure = true;
  log_booted = true;
}

void write_log(String &content) {
  log_file.println(content);
  Serial.println(content);
}

void handle_event(LogEvent &event) {
  String *content = event.value.get_if<String>();
  if (event != nullptr) {
    write_log("[" + String(event.timestamp) + "]" + content);
  }
}

void loop1() {
  if (log_failure) {
    delay(10);
    return;
  }

  LogEvent event;
  while (true) {
    while (events.getQ(event)) {
      handle_event(event);
    }

    delay(1);
  }
}

