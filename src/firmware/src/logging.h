#ifndef LOGGING_H
#define LOGGING_H

#include <variant>

#include "util.h"

struct ModeChange {
  BoardMode old;
  BoardMode next;

  ModeChange(BoardMode old, BoardMode next) : old(old), next(next) {
  }
};

typedef std::variant<String, ModeChange> Message;

void log_message(Message &&content);

bool wait_log_boot();

#endif

