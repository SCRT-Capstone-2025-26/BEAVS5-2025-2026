#ifndef LOGGING_H
#define LOGGING_H

#include <variant>

typedef std::variant<String> Message;

void log_message(Message &content);

bool wait_log_boot();

#endif

