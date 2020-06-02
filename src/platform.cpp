#include "platform.h"

bool PublishFuture::isDone() {}

bool PublishFuture::isSucceeded() {}

bool NetworkTime::isValid() {}

uint32_t NetworkTime::now() // Figure out return value
{}

PublishFuture Platform::publish(const char *eventName, const char *eventData) {}

bool Platform::connected() {}

bool Platform::function(const char *callName, int (*fnct)(String)) {}
