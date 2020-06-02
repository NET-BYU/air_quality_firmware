#ifndef PLATFORM_H_
#define PLATFORM_H_

class PublishFuture {
  public:
    bool isDone();
    bool isSucceeded();
};

class NetworkTime {
  public:
    bool isValid();
    uint32_t now(); // Figure out return value
};

class Platform {
  public:
    PublishFuture publish(const char *eventName, const char *eventData);
    bool connected();
    bool function(const char *callName, int (*fnct)(String));
};

#endif