#pragma once

#include <Arduino.h>

#define MAX_CALLBACKS 16
#define TYPE_ALL 0

template <typename T> class Message {
 public:
    uint8_t seq;
    uint8_t type;
    uint8_t index;
    T       payload;
};


typedef void (*Callback)(const Message<uint32_t>& payload);

class Comm {
private:
    uint8_t id;

    uint8_t seq = 0;

    uint8_t callback_types[MAX_CALLBACKS];
    Callback callbacks[MAX_CALLBACKS];
    uint8_t callback_count = 0;

public:

    Comm(uint8_t id): id(id) {};

    void nextSequence() {seq++;};

    // Receiving
    template <typename T>
        void listen(uint8_t type,
                    void (*callback)(const Message<T>& message)) {
        listen(type, reinterpret_cast<Callback>(callback));
    }

    // Sending
    void send(uint8_t type, uint8_t index);
    template <typename T>
        void send(uint8_t type, uint8_t index, const T& payload) {
        return send(type, index, reinterpret_cast<const uint32_t>(&payload));
    }

    bool receive(String &str);

private:
    void listen(uint8_t type,
                void (*callback)(const Message<uint32_t>& message));
    void send(uint8_t type, uint8_t index, const uint32_t payload);

protected:
    virtual void writeLine(char* str) = 0;
};

class SerialComm: public Comm {
private:
    Stream &stream;
public:
    SerialComm(uint8_t id, Stream &stream = Serial):
        Comm::Comm(id), stream(stream) {};
    void update();

private:
    void writeLine(char* str);
};
