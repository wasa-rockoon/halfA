#include "Comm.h"

Message<uint32_t> makeMessage(byte *bytes) {
    Message<uint32_t> m;
    m.seq        = bytes[1];
    m.type       = bytes[2];
    m.index      = bytes[3];
    byte* p = reinterpret_cast<byte*>(&m.payload);
    p[0] = bytes[4];
    p[1] = bytes[5];
    p[2] = bytes[6];
    p[3] = bytes[7];
    return m;
}


void Comm::listen(uint8_t type,
            void (*callback)(const Message<uint32_t>& message))  {
    if (callback_count >= MAX_CALLBACKS) return;
    callback_types[callback_count] = type;
    callbacks[callback_count] = callback;
    callback_count++;
}


bool Comm::receive(String &str) {
    // Serial.println("receive: " + str);
    char *err = NULL;
    byte bytes[8];
    for (int i = 0; i < 8; i++) {
        char hex[3];
        hex[0] = str.charAt(2 * i);
        hex[1] = str.charAt(2 * i + 1);
        hex[2] = '\0';
        bytes[i] = (uint8_t)strtol(hex, &err, 16);
        // Serial.print(bytes[i]);
        // Serial.print(" ");
    }
    // Serial.println(" ");
    if (*err != '\0')   return false;
    if (bytes[0] != id) return false;


    Message<uint32_t> message = makeMessage(bytes);

    // Message<float> * m2 = reinterpret_cast<Message<float> *>(&message);
    // log_d("%f", *reinterpret_cast<float*>(bytes + 4));

    for (int i = 0; i < callback_count; i++) {
        if (callback_types[i] == message.type ||
            callback_types[i] == TYPE_ALL)
            callbacks[i](message);
    }

    return true;
}

void Comm::send(uint8_t type, uint8_t index, const uint32_t payload) {
    byte data[8] = {id, seq, type, index};
    byte* p = reinterpret_cast<byte*>(payload);
    for (int i = 0; i < 4; i++) data[4+i] = p[i];

    char str[17];

    for (int i = 0; i < 8; i++) {
        char hex[3];
        sprintf(hex, "%X", data[i]);
        if (data[i] < 16) {
            str[i*2]   = '0';
            str[i*2+1] = hex[0];
        }
        else {
            str[i*2]   = hex[0];
            str[i*2+1] = hex[1];
        }
    }
    str[16] = '\0';
    writeLine(str);
    // log_d("%s", str);
    // log_d("%f %d", payload, payload);
}

void Comm::send(uint8_t type, uint8_t index) {
    byte empty[4];
    send(type, index, empty);
}


void SerialComm::update() {
    while (stream.available() > 0) {
        String str = Serial.readStringUntil('\n');
        receive(str);
    }
}

void SerialComm::writeLine(char* str) {
    stream.println(str);
}
