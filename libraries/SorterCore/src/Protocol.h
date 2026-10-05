#ifndef SORTER_PROTOCOL_H
#define SORTER_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

namespace sorter {

constexpr size_t kMaxPayload = 400;
constexpr size_t kMaxBody = 480;
constexpr size_t kMaxFrame = 512;
constexpr size_t kMaxObjects = 8;

struct Packet {
    char type = 0;
    uint32_t session = 0;
    uint32_t seq = 0;
    char payload[kMaxPayload + 1] = {};
};
struct Detection {
    uint8_t classId = 0;
    int32_t x10 = 0;
    int32_t y10 = 0;
    uint32_t pixels = 0;
};
struct Scene {
    uint32_t cameraBoot = 0;
    uint32_t calibrationId = 0;
    uint8_t count = 0;
    Detection objects[kMaxObjects] = {};
};

inline uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

inline bool parseU32(const char* text, uint32_t& out) {
    if (!text || !*text) return false;
    uint32_t value = 0;
    for (const char* p = text; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        const uint32_t digit = static_cast<uint32_t>(*p - '0');
        if (value > (UINT32_MAX - digit) / 10U) return false;
        value = value * 10U + digit;
    }
    out = value;
    return true;
}

inline bool parseI32(const char* text, int32_t& out) {
    if (!text || !*text) return false;
    const bool negative = text[0] == '-';
    uint32_t value;
    if (!parseU32(text + (negative ? 1 : 0), value)) return false;
    if (negative) {
        if (value > 2147483648UL) return false;
        out = value == 2147483648UL ? INT32_MIN : -static_cast<int32_t>(value);
    } else {
        if (value > INT32_MAX) return false;
        out = static_cast<int32_t>(value);
    }
    return true;
}

inline bool splitExact(char* text, char delim, char** fields, size_t n) {
    if (!text || !n) return false;
    fields[0] = text;
    for (size_t i = 1; i < n; ++i) {
        char* p = strchr(fields[i - 1], delim);
        if (!p) return false;
        *p = 0;
        fields[i] = p + 1;
    }
    if (strchr(fields[n - 1], delim)) return false;
    for (size_t i = 0; i < n; ++i) if (!*fields[i]) return false;
    return true;
}

inline bool encodeScene(const Scene& scene, char* output, size_t capacity) {
    if (!output || !capacity) return false;
    output[0] = 0;
    if (!scene.cameraBoot || !scene.calibrationId || scene.count > kMaxObjects) return false;
    int n = snprintf(output, capacity, "%lu,%lu,%u", static_cast<unsigned long>(scene.cameraBoot),
                     static_cast<unsigned long>(scene.calibrationId), static_cast<unsigned>(scene.count));
    if (n < 0 || static_cast<size_t>(n) >= capacity) { output[0] = 0; return false; }
    size_t used = static_cast<size_t>(n);
    for (size_t i = 0; i < scene.count; ++i) {
        const Detection& d = scene.objects[i];
        if (d.classId < 1 || d.classId > 3 || !d.pixels) { output[0] = 0; return false; }
        n = snprintf(output + used, capacity - used, ";%u,%ld,%ld,%lu", static_cast<unsigned>(d.classId),
                     static_cast<long>(d.x10), static_cast<long>(d.y10), static_cast<unsigned long>(d.pixels));
        if (n < 0 || static_cast<size_t>(n) >= capacity - used) { output[0] = 0; return false; }
        used += static_cast<size_t>(n);
    }
    return true;
}

inline bool decodeScene(const char* payload, Scene& output) {
    if (!payload) return false;
    const size_t n = strlen(payload);
    if (n > kMaxPayload) return false;
    char copy[kMaxPayload + 1];
    memcpy(copy, payload, n + 1);
    char* objectStart = strchr(copy, ';');
    if (objectStart) *objectStart++ = 0;
    char* head[3];
    uint32_t count;
    Scene candidate;
    if (!splitExact(copy, ',', head, 3) || !parseU32(head[0], candidate.cameraBoot) ||
        !parseU32(head[1], candidate.calibrationId) || !parseU32(head[2], count) ||
        !candidate.cameraBoot || !candidate.calibrationId || count > kMaxObjects) return false;
    candidate.count = static_cast<uint8_t>(count);
    if (!count && objectStart) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!objectStart || !*objectStart) return false;
        char* next = strchr(objectStart, ';');
        if (next) *next++ = 0;
        char* f[4];
        uint32_t cls;
        Detection& d = candidate.objects[i];
        if (!splitExact(objectStart, ',', f, 4) || !parseU32(f[0], cls) || cls < 1 || cls > 3 ||
            !parseI32(f[1], d.x10) || !parseI32(f[2], d.y10) || !parseU32(f[3], d.pixels) || !d.pixels)
            return false;
        d.classId = static_cast<uint8_t>(cls);
        objectStart = next;
    }
    if (objectStart) return false;
    output = candidate;
    return true;
}

inline bool validPayload(const Packet& packet) {
    size_t len = 0;
    while (len <= kMaxPayload && packet.payload[len]) {
        const unsigned char c = static_cast<unsigned char>(packet.payload[len]);
        if (c < 32 || c > 126 || c == '@' || c == '|' || c == '*') return false;
        ++len;
    }
    if (!len || len > kMaxPayload) return false;
    if (packet.type == 'Q') {
        uint32_t id;
        return parseU32(packet.payload, id) && id != 0;
    }
    if (packet.type == 'D') {
        Scene scene;
        return decodeScene(packet.payload, scene);
    }
    if (packet.type == 'E') {
        if (len > 32 || packet.payload[0] < 'A' || packet.payload[0] > 'Z') return false;
        for (size_t i = 1; i < len; ++i) {
            const char c = packet.payload[i];
            if (!(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9') && c != '_') return false;
        }
        return true;
    }
    return false;
}

inline bool encodePacket(const Packet& packet, char* output, size_t capacity) {
    if (!output || !capacity) return false;
    output[0] = 0;
    if (!packet.session || !packet.seq || !validPayload(packet)) return false;
    char body[kMaxBody + 1];
    const int n = snprintf(body, sizeof(body), "1|%c|%lu|%lu|%s", packet.type,
                           static_cast<unsigned long>(packet.session), static_cast<unsigned long>(packet.seq),
                           packet.payload);
    if (n < 0 || static_cast<size_t>(n) > kMaxBody) return false;
    const uint16_t check = crc16(reinterpret_cast<const uint8_t*>(body), static_cast<size_t>(n));
    const int written = snprintf(output, capacity, "@%s*%04X\n", body, static_cast<unsigned>(check));
    if (written < 0 || static_cast<size_t>(written) >= capacity) { output[0] = 0; return false; }
    return true;
}

inline int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

inline bool decodePacket(const char* frame, Packet& output) {
    if (!frame || frame[0] != '@') return false;
    size_t length = strlen(frame);
    if (length >= kMaxFrame) return false;
    if (length && frame[length - 1] == '\n') --length;
    if (length && frame[length - 1] == '\r') --length;
    const char* star = strchr(frame, '*');
    if (!star || star - frame < 2 || static_cast<size_t>(star - frame) + 5 != length) return false;
    const size_t bodyLength = static_cast<size_t>(star - frame - 1);
    if (bodyLength > kMaxBody) return false;
    uint16_t received = 0;
    for (int i = 1; i <= 4; ++i) {
        const int value = hexValue(star[i]);
        if (value < 0) return false;
        received = static_cast<uint16_t>((received << 4) | value);
    }
    if (received != crc16(reinterpret_cast<const uint8_t*>(frame + 1), bodyLength)) return false;
    char body[kMaxBody + 1];
    memcpy(body, frame + 1, bodyLength); body[bodyLength] = 0;
    char* fields[5];
    Packet candidate;
    if (!splitExact(body, '|', fields, 5) || strcmp(fields[0], "1") != 0 || strlen(fields[1]) != 1 ||
        !parseU32(fields[2], candidate.session) || !candidate.session ||
        !parseU32(fields[3], candidate.seq) || !candidate.seq || strlen(fields[4]) > kMaxPayload) return false;
    candidate.type = fields[1][0];
    strcpy(candidate.payload, fields[4]);
    if (!validPayload(candidate)) return false;
    output = candidate;
    return true;
}

class LineParser {
 public:
    bool feed(char c, Packet& output) {
        if (c == '@') {
            if (used_) ++errors_;
            used_ = 0; buffer_[used_++] = c;
            return false;
        }
        if (!used_) return false;
        if (c == '\n') {
            buffer_[used_] = 0;
            const bool ok = decodePacket(buffer_, output);
            used_ = 0;
            if (!ok) ++errors_;
            return ok;
        }
        if (used_ >= kMaxFrame - 2 || c == 0) {
            used_ = 0; ++errors_; return false;
        }
        buffer_[used_++] = c;
        return false;
    }
    void reset() { used_ = 0; }
    uint32_t errors() const { return errors_; }
 private:
    char buffer_[kMaxFrame] = {};
    size_t used_ = 0;
    uint32_t errors_ = 0;
};
}  // namespace sorter
#endif
