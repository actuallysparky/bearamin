#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dpt_multimode
{
static constexpr size_t kPresetMaxJsonBytes = 12000;
static constexpr size_t kPresetHexChunkBytes = 36;
static constexpr size_t kPresetMaxNameLength = 48;

inline bool IsPresetNameChar(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
           || (c >= '0' && c <= '9') || c == '_' || c == '-';
}

inline bool IsValidPresetName(const char* name)
{
    if(name == nullptr || name[0] == '\0')
        return false;
    size_t length = 0;
    for(size_t i = 0; name[i] != '\0'; i++)
    {
        if(!IsPresetNameChar(name[i]))
            return false;
        length++;
        if(length > kPresetMaxNameLength)
            return false;
    }
    return true;
}

inline uint16_t PresetCrc16(const uint8_t* data, size_t size)
{
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < size; i++)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for(uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000)
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                      : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

inline uint16_t PresetCrc16Update(uint16_t crc, const uint8_t* data, size_t size)
{
    for(size_t i = 0; i < size; i++)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for(uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000)
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                      : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

inline char PresetHexNibble(uint8_t value)
{
    value &= 0x0F;
    return value < 10 ? static_cast<char>('0' + value)
                      : static_cast<char>('A' + value - 10);
}

inline int PresetHexValue(char c)
{
    if(c >= '0' && c <= '9')
        return c - '0';
    if(c >= 'a' && c <= 'f')
        return 10 + c - 'a';
    if(c >= 'A' && c <= 'F')
        return 10 + c - 'A';
    return -1;
}

inline void PresetHexEncode(const uint8_t* data,
                            size_t         size,
                            char*          out,
                            size_t         capacity)
{
    if(capacity == 0)
        return;
    size_t index = 0;
    for(size_t i = 0; i < size && index + 2 < capacity; i++)
    {
        out[index++] = PresetHexNibble(static_cast<uint8_t>(data[i] >> 4));
        out[index++] = PresetHexNibble(data[i]);
    }
    out[index] = '\0';
}

inline bool PresetHexDecode(const char* text,
                            uint8_t*    out,
                            size_t      capacity,
                            size_t&     decoded)
{
    decoded = 0;
    if(text == nullptr)
        return false;
    for(size_t i = 0; text[i] != '\0'; i += 2)
    {
        if(text[i + 1] == '\0' || decoded >= capacity)
            return false;
        const int hi = PresetHexValue(text[i]);
        const int lo = PresetHexValue(text[i + 1]);
        if(hi < 0 || lo < 0)
            return false;
        out[decoded++] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
}

inline bool PresetLineHasPrefix(const char* line, const char* prefix)
{
    if(line == nullptr || prefix == nullptr)
        return false;
    for(size_t i = 0; prefix[i] != '\0'; i++)
    {
        if(line[i] != prefix[i])
            return false;
    }
    return true;
}

} // namespace dpt_multimode
