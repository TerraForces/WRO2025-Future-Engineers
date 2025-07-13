/**
 * OneWire.hpp
 * by Paul Stoffregen
 * modified by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef ONEWIRE_HPP
#define ONEWIRE_HPP

// include basic types and arduino framework functions
#include <Arduino.h>

class OneWire {
private:
    uint8_t _pin;

    // global search state
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;

public:
    OneWire(uint8_t pin);

    bool reset();
    void select(uint8_t address[8]);

    bool read_bit();
    uint8_t read_byte();

    void write_bit(bool bit);
    void write_byte(uint8_t byte);

    void reset_search();
    bool search(uint8_t* next);
};

#endif