/**
 * OneWire.cpp
 * by Paul Stoffregen
 * modified by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

#include "OneWire.hpp"

#include <driver/rtc_io.h>

static inline __attribute__((always_inline))
uint32_t directRead(uint32_t pin) {
    if(pin < 32) return (GPIO.in >> pin) & 0x1;
    else if(pin < 46) return (GPIO.in1.val >> (pin - 32)) & 0x1;
    return 0;
}

static inline __attribute__((always_inline))
void directWriteLow(uint32_t pin) {
    if(pin < 32) GPIO.out_w1tc = ((uint32_t)1 << pin);
    else if (pin < 46) GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32));
}

static inline __attribute__((always_inline))
void directWriteHigh(uint32_t pin) {
    if(pin < 32) GPIO.out_w1ts = ((uint32_t)1 << pin);
    else if(pin < 46) GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32));
}

static inline __attribute__((always_inline))
void directModeInput(uint32_t pin) {
    if(digitalPinIsValid(pin)) {
        if(pin < 32) GPIO.enable_w1tc = ((uint32_t)1 << pin);
        else GPIO.enable1_w1tc.val = ((uint32_t)1 << (pin - 32));
    }
}

static inline __attribute__((always_inline))
void directModeOutput(uint32_t pin) {
    if(digitalPinIsValid(pin) && (pin <= 33)) {
        if (pin < 32) GPIO.enable_w1ts = ((uint32_t)1 << pin);
        else GPIO.enable1_w1ts.val = ((uint32_t)1 << (pin - 32));
    }
}

#define _noInterrupts() {portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux)
#define _interrupts() portEXIT_CRITICAL(&mux);}

OneWire::OneWire(uint8_t pin) : _pin(pin) {
    pinMode(_pin, INPUT);
	reset_search();
}

bool IRAM_ATTR OneWire::reset() {
	bool result;
	uint8_t retries = 125;
	_noInterrupts();
	directModeInput(_pin);
	_interrupts();
	do {
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while ( !directRead(_pin));
	_noInterrupts();
	directWriteLow(_pin);
	directModeOutput(_pin);
	_interrupts();
	delayMicroseconds(480);
	_noInterrupts();
	directModeInput(_pin);
	delayMicroseconds(70);
	result = !directRead(_pin);
	_interrupts();
	delayMicroseconds(410);
	return result;
}

bool IRAM_ATTR OneWire::read_bit(void) {
	bool result;
    _noInterrupts();
	directModeOutput(_pin);
	directWriteLow(_pin);
	delayMicroseconds(3);
	directModeInput(_pin);
	delayMicroseconds(10);
	result = directRead(_pin);
	_interrupts();
	delayMicroseconds(53);
	return result;
}

uint8_t OneWire::read_byte() {
    uint8_t result = 0;
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1) if (read_bit()) result |= bitMask;
    return result;
}

void IRAM_ATTR OneWire::write_bit(bool bit) {
    _noInterrupts();
    directWriteLow(_pin);
    directModeOutput(_pin);
    delayMicroseconds(bit ? 10 : 65);
    directWriteHigh(_pin);
    _interrupts();
    delayMicroseconds(bit ? 55 : 5);
}

void OneWire::write_byte(uint8_t byte) {
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1) write_bit((bitMask & byte) ? 1 : 0);
	_noInterrupts();
	directModeInput(_pin);
	directWriteLow(_pin);
	_interrupts();
}

void OneWire::select(uint8_t rom[8]) {
    write_byte(0x55);
    for(uint8_t i = 0; i < 8; i++) write_byte(rom[i]);
}







//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void OneWire::reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
bool OneWire::search(uint8_t *newAddr) {
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number;
    bool    search_result;
    uint8_t id_bit, cmp_id_bit;
    unsigned char rom_byte_mask, search_direction;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = false;

    // if the last call was not the last one
    if (!LastDeviceFlag) {
        // 1-Wire reset
        if (!reset()) {
            // reset the search
            LastDiscrepancy = 0;
            LastDeviceFlag = false;
            LastFamilyDiscrepancy = 0;
            return false;
        }

        // issue the search command
        write_byte(0xF0);

        // loop to do the search
        do
        {
            // read a bit and its complement
            id_bit = read_bit();
            cmp_id_bit = read_bit();

            // check for no devices on 1-wire
            if ((id_bit == 1) && (cmp_id_bit == 1)) {
                break;
            } else {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit) {
                search_direction = id_bit;  // bit write value for search
                } else {
                // if this discrepancy if before the Last Discrepancy
                // on a previous next then pick the same as last time
                if (id_bit_number < LastDiscrepancy) {
                    search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                } else {
                    // if equal to last pick 1, if not then pick 0
                    search_direction = (id_bit_number == LastDiscrepancy);
                }
                // if 0 was picked then record its position in LastZero
                if (search_direction == 0) {
                    last_zero = id_bit_number;

                    // check for Last discrepancy in family
                    if (last_zero < 9)
                        LastFamilyDiscrepancy = last_zero;
                }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                ROM_NO[rom_byte_number] |= rom_byte_mask;
                else
                ROM_NO[rom_byte_number] &= ~rom_byte_mask;

                // serial number search direction write bit
                write_bit(search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0) {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)) {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
  }