#ifndef _BUCKETESC_H_
#define _BUCKETESC_H_

#include "I2Cdev.h"

/*
	Refer http://docs.bluerobotics.com/bluesc/#i2c-protocol for I2C registers.
*/

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM /* empty */
#define pgm_read_byte(x) (*(x))
#define pgm_read_word(x) (*(x))
#define pgm_read_float(x) (*(x))
#define PSTR(STR) STR
#endif

#define ESC_DEFAULT_ADDR			0x29
#define ESC_ALIVE					0xAB

#define ESC_ADDR_THROTTLE_HIGH		0x00
#define ESC_ADDR_THROTTLE_LOW		0x01

#define ESC_ADDR_PULSECOUNT_HIGH	0x02
#define ESC_ADDR_PULSECOUNT_LOW		0x03
#define ESC_ADDR_VOLTAGE_HIGH		0x04
#define ESC_ADDR_VOLTAGE_LOW		0x05
#define ESC_ADDR_TEMPERATURE_HIGH	0x06
#define ESC_ADDR_TEMPERATURE_LOW	0x07
#define ESC_ADDR_CURRENT_HIGH		0x08
#define ESC_ADDR_CURRENT_LOW		0x09
#define ESC_ADDR_ALIVE				0x0A

class BucketESC {
    public:
        BucketESC();
        BucketESC(uint8_t address);

        bool		testConnection();

		void		setThrottle(int16_t throttle);

		uint16_t	getPulseCount();
		uint8_t		getAlive();

    private:
        uint8_t		devAddr;
        uint8_t		buffer8;
		uint16_t	buffer16;
};

#endif /* _BUCKETESC_H_ */
