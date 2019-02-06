/*********************************************************************
 *
 *                HT16K33 16x8 LED controller/driver for Fraise
 *
 *********************************************************************
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Antoine Rousseau  feb 2019     From Adafruit Arduino library 
 *                (https://github.com/adafruit/Adafruit_LED_Backpack)
 ********************************************************************/

/*
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA  02110-1301, USA.
*/

#include <fruit.h>
#include <ht16k33.h>
#include <i2c_master.h>

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0


static const uint8_t numbertable[] = {
	0x3F, /* 0 */
	0x06, /* 1 */
	0x5B, /* 2 */
	0x4F, /* 3 */
	0x66, /* 4 */
	0x6D, /* 5 */
	0x7D, /* 6 */
	0x07, /* 7 */
	0x7F, /* 8 */
	0x6F, /* 9 */
	0x77, /* a */
	0x7C, /* b */
	0x39, /* C */
	0x5E, /* d */
	0x79, /* E */
	0x71, /* F */
};

void ht16k33_setBrightness(ht16k33 *dev, uint8_t b) {
	if (b > 15) b = 15;
	i2cm_begin(dev->i2c_addr, 0);
	i2cm_writechar(HT16K33_CMD_BRIGHTNESS | b);
	i2cm_stop();
}

void ht16k33_blinkRate(ht16k33 *dev, uint8_t b) {
	if (b > 3) b = 0; // turn off if not sure
	i2cm_begin(dev->i2c_addr, 0);
	i2cm_writechar(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
	i2cm_stop();
}

void ht16k33_init(ht16k33 *dev, uint8_t i2c_address) {
	dev->i2c_addr = i2c_address;

	i2cm_begin(dev->i2c_addr, 0);
	i2cm_writechar(0x21);  // turn on oscillator
	i2cm_stop();
	ht16k33_blinkRate(dev, HT16K33_BLINK_OFF);
	ht16k33_setBrightness(dev, 15); // max brightness
}

void ht16k33_writeDisplay(ht16k33 *dev) {
	i2cm_begin(dev->i2c_addr, 0);
	i2cm_writechar(0); // start at address $00

	for (uint8_t i=0; i<8; i++) {
		i2cm_writechar(dev->displaybuffer[i] & 0xFF);
		i2cm_writechar(dev->displaybuffer[i] >> 8);
	}
	i2cm_stop();
}

void ht16k33_clear(ht16k33 *dev) {
	for (uint8_t i=0; i<8; i++) {
		dev->displaybuffer[i] = 0;
	}
}

void ht16k33_writeDigitRaw(ht16k33 *dev, uint8_t digit, uint8_t bitmask)
{
	if (digit > 4) return;
	dev->displaybuffer[digit] = bitmask;
}

void ht16k33_writeDigitNum(ht16k33 *dev, uint8_t digit, uint8_t num, char dot /*= false*/) {
	if (digit > 4) return;
	dev->displaybuffer[digit] = numbertable[num] | (dot << 7);
}

void ht16k33_printNumber(ht16k33 *dev, int n, uint8_t base) {
	int8_t displayPos = 4;
	
	if(n > 9999) n = 9999;
	
	if (n)  //if n is not 0
	{
		for(uint8_t i = 0; n ; ++i) {
			ht16k33_writeDigitNum(dev, displayPos--, n % base, 0);
			if(displayPos == 2) ht16k33_writeDigitRaw(dev, displayPos--, 0x00);
			n /= base;
		}
	}
	else {
		ht16k33_writeDigitNum(dev, displayPos--, 0, 0);
	}
	
	// clear remaining display positions
	while(displayPos >= 0) ht16k33_writeDigitRaw(dev, displayPos--, 0x00);
}
