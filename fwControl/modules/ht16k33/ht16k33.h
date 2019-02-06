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
#ifndef _HT16K33_H_
#define _HT16K33_H_

typedef struct {
	byte i2c_addr;
	uint16_t displaybuffer[8];
} ht16k33;

void ht16k33_init(ht16k33 *dev, uint8_t i2c_address /*= 0x70*/);
void ht16k33_setBrightness(ht16k33 *dev, uint8_t b);
void ht16k33_blinkRate(ht16k33 *dev, uint8_t b);
void ht16k33_writeDisplay(ht16k33 *dev);
void ht16k33_clear(ht16k33 *dev);
void ht16k33_writeDigitRaw(ht16k33 *dev, uint8_t digit, uint8_t bitmask);
/*
    digit 2:
    0x02 - center colon (both dots)
    0x04 - left colon - lower dot
    0x08 - left colon - upper dot
    0x10 - decimal point
*/

void ht16k33_writeDigitNum(ht16k33 *dev, uint8_t x, uint8_t num, char dot /*= false*/);
void ht16k33_printNumber(ht16k33 *dev, int n, uint8_t base);

#endif	/* _HT16K33_H_ */
