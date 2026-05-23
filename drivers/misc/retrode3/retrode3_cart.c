// SPDX-License-Identifier: GPL-2.0

/*
 * driver for Retrode 3 game cartridge driver
 *
 * based on ideas and fragments from
 *  drivers/char/mem.c
 *  drivers/gnss/core.c
 *  arch/arm/common/locomo.c
 *
 *  Copyright (C) 2022-26, H. Nikolaus Schaller
 *
 * FIXME: make this an independent driver module
 */

#include "retrode3_bus.h"
#include <linux/delay.h>

// #define RETRODE3_MAJOR FLOPPY_MAJOR
#define RETRODE3_MINORS	16
static DEFINE_IDA(retrode3_minors);
static dev_t retrode3_first;
static struct class *retrode3_class;

/*
 * game cart driver
 */

/*
 * This function reads the slot. The ppos points directly to the
 * memory location. But also includes a special access mode in the upper bits.
 */

/* special mode constants - should be exported as ABI to user-space */

#define MODE_SIMPLE_BUS	0	// default read/write with just CE, RD/WR0/WR8
#define MD_ROM		MODE_SIMPLE_BUS
#define MD_P10		1	// 10 toggle pulses on CLK
#define MD_P1		2	// 1 toggle pulses on CLK
#define MD_TIME		3	// read/write with TIME impulse
#define MD_FLASH	unused 0x04 unused
#define MD_ENSRAM	5	// TIME impulse without WE (despite write?)
#define MD_EEPMODE	6
#define MD_FRAM		7	// read with WE0 for special SONIC3 FRAM

#define TIME_HIGH (gpiod_set_value(slot->bus->time, 0))			// TIME = low
#define TIME_LOW (gpiod_set_value(slot->bus->time, 1))			// TIME = high

#define SNES_REGULAR	MODE_SIMPLE_BUS
#define SNES_HIROM	9

// define SNES specific gpios:	gpio-? =

// for NES address mapping: https://www.nesdev.org/wiki/CPU_memory_map

// can we simplify this? E.g. make the distinction between ROM (PRG) and RAM depend on address?
// so that /dev/slot mimicks the CPU_memory_map

#define NES_PRG		10	// CPU d0..d7	ROM $8000-$ffff
#define NES_CHR		11	// PPU d8..d15	ROM
#define NES_CHR_M2	12	// PPU d8..d15	?
#define NES_MMC5_SRAM	13	// CPU d0..d7	special RAM? $0000-$07ff
#define NES_REG		14	// PPU d8..d15	PPU registers? $2000-$2007
#define NES_RAM		15	// PPU d8..d15	internal RAM? $0000-$07ff
#define NES_WRAM	16	// PPU d8..d15?	Cartridge RAM  $6000-$7fff

/* should depend on .compatible since it controls hardware peculiarities */

#define IS_MD()	(mode >= MD_ROM && mode <= MD_EEPMODE)
#define IS_SNES()	(mode == SNES_REGULAR || mode == SNES_HIROM)
#define IS_NES()	(mode >= NES_PRG && mode <= NES_WRAM)

/* Retrode gpios to NES special wiring
 d0..d7  <-> CPU (PRG = Program ROM/RAM - CPU = Central Processing Unit 6502)
 d8..d15 <-> PPU (CHR = Character ROM/RAM - PPU = Picture Processing Unit)
 a0..a14  -> A0..A14
 ce-nes & !a15 -> PRG/CE
 a16..a23 -> ignored
 romsel   -> A15
 ppu/a13  -> inverted A16
 unused   -> A17..A23
 we0      -> CPU_R/W
 rd / we8 -> PPU_/R/PPU_/W
 ce_nes   -> CPU_M2 (PHI2)
 for an example: https://www.nesdev.org/wiki/UxROM
*/

/* these macros map the OSCR software variables and functions to Retrode 3 kernel accessors */

/* 6502 bus timing
 * see Figure 6-3 of https://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf
 * or even better: https://www.princeton.edu/~mae412/HANDOUTS/timing.png
 *
 * Addr and R/W may change after PHI2_LOW
 * data can be read on PHI2 edge from L to H
 * write data is available some time after the rising edge
 *
 * this means:
 * we start assuming PHI2_LOW
 * we set up address and WE
 * we sample the data on read
 * then we issue PHI2_HI - this may remove ROMSEL and write badly?
 * if we write, we drive the bus
 * we do a PHI2_LO
 * we leave the bus driven or not as is until the next activity
 */

#define _delay_us(us) usleep_range(us, us)

#define PORTK_PRG(data) (set_half(slot->bus, data, 1))		// CPU/PRG D0..D7 := data (and output)
#define PORTK_CHR(data) (set_half(slot->bus, data, 0))		// PPU/CHR D8..D15 := data (and output)
#define PINK_PRG read_half(slot->bus, 1)
#define PINK_CHR read_half(slot->bus, 0)

// ROMSEL = PRG_/CE
static uint16_t romsel;
#define ROMSEL_HI { romsel = BIT(15); set_address(slot->bus->current_addr); }	// A15 = 1 makes ROMSEL 1
#define ROMSEL_LOW { romsel = 0; set_address(slot->bus->current_addr); }	// A15 = 0 makes ROMSEL 0
#define set_romsel(addr) { if ((addr) & BIT(15)) { ROMSEL_LOW ; } else { ROMSEL_HI; } }

// PHI2 = CPU_M2
/* on 6502 there is a PHI2_LOW at the start of every cycle and a PHI2_HI at the end and data sampling typically occurs on or after rising edge */
#define PHI2_HI (gpiod_set_value(slot->bus->phi2, 0))			// CPU_M2 = low
#define PHI2_LOW (gpiod_set_value(slot->bus->phi2, 1))			// CPU_M2 = high
// PRG = CPU = D0..D7 / CPU_R/W = WE0
#define PRG_READ (gpiod_set_value(slot->bus->we->desc[0], 0))	// /WE0 = inactive
#define PRG_WRITE (gpiod_set_value(slot->bus->we->desc[0], 1))	// /WE0 = active
// CHR = PPU = D8..D15 / PPU_R/W = WE8
// for OE/RD this is the pin level although we have "active low" in the device tree - reason unknown
#define CHR_READ_HI (gpiod_set_value(slot->bus->oe, 1), gpiod_set_value(slot->bus->we->desc[1], 0))	// RD = inactive, WE8 = inactive
#define CHR_READ_LOW (gpiod_set_value(slot->bus->oe, 0), gpiod_set_value(slot->bus->we->desc[1], 0))	// RD = active, WE8 = inactive
#define CHR_WRITE_HI (gpiod_set_value(slot->bus->oe, 1), gpiod_set_value(slot->bus->we->desc[1], 0))	// RD = inactive, WE8 = inactive
#define CHR_WRITE_LOW (gpiod_set_value(slot->bus->oe, 1), gpiod_set_value(slot->bus->we->desc[1], 1))	// RD = inactive, WE8 = active

#define MODE_READ (end_drive_word(slot->bus))			// D0..D15 = input
#define MODE_WRITE						// done automatically by set_half

static inline int _set_address(int mode, struct retrode3_slot *slot, uint32_t addr)
{
	if (IS_NES()) {
		addr = addr & 0x7fff;
		addr |= romsel;
		if (!(addr & BIT(13)))
			addr |= BIT(16);
	}
// if ((addr &0xff) == 0) printk("%s: %08x\n", __func__, addr);
	return set_address(slot->bus, addr);
}

#define set_address(addr) if ((err = _set_address(mode, slot, (addr))) < 0) goto failed

/* this section needs cleanup!
 * it is a partial and modified and adapted copy of:
 *    https://github.com/sanni/cartreader/blob/0b8ac2ec14675ae55952ddf0b4590ba8d2899664/Cart_Reader/NES.ino#L1012
 */

static ssize_t retrode3_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct retrode3_slot *slot = file->private_data;
	uint32_t addr = *ppos & 0xffffff;	/* max address is 24 bit */
	uint32_t mode = *ppos >> 24;		/* access mode control */
	int err;

#ifndef CONFIG_RETRODE3_BUFFER
	ssize_t read, sz;
#else
	ssize_t read;
	unsigned long remaining;
	unsigned char buffer[1024];
	unsigned int fill = 0;
#endif

// dev_info(&slot->dev, "%s: %08llx\n", __func__, *ppos);

// if (mode) dev_warn(&slot->dev, "%s: mode = %d\n", __func__, mode);

	read = 0;

	select_slot(slot->bus, slot);

	if (addr + count >= EOF) {
		if (addr >= EOF)
			count = 0;	// read nothing
		else
			count = EOF - addr;	// limit to EOF
	}

if ((addr &0xff) == 0) dev_info(&slot->dev, "%s: read mode=%02x %08x\n", __func__, mode, addr);
	while (count > 0) {
#ifndef CONFIG_RETRODE3_BUFFER
		unsigned long remaining;
#endif

		if (mode == MODE_SIMPLE_BUS && slot->bus_width == 16 && count >= 2 && ((addr & 1) == 0)) {
			uint16_t word;

			err = _set_address(mode, slot, addr);	// 24 bit address and A0 determines lower/upper byte
			if(err < 0)
				goto failed;

			err = read_word(slot->bus);
			if(err < 0)
				goto failed;

#ifndef CONFIG_RETRODE3_BUFFER
			sz = 2;	// handle word reads
			word = swab16(err);	// use htons()?

			remaining = copy_to_user(buf, (char *) &word, sz);
#else

			word = err;

			if (fill == sizeof(buffer)) { // flush buffer
				remaining = copy_to_user(buf+read, buffer, fill);
				if (remaining)
		// FIXME: err = something?
					goto failed;
				fill = 0;
			}
			*((uint16_t *) &buffer[fill]) = swab16(word);	// use htons()?
			fill += 2;
			*ppos += 2;
			addr += 2;
			count -= 2;
			read += 2;
#endif
		} else {
			uint8_t byte;

			if (slot->bus_width == 16) { // megadrive
				err = _set_address(mode, slot, addr);	// 24 bit address and A0 determines lower/upper byte
				if(err < 0)
					goto failed;
				switch (mode) {
					case MD_TIME:
						TIME_LOW;
						byte = err = read_byte(slot->bus);	// read half based on a0
						TIME_HIGH;
						break;
					// other modes
					case MD_FRAM:	// special SONIC3 trick to read without !AS impulse on B18
						TIME_LOW;	// activate OE (and deactivate WE)
						PRG_WRITE;	// reset FF to control CE
						byte = err = read_byte(slot->bus);	// read half based on a0
						PRG_READ;	// release FF
						TIME_HIGH;
						break;
					default:
						byte = err = read_byte(slot->bus);	// read half based on a0
				}
			}
			else { // 8 bit bus SNES or NES
				switch (mode) {
					case MODE_SIMPLE_BUS:
						err = _set_address(mode, slot, addr);	// 24 bit address and A0 determines lower/upper byte
						if(err < 0)
							goto failed;
						byte = err = read_half(slot->bus, 1);	// D0..D7
if ((addr &0xff) == 0) dev_info(&slot->dev, "%s: read SIMPLE %08x %02x\n", __func__, addr, byte);
						break;
					case NES_PRG:
/*
static unsigned char read_prg_byte(unsigned int address) {
  MODE_READ;
  PRG_READ;
  ROMSEL_HI;
  set_address(address);
  PHI2_HI;
  set_romsel(address);
  _delay_us(1);
  return PINK;
}
*/

						PHI2_LOW;		// CPU_M2 = L
						MODE_READ;
						PRG_READ;
						/* ROMSEL_HI; */ set_romsel(addr);
						set_address(addr);
						PHI2_HI;
						set_romsel(addr);
						_delay_us(1);
						byte = err = PINK_PRG;	// NES CPU bus = D0..D7
if ((addr &0xff) == 0) dev_info(&slot->dev, "%s: read NES CPU/PRG/SRAM %08x %02x\n", __func__, addr, byte);
						break;
					case NES_CHR:		// OSCR does not clock PHI2...
					case NES_CHR_M2:
/*
static unsigned char read_chr_byte(unsigned int address) {
  MODE_READ;
  PHI2_HI;
  ROMSEL_HI;
  set_address(address);
  CHR_READ_LOW;
  _delay_us(1);
  uint8_t result = PINK;
  CHR_READ_HI;
  return result;
}
*/
						PHI2_LOW;		// CPU_M2 = L
						MODE_READ;
						PHI2_HI;
						ROMSEL_HI;
						set_address(addr);
						CHR_READ_LOW;
						_delay_us(1);
						byte = PINK_CHR;	// NES PPU bus = D8..D15
						CHR_READ_HI;
if ((addr &0xff) == 0) dev_info(&slot->dev, "%s: read NES PPU/CHR %08x %02x\n", __func__, addr, byte);
						break;
					case NES_MMC5_SRAM:
					case NES_REG:		// hier evtl. A15 = 1?
					case NES_RAM:
					case NES_WRAM:
					default:
						dev_err(&slot->dev, "%s: mode (%d) not implemented\n", __func__, mode);
						err = -EIO;
				}
			}
			if(err < 0)
				goto failed;

#ifndef CONFIG_RETRODE3_BUFFER
			sz = 1;	// handle byte read

			remaining = copy_to_user(buf, (char *) &byte, sz);
#else

			if (fill == sizeof(buffer)) { // flush buffer
				remaining = copy_to_user(buf+read, buffer, fill);
				if (remaining)
					goto failed;
				fill = 0;
			}
			*((uint8_t *) &buffer[fill]) = byte;
			fill += 1;
			*ppos += 1;
			addr += 1;
			count -= 1;
			read += 1;
#endif
		}

#ifndef CONFIG_RETRODE3_BUFFER
                if (remaining)
                        goto failed;

		*ppos += sz;
		addr += sz;
		buf += sz;
		count -= sz;
		read += sz;
#endif
	}

#ifdef CONFIG_RETRODE3_BUFFER
#warning buffered
	if (fill > 0) { // flush remaining bytes
		remaining = copy_to_user(buf, buffer, fill);
		if (remaining)
			goto failed;
	}
#endif

	select_slot(slot->bus, NULL);

	return read;

failed:
	select_slot(slot->bus, NULL);

	return err;
}

static ssize_t retrode3_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct retrode3_slot *slot = file->private_data;
	uint32_t addr = *ppos & 0xffffff;	/* max address is 24 bit */
	uint32_t mode = *ppos >> 24;		/* access mode control */
	ssize_t written, sz;
	unsigned long copied;
	int err;

//	dev_info(&slot->dev, "%s\n", __func__);

// if (mode) dev_warn(&slot->dev, "%s: mode = %d\n", __func__, mode);

	written = 0;

	select_slot(slot->bus, slot);

	if (addr + count >= EOF) {
		if (addr >= EOF)
			count = 0;	// write nothing
		else
			count = EOF - addr;	// limit to EOF
	}

if ((addr &0xff) == 0) dev_info(&slot->dev, "%s: write mode=%02x %08x\n", __func__, mode, addr);
	while (count > 0) {
		// int allowed;
		uint8_t byte;

// FIXME: loop over bytes and write them to consecutive addresses
// handle words for 16 bit bus and faster write

		sz = sizeof(byte);

		copied = copy_from_user(&byte, buf, sz);

		switch (mode) {
			case MODE_SIMPLE_BUS:
				dev_info(&slot->dev, "%s: write BUS %08x %02x\n", __func__, addr, byte);
				err = _set_address(mode, slot, addr);	// 24 bit address and A0 determines lower/upper byte
				if(err < 0)
					goto failed;
				write_byte(slot->bus, byte);	// a0 should be 1 for D0..D7 and WE0
// CHECKME: do we have to play the WE0/WE8 differently?
				break;
			case MD_TIME:	// write with active TIME impulse
				dev_info(&slot->dev, "%s: write MD_TIME+WE %08x %02x\n", __func__, addr, byte);
				err = _set_address(mode, slot, addr);	// 24 bit address and A0 determines lower/upper byte
				if(err < 0)
					goto failed;
				TIME_LOW;
				write_byte(slot->bus, byte);	// a0 should be 1 for D0..D7 and WE0
				TIME_HIGH;
				break;
				;;
			case MD_ENSRAM:	// write with active TIME impulse but no WE
				dev_info(&slot->dev, "%s: write MD_TIME %08x %02x\n", __func__, addr, byte);
#if 0	// ignore address
				err = _set_address(mode, slot, addr);	// 24 bit address and A0 determines lower/upper byte
				if(err < 0)
					goto failed;
#endif
				set_half(slot->bus, byte, slot->bus->current_addr & 1);	// a0 should be 1 for D0..D7 but no WE0
				TIME_LOW;
				_delay_us(1);
				TIME_HIGH;
				end_drive_word(slot->bus);
				break;
				;;
			// other MD modes
			case NES_PRG:
			case NES_MMC5_SRAM:
				dev_info(&slot->dev, "%s: write NES CPU/PRG/SRAM %08x %02x\n", __func__, addr, byte);
				// err = write_prg_byte(addr, byte);

/*
static void write_prg_byte(unsigned int address, uint8_t data) {
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PRG_WRITE;
  PORTK = data;

  set_address(address);  // PHI2 low, ROMSEL always HIGH
  //  _delay_us(1);
  PHI2_HI;
  //_delay_us(10);
  set_romsel(address);  // ROMSEL is low if need, PHI2 high
  _delay_us(1);         // WRITING
  //_delay_ms(1); // WRITING
  // PHI2 low, ROMSEL high
  PHI2_LOW;
  _delay_us(1);
  ROMSEL_HI;
  // Back to read mode
  //  _delay_us(1);
  PRG_READ;
  MODE_READ;
  set_address(0);
  // Set phi2 to high state to keep cartridge unreseted
  //  _delay_us(1);
  PHI2_HI;
  //  _delay_us(1);
}
*/

				PHI2_LOW;		// CPU_M2 = L
				ROMSEL_HI;		// disable ROM
				MODE_WRITE;		// switch data to output
				PRG_WRITE;		// activate CPU_R/W (WE0)
				PORTK_PRG(byte);	// set data bus
				set_address(addr);	// set address
				//  _delay_us(1);
				PHI2_HI;		// this edge should clock data into mapper
				//_delay_us(10);
				set_romsel(addr);	// ROMSEL is low if need, PHI2 high
				_delay_us(1);		// WRITING
				//_delay_ms(1); // WRITING
				// PHI2 low, ROMSEL high
				PHI2_LOW;		// start new (read) cycle
				_delay_us(1);
				ROMSEL_HI;		// disable ROM
				// Back to read mode
				//  _delay_us(1);
				PRG_READ;
				MODE_READ;
				set_address(0);
				// Set phi2 to high state to keep cartridge unreseted
				//  _delay_us(1);
				PHI2_HI;		// this clocks a dummy read to prepare the mapper
				//  _delay_us(1);
				break;
			case NES_CHR:
			case NES_CHR_M2:	// FIXME: what is the difference? Clocking M2 = CE-NES
/*
static void write_chr_byte(unsigned int address, uint8_t data) {
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PORTK = data;
  set_address(address);  // PHI2 low, ROMSEL always HIGH
  _delay_us(1);

  CHR_WRITE_LOW;

  _delay_us(1);  // WRITING

  CHR_WRITE_HI;

  _delay_us(1);

  MODE_READ;
  set_address(0);
  PHI2_HI;

  //_delay_us(1);
}

*/
				dev_info(&slot->dev, "%s: write NES PPU/CHR %08x %02x\n", __func__, addr, byte);
				PHI2_LOW;
				ROMSEL_HI;
				MODE_WRITE;
				PORTK_CHR(byte);
 				set_address(addr);
				_delay_us(1);

				CHR_WRITE_LOW;

				_delay_us(1);  // WRITING

				CHR_WRITE_HI;

				_delay_us(1);

				MODE_READ;
				set_address(0);
				PHI2_HI;

				//_delay_us(1);
				break;
			case NES_REG:
/*
static void write_reg_byte(unsigned int address, uint8_t data) {  // FIX FOR MMC1 RAM CORRUPTION
  PHI2_LOW;
  ROMSEL_HI;  // A15 HI = E000
  MODE_WRITE;
  PRG_WRITE;  // CPU R/W LO
  PORTK = data;

  set_address(address);  // PHI2 low, ROMSEL always HIGH
  // DIRECT PIN TO PREVENT RAM CORRUPTION
  // DIFFERENCE BETWEEN M2 LO AND ROMSEL HI MUST BE AROUND 33ns
  // IF TIME IS GREATER THAN 33ns THEN WRITES TO 0xE000/0xF000 WILL CORRUPT RAM AT 0x6000/0x7000
  PORTF = 0b01111101;  // ROMSEL LO/M2 HI
  PORTF = 0b01111110;  // ROMSEL HI/M2 LO
  _delay_us(1);
  // Back to read mode
  PRG_READ;
  MODE_READ;
  set_address(0);
  // Set phi2 to high state to keep cartridge unreseted
  PHI2_HI;
}
*/
				PHI2_LOW;
				ROMSEL_HI;  // A15 HI = E000
				MODE_WRITE;
				PRG_WRITE;  // CPU R/W LO
				PORTK_PRG(byte);
				set_address(addr);  // PHI2 low, ROMSEL always HIGH
				// DIRECT PIN TO PREVENT RAM CORRUPTION
				// DIFFERENCE BETWEEN M2 LO AND ROMSEL HI MUST BE AROUND 33ns
				// IF TIME IS GREATER THAN 33ns THEN WRITES TO 0xE000/0xF000 WILL CORRUPT RAM AT 0x6000/0x7000
#ifdef NIMP
	// we can't implement this through gpiod() in Linux kernel
	return -EINVAL;
	// rather, we must ioremap the gpio controller registers
	// also note: on x1600 ROMSEL is controlled by A15/CE-NES and M2 through GPC26 on different gpio controllers
	// and try to do this with interrupts locked
				PORTF = 0b01111101;  // ROMSEL LO/M2 HI
				PORTF = 0b01111110;  // ROMSEL HI/M2 LO
#endif
				_delay_us(1);
				// Back to read mode
				PRG_READ;
				MODE_READ;
				set_address(0);
				// Set phi2 to high state to keep cartridge unreseted
				PHI2_HI;
				break;
			case NES_RAM:
/*
static void write_ram_byte(unsigned int address, uint8_t data) {  // Mapper 19 (Namco 106/163) WRITE RAM SAFE ($E000-$FFFF)
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PRG_WRITE;
  PORTK = data;

  set_address(address);  // PHI2 low, ROMSEL always HIGH
  PHI2_HI;
  ROMSEL_LOW;    // SET /ROMSEL LOW OTHERWISE CORRUPTS RAM
  _delay_us(1);  // WRITING
  // PHI2 low, ROMSEL high
  PHI2_LOW;
  _delay_us(1);
  ROMSEL_HI;
  // Back to read mode
  PRG_READ;
  MODE_READ;
  set_address(0);
  // Set phi2 to high state to keep cartridge unreseted
  PHI2_HI;
}
*/
				PHI2_LOW;
				ROMSEL_HI;
				MODE_WRITE;
				PRG_WRITE;
				PORTK_PRG(byte);
				set_address(addr);  // PHI2 low, ROMSEL always HIGH
				PHI2_HI;
				ROMSEL_LOW;    // SET /ROMSEL LOW OTHERWISE CORRUPTS RAM
				_delay_us(1);  // WRITING
				// PHI2 low, ROMSEL high
				PHI2_LOW;
				_delay_us(1);
				ROMSEL_HI;
				// Back to read mode
				PRG_READ;
				MODE_READ;
				set_address(0);
				// Set phi2 to high state to keep cartridge unreseted
				PHI2_HI;
				break;
			case NES_WRAM:
/*
static void write_wram_byte(unsigned int address, uint8_t data) {  // Mapper 5 (MMC5) RAM
  PHI2_LOW;
  ROMSEL_HI;
  set_address(address);
  PORTK = data;

  _delay_us(1);
  MODE_WRITE;
  PRG_WRITE;
  PHI2_HI;
  _delay_us(1);  // WRITING
  PHI2_LOW;
  ROMSEL_HI;
  // Back to read mode
  PRG_READ;
  MODE_READ;
  set_address(0);
  // Set phi2 to high state to keep cartridge unreseted
  PHI2_HI;
}
*/

				PHI2_LOW;
				ROMSEL_HI;
				set_address(addr);
				PORTK_PRG(byte);
				_delay_us(1);
				MODE_WRITE;
				PRG_WRITE;
				PHI2_HI;
				_delay_us(1);  // WRITING
				PHI2_LOW;
				ROMSEL_HI;
				// Back to read mode
				PRG_READ;
				MODE_READ;
 				set_address(0);
				// Set phi2 to high state to keep cartridge unreseted
				PHI2_HI;
				break;

/*

// NOTE: there are even more modes in NES.ino
// write_reg_m59()
// write_prg_pulsem2()
// read_prg_pulsem2()
// read_chr_pulsem2()

*/

			default:
				dev_err(&slot->dev, "%s: mode (%d) not implemented\n", __func__, mode);
				err = -EIO;
				goto failed;
		}

		if (copied) {
			written += sz - copied;
			if (written)
				break;
			err = -EFAULT;
			goto failed;
		}

		*ppos += sz;
		addr += sz;
		buf += sz;
		count -= sz;
		written += sz;
	}

	select_slot(slot->bus, NULL);

	return written;
failed:
	select_slot(slot->bus, NULL);

	return err;
}

/*
 * The memory devices use the full 32/64 bits of the offset, and so we cannot
 * check against negative addresses: they are ok. The return value is weird,
 * though, in that case (0).
 *
 * also note that seeking relative to the "end of file" isn't supported:
 * it has no meaning, so it returns -EINVAL.
 */
static loff_t retrode3_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;

	inode_lock(file_inode(file));
	switch (orig) {
	case SEEK_CUR:
		offset += file->f_pos;
		fallthrough;
	case SEEK_SET:
		/* to avoid userland mistaking f_pos=-9 as -EBADF=-9 */
		if ((unsigned long long)offset >= -MAX_ERRNO) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = offset;
		ret = file->f_pos;
		force_successful_syscall_return();
		break;
	default:
		ret = -EINVAL;
	}
	inode_unlock(file_inode(file));
	return ret;
}

static int retrode3_open(struct inode *inode, struct file *file)
{
	struct retrode3_slot *slot;
	int ret = 0;

	slot = container_of(inode->i_cdev, struct retrode3_slot, cdev);

        dev_dbg(&slot->dev, "%s\n", __func__);

	ret = gpiod_get_value(slot->cd);	// check cart detect

	if (!ret)
		return -ENODEV;	// no cart is inserted

	get_device(&slot->dev);	// increment the reference count for the device

	ret = generic_file_open(inode, file);
	if (ret < 0)
		return ret;

	file->private_data = slot;

	return ret;
}

static int retrode3_release(struct inode *inode, struct file *file)
{
	struct retrode3_slot *slot = file->private_data;

        dev_dbg(&slot->dev, "%s\n", __func__);

	put_device(&slot->dev);

	return 0;
}

static const struct file_operations slot_fops = {
	.llseek		= retrode3_lseek,
	.read		= retrode3_read,
	.write		= retrode3_write,
	.open		= retrode3_open,
	.release	= retrode3_release,
};

// FIXME: duplicate to retrode3_release??

static void retrode3_slot_release(struct device *dev)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);

        dev_dbg(&slot->dev, "%s\n", __func__);

	ida_free(&retrode3_minors, slot->id);
	// FIXME: what else to release
	kfree(slot);
}

int retrode3_probe_slot(struct retrode3_slot *slot, struct device_node *child)
{
	struct device *	dev = &slot->dev;
	int id;
	int ret;
	const char *name = "unknown";

	id = ida_alloc_max(&retrode3_minors, RETRODE3_MINORS-1, GFP_KERNEL);
	if (id < 0)
		return id;

	slot->id = id;

	device_initialize(dev);
	dev->devt = retrode3_first + id;
	dev->class = retrode3_class;
	dev->release = retrode3_slot_release;
	dev_set_drvdata(dev, slot);
	of_property_read_string(child, "name", &name);
	dev_set_name(dev, "slot-%s", name);
	dev->of_node = child;

	slot->ce = devm_gpiod_get(dev, "ce", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
	gpiod_set_value(slot->ce, 0);	// turn inactive
	slot->cd = devm_gpiod_get(dev, "cd", GPIOD_IN);
	slot->led = devm_gpiod_get(dev, "status", GPIOD_OUT_HIGH);
	if (!IS_ERR_OR_NULL(slot->led))
		gpiod_set_value(slot->led, 0);	// turn inactive
	slot->power = devm_gpiod_get(dev, "power", GPIOD_IN);
	of_property_read_u32_index(child, "address-width", 0, &slot->addr_width);
	of_property_read_u32_index(child, "bus-width", 0, &slot->bus_width);

	// access reference "status-led" (if available)

	cdev_init(&slot->cdev, &slot_fops);
	slot->cdev.owner = THIS_MODULE;

// FIXME: should be some devm_cdev_device_add
	ret = cdev_device_add(&slot->cdev, &slot->dev);
	if (ret)
		return ret;

	if(!IS_ERR_OR_NULL(slot->power)) {
		if (set_slot_power_mV(slot, 3300) < 0)	// switch to 3.3V if possible
			set_slot_power_mV(slot, 5000);	// fall back to 5V
	}

	slot->cd_state = -1;	// enforce a state update event for initial state
#if 1	// use polling
	INIT_DELAYED_WORK(&slot->work, retrode3_cd_work);
	schedule_delayed_work(&slot->work,
			msecs_to_jiffies(50));	// start first check

#else	// use interrupt (untested)
	ret = devm_request_threaded_irq(dev, gpiod_to_irq(slot->cd),
		NULL, retrode3_gpio_cd_irqt,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"cart-detect", slot);
#endif

	return 0;
}

/*
 * cart detect and /sysfs
 */

static ssize_t sense_show(struct device *dev, struct device_attribute *attr,
				char *buf);

static void retrode3_update_cd(struct retrode3_slot *slot)
{
	int cd_state = gpiod_get_value(slot->cd);	// check cart detect pin

	if (cd_state < 0)
		return;	// ignore

	if (cd_state != slot-> cd_state) {
		char *envp[3];
		char buf[20];
		int len;

// printk("%s: state changed to %d\n", __func__, cd_state);

		slot->cd_state = cd_state;

		// automatically control status LED if defined?

		if (slot->bus_width == 16 && cd_state == 0)
			set_slot_power_mV(slot, 3300);	// turn off 5V mode (for MD slot)

		envp[0] = kasprintf(GFP_KERNEL, "SLOT=%s", dev_name(&slot->dev));
		len = sense_show(&slot->dev, NULL, buf);
		if (len < 0)
			return;
		if (buf[len - 1] == '\n')
			buf[len - 1] = 0;	// strip off \n
		envp[1] = kasprintf(GFP_KERNEL, "SENSE=%s", buf);
		envp[2] = NULL;
// printk("%s: %s %s\n", __func__, envp[0], envp[1]);
		// check with: udevadm monitor --environment
		kobject_uevent_env(&slot->dev.kobj, KOBJ_CHANGE, envp);

	}
}

#if 0
static irqreturn_t retrode3_gpio_cd_irqt(int irq, void *dev_id)
{
	struct retrode3_slot *slot = dev_id;

	retrode3_update_cd(slot);

	return IRQ_HANDLED;
}
#endif

static void retrode3_cd_work(struct work_struct *work)
{
	struct retrode3_slot *slot = container_of(work, struct retrode3_slot, work.work);

	retrode3_update_cd(slot);

	schedule_delayed_work(&slot->work,
			msecs_to_jiffies(50));	// start next check
}

#if FIXME

static const struct retrode3_device_id retrode3_slot_idtable[] = {
	{ "retrode3-slot", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, retrode3_slot_idtable);

static const struct of_device_id retrode3_slot_of_match[] = {
	{ .compatible = "openpandora,retrode3-slot" },
/* not yet used */
	{ .compatible = "openpandora,retrode3-slot-snes", .data = (void *) DEVICE_SNES },
	{ .compatible = "openpandora,retrode3-slot-megadrive", .data = (void *) DEVICE_MD },
	{ .compatible = "openpandora,retrode3-slot-nes", .data = (void *) DEVICE_NES },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, retrode3_slot_of_match);

static struct retrode3_driver retrode3_slot_driver = {
	.driver = {
		.name	= "retrode3-controller",
		.of_match_table = retrode3_slot_of_match,
	},
	.id_table	= retrode3_slot_idtable,
	.probe		= retrode3_probe_slot,
};

module_retrode3_driver(retrode3_slot_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("Retrode3 Game Cartridge Slot Driver");
MODULE_LICENSE("GPL");

#endif

#undef set_address
