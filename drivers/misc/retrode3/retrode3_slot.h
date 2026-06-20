// SPDX-License-Identifier: GPL-2.0

/*
 * based on ideas and fragents from
 *  drivers/char/mem.c
 *  drivers/gnss/core.c
 *  arch/arm/common/locomo.c
 *
 *  Copyright (C) 2022-26, H. Nikolaus Schaller
 *
 */

#ifndef _RETRODE3_SLOT_H
#define _RETRODE3_SLOT_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>

/* special mode constants - should be exported as ABI to user-space */

#define MODE_SIMPLE_BUS	0	// default read/write with just CE, RD/WR0/WR8
#define MD_ROM		MODE_SIMPLE_BUS
#define MD_P10		1	// 10 toggle pulses on CLK
#define MD_P1		2	// 1 toggle pulses on CLK
#define MD_TIME		3	// read/write with TIME impulse
#define MD_FLASH	unused 0x04 unused
#define MD_ENSRAM	5	// write D0 with TIME impulse (address ignored) without WE - switches to FRAM mode
#define MD_EEPMODE	6

#define SNES_REGULAR	MODE_SIMPLE_BUS
#define SNES_HIROM	9

// for NES address mapping: https://www.nesdev.org/wiki/CPU_memory_map

// can we simplify this? E.g. make the distinction between ROM (PRG) and RAM depend on address?
// so that /dev/slot mimicks the CPU_memory_map?

#define NES_PRG		10	// CPU d0..d7	ROM $8000-$ffff
#define NES_CHR		11	// PPU d8..d15	ROM
#define NES_CHR_M2	12	// PPU d8..d15	?
#define NES_MMC5_SRAM	13	// CPU d0..d7	special RAM? $0000-$07ff
#define NES_REG		14	// PPU d8..d15	PPU registers? $2000-$2007
#define NES_RAM		15	// PPU d8..d15	internal RAM? $0000-$07ff
#define NES_WRAM	16	// PPU d8..d15?	Cartridge RAM  $6000-$7fff

/* one slot */

struct retrode3_controller {
	struct input_dev *input;
	u32 data_offset;	// 0 or 8
	u64 last_state;
	int state_valid;
};

// this is a little similar to struct spi_device
struct retrode3_slot {
	struct device dev;
	struct retrode3_bus *bus;	// backpointer to bus
	struct gpio_desc *ce;	// slot enable
	struct gpio_desc *cd;	// slot detect
	struct gpio_desc *led;	// status led (optional)
	struct gpio_desc *power;	// power control (optional)
	struct delayed_work work;
	int cd_state;
	u32 addr_width;
	u32 bus_width;
// nur für Spezialisierung auf memory-slot
	struct cdev cdev;	// the /dev/slot character device
	int id;			// slot id
// nur für Spezialisierung auf controller-slot
	struct retrode3_controller controllers[2];
// nur für MD und FRAM
	bool fram_mode;		// FRAM mode (SONIC3) has been activated through MD_MODE_ENSRAM
};

/* retrode3 bus client match table */

#define RETRODE3_NAME_SIZE	20
#define RETRODE3_MODULE_PREFIX	"retrode3:"

struct retrode3_device_id {
	char name[RETRODE3_NAME_SIZE];
	kernel_ulong_t driver_data;	/* Data private to the driver */
};

// this is a little similar to struct spi_driver
struct retrode3_driver {
	const struct retrode3_device_id *id_table;
	int			(*probe)(struct retrode3_slot *slot);
	void			(*remove)(struct retrode3_slot *slot);
	void			(*shutdown)(struct retrode3_slot *slot);
	struct device_driver	driver;
};

extern int retrode3_register_driver(struct module *owner, struct retrode3_driver *sdrv);

static inline void retrode3_unregister_driver(struct retrode3_driver *drv)
{
	if (drv)
		driver_unregister(&drv->driver);
}

#define retrode3_register_driver(driver) \
	__retrode3_register_driver(THIS_MODULE, driver)
#define module_retrode3_driver(__retrode3_driver) \
	module_driver(__retrode3_driver, retrode3_register_driver, \
			retrode3_unregister_driver)

/* cart select */

// switch power for this slot
static int get_slot_power_mV(struct retrode3_slot *slot);
static int set_slot_power_mV(struct retrode3_slot *slot, int mV);

// select this slot on the bus (mutually exclusive!)
// FIXME: ein Slot kennt seinen bus! - aber was mit SLOT==NULL?
static int is_selected(struct retrode3_slot *slot);
static void select_slot(struct retrode3_bus *bus, struct retrode3_slot *slot);

// update cart-detect
static void retrode3_update_cd(struct retrode3_slot *slot);

// cart detect polling worker
static void retrode3_cd_work(struct work_struct *work);

// show cart detect sense status
static ssize_t sense_show(struct device *dev, struct device_attribute *attr,
				char *buf);

int retrode3_probe_slot(struct retrode3_slot *slot, struct device_node *child);
int retrode3_probe_controller(struct retrode3_slot *slot, struct device_node*child);

#endif
