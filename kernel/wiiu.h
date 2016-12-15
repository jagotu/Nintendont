/*
wiiu.c for Nintendont (Kernel)

Copyright (C) 2016 JaGoTu

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation version 2.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef __WIIU_H__
#define __WIIU_H__

#include "global.h"
#include "alloc.h"
#include "common.h"
#include "string.h"

#define	I2C_BASE		0xD000000
#define I2C_ENABLED		(I2C_BASE+0x150)	
#define I2C_DATA_IN		(I2C_BASE+0x154)
#define I2C_DATA_OUT	(I2C_BASE+0x158)


#define I2C_DRH			(I2C_BASE+0x570)
#define I2C_DRH_DATA	(I2C_BASE+0x584)
#define I2C_TV			(I2C_BASE+0x250)
#define I2C_TV_DATA		(I2C_BASE+0x6C)

#define LATTE_BASE		0xD800000

#define LT_CLOCKINFO	(LATTE_BASE+0x190)
#define LT_CLOCKUNK1	(LATTE_BASE+0x1B0)
#define LT_CLOCKUNK2	(LATTE_BASE+0x1B4)
#define LT_ASICREV_ACR	(LATTE_BASE+0x214)

#define TIMER_MESSAGE 1

enum WiiUI2CDevice
{
	WIIU_DEV_TV = 0,
	WIIU_DEV_DRH = 1
};


typedef struct I2c_Config
{
  u8 device;
  u8 par;
  u8 padding[2];
  u32 baudrate;
  u32 busspeedratio;
  u32 address;
} i2c_config;

typedef struct I2C_Packet
{
  u8 data[0x80];
  u8 datalength;
  u8 padding[3];
} i2c_packet;

s32 wiiu_i2c_init();
#endif