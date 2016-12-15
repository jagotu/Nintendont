#include "wiiu.h"

//device 0(HDMIController) or 1(DRH)


i2c_config configurations[2];
i2c_packet packetsbuff[8];
static u32 DRCRead_thread = 0;
static u32 DRCRead_mqueue = 0;
static u8* DRCRead_heap = 0;
static u32 DRCRead_timer = 0;
static u8 i2cBuffer[0x22];
extern char __gamepad_read_stack_addr, __gamepad_read_stack_size;

static s32 wiiu_i2c_enable();
static s32 wiiu_config_init(u8 device, u8 par, u32 baudrate);
s32 wiiu_i2c_init();
static u32 DRCRead_Proc();
static s32 i2cmCtrlDrvRead(u8 device, u8 command, u8* data, u8 datalength);
static s32 i2c_read_gamepad_data(u8* dataptr);
static s32 i2cmCtrlDrvWrite(u8 device, u8 command, u8* data, u8 datalength);
static s32 i2c_clockconvert(u32 value);
static s32 i2c_getclock();
static void read_LT_ASICREV_ACR(u32* VERHI, u32* VERLO);
static void i2c_syncDRH(bool start);
static void i2c_begin_write(i2c_config conf);
static s32 i2c_send(i2c_config conf, u8 something, u8 datalength);
static s32 i2csCtrlDrvPoll(u8* buffer, u32 length);
static void i2c_write(i2c_config conf, u8* data, u8 datalength, bool last);
unsigned short crc16(unsigned char *data_p, unsigned short length);

static s32 wiiu_i2c_enable()
{
	write32(I2C_ENABLED, 1);
	return 0;
}

static s32 wiiu_config_init(u8 device, u8 par, u32 baudrate)
{
	if(device > 1) return -4;
	if(par > 1) return -4;
	
	configurations[device].device = device;
	configurations[device].par = par;
	configurations[device].baudrate = baudrate;
	if(device == WIIU_DEV_TV)
	{
		configurations[device].address = I2C_TV;
	} else {
		configurations[device].address = I2C_DRH;
	}
	
	s32 busclock = GetBUSClock();
	
	u32 res1 = ((busclock*1000000) / baudrate) * 2 - 1; //r10
	u32 res2 = 6250000/baudrate;
	u32 res3 = 1000000000/(baudrate*8);
	u32 res4 = (busclock*1000000)/res2;
	u32 res5 = (1000000000/res4) * 8;
	
	u32 result = res3 + res5;
	configurations[device].busspeedratio = result;
	
	if(res1 < res2) return -4;
	if(res1 > 0xFFFF) return -4;
	if(res2 == 0) return -4;
	if(res2 > 0xFF) return -4;
	
	u32 tmp = res2 << 16;
	tmp |= res1 << 8;
	tmp |= 1;
	if(par == 1) tmp |= 2;
	
	write32(configurations[device].address, tmp);
	write32(configurations[device].address + 8, 0);
	
	return 0;
}

s32 wiiu_i2c_init()
{
	s32 return_value;
	
	return_value = wiiu_i2c_enable();
	if(return_value < 0)
		goto CleanUp;
	
	return_value = wiiu_config_init(WIIU_DEV_TV, 0, 40000);
	if(return_value < 0)
		goto CleanUp;
	
	return_value = wiiu_config_init(WIIU_DEV_DRH, 0, 4000);
	if(return_value < 0)
		goto CleanUp;
	
	u32 priority = thread_get_priority(0);
	DRCRead_thread = thread_create(DRCRead_Proc, NULL, ((u32*)&__gamepad_read_stack_addr), ((u32)(&__gamepad_read_stack_size)) / sizeof(u32), priority, 0);
	if(DRCRead_thread < 0)
		goto CleanUp;
	
	DRCRead_heap = (u8*)malloca(32,32);
	DRCRead_mqueue = mqueue_create(DRCRead_heap, 1);	
	if(DRCRead_mqueue < 0)
		goto CleanUp;
	
	DRCRead_timer = TimerCreate(0, 5000, DRCRead_mqueue, TIMER_MESSAGE);
	if(DRCRead_timer < 0)
		goto CleanUp;
	
	return_value = thread_continue(DRCRead_thread);
	if(return_value < 0)
		goto CleanUp;
	
	memset32(i2cBuffer, 0xFF, 0x22);

	return 0;
	
	
CleanUp:
	if(DRCRead_thread != 0)
		thread_cancel(DRCRead_thread, 0);
	if(DRCRead_mqueue != 0)
		mqueue_destroy(DRCRead_mqueue);
	if(DRCRead_timer != 0)
		TimerDestroy(DRCRead_timer);
	memset32(i2cBuffer, 0xFF, 0x22);
		
	return return_value;
}

u32 i2cDRCreads = 0;
u32 i2cHASHfails = 0;
u32 i2cgeneralfails = 0;
u32 i2ccycles = 0;
u32 i2cpacketscount = 0;
u32 i2cTVfailedredirects = 0;
u32 i2cDRHfailedredirects = 0;
u32 i2cmaxlength = 0;

u8 i2cDRCdata[9];

static u32 DRCRead_Proc()
{
	struct ipcmessage *msg = NULL;
	
	
	u32 return_value;
	u8 unknown = 0;
	bool fatalerror = false;
	s32 result;
	while(1)
	{
		return_value = mqueue_recv(DRCRead_mqueue, &msg, 0);
		if(return_value < 0)
			break;
		
		//Read incoming messages
		int packetno = 0;
		while(packetno < 8)
		{
			u32 bytes_read = i2csCtrlDrvPoll(packetsbuff[packetno].data, sizeof(packetsbuff[packetno].data));
			
			if(bytes_read<0)
			{
				continue;
			} else if (bytes_read == 0)
			{
				break;
			} else 
			{
				if(bytes_read == 0x22) 
				{
					if(memcmp(i2cBuffer, packetsbuff[packetno].data, 0x22) != 0)
					{
						memcpy(i2cBuffer, packetsbuff[packetno].data, 0x22);
					}
				} else {
					memset(i2cBuffer, 0xFF, 0x22);
				}
				unknown = 0xA;
			}
		}
		
		//Redirect incoming messages every cycle
		u16 totallength = 0;
		int i;
		for(i = 0;i<packetno;i++)
		{
			totallength += packetsbuff[i].datalength;
			i2cpacketscount++;
			result = i2cmCtrlDrvWrite(WIIU_DEV_TV, 0x70, packetsbuff[i].data, packetsbuff[i].datalength); //forward to TV
			if(result<0)
			{
				//OHCI1: ERROR %x while forwarding command 0x%02x of size %d to TV
				i2cTVfailedredirects++;
			}
			
			i2c_syncDRH(0);			
			result = i2cmCtrlDrvWrite(WIIU_DEV_DRH, 0x70, packetsbuff[i].data, packetsbuff[i].datalength);
			i2c_syncDRH(1);
			if(result<0)
			{
				//OHCI1: ERROR %x while forwarding command 0x%02x of size %d DRH.
				i2cDRHfailedredirects++;
			}
			
		}
		
		if(totallength > i2cmaxlength)
		{
			i2cmaxlength = totallength;
		}
		
		if(unknown > 0)
		{
			unknown--;
		}
		
		if(unknown > 0)
		{
			i2ccycles++;
			continue;
		}
		
		//Read the gamepad every 10 cycles
		i2c_syncDRH(0);
		result = i2c_read_gamepad_data(i2cDRCdata);
		i2c_syncDRH(1);
		
		//DRCMSGProc(result);
		i2cDRCreads++;
		if(result == -20)
		{
			i2cHASHfails++;
		} else if (result <0)
		{
			i2cgeneralfails++;
		}
		
		if(fatalerror)
		{
			continue;
		}
		
		if(i2cDRCdata[1] & 0x80)
		{
			s32 stm_handle = IOS_Open("/dev/stm/immediate", 2);
			if(stm_handle < 0)
			{
				fatalerror = 1;
				continue;
			}
			IOS_Ioctl(stm_handle, 0x2006, 0, 0, 0, 0); //0x2006 = ISTM_CMD_GPIOShutdown apparently
			IOS_Close(stm_handle);
			fatalerror = 1;
		}
		
	}
	if(return_value < 0)
		return return_value;
	return 0;
}

static s32 i2cmCtrlDrvRead(u8 device, u8 command, u8* data, u8 datalength)
{
	s32 return_value;
	if(data == NULL)
		return -1;
	if(datalength > 64)
		return -4;
	if(device > 1)
		return -6;
	
	u8 command2 = (command << 1) | 1; //SP = R7
	u32 address = configurations[device].address;
	if(address == 0)
		return -10;
		
	i2c_begin_write(configurations[device]);
	i2c_write(configurations[device], &command2, 1, false);
	i2c_write(configurations[device], 0, datalength, true);
	return_value = i2c_send(configurations[device], 0x72, datalength);
	if(return_value < 0)
		return return_value;
	
	u32 value = read32(address+0xC);
	u8 bytes_available = (value >> 16) & 0xFF;
	if(bytes_available != datalength)
	{
		//I2CM: i2cmCtrlDrvRead() FIFO data count mismatch - expected datalength bytes, bytes_available available.\n
		return -4;
	}
	
	data[0] = bytes_available;
	if(bytes_available < 1)
	{
		goto Error;
		
	}
	int i;
	for(i = 1;i<datalength;i++)
	{
		value = read32(address+0xC);
		data[i] = value & 0xFF;
		if((value & 0xFF0000) == 0)
		{
			//I2CM: i2cmCtrlDrvRead() FIFO underflow - expected %d bytes, empty at offset %d.
			return_value = -4;
			goto Error;
		}
	}
	
Error:
	if(read32(address+0xC) & 0xFF0000)
	{
		//I2CM: i2cmCtrlDrvRead() FIFO overflow - expected datalength bytes, bytes_available remain.
		return -4;
	} else {
		if(return_value >= 0)
		{
			return datalength;
		} else {
			return return_value;
		}
	}
}

u8 gamepaddatax = 0;
u8 gamepaddatay = 0;
u8 gamepaddataz = 0;
static s32 i2c_read_gamepad_data(u8* dataptr)
{
	s32 return_value = 0;
	u8 command = -16;
	u8 read_data[11];
	
	return_value = i2cmCtrlDrvWrite(WIIU_DEV_DRH, 0x70, &command, 1);
	if(return_value < 0)
		goto gamepaderror;
	
	return_value = i2cmCtrlDrvRead(WIIU_DEV_DRH, 0x70, read_data, 11);
	if(return_value < 0)
	{
		goto gamepaderror;
	}
	
	if(return_value != 11)
	{
		return_value = -23;
		goto gamepaderror;
	}
	
	u16 hash = crc16(read_data, 9);
	u16 desiredhash = (read_data[9] << 8) && read_data[10];
	if(hash != desiredhash)
	{
		return -20;
	}
	
	if(read_data[0] != gamepaddatax && (read_data[1] & 0x7F) != gamepaddatay)
	{
		if(gamepaddataz > 1)
			return -20;
		gamepaddataz = (gamepaddataz + 1) & 0xFF;
		if(gamepaddataz > 1)
			return -20;
	} else {
		gamepaddatax = read_data[0];
		gamepaddatay = read_data[1] & 0x7F;
		gamepaddataz = 0;
	}
	
	memcpy(dataptr, read_data, 9);
	return 0;
	
gamepaderror:
	if(return_value < 0)
		return return_value;
	return 0;
}

static s32 i2cmCtrlDrvWrite(u8 device, u8 command, u8* data, u8 datalength)
{
	if(data == NULL)
		return -1;
	if(datalength > 64)
		return -4;
	if(device > 1)
		return -6;
	
	u8 command2 = command << 1; //SP = R7
	u32 address = configurations[device].address;
	if(address == 0)
		return -10;
		
	i2c_begin_write(configurations[device]);
	i2c_write(configurations[device], &command2, 1, false);
	i2c_write(configurations[device], data, datalength, true);
	return i2c_send(configurations[device], 0x77, datalength);
}

static s32 i2c_clockconvert(u32 value)
{
	u32 clock = i2c_getclock();
	if(clock == 0x51) {
		return (value >> 1) + (value >> 3) + (value >> 7);
	} else if(clock == 0x6C) {
		return (value >> 1) + (value >> 2) + (value >> 4) + (value >> 5);
	} else if (clock == 0xA2) {
		return value + (value >> 2) + (value >> 6);
	} else if(clock == 0x36) {
		return (value >> 2) + (value >> 3) + (value >> 5) + (value >> 6);
	} else {
		return value + (value >> 1) + (value >> 6) + (value >> 2) + (value >> 3) + (value >> 7);
	}
				
}

static s32 i2c_getclock()
{
	u32 verhi;
	u32 verlo;
	read_LT_ASICREV_ACR(&verhi, &verlo);
	
	u32 clockinfo = read32(LT_CLOCKINFO);
	if((clockinfo >> 1) & 1)
		return 0xA2;
	
	u32 clockunk1 = read32(LT_CLOCKUNK1);
	u32 clockunk2 = read32(LT_CLOCKUNK2);
	
	u32 divident = 0x798;
	if(verhi < 1){
		divident = divident << 1;
	}
	
	if((clockinfo & 1) == 0)
	{
		return divident / ((clockunk1 >> 0x17) & 0xF);
	} else
	{
		return divident / (clockunk2 & 0x1FF);
	}
	
	
}

static void read_LT_ASICREV_ACR(u32* VERHI, u32* VERLO)
{
	u32 ver = read32(LT_ASICREV_ACR);
	*VERHI = (ver >> 4) & 0xF;
	*VERLO = (ver) & 0xF;
}

/*static u32 dosomemath(u32 a, u32 b, u32 c, u32 d)
{
	d*a + b*c + (d >> 16) * (b >> 16) + 0x10000 + ( ((b & 0xFF)*(d & 0xFF))>>16 + (b & 0xFF) * (d >> 16) + (d & 0xFF) * (b >> 16)) >> 16
}*/

/*static u32 multiplylarge(u32 a, u32 b)
{
	u32 a1 = (a&0xFF)*(b&0xFF);
    u32 a2 = (a&0xFF)*(b>>16);
    u32 a3 = (a>>16)*(b&0xFF);
    u32 a4 = (a>>16)*(b>>16);

    return (((a1 >> 16) + a2 + a3)>>16) + a4;
}*/
/*
static u64 holyshit(u64 a, u64 b)
{
	//TODO
}*/

u32 sync_time;
u32 sync_data1;
u32 sync_data2;
static void i2c_syncDRH(bool start)
{
	if(start)
	{
		sync_time = time_now();
	} else {
		u32 time_passed = time_now() - sync_time;
		u32 busclock = GetBUSClock();
		u64 result1 = (u64)time_passed * (u64)1000000;
		u64 result2 = (u64)busclock *(u64)1000000;
		u64 result3 = result1 / (result2 >> 7);
		if(result3 < 499)
		{
			udelay(500-time_passed);
			sync_data1++;
			sync_data2 = 500-time_passed;
		}
	}
}

static void i2c_begin_write(i2c_config conf)
{
	if(conf.device == WIIU_DEV_DRH)
	{
		write32(I2C_TV_DATA, 0xCE0);
	} else {
		write32(I2C_DRH_DATA, 0x1F);
	}
}

static s32 i2c_send(i2c_config conf, u8 something, u8 datalength)
{
	u32 delay = (datalength * conf.busspeedratio) / 1000;
	u32 converted = i2c_clockconvert(20000);
	u32 starttime = time_now();
	
	u32 counter = 0;
	u32 buffer = 0;
	s32 return_value = 0;
	while(1)
	{
		counter++;
		if(counter == 1)
		{
			udelay(0x1F4);
		} else {
			udelay(delay);
		}
		
		if(conf.device == WIIU_DEV_TV)
		{
			buffer = read32(I2C_TV_DATA);
			if(((buffer >> 11) & 1) || ((buffer >> 10) & 1) || ((buffer >> 7) & 1))
			{
				return_value = -21;
				goto sendend;
			}
			
			if(something == 0x72)
			{
				if((buffer >> 5) & 1)
				{
					return_value = 0;
					goto sendend;
				}
			} else if(something == 0x77)
			{
				if((buffer >> 12) & 1)
				{
					return_value = 0;
					goto sendend;
				}
			}
		} else {
			buffer = read32(I2C_DRH_DATA);
			if(((buffer >> 4) & 1) || ((buffer >> 3) & 1) || ((buffer >> 2) & 1))
			{
				return_value = -21;
				goto sendend;
			}
			
			if(something == 0x72)
			{
				if(buffer & 1)
				{
					return_value = 0;
					goto sendend;
				}
			} else if(something == 0x77)
			{
				if((buffer >> 1) & 1)
				{
					return_value = 0;
					goto sendend;
				}
			}
		}
		u32 now = time_now();
		if((now - starttime) >= converted)
		{
			break;
		}
		
	}
	return_value = -10;

sendend:
	if(conf.device == WIIU_DEV_TV)
	{
		write32(buffer & 0xCE0, I2C_TV_DATA);
	} else {
		write32(buffer & 0x1F, I2C_DRH_DATA);
	}
	return return_value;
}

//LSLS x, n; BMI
//jump if (x >> (31-n)) & 0x1

static void i2c_write(i2c_config conf, u8* data, u8 datalength, bool last)
{
	if(datalength == 0)
		return;
	int i;
	for(i = 0;i<datalength;i++)
	{
		u8 byte;
		if(data == NULL)
		{
			byte = datalength;
		} else {
			byte = data[i];
		}
		
		if(i == datalength - 1 && last)
		{
			byte |= 0x100;
		}
		
		write32(conf.address+4, (u32)byte);
		write32(conf.address+8, 1);
		
	}
	
}

static s32 i2csCtrlDrvPoll(u8* buffer, u32 length)
{
	u32 data_in = 0;
	data_in = read32(I2C_DATA_IN);
	if(data_in << 15 >> 24) //We have an I2c address
	{
		u32 readlength;
		u8 finished = 0;
		for(readlength = 0;readlength<length;readlength++)
		{
			write32(I2C_DATA_OUT, 1); //I2C send ACK
			data_in = read32(I2C_DATA_IN);
			if(readlength == 0)
			{
				buffer[readlength++] = (data_in >> 17) & 0xFF; //Store address on the beginning of buffer
																 //Since we already ACKed the first byte, we now already have address + byte0
			}
			buffer[readlength] = data_in & 0xFF;
			if(data_in >> 8 & 0x1) //This byte was last, terminate connection
			{
				finished = 1;
				break;
			}
		}
		
		if(!finished)
		{
			return -5; //buffer overflow
		} else {
			return readlength;
		}
	}
	return 0;
}

#define POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/

unsigned short crc16(unsigned char *data_p, unsigned short length)
{
      unsigned char i;
      unsigned int data;
      unsigned int crc = 0xffff;

      if (length == 0)
            return (~crc);

      do
      {
            for (i=0, data=(unsigned int)0xff & *data_p++;
                 i < 8; 
                 i++, data >>= 1)
            {
                  if ((crc & 0x0001) ^ (data & 0x0001))
                        crc = (crc >> 1) ^ POLY;
                  else  crc >>= 1;
            }
      } while (--length);

      crc = ~crc;
      data = crc;
      crc = (crc << 8) | (data >> 8 & 0xff);

      return (crc);
}