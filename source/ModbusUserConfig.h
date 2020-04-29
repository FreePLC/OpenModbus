#ifndef __MODBUS_USERCONFIG_H__

#define __MODBUS_USERCONFIG_H__


#define MODBUS_MASTER_USED
#define MODBUS_SLAVE_USED



#ifdef MODBUS_SLAVE_USED
#define SLAVE_PORT0					0
#define MODBUS_SLAVE_NUMBER    		1
#define MODBUS_SLAVE0_ADDRESS       51
#endif

#ifdef MODBUS_MASTER_USED
#define MASTER_PORT0				0
#define MODBUS_MASTER_NUMBER    	1
#define SLAVE_READ_NUM			5
#define SLAVE_WRITE_NUM			5
#endif













#endif

