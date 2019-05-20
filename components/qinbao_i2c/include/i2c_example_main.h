#ifndef _QINBAO_I2C_H_
#define _QINBAO_I2C_H_

#define	IIC_COMMAND_WIFI_SCAN_BEGIN			"bgn\x0d"
#define	IIC_COMMAND_WIFI_SCAN_STOP			"stp\x0d"
#define	IIC_COMMAND_OFF						"off\x0d"

void qinbao_i2c();

#endif