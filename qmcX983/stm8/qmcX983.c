
#include "stm8s.h"
#include "qst_i2c.h"
#include "delay.h"

#if defined(QST_CONFIG_QMCX983)
#define QMC6983_A1_D1				0
#define QMC6983_E1					1	
#define QMC7983						2
#define QMC7983_LOW_SETRESET		3
#define QMC6983_E1_Metal			4
#define QMC7983_Vertical			5
#define QMC7983_Slope				6

//#define QMC5883L

#if defined(QMC5883L)
#define QMC_II_ADDR		(0x0c<<1)
#else
#define QMC_II_ADDR		(0x2c<<1)
#endif
extern void qst_printf(const char *format, ...);
static uint8_t mag_chip_id=QMC7983_Vertical;
static int otp_kx, otp_ky;

uint8_t qmcX983_read_xyz(void)
{
	uint8_t reg_data[6];
	int16_t raw[3];
	float out_data[3];
	uint8_t err = 0;

	reg_data[0] = 0;

	while((!(reg_data[0]&0x07)) && (err<3))
	{
		qst_iic_read(QMC_II_ADDR, 0x06, &reg_data[0], 1);
		//qst_printf("qmcX983 value=%d\n",reg_data[0]);
		err++;
	}
	if(err >= 3)
		return 0;
	err = qst_iic_read(QMC_II_ADDR, 0x00, reg_data, 6);
 	raw[0] = (int16_t)((reg_data[1]<<8)|(reg_data[0]));
	raw[1] = (int16_t)((reg_data[3]<<8)|(reg_data[2]));
	raw[2] = (int16_t)((reg_data[5]<<8)|(reg_data[4]));

#if defined(QMC5883L)
	qst_printf("mag	%f %f %f\n",(float)raw[0]/30.0f,(float)raw[1]/30.0f,(float)raw[2]/30.0f);
#else
	out_data[0] = (float)raw[0]/25.0f;
	out_data[1] = (float)raw[1]/25.0f;
	out_data[2] = (float)raw[2]/25.0f;
	out_data[2] = out_data[2] - out_data[0]*otp_kx/50.0f - out_data[1]*otp_ky/50.0f;

	qst_printf("mag	%f	%f	%f\n",out_data[0],out_data[1],out_data[2]);
#endif
	return err;
}

void qmcX983_get_otp(void)
{
	unsigned char databuf[2] = {0};
	unsigned char value[2] = {0};
	uint8_t ret = 0;

	otp_kx = 0;
	otp_ky = 0;

	if(mag_chip_id == QMC6983_A1_D1)
	{
		otp_kx = 0;
		otp_ky = 0;
		return;
	}
	else
	{
		ret = qst_iic_write(QMC_II_ADDR, 0x2e, 0x0a);
		if(ret == 0)
		{
			qst_printf("%s: I2C_TxData failed\n",__func__);
			return;			
		}
        delay_ms(10);
		ret = qst_iic_read(QMC_II_ADDR, 0x2f, databuf, 1);
		if(ret == 0)
		{
			qst_printf("%s: I2C_RxData failed\n",__func__);
			return;
		}
        value[0] = databuf[0];
    
		if(((value[0]&0x3f) >> 5) == 1)
		{
			otp_kx = (value[0]&0x1f)-32;
		}
		else
		{
			otp_kx = value[0]&0x1f;	
		}
		ret = qst_iic_write(QMC_II_ADDR, 0x2e, 0x0d);
		if(ret == 0)
		{
			qst_printf("%s: I2C_TxData failed\n",__func__);
			return;			
		}
        delay_ms(10);
		ret = qst_iic_read(QMC_II_ADDR, 0x2f, databuf, 1);
		if(ret == 0)
		{
			qst_printf("%s: I2C_RxData failed\n",__func__);
			return;
		}
        value[0] = databuf[0];
		delay_ms(10);
		ret = qst_iic_write(QMC_II_ADDR, 0x2e, 0x0f);
		if(ret == 0)
		{
			qst_printf("%s: I2C_TxData failed\n",__func__);
			return;			
		}
        delay_ms(10);
		ret = qst_iic_read(QMC_II_ADDR, 0x2f, databuf, 1);
		if(ret == 0)
		{
			qst_printf("%s: I2C_RxData failed\n",__func__);
			return;
		}
        value[1] = databuf[0];
		qst_printf("otp-y value:[%d  %d] \n",value[0], value[1]);
		if((value[0] >> 7) == 1)
			otp_ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6))-32;
		else
			otp_ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6));	
	}
	qst_printf("kx:%d ky:%d \n",otp_kx, otp_ky);
}

uint8_t qmcX983_init(void)
{
	uint8_t chip;

#if defined(QMC5883L)
	qst_iic_write(QMC_II_ADDR,0x0b, 0x01);
	qst_iic_write(QMC_II_ADDR,0x09,0x1d);
	delay_ms(10);
	qst_iic_read(QMC_II_ADDR, 0x0c, &chip, 1);
	qst_printf("0x0c=%d \n", chip);
	qst_iic_read(QMC_II_ADDR,0x0d, &chip, 1);
	qst_printf("0x0d=%d \n", chip);

	return 1;
#else
	qst_iic_write(QMC_II_ADDR, 0x09, 0x1d);
	delay_ms(2);
	qst_iic_read(QMC_II_ADDR, 0x0d, &chip, 1);
	qst_printf("qmcX983_init id=%d\n", (int16_t)chip);
	if(0x31 == chip)
	{
		mag_chip_id = QMC6983_E1;
	}
	else if(0x32 == chip)
	{
		qst_iic_write(QMC_II_ADDR, 0x2e, 0x01);
		qst_iic_read(QMC_II_ADDR,0x2f, &chip, 1);
		if(((chip&0x04 )>> 2))
		{
			mag_chip_id = QMC6983_E1_Metal;
		}
		else
		{
			qst_iic_write(QMC_II_ADDR,0x2e, 0x0f);
			qst_iic_read(QMC_II_ADDR,0x2f, &chip, 1);
			if(0x02 == ((chip&0x3c)>>2))
			{
				mag_chip_id = QMC7983_Vertical;
			}
			if(0x03 == ((chip&0x3c)>>2))
			{
				mag_chip_id = QMC7983_Slope;
			}
		}
	}
	else
	{
		return 0;
	}

	qst_printf("mag_chip_id=%d\n", (int)mag_chip_id);
	qst_iic_write(QMC_II_ADDR, 0x21, 0x01);
	qst_iic_write(QMC_II_ADDR, 0x20, 0x40);
	if(mag_chip_id != QMC6983_A1_D1)
	{
		qst_iic_write(QMC_II_ADDR, 0x29, 0x80);
		qst_iic_write(QMC_II_ADDR, 0x0a, 0x0c);
	}

	if(mag_chip_id == QMC6983_E1_Metal || mag_chip_id == QMC7983_Slope)
	{		
		qst_iic_write(QMC_II_ADDR, 0x1b, 0x80);
	}

	qst_iic_write(QMC_II_ADDR, 0x0b, 0x01);
	qst_iic_write(QMC_II_ADDR, 0x09, 0x1d);
	qmcX983_get_otp();
#endif

	return chip;
}
#endif

