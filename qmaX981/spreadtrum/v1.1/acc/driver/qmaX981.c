/*****************************************************************************
 *
 * Copyright (c) 2013 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the QST Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of QST Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("QST Software")
 * have been modified by QST Inc. All revisions are subject to any receiver's
 * applicable license agreements with QST Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *****************************************************************************/
#include<linux/module.h>
#include<linux/err.h>
#include<linux/errno.h>
#include<linux/delay.h>
#include<linux/fs.h>
#include<linux/i2c.h>

#include<linux/input.h>
#include<linux/input-polldev.h>
#include<linux/miscdevice.h>
#include<linux/uaccess.h>
#include<linux/slab.h>

#include<linux/workqueue.h>
#include<linux/irq.h>
#include<linux/gpio.h>
#include<linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif
#include<linux/of_device.h>
#include<linux/of_address.h>
#include<linux/of_gpio.h>

#include<linux/wakelock.h>
#include<linux/mutex.h>

#include "qmaX981.h"

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t acc_flag = ATOMIC_INIT(0);
#if defined(QMAX981_STEP_COUNTER)
static atomic_t sc_flag = ATOMIC_INIT(0);
#endif
static unsigned char v_open_flag = 0x00;


enum qmaX981_axis {
	QMAX981_AXIS_X = 0,
	QMAX981_AXIS_Y,
	QMAX981_AXIS_Z,
	QMAX981_AXIS_NUM
};

struct qmaX981_acc {
	signed short x;
	signed short y;
	signed short z;	
#ifdef QMAX981_STEP_COUNTER
	unsigned int stepcount;
#endif
};

struct hwmsen_convert {
	signed char sign[3];
	unsigned char map[3];
};

#ifdef QMA6981_STEP_COUNTER_USE_INT

#define STEP_INT_START_VLUE	4
#define STEP_END	false
#define STEP_START	true

static int STEP_DUMMY_VLUE = 10;

static bool wake_lock_status = false;
static struct wake_lock sc_wakelock;
struct qma6981_stepcount{
	//int stepcounter_pre_start;
	int stepcounter_pre_end;  
	int stepcounter_next_start;
	int stepcounter_next_end;  
	int stepcounter_pre;
	int stepcounter_pre_fix;
	bool stepcounter_statu;
	bool stepcounter_start_int_update_diff;
	int back;
	int step_diff;
};
bool int_statu_flag = false;
static struct qma6981_stepcount step_count_index;
#endif


struct qmaX981_data {
	atomic_t enable;                /* attribute value */
	atomic_t step_count_enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	atomic_t position;

	struct qmaX981_acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex op_mutex;
	
	struct i2c_client *client;
	struct input_dev *input;
	
	struct delayed_work work;
	
	unsigned int resolution;

#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend; 
#endif

#if defined(QMAX981_STEP_COUNTER)
	struct input_dev *input_sc;
	struct delayed_work sc_work;
	int sc_dely;
#endif

#ifdef QMA6981_STEP_COUNTER_USE_INT
	struct 	work_struct  eint_work;
	struct  device_node *irq_node;
	int 	irq;
#endif

};

static struct i2c_client *this_client = NULL;

#define QMAX981_DEV_NAME       "qmax981"
#define QMAX981_DEV_VERSION    "1.0.1"
#define QMAX981_INPUT_NAME     "accelerometer"
#define QMAX981_I2C_ADDR       QMAX981_ACC_I2C_ADDR

#define QMAX981_SC_INPUT_NAME     "step_counter"


static unsigned char qmaX981_current_placement = 0; // current soldered placement

#define LOG_TAG				        "[QMA-Gsensor] "
#define GSE_FUN( )			        printk(KERN_INFO LOG_TAG"%s\n", __FUNCTION__)
#define GSE_LOG(fmt, args...)		printk(KERN_INFO LOG_TAG fmt, ##args)
#define GSE_ERR(fmt, args...)		printk(KERN_ERR  LOG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

// Transformation matrix for chip mounting position
static struct hwmsen_convert map[] = {
    { { 1, 1, 1}, {0, 1, 2} },
    { {-1, 1, 1}, {1, 0, 2} },
    { {-1,-1, 1}, {0, 1, 2} },
    { { 1,-1, 1}, {1, 0, 2} },

    { {-1, 1,-1}, {0, 1, 2} },
    { { 1, 1,-1}, {1, 0, 2} },
    { { 1,-1,-1}, {0, 1, 2} },
    { {-1,-1,-1}, {1, 0, 2} },      

};

static inline int qmaX981_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	
	signed int dummy = 0;
	mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	*data = dummy & 0x000000ff;
	return 0;
}

static inline int qmaX981_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	signed int dummy = 0;
    mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static inline int qmaX981_smbus_read_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);	
	signed int dummy = 0;

	mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}

static inline int qmaX981_smbus_write_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	signed int dummy = 0;

	mutex_lock(&qmaX981->op_mutex);
	dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
	mutex_unlock(&qmaX981->op_mutex);
	if (dummy < 0)
		return dummy;
	return 0;
}



static int qmaX981_read_chip_id(struct i2c_client *client, char *buf)
{
	unsigned char data[3];
	
	int res = 0;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	GSE_FUN();

	res = qmaX981_smbus_read_byte(qmaX981->client, QMA6981_CHIP_ID, &data[0]);
	if (res)
	{
		GSE_ERR("%s: i2c error!\n", __func__);
		return -1;
	}

	if (0xA9 == data[0])
	{
		data[1] = QMA6981_DIE_ID;
	}
	else
	{
		data[1] = QMA6981_DIE_ID_V2;
	}

	res = qmaX981_smbus_read_byte(qmaX981->client, data[1], &data[2]);
	if (res)
	{
		GSE_ERR("%s: i2c error!\n", __func__);
		return -1;
	}
		

	return sprintf(buf, "%02x-%02x\n", data[0], data[2]);
}

static int qmaX981_set_mode(struct i2c_client *client, unsigned char mode)
{
	int rc = 0;
	unsigned char data = 0;

#ifdef QMAX981_STEP_COUNTER
	return 0;
#endif
	if (mode == QMAX981_MODE_STANDBY)
	{
		#ifndef QMAX981_STEP_COUNTER
			data = 0x00;
		#else
			data = 0x80;
		#endif	
		rc = qmaX981_smbus_write_byte(client,QMA6981_MODE,&data);

	}
	else if (mode == QMAX981_MODE_ACTIVE)
	{
		data = 0x80;
		rc = qmaX981_smbus_write_byte(client,QMA6981_MODE,&data);
		msleep(1);
	}

	return 0;
}


#ifdef QMAX981_STEP_COUNTER
static int qmaX981_clear_stepcount(struct i2c_client *client)
{
	int rc = 0;
	unsigned char data = 0;

	data = 0x80;
	
	rc = qmaX981_smbus_write_byte(client,0x13,&data);
#ifdef QMA6981_STEP_COUNTER_USE_INT
	memset(&step_count_index, 0, sizeof(step_count_index));
#endif
	return 0;
}
#endif

static int qmaX981_get_delay(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return atomic_read(&qmaX981->delay);
}

static void qmaX981_set_delay(struct device *dev, int delay)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);	
	int delay_last =  atomic_read(&qmaX981->delay);

	GSE_FUN();
	
	atomic_set(&qmaX981->delay, delay);

	mutex_lock(&qmaX981->enable_mutex);

	if (delay_last != delay) {
		cancel_delayed_work_sync(&qmaX981->work);
		schedule_delayed_work(&qmaX981->work, msecs_to_jiffies(delay) + 1);
	}

	mutex_unlock(&qmaX981->enable_mutex);
}

static int qma6981_initialize(struct qmaX981_data *qmaX981)
{
	int rc = 0;
	unsigned char data = 0;

	// reset all regs
	data = 0xb6;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x36,&data);
	mdelay(5);
	// reset all regs

	//0x01 range  2g , 3.9mg/LSB  @2g
	//0x04 range  8g , 15.6mg/LSB @8g
#if defined(QMAX981_STEP_COUNTER)
	data = QMA6981_RANGE_8G;
	qmaX981->resolution = 64;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x0F,&data);

	data = 0x2a;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x10,&data);

	data = 0x8f;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x12,&data);

	data = 0x10;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x13,&data);

	data = 0x14;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x14,&data);

	data = 0x10;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x15,&data);

	data = 0x0c;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x16,&data);

#ifdef QMA6981_STEP_COUNTER_USE_INT	
	data = 0x08;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x19,&data);
#endif

	data = QMAX981_OFFSET;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x27,&data);

	data = QMAX981_OFFSET;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x28,&data);

	data = QMAX981_OFFSET;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x29,&data);
	
	//0x32,0x00 xy;0x01 yz; 0x02 xz;
	data = 0x02;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x32,&data);
	

	data = 0x80;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x11,&data);

#else
	data = QMA6981_RANGE_4G;
	qmaX981->resolution = 128;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x0F,&data);

	data = QMA6981_ODR_125HZ;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x10,&data);

	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x27,&data);

	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x28,&data);

	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x29,&data);

	data = 0x0c;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x16,&data);
	
	data = 0x00;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x19,&data);

	data = 0x80;
	rc = qmaX981_smbus_write_byte(qmaX981->client,0x11,&data);
#endif

	return rc;
}


static int qmaX981_read_accel_xyz(struct qmaX981_data *qmaX981, struct qmaX981_acc *acc, int orient)
{
	int res = -1;
	unsigned char data[6] = { 0 };

	s16 raw[3] = { 0 };
	int i;
	const struct hwmsen_convert *cvt = NULL;

	res = qmaX981_smbus_read_block(qmaX981->client, QMA6981_XOUTL, data, 6);
		raw[0] = (s16)((data[1]<<2) |( data[0]>>6));
		raw[1] = (s16)((data[3]<<2) |( data[2]>>6));
		raw[2] = (s16)((data[5]<<2) |( data[4]>>6));

	for(i=0;i<3;i++)
	{
		if ( raw[i] == 0x0200 )
			raw[i]= -512;
		else if ( raw[i] & 0x0200 ){
			raw[i] -= 0x1;	
			raw[i] = ~raw[i];
			raw[i] &= 0x01ff;
			raw[i] = -raw[i];
		}
	}

#ifdef QMAX981_STEP_COUNTER
	raw[0] -= QMAX981_OFFSET;
	raw[1] -= QMAX981_OFFSET;
	raw[2] -= QMAX981_OFFSET;
#endif	
	
	raw[0] = ((raw[0]*1000)/qmaX981->resolution);
	raw[1] = ((raw[1]*1000)/qmaX981->resolution);
	raw[2] = ((raw[2]*1000)/qmaX981->resolution);
	GSE_LOG("%s_A: %d %d %d\n",__func__,raw[0],raw[1],raw[2]);


	cvt = &map[orient];
	acc->x = cvt->sign[QMAX981_AXIS_X] * raw[cvt->map[QMAX981_AXIS_X]];
	acc->y = cvt->sign[QMAX981_AXIS_Y] * raw[cvt->map[QMAX981_AXIS_Y]];
	acc->z = cvt->sign[QMAX981_AXIS_Z] * raw[cvt->map[QMAX981_AXIS_Z]];
	
	GSE_LOG("%s_B: %d %d %d\n",__func__,acc->x,acc->y,acc->z);

	return res;
}


#ifdef QMA6981_STEP_COUNTER_USE_INT
static void qma6981_set_wakelock(int en)
{
	#if 1
	if((en == 1)&&(wake_lock_status == false))
	{
		wake_lock(&sc_wakelock);
		wake_lock_status = true;
		GSE_LOG("yzqlock enable wakelock\n");
	}
	else if((en == 0)&&(wake_lock_status == true))
	{
		wake_unlock(&sc_wakelock);		
		wake_lock_status = false;
		GSE_LOG("yzqlock disable wakelock\n");
	}
	#endif
}

/*
void QMA6981_eint_func(void)
{
	if(qmaX981 == NULL)
	{
		return;
	}	

	schedule_work(&qmaX981->eint_work);
}*/

static irqreturn_t QMA6981_eint_handler(int irq, void *desc)
{
	disable_irq_nosync(irq);		// add by yangzhiqiang 2017/03/02

	schedule_work(&qmaX981->eint_work);

	return IRQ_HANDLED;
}


static  int   QMA6981_setup_eint_ok = 0;


int QMA6981_setup_eint(struct i2c_client *client)
{
	struct qmaX981_data *obj = i2c_get_clientdata(client);        
	int err = 0;	

	if (QMA6981_setup_eint_ok == 1) 
	{
		//enable_irq(gpio_to_irq(QMAX981_IRQ_NUMBER));
		
		//return 0;
	}

	memset(&step_count_index, 0, sizeof(step_count_index));

	gpio_request(QMAX981_IRQ_NUMBER, "gsensor_irq_pin");
	gpio_direction_input(QMAX981_IRQ_NUMBER);	
	obj->irq = gpio_to_irq(QMAX981_IRQ_NUMBER);
	// err = request_irq(obj->irq, QMA6981_eint_handler, IRQF_TRIGGER_RISING |IRQF_NO_SUSPEND, "gsensor_irq_pin", NULL);
	err = request_irq(obj->irq, QMA6981_eint_handler, IRQF_TRIGGER_HIGH, "gsensor_irq_pin", NULL);


	QMA6981_setup_eint_ok = 1;

	disable_irq(obj->irq);
	enable_irq(obj->irq);
    return 0;
}

static void QMA6981_eint_work(struct work_struct *work)
{
	//struct qmaX981_data *obj = container_of((struct delayed_work *)work, struct qmaX981_data, work);//error
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	//int gpio39_status = -1;	
	int res = 0,temp = 0;
	unsigned char databuf[6]; 
	u16 data = 0;
//	gpio39_status = gpio_get_value(QMAX981_IRQ_NUMBER);

	
	qma6981_set_wakelock(1);
	if(qmaX981 && qmaX981->client)
	{
		res	= qmaX981_smbus_read_block(qmaX981->client, QMA6981_STEPCOUNT, databuf, 2);
		if(res)
		{		
			res	= qmaX981_smbus_read_block(qmaX981->client, QMA6981_STEPCOUNT, databuf, 2);
			if(res)
			{
				qma6981_set_wakelock(0);
				return;
			}
		}
	}
	else
	{
		qma6981_set_wakelock(0);
		return;
	}
	qma6981_set_wakelock(0);
	data = (u16)(databuf[1]<<8)|databuf[0];

	if(int_statu_flag)
	{	
		int_statu_flag = false;
		step_count_index.stepcounter_next_end = data;
		step_count_index.stepcounter_statu = STEP_END;
		GSE_LOG("qma6981** stepcounter_next_end = %d stepcounter_next_start = %d\n",step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start);
		if (step_count_index.stepcounter_next_end < step_count_index.stepcounter_next_start)
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start;
		}
		GSE_LOG("qma6981**step_int_end  temp =%d,step_count_index.step_diff =%d\n" ,temp,step_count_index.step_diff);
		if (temp < STEP_DUMMY_VLUE)
		{
			step_count_index.step_diff += (temp+STEP_INT_START_VLUE);
			// add by yangzhiqiang for step_diff
			if(step_count_index.step_diff > data)
			{
				step_count_index.step_diff = data;
			}
			// yangzhiqiang for step_diff
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;
			
		}
		else
		{
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;
		}
		
		GSE_LOG("qma6981**step_int_end  stepcounter_pre_end = %d , stepcounter_next_end = %d stepcounter_next_start = %d,step_count_index.step_diff =%d\n",	step_count_index.stepcounter_pre_end ,step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start,step_count_index.step_diff );
		//res = request_irq(qmaX981->irq, QMA6981_eint_handler, IRQF_TRIGGER_RISING, "gsensor-eint", NULL);
		irq_set_irq_type(qmaX981->irq,IRQF_TRIGGER_HIGH);
		GSE_LOG("qma6981**step_int_end\n" );
	}
	else
	{
		int_statu_flag = true;
		step_count_index.stepcounter_next_start= data;
		step_count_index.stepcounter_statu = STEP_START;
		GSE_LOG("qma6981** stepcounter_next_start = %d stepcounter_pre_end = %d\n", step_count_index.stepcounter_next_start,step_count_index.stepcounter_pre_end);
		if (step_count_index.stepcounter_next_start < step_count_index.stepcounter_pre_end)
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end;
		}
		GSE_LOG("qma6981**step_int_start  temp =%d,step_count_index.step_diff =%d\n" ,temp,step_count_index.step_diff);
		if (temp >STEP_INT_START_VLUE)
		{
			step_count_index.step_diff += (temp - STEP_INT_START_VLUE);
		}
		GSE_LOG("qma6981**start  stepcounter_pre_end = %d , stepcounter_next_end = %d stepcounter_next_start = %d,step_count_index.step_diff =%d\n",	step_count_index.stepcounter_pre_end ,step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start,step_count_index.step_diff );
		irq_set_irq_type(qmaX981->irq,IRQF_TRIGGER_LOW);
		GSE_LOG("step_int_start\n" );
	}
	
	//disable_irq(qmaX981->irq);
	enable_irq(qmaX981->irq);
	return;
}

#endif

static void acc_work_func(struct work_struct *work)
{
	static struct qmaX981_acc acc = { 0 };
	struct qmaX981_data *qmaX981 = container_of((struct delayed_work *)work, struct qmaX981_data, work);

	int comres = -1;
	
	GSE_FUN();
	comres = qmaX981_read_accel_xyz(qmaX981, &acc, qmaX981_current_placement);
	if(comres)
	{
		comres = qmaX981_read_accel_xyz(qmaX981, &acc, qmaX981_current_placement);
		if(comres)
		{		
			schedule_delayed_work(&qmaX981->work, msecs_to_jiffies(atomic_read(&qmaX981->delay)));
			return;
		}
	}
	
	qmaX981->value.x = acc.x;
	qmaX981->value.y = acc.y;
	qmaX981->value.z = acc.z;
	
	input_report_abs(qmaX981->input, ABS_X, acc.x);
	input_report_abs(qmaX981->input, ABS_Y, acc.y);
	input_report_abs(qmaX981->input, ABS_Z, acc.z);
	input_report_abs(qmaX981->input, ABS_THROTTLE, 3);
	
	input_sync(qmaX981->input);
		
	GSE_LOG("%s: [%d %d %d ]\n",__func__,acc.x,acc.y,acc.z);
	schedule_delayed_work(&qmaX981->work, msecs_to_jiffies(atomic_read(&qmaX981->delay)));
}

#if defined(QMAX981_STEP_COUNTER)
static void step_c_work_func(struct work_struct *work){
	int ret;
	struct qmaX981_data *qmaX981;
	unsigned char data[6] = { 0 };
	
	//u16 raw = 0;
	u16 resut = 0;
	u16 resut__ext = 0;
#ifdef QMA6981_STEP_COUNTER_USE_INT 
	u16 tempp = 0;
#endif

	GSE_FUN();
	qmaX981 = container_of((struct delayed_work *)work, struct qmaX981_data, sc_work);

	ret = qmaX981_smbus_read_block(qmaX981->client, QMA6981_STEPCOUNT, data, 2);
	if(ret)
	{
		ret = qmaX981_smbus_read_block(qmaX981->client, QMA6981_STEPCOUNT, data, 2);
		if(ret)
		{		
			schedule_delayed_work(&qmaX981->sc_work, msecs_to_jiffies(qmaX981->sc_dely));
			return;
		}
	}
	
	resut = (u16)((data[1]<<8) |( data[0]));
	GSE_LOG("%s: %d\n",__func__,resut);

#ifdef QMA6981_STEP_COUNTER_USE_INT 
	if (resut < step_count_index.stepcounter_pre)
	{
		step_count_index.back++;
		resut__ext = resut- step_count_index.stepcounter_pre + 65536;
		step_count_index.stepcounter_pre = resut;
	}
	else
	{
		//nothing
		step_count_index.stepcounter_pre = resut;
		resut__ext = resut;
	}

	if (step_count_index.stepcounter_statu == STEP_START)
	{	
		if (resut__ext >= step_count_index.stepcounter_next_start)
		{
			tempp = resut__ext - step_count_index.stepcounter_next_start + 4;
		}
		else
		{
			tempp = resut__ext - step_count_index.stepcounter_next_start +65540;
		}

		GSE_LOG("ReadStepCounter_running resut__ext= %d,stepcounter_next_start = %d,tempp = %d,stepcounter_pre_end =%d \n",resut__ext,step_count_index.stepcounter_next_start,tempp,step_count_index.stepcounter_pre_end);
		GSE_LOG("ReadStepCounter_running 00 step_count_index.step_diff =%d\n" ,step_count_index.step_diff);
		if (tempp < (STEP_INT_START_VLUE+STEP_DUMMY_VLUE))
		{
			resut__ext = step_count_index.stepcounter_pre_fix;
			GSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
		}
		else
		{
			if (step_count_index.step_diff >resut__ext)
			{
				step_count_index.step_diff = 0;
			}
			else
			{
				resut__ext = resut__ext -  step_count_index.step_diff;
				step_count_index.stepcounter_pre_fix = resut__ext ;
				GSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
			}
		}
	
	}
	else 
	{
		GSE_LOG("ReadStepCounter_running 11 step_count_index.step_diff =%d\n" ,step_count_index.step_diff);
		// add by yangzhiqiang for step_diff
		if(step_count_index.step_diff > resut__ext)
		{
			step_count_index.step_diff = resut__ext;
		}
		// yangzhiqiang for step_diff		

		step_count_index.stepcounter_pre_end = resut__ext;

		resut__ext  = resut__ext -  step_count_index.step_diff;
		step_count_index.stepcounter_pre_fix = resut__ext;
		GSE_LOG("ReadStepCounter_end stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
	}
	GSE_LOG("ReadStepCounter=%d, step_diff= %d\n",resut__ext,step_count_index.step_diff );
#else
	resut__ext = resut;
#endif

	qmaX981->value.stepcount = resut__ext;
	
	

	input_report_abs(qmaX981->input_sc, ABS_RX, qmaX981->value.stepcount);
	
	input_sync(qmaX981->input_sc);
	
	GSE_LOG("%s: step_value: %d \n",__func__,resut__ext);
	schedule_delayed_work(&qmaX981->sc_work, msecs_to_jiffies(qmaX981->sc_dely));
}
#endif

static int qmaX981_input_init(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = NULL;
	int err = 0;

	GSE_LOG("%s called\n", __func__);
	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}

	dev->name = QMAX981_INPUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_capability(dev, EV_ABS, ABS_THROTTLE);
	
	input_set_abs_params(dev, ABS_X, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_8G, ABSMAX_8G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_8G, ABSMAX_8G, 0, 0);
	
	input_set_abs_params(dev, ABS_THROTTLE, 0, 3, 0, 0);
	
	input_set_drvdata(dev, qmaX981);

	err = input_register_device(dev);
	
	if (err < 0) {
		pr_err("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}
	
	qmaX981->input = dev;

	return 0;
}


static void qmaX981_input_deinit(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = qmaX981->input;

	GSE_LOG("%s called\n", __func__);
	input_unregister_device(dev);
	
	input_free_device(dev);
}

#ifdef QMAX981_STEP_COUNTER
static int qmaX981_sc_input_init(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = NULL;
	int err = 0;

	GSE_LOG("%s called\n", __func__);
	
	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}
	dev->name = QMAX981_SC_INPUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, 65535, 0, 0);
	input_set_abs_params(dev, ABS_Z, 0, 65535, 0, 0);
	input_set_abs_params(dev, ABS_RX, 0, 65535, 0, 0);
	
	input_set_drvdata(dev, qmaX981);

	err = input_register_device(dev);
	
	if (err < 0) {
		pr_err("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}
	
	qmaX981->input_sc = dev;

	return 0;
}

static void qmaX981_sc_input_deinit(struct qmaX981_data *qmaX981)
{
	struct input_dev *dev = qmaX981->input_sc;

	GSE_LOG("%s called\n", __func__);

	input_unregister_device(dev);
	
	input_free_device(dev);
}

#endif

static int qmaX981_get_enable(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return atomic_read(&acc_flag);
}

static int qmaX981_set_enable(struct device *dev, int enable)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_LOG("%s: enable :%d\n",__func__,enable);

	if (enable) 
	{
		atomic_set(&acc_flag, 1);	
		if(atomic_read(&open_flag) == 0){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_ACTIVE);
		}
		v_open_flag |= 0x01;
		schedule_delayed_work(&qmaX981->work,msecs_to_jiffies(atomic_read(&qmaX981->delay)));
	}else{
		atomic_set(&acc_flag, 0);
		v_open_flag &= 0x3e;
		if(atomic_read(&open_flag)){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY);
		}	
		cancel_delayed_work_sync(&qmaX981->work);		
	}
	
	if(v_open_flag == 0){
		qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY); //disable for step counter
	}

	atomic_set(&open_flag, v_open_flag);
	GSE_LOG("%s: open_flag :%d sc_flag %d\n",__func__,atomic_read(&open_flag),atomic_read(&acc_flag));
	return 0;
}

#ifdef QMAX981_STEP_COUNTER
static int qmaX981_get_stepcount_enable(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return atomic_read(&sc_flag);
}


static int qmaX981_set_stepcount_enable(struct device *dev, int enable)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	
	GSE_LOG("%s: enable :%d\n",__func__,enable);

	if (enable) 
	{
		atomic_set(&sc_flag, 1);	
		if(atomic_read(&open_flag) == 0){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_ACTIVE);
		}
		v_open_flag |= 0x02;
		schedule_delayed_work(&qmaX981->sc_work,msecs_to_jiffies(qmaX981->sc_dely));
	}else{
		atomic_set(&sc_flag, 0);
		
		if(atomic_read(&open_flag)){
			qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY);
		}
		v_open_flag &= 0x3d;		
		cancel_delayed_work_sync(&qmaX981->sc_work);		
	}
	
	if(v_open_flag == 0){
		qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY); //disable for step counter
	}
	
	atomic_set(&open_flag, v_open_flag);
	GSE_LOG("%s: open_flag :%d sc_flag %d\n",__func__,atomic_read(&open_flag),atomic_read(&sc_flag));
	return 0;
}
#endif

#ifdef QMAX981_STEP_COUNTER
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_power_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	ret = qmaX981_smbus_read_byte(qmaX981->client, 0x11, &databuf[0]);

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter power status 0x%2x\n", output[0]);

}

static ssize_t store_sc_power_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;
	//unsigned char data[2] = {0};
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(qmaX981->client,0x11,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_count_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	ret = qmaX981_smbus_read_byte(qmaX981->client, 0x12, &databuf[0]);

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter sample count 0x%2x\n", output[0]);

}

static ssize_t store_sc_count_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(qmaX981->client,0x12,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_recision_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	ret = qmaX981_smbus_read_byte(qmaX981->client, 0x13, &databuf[0]);

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter precision 0x%2x\n", output[0]);

}

static ssize_t store_sc_recision_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(qmaX981->client,0x13,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_timelow_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	ret = qmaX981_smbus_read_byte(qmaX981->client, 0x14, &databuf[0]);

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter time low 0x%2x\n", output[0]);

}

static ssize_t store_sc_timelow_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(qmaX981->client,0x14,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_timeup_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	ret = qmaX981_smbus_read_byte(qmaX981->client, 0x15, &databuf[0]);

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter time up 0x%2x\n", output[0]);

}

static ssize_t store_sc_timeup_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(qmaX981->client,0x15,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_axis_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret=0;

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	ret = qmaX981_smbus_read_byte(qmaX981->client, 0x32, &databuf[0]);

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter saxis is 0x%2x\n", output[0]);
}


static ssize_t store_sc_axis_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value = 0,ret;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	unsigned char data;
	
	if(1 == sscanf(buf, "%x", &value))
	{
		data = (unsigned char)value;
		ret = qmaX981_smbus_write_byte(qmaX981->client,0x32,&data);
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t qmaX981_sc_value_show(struct device *dev,
		struct device_attribute *attr, char *buf){
				
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);	
	int comres;
	u16 step_value;
	unsigned char data[2] = { 0 };
	
	comres = qmaX981_smbus_read_block(qmaX981->client, QMA6981_STEPCOUNT, data, 2);
	
	step_value = (u16)((data[1]<<8) |( data[0]));
	
	return sprintf(buf, "sc_value (%d)\n",step_value);			
}

static ssize_t qmaX981_clear_stepcount_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data = 0;
	int error = 0;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();
#ifdef QMA6981_STEP_COUNTER_USE_INT
	memset(&step_count_index, 0, sizeof(step_count_index));
#endif 
	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (qmaX981_clear_stepcount(qmaX981->client)< 0)
		return -EINVAL;

	return count;
}
#endif

static ssize_t show_chipinfo_value(struct device *dev,
		struct device_attribute *attr, char *buf){
			
	char strbuf[256];
	int output;
	unsigned char databuf;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	qmaX981_smbus_read_byte(qmaX981->client, QMA6981_CHIP_ID, &databuf);

	output = (int)databuf;

	sprintf(strbuf, "chipid:%d \n", output);

	return sprintf(buf, "%s\n", strbuf);	
}

static ssize_t show_waferid_value(struct device *dev,
		struct device_attribute *attr, char *buf){
			
	int res;
	
	unsigned int chipid;
	unsigned char chipidh;
	unsigned char chipidl;
	
	unsigned char waferid;
	unsigned char waferid1;
	unsigned char waferid2;
	unsigned char waferid3;
	
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	
	res = qmaX981_smbus_read_byte(qmaX981->client, 0x48, &chipidh);

	res = qmaX981_smbus_read_byte(qmaX981->client, 0x47, &chipidl);
	chipidl = 0x47;

	GSE_LOG("read wafer chip H:0x%x L:0x%x", chipidh, chipidl);
	chipid = (chipidh<<8)|chipidl;

	res = qmaX981_smbus_read_byte(qmaX981->client, 0x59, &waferid1);	
	
	res = qmaX981_smbus_read_byte(qmaX981->client, 0x41, &waferid2);

	res = qmaX981_smbus_read_byte(qmaX981->client, 0x40, &waferid3);	

	GSE_LOG("wafer ID: 0x%x 0x%x 0x%x\n", waferid1, waferid2, waferid3);
	
	waferid = (waferid1&0x10)|((waferid2>>4)&0x0c)|((waferid3>>6)&0x03);

	return sprintf(buf, " Chip id:0x%x \n Wafer ID 0x%02x\n", chipid, waferid);			
}

static ssize_t show_sensordata_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	
	struct qmaX981_acc acc;
	
	qmaX981_read_accel_xyz(qmaX981, &acc, qmaX981_current_placement);

	return sprintf(buf, "(%d %d %d)\n",acc.x,acc.y,acc.z);
}
		
static ssize_t show_dumpallreg_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
	char strbuf[1024];
	char tempstrbuf[24];
	unsigned char databuf[2];
	int length=0;
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	GSE_FUN();

	/* Check status register for data availability */
	for(i =0;i<64;i++)
	{
		databuf[0] = i;
		res = qmaX981_smbus_read_byte(qmaX981->client, databuf[0], &databuf[1]);
		if(res < 0)
			GSE_LOG("qma6981 dump registers 0x%02x failed !\n", i);

		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "reg[0x%2x] =  0x%2x \n",i, databuf[1]);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "%s \n",tempstrbuf);
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return sprintf(buf, "(%d %d)\n",atomic_read(&qmaX981->position),qmaX981_current_placement);
}

static ssize_t store_layout_value(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned int position = 0;

	GSE_FUN();

	position = simple_strtoul(buf, NULL,10);
	if ((position >= 0) && (position <= 7)) {
		qmaX981_current_placement = position;
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t qmaX981_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return sprintf(buf, "%d\n", atomic_read(&qmaX981->enable));
}

static ssize_t qmaX981_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int enable = simple_strtoul(buf, NULL, 10);

	GSE_FUN();

	if (enable)
		qmaX981_set_enable(&(this_client->dev), 1);
	else
		qmaX981_set_enable(&(this_client->dev), 0);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t qmaX981_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);

	GSE_FUN();

	return sprintf(buf, "%d\n", atomic_read(&qmaX981->delay));
}

static ssize_t qmaX981_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	//struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	GSE_FUN();

	if (delay > QMAX981_MAX_DELAY)
		delay = QMAX981_MAX_DELAY;
	
	//atomic_set(&qmaX981->delay, delay);

	qmaX981_set_delay(&(this_client->dev),delay);
	
	return count;
}
/*----------------------------------------------------------------------------*/
#ifdef QMAX981_STEP_COUNTER
static DEVICE_ATTR(spower,      S_IRUGO | S_IWUSR, show_sc_power_value, store_sc_power_value);
static DEVICE_ATTR(sspcount,      S_IRUGO | S_IWUSR, show_sc_count_value, store_sc_count_value);
static DEVICE_ATTR(sprecision,      S_IRUGO | S_IWUSR, show_sc_recision_value, store_sc_recision_value);
static DEVICE_ATTR(stimelow,      S_IRUGO | S_IWUSR, show_sc_timelow_value, store_sc_timelow_value);
static DEVICE_ATTR(stimeup,      S_IRUGO | S_IWUSR, show_sc_timeup_value, store_sc_timeup_value);
static DEVICE_ATTR(saxis,	S_IRUGO | S_IWUSR, show_sc_axis_value, store_sc_axis_value);
static DEVICE_ATTR(sc_value,	S_IRUGO, qmaX981_sc_value_show , NULL);
static DEVICE_ATTR(sc_cls,	S_IWUSR, NULL , qmaX981_clear_stepcount_store);
#endif 

static DEVICE_ATTR(chipinfo,	S_IRUGO, show_chipinfo_value, NULL);
static DEVICE_ATTR(waferid,		S_IRUGO, show_waferid_value, NULL);
static DEVICE_ATTR(sensordata,	S_IRUGO, show_sensordata_value,    NULL);
static DEVICE_ATTR(dumpallreg,	S_IRUGO , show_dumpallreg_value, NULL);
static DEVICE_ATTR(layout,	S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DEVICE_ATTR(enable_acc,	 S_IRUGO | S_IWUSR , qmaX981_enable_show , qmaX981_enable_store);
static DEVICE_ATTR(delay_acc,	 S_IRUGO | S_IWUSR , qmaX981_delay_show , qmaX981_delay_store);

static struct attribute *qmaX981_attributes[] = {
#ifdef QMAX981_STEP_COUNTER
	&dev_attr_spower.attr,
	&dev_attr_sspcount.attr,
	&dev_attr_sprecision.attr,
	&dev_attr_stimelow.attr,
	&dev_attr_stimeup.attr,
	&dev_attr_saxis.attr,
	&dev_attr_sc_value.attr,
	&dev_attr_sc_cls.attr,
#endif
	&dev_attr_chipinfo.attr,
	&dev_attr_waferid.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_layout.attr,
	&dev_attr_enable_acc.attr,
	&dev_attr_delay_acc.attr,
	NULL
};

static struct attribute_group qmaX981_attribute_group = {
	.attrs = qmaX981_attributes
};

static long qmaX981_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;
	struct qmaX981_data *qmaX981 = file->private_data;
	int temp = 0;

	GSE_LOG("%s: cmd %x\n",__func__, cmd);

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));

	if(err)
	{
		GSE_ERR("%s: access isn't ok!\n", __func__);
		return -EFAULT;
	}

	if (NULL == this_client)
	{
		GSE_ERR("%s: i2c client isn't exist!\n", __func__);
		return -EFAULT;
	}
	
	switch(cmd)
	{
	case QMAX981_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&temp, argp, sizeof(temp)))
		{
			GSE_ERR("%s: set delay copy error!\n", __func__);
			return -EFAULT;
		}
		if (temp < 0 || temp > 1000)
		{
			GSE_ERR("%s: set delay over limit!\n", __func__);
			return -EINVAL;
		}
		qmaX981_set_delay(&(qmaX981->client->dev),temp);
		break;

	case QMAX981_ACC_IOCTL_GET_DELAY:
		temp = qmaX981_get_delay(&(qmaX981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp)))
		{
			GSE_ERR("%s: get delay copy error!\n", __func__);
			return -EFAULT;
		}
		break;

	case QMAX981_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&temp, argp, sizeof(temp)))
		{
			GSE_ERR("%s: set enable copy error!\n", __func__);
			return -EFAULT;
		}

		if (1 == temp)
			qmaX981_set_enable(&(qmaX981->client->dev), 1);
		else if (0 == temp)
			qmaX981_set_enable(&(qmaX981->client->dev), 0);
		else
		{
			GSE_ERR("%s: set enable over limit!\n", __func__);
			return -EINVAL;
		}
			
		break;

	case QMAX981_ACC_IOCTL_GET_ENABLE:
		temp = qmaX981_get_enable(&(qmaX981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp)))
		{
			GSE_ERR("%s: get enable copy error!\n", __func__);
			return -EINVAL;
		}
		break;

#ifdef QMAX981_STEP_COUNTER
	case QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE:
		GSE_LOG("QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE START!\n");
		if (copy_from_user(&temp, argp, sizeof(temp)))
		{
			GSE_ERR("%s: set enable copy error!\n", __func__);
			return -EFAULT;
		}
		GSE_LOG("QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE OK! %d\n",temp);
		if (1 == temp)
			qmaX981_set_stepcount_enable(&(qmaX981->client->dev), 1);
		else if (0 == temp)
			qmaX981_set_stepcount_enable(&(qmaX981->client->dev), 0);
		else
		{
			GSE_ERR("%s: set enable over limit!\n", __func__);
			return -EINVAL;
		}
			
		break;

	case QMAX981_ACC_IOCTL_GET_STEPCOUNT_ENABLE:
		temp = qmaX981_get_stepcount_enable(&(qmaX981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp)))
		{
			GSE_ERR("%s: get enable copy error!\n", __func__);
			return -EINVAL;
		}
		break;
#endif
			
	case QMAX981_ACC_IOCTL_CALIBRATION:
		GSE_ERR("%s: don't handle the command!\n", __func__);
		return -EINVAL;
		

	default:
		GSE_ERR("%s: can't recognize the cmd!\n", __func__);
		return 0;
	}
	
    return 0;
}


static int qmaX981_open(struct inode *inode, struct file *file)
{
	int err = 0;

	GSE_FUN();

	err = nonseekable_open(inode, file);
	if (err < 0)
	{
		GSE_ERR("%s: open fail!\n", __func__);
		return err;
	}

	file->private_data = i2c_get_clientdata(this_client);

	return 0;
}

static int qmaX981_release(struct inode *inode, struct file *file)
{
	GSE_FUN();
	file->private_data = NULL;
	return 0;
}

static const struct file_operations qmaX981_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = qmaX981_open,
	.release = qmaX981_release,
	.unlocked_ioctl = qmaX981_unlocked_ioctl,
};

static struct miscdevice qmaX981_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMAX981_DEV_NAME,
	.fops = &qmaX981_acc_misc_fops,
};

#ifdef CONFIG_OF
static struct QMAX981_acc_platform_data *qmaX981_acc_parse_dt(struct device *dev)
{
	struct QMAX981_acc_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct lis3dh_acc_platform_data");
		return NULL;
	}

/*	pdata->gpio_int1 = of_get_gpio(np, 0);
	GSE_LOG(" %s:get gpio_int1  %d\n",__func__,pdata->gpio_int1);
	if(pdata->gpio_int1 < 0){
		dev_err(dev, "fail to get irq_gpio_number\n");
		GSE_LOG(" get irq1_gpio_number fail\n");
		goto fail;
	}*/
	
	ret = of_property_read_u32(np, "layout", &pdata->layout);
	if(ret){
		dev_err(dev, "fail to get g_range\n");
		goto fail;
	}
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static int qmaX981_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{

	static struct qmaX981_data *qmaX981;

	struct QMAX981_acc_platform_data *pdata;
	int err = 0;
	char buf[10] = {0} ;

	GSE_LOG("%s: start\n",__func__);

	/* check i2c function */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GSE_ERR("%s: i2c function not support!\n", __func__);
		err = -ENODEV;
		goto exit;
	}
	
	/* setup private data */
	qmaX981 = kzalloc(sizeof(struct qmaX981_data), GFP_KERNEL);
	if (!qmaX981) {
		GSE_ERR("%s: can't allocate memory for qmaX981_data!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	
#ifdef CONFIG_OF
	if (client->dev.of_node)
	{
		pdata = qmaX981_acc_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
			atomic_set(&qmaX981->position, pdata->layout);
			qmaX981_current_placement = pdata->layout;			
		}
		else{
			atomic_set(&qmaX981->position, qmaX981_current_placement);
		}
	}
#endif

	mutex_init(&qmaX981->enable_mutex);
	mutex_init(&qmaX981->value_mutex);	
	mutex_init(&qmaX981->op_mutex);
	
	atomic_set(&qmaX981->delay, QMAX981_DEFAULT_DELAY);
	
	this_client = client;
	qmaX981->client = client;
	i2c_set_clientdata(client, qmaX981);

#ifdef QMA6981_STEP_COUNTER_USE_INT
    wake_lock_init(&sc_wakelock,WAKE_LOCK_SUSPEND,"sc wakelock");
#endif
	
	memset(buf, 0, sizeof(buf));
	
	err = qmaX981_read_chip_id(this_client, buf);
	if (err < 0) {
		goto exit1;
		GSE_ERR("%s: qmaX981 read id fail!\n", __func__);
	}

	qma6981_initialize(qmaX981);

	INIT_DELAYED_WORK(&qmaX981->work, acc_work_func);
#ifdef QMAX981_STEP_COUNTER
	INIT_DELAYED_WORK(&qmaX981->sc_work, step_c_work_func);
	qmaX981->sc_dely = 200;
#endif 

#ifdef QMA6981_STEP_COUNTER_USE_INT
	INIT_WORK(&qmaX981->eint_work, QMA6981_eint_work); 
	QMA6981_setup_eint(this_client);	
#endif

	err = qmaX981_input_init(qmaX981);
	if (err < 0) {
		goto exit1;
		GSE_ERR("input init fail!\n");
	}

#ifdef QMAX981_STEP_COUNTER
	err = qmaX981_sc_input_init(qmaX981);
	if (err < 0) {
		GSE_ERR("qmaX981_sc_input_init fail!\n");
		goto exit2;
	}
#endif

	err = sysfs_create_group(&qmaX981->input->dev.kobj, &qmaX981_attribute_group);
	if (err < 0) {
		GSE_ERR("%s: create group fail!\n", __func__);
		#ifdef QMAX981_STEP_COUNTER
			goto exit3;
		#else
			goto exit2;
		#endif
	}

	err = misc_register(&qmaX981_device);
	if (err) {
		GSE_ERR("%s: create register fail!\n", __func__);
		goto exit4;
	}
	return 0;
exit4:
	sysfs_remove_group(&qmaX981->input->dev.kobj, &qmaX981_attribute_group);	
#ifdef QMAX981_STEP_COUNTER
exit3:
	qmaX981_sc_input_deinit(qmaX981);
#endif 
exit2:
	qmaX981_input_deinit(qmaX981);	
exit1:
	kfree(qmaX981);
exit:
	return err;	
}

static int qmaX981_remove(struct i2c_client *client)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	GSE_FUN();
	sysfs_remove_group(&qmaX981->input->dev.kobj, &qmaX981_attribute_group);
#ifdef QMAX981_STEP_COUNTER
	qmaX981_sc_input_deinit(qmaX981);
#endif 	
	qmaX981_input_deinit(qmaX981);
	kfree(qmaX981);
	return 0;
}

static int qmaX981_i2c_remove(struct i2c_client *client)
{
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(this_client);
	
	int rc;
	unsigned char data = 0x00;

	GSE_FUN();
	rc = qmaX981_smbus_write_byte(this_client,QMA6981_MODE,&data);
	
	return qmaX981_remove(qmaX981->client);
}

static int qmaX981_suspend(struct i2c_client *client, pm_message_t mesg)
{

	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);

	GSE_FUN();
	
//	mutex_lock(&qmaX981->enable_mutex);
	if(atomic_read(&acc_flag))
		cancel_delayed_work_sync(&qmaX981->work);
#if defined(QMAX981_STEP_COUNTER)
	if(atomic_read(&sc_flag))
		cancel_delayed_work_sync(&qmaX981->sc_work);
#endif
	qmaX981_set_mode(qmaX981->client,QMAX981_MODE_STANDBY);
	//atomic_set(&open_flag,0);
	
//	mutex_unlock(&qmaX981->enable_mutex);

	return 0;
}

static int qmaX981_resume(struct i2c_client *client)
{
//#if !defined(QMAX981_STEP_COUNTER)
	struct qmaX981_data *qmaX981 = i2c_get_clientdata(client);
	int delay = qmaX981_get_delay(&(client->dev));

	GSE_FUN();

	//qmaX981_set_delay(&client->dev, delay);

//	mutex_lock(&qmaX981->enable_mutex);

	qmaX981_set_mode(qmaX981->client, QMAX981_MODE_ACTIVE);
#if defined(QMAX981_STEP_COUNTER)
	if(atomic_read(&sc_flag))
		schedule_delayed_work(&qmaX981->sc_work,msecs_to_jiffies(qmaX981->sc_dely));
#endif
	if(atomic_read(&acc_flag))
		schedule_delayed_work(&qmaX981->work,msecs_to_jiffies(delay));

//	mutex_unlock(&qmaX981->enable_mutex);
//#endif
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void qmaX981_early_suspend (struct early_suspend* es)
{
	GSE_FUN();
	qmaX981_suspend(this_client,(pm_message_t){.event=0});
}

static void qmaX981_early_resume (struct early_suspend* es)
{
	GSE_FUN();

	qmaX981_resume(this_client);
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id qmaX981_id[] = {
	{QMAX981_DEV_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, qmaX981_id);

static const struct of_device_id qmaX981_acc_of_match[] = {
       { .compatible = "QST,qmaX981", },
       { }
};

MODULE_DEVICE_TABLE(of, qmaX981_acc_of_match);
static struct i2c_driver qmaX981_driver = {
	.driver = {
		.name = QMAX981_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = qmaX981_acc_of_match,
	},
	.probe    = qmaX981_i2c_probe,
	.remove   = qmaX981_i2c_remove,
	.suspend  = qmaX981_suspend,
	.resume   = qmaX981_resume,
	.id_table = qmaX981_id,
};

static int __init qmaX981_i2c_init(void)
{
	GSE_LOG("%s accelerometer driver: init\n", QMAX981_DEV_NAME);

	return i2c_add_driver(&qmaX981_driver);
}

static void __exit qmaX981_i2c_exit(void)
{
	GSE_LOG("%s accelerometer driver exit\n", QMAX981_DEV_NAME);

	i2c_del_driver(&qmaX981_driver);
}


module_init(qmaX981_i2c_init);
module_exit(qmaX981_i2c_exit);

MODULE_DESCRIPTION("qmaX981 accelerometer driver");
MODULE_AUTHOR("QST-inc");
MODULE_LICENSE("GPL");
MODULE_VERSION(QMAX981_DEV_VERSION);
