#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x0x_ts.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/gpio.h>


static struct i2c_client *this_client;

#define CONFIG_FT5X0X_MULTITOUCH 1

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];              //x coordinate
	u16 au16_y[CFG_MAX_TOUCH_POINTS];              //y coordinate
	u8  au8_touch_event[CFG_MAX_TOUCH_POINTS];     //touch event:  0 -- down; 1-- contact; 2 -- contact
	u8  au8_finger_id[CFG_MAX_TOUCH_POINTS];       //touch ID
	u16 pressure;
	u8  touch_point;
};

struct ft5x0x_ts_data {
	struct input_dev		*input_dev;
	struct i2c_client		*client;
	struct ts_event			event;
	struct work_struct		pen_event_work;
	struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend    early_suspend;
#endif
	int 			irq;
	int 			wake_gpio;
	s32 			irq_is_disable;
	spinlock_t 		irq_lock;
	u16 abs_x_max;
	u16 abs_y_max;
};




#if CFG_SUPPORT_TOUCH_KEY
int tsp_keycodes[CFG_NUMOFKEYS] = {
	KEY_MENU,
	KEY_HOME,
	KEY_BACK,
	KEY_SEARCH
};

char *tsp_keyname[CFG_NUMOFKEYS] = {
	"Menu",
	"Home",
	"Back",
	"Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr    = this_client->addr,
			.flags    = 0,
			.len    = length,
			.buf    = txdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

///write register of ft5x0x
static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}



/// read register of ft5x0x
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msgs[2];

	buf[0] = addr;    //register address

	msgs[0].addr = this_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;

	msgs[1].addr = this_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	
	return ret;
}

/// read TP firmware version
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;

	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);

	return(ver);
}

#if CFG_DBG_RAW_DATA_READ

#define FTS_TX_NUM 28
#define FTS_RX_NUM 16
static unsigned char _suc_raw_data[FTS_TX_NUM][FTS_RX_NUM * 2];

int fts_read_line(unsigned char uc_row, unsigned char uc_buf[], unsigned char uc_rx_num)
{
	int        i_ret;

	//set the row address
	i_ret = ft5x0x_write_reg(FT5X0X_REG_GEST_ID, uc_row);
	if (i_ret < 0 ) {
		return -1;
	}

	uc_buf[0] = 0x10;
	i_ret = ft5x0x_i2c_rxdata(uc_buf, uc_rx_num * 2);
	if (i_ret < 0) {
		return -2;
	}

	return 0;
}

int fts_raw_data_test(void)
{
	int i, j, i_test;
	unsigned short uc_data;

	msleep(200);  //make sure the CTP already start up normally

	ft5x0x_write_reg(FT5X0X_REG_DEVIDE_MODE, 0x40); //switch to factory mode 

	msleep(100);  //make sure already enter factory mode

	printk("[TSP] raw data test start ----------------------- \n");

	for (i_test = 0; i_test < 10; i_test ++) {
		printk("frame %d *********\n", i_test + 1);
		ft5x0x_write_reg(FT5X0X_REG_DEVIDE_MODE, 0xC0); //trigger start raw data scan bit
		msleep(10);
		for ( i = 0; i < FTS_TX_NUM; i ++) {
			msleep(1);
			fts_read_line(i, _suc_raw_data[i], FTS_RX_NUM);
			for ( j = 0; j< FTS_RX_NUM; j++) {
				uc_data = _suc_raw_data[i][2*j];
				uc_data <<= 8;
				uc_data |= _suc_raw_data[i][2*j + 1];
				printk("%5d ", uc_data);
			}
			printk("\n");
		}

		msleep(30);
	}

	printk("[TSP] raw data test end ----------------------- \n");
	return 0;
}

#endif




#if CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
	int i;
	int key_id;

	if ( y < 894 && y > 834) {
		key_id = 1;
	}
	else if ( y < 638 && y > 578) {
		key_id = 0;
	}
	else if ( y < 382 && y > 322)
	{
		key_id = 2;
	}  
	else if (y < 126 && y > 36) {
		key_id = 3;
	}
	else {
		key_id = 0xf;
	}

	for(i = 0; i <CFG_NUMOFKEYS; i++ ) {
		if(tsp_keystatus[i]) {
			input_report_key(dev, tsp_keycodes[i], 0);

			printk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);

			tsp_keystatus[i] = KEY_RELEASE;
		}
		else if( key_id == i ) {
			if( touch_event == 0) { // detect
				input_report_key(dev, tsp_keycodes[i], 1);
				printk( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
				tsp_keystatus[i] = KEY_PRESS;
			}
		}
	}

	return 0;
}    
#endif



/// read touch point information
static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[CFG_POINT_READ_BUF] = {0};
	int ret = -1;
	int i;
	
	ret = ft5x0x_i2c_rxdata(buf, CFG_POINT_READ_BUF);
	if (ret < 0) {
		printk("%s i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07; 

	
	if (event->touch_point > CFG_MAX_TOUCH_POINTS) {
		event->touch_point = CFG_MAX_TOUCH_POINTS;
	}

	//printk("touch_point = %d\n", event->touch_point);
/*
	for (i = 0; i < CFG_POINT_READ_BUF; i++) {
		printk("0x%x ", buf[i]);
	}
	printk("\n");
*/

	for (i = 0; i < event->touch_point; i++) {
		event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
		event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
		event->au8_touch_event[i] = buf[0x3 + 6*i] >> 6;
		event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
		//printk("%d, %d\n", event->au16_x[i], event->au16_y[i]);
	}

	event->pressure = PRESS_WIDTH;

	return 0;
}


static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	
#if FT5X0X_MULTI_TOUCH
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif

	input_sync(data->input_dev);
}

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	int i;

	for (i  = 0; i < event->touch_point; i++) {
		
		if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y) {
			// LCD view area
#if FT5X0X_MULTI_TOUCH
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
			/* TD_STATUS touch event
							00b: put down
							01b: put up
							10b: contact
							11b: no event
			*/
			/// event->au8_touch_event[i]== 1   => event->touch_point = 0
			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2) {
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			}
			else {
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			}
#else
			if (event->touch_point == 1) {
				/// report order for support tslib 
				input_report_abs(data->input_dev, ABS_X, event->au16_x[i]);
				input_report_abs(data->input_dev, ABS_Y, event->au16_y[i]);
				input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
				input_report_key(data->input_dev, BTN_TOUCH, 1);	///touch type key
				
				dev_dbg(&this_client->dev,"x = %d y = %d \n", event->au16_x[i], event->au16_y[i]);
			}
#endif
		}
		else { //maybe the touch key area
#if CFG_SUPPORT_TOUCH_KEY
			if (event->au16_x[i] >= SCREEN_MAX_X) {
				//printk("key x = %d\n", event->au16_y[i]);
				ft5x0x_touch_key_process(data->input_dev, event->au16_x[i],
											event->au16_y[i], event->au8_touch_event[i]);
			}
#endif
		}
#if FT5X0X_MULTI_TOUCH
		input_mt_sync(data->input_dev);
#endif
	}
	input_sync(data->input_dev);

	if (event->touch_point == 0) {
		ft5x0x_ts_release();
	}
}



static void ft5x0x_irq_disable(struct ft5x0x_ts_data *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable) {
		ts->irq_is_disable = 1; 
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


static void ft5x0x_irq_enable(struct ft5x0x_ts_data *ts)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	int ret = -1;

	ft5x0x_ts = container_of(work, struct ft5x0x_ts_data, pen_event_work);

	ret = ft5x0x_read_data();
	if (ret == 0) {
		ft5x0x_report_value();
	}

	ft5x0x_irq_enable(ft5x0x_ts);
}


static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	
	ft5x0x_irq_disable(ft5x0x_ts);
	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}



#ifdef CONFIG_HAS_EARLYSUSPEND

static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	ft5x0x_ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);


	printk("==ft5x0x_ts_suspend=\n");
	disable_irq(this_client->irq);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	flush_workqueue(ft5x0x_ts->ts_workqueue);
	// ==set mode ==
	ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	ft5x0x_ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk("==ft5x0x_ts_resume=\n");
	// wake the mode
	gpio_direction_output(ft5x0x_ts->wake_gpio, 0); //set wake = 0,base on system
	msleep(100);
	gpio_direction_output(ft5x0x_ts->wake_gpio, 1); //set wake = 1,base on system
	msleep(100);
	enable_irq(this_client->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND



static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
#endif

#if CFG_SUPPORT_TOUCH_KEY
	int i;
#endif

	printk("[FTS] ft5x0x_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

//#ifdef CONFIG_OF	/* device tree support */
//		if (np) {
//			ft5x0x_ts->wake_gpio = of_get_named_gpio(np, "ft5x0x,rst-gpio", 0);

//		}
//#else
		ft5x0x_ts->wake_gpio = GTP_WAKE_PORT;

//#endif

	err = gpio_request(ft5x0x_ts->wake_gpio, "GTP wake gpio");
	if (err < 0) {
		dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d",(s32)ft5x0x_ts->wake_gpio, err);
		err = -ENODEV;
		goto exit_alloc_data_failed;
	}

	/// Reset ft5x0x
	gpio_direction_output(ft5x0x_ts->wake_gpio, 0);
	msleep(2);
	gpio_direction_output(ft5x0x_ts->wake_gpio, 1);

	this_client = client;

	ft5x0x_ts->client = client;
	ft5x0x_ts->irq = client->irq;
	
	i2c_set_clientdata(client, ft5x0x_ts);

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	err = request_irq(ft5x0x_ts->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", 
													ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(ft5x0x_ts->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

#if FT5X0X_MULTI_TOUCH
	/* android multi touch*/
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,  0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,  0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, PRESS_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);
#else

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_WIDTH, 0, 0);
#endif

#if CFG_SUPPORT_TOUCH_KEY
	/// setup key code area
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_dev->keycode = tsp_keycodes;
	for(i = 0; i < CFG_NUMOFKEYS; i++) {
		input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
		tsp_keystatus[i] = KEY_RELEASE;
	}
#endif

	input_dev->name = FT5X0X_NAME;	///dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
				"ft5x0x_ts_probe: failed to register input device: %s\n",
				dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume    = ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	msleep(150);  //make sure CTP already finish startup process

	//get some register information
	uc_reg_value = ft5x0x_read_fw_ver();
	printk("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	ft5x0x_read_reg(FT5X0X_REG_PERIODACTIVE, &uc_reg_value);
	printk("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
	ft5x0x_read_reg(FT5X0X_REG_THGROUP, &uc_reg_value);
	printk("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

 

#if CFG_DBG_RAW_DATA_READ
	fts_raw_data_test();
#endif


	enable_irq(ft5x0x_ts->irq);

	printk("[FTS] ==probe over =\n");
	
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
exit_irq_request_failed:
	//exit_platform_data_null:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	printk("==ft5x0x_ts_remove=\n");
	ft5x0x_ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);


#ifdef CONFIG_OF
static const struct of_device_id ft5x0x_of_match[] = {
	{ .compatible = FT5X0X_NAME },
};

MODULE_DEVICE_TABLE(of, ft5x0x_of_match);
#endif

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= ft5x0x_ts_remove,
	.id_table	= ft5x0x_ts_id,
	.driver		= {
		.name  	= FT5X0X_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ft5x0x_of_match),
	},
};


module_i2c_driver(ft5x0x_ts_driver);

MODULE_AUTHOR("peixiuhui@163.com");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");

