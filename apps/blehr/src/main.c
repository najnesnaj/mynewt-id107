/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include "os/mynewt.h"
#include "console/console.h"
#include "config/config.h"
#include "sensor/sensor.h"
#include "sensor/light.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "blehr_sens.h"


#include <si1143/si1143.h>
//#include <kx022/kx022.h>
#include <time.h>
#include <compile_time.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "os/os_time.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#include "hal/hal_spi.h"
#include "ssd1306/ssd1306.h"

#define ST 4      // Sampling time in s. WARNING: if you change ST, then you MUST recalcuate the sum_X2 parameter below!
#define FS 25     // Sampling frequency in Hz. WARNING: if you change FS, then you MUST recalcuate the sum_X2 parameter below!




#define READ_SENSOR_INTERVAL (OS_TICKS_PER_SEC/32) // take 32 samples per second 
// for a low 30 beats per minute, you have to sample 2 seconds which equals 64 samples which corresponds to the displaywidth 



#define MAX_SAMPLES ST*FS 

#define TYPICAL_HR 60
#define MAX_HR 125
#define MIN_HR 40

#define FS60  1500  // FS*60  Conversion factor for heart rate from bps to bpm
const int32_t LOWEST_PERIOD = FS60/MAX_HR; // Minimal distance between peaks
const int32_t HIGHEST_PERIOD = FS60/MIN_HR; // Maximal distance between peaks
const int32_t INIT_INTERVAL = FS60/TYPICAL_HR; // Seed value for heart rate determination routine

const float min_autocorrelation_ratio = 0.5;

const float mean_X = (float)(MAX_SAMPLES-1)/2.0; // Mean value of the set of integers from 0 to BUFFER_SIZE-1. For ST=4 and FS=25 it's equal to 49.5
const float sum_X2 = 83325; // WARNING: you MUST recalculate this sum if you changed either ST or FS above!


#define MAX_HR 125  // Maximal heart rate. To eliminate erroneous signals, calculated HR should never be greater than this number.
#define MIN_HR 40   // Minimal heart rate. To eliminate erroneous signals, calculated HR should never be lower than this number.

#define MY_SENSOR_DEVICE_HEART "si1143_0"
#define READ_CB 2

static struct os_callout sensor_callout;
struct os_timeval os_time;
struct tm  time_struct;
static volatile int g_task1_loops;

static struct sensor *my_sensor;




/* For LED toggling */
int g_led_pin,g_front_button_pin,g_side_button_pin;


unsigned long beats[MAX_SAMPLES+1];
int teller;


struct log blehr_log;

static bool notify_state;

static const char *device_name = "blehr_sensor";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;

/* Sending notify data timer */
static struct os_callout blehr_tx_timer;

/* Variable to simulate heart beats */
static uint8_t heartrate = 90; // this is the heartrate transmitted over bluetooth

/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
	static void
blehr_advertise(void)
{
	struct ble_gap_adv_params adv_params;
	struct ble_hs_adv_fields fields;
	int rc;

	/*
	 *  Set the advertisement data included in our advertisements:
	 *     o Flags (indicates advertisement type and other general info)
	 *     o Advertising tx power
	 *     o Device name
	 */
	memset(&fields, 0, sizeof(fields));

	/*
	 * Advertise two flags:
	 *      o Discoverability in forthcoming advertisement (general)
	 *      o BLE-only (BR/EDR unsupported)
	 */
	fields.flags = BLE_HS_ADV_F_DISC_GEN |
		BLE_HS_ADV_F_BREDR_UNSUP;

	/*
	 * Indicate that the TX power level field should be included; have the
	 * stack fill this value automatically.  This is done by assigning the
	 * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
	 */
	fields.tx_pwr_lvl_is_present = 1;
	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	fields.name = (uint8_t *)device_name;
	fields.name_len = strlen(device_name);
	fields.name_is_complete = 1;

	rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		BLEHR_LOG(ERROR, "error setting advertisement data; rc=%d\n", rc);
		return;
	}

	/* Begin advertising */
	memset(&adv_params, 0, sizeof(adv_params));
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
			&adv_params, blehr_gap_event, NULL);
	if (rc != 0) {
		BLEHR_LOG(ERROR, "error enabling advertisement; rc=%d\n", rc);
		return;
	}
}

	static void
blehr_tx_hrate_stop(void)
{
	os_callout_stop(&blehr_tx_timer);
}

/* Reset heartrate measurment */
	static void
blehr_tx_hrate_reset(void)
{
	int rc;

	rc = os_callout_reset(&blehr_tx_timer, OS_TICKS_PER_SEC);
	assert(rc == 0);
}

/* This functions simulates heart beat and notifies it to the client */
	static void
blehr_tx_hrate(struct os_event *ev)
{
	static uint8_t hrm[2];
	int rc;
	struct os_mbuf *om;

	if (!notify_state) {
		blehr_tx_hrate_stop();
		//		heartrate = 90;
		return;
	}

	hrm[0] = 0x06; /* contact of a sensor */
	hrm[1] = heartrate; /* storing dummy data */

	/* Simulation of heart beats */
	//	heartrate++;
	//	if (heartrate == 160) {
	//		heartrate = 90;
	//	}

	om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));

	rc = ble_gattc_notify_custom(notify_state, hrs_hrm_handle, om);

	assert(rc == 0);
	blehr_tx_hrate_reset();
}

	static int
blehr_gap_event(struct ble_gap_event *event, void *arg)
{
	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT:
			/* A new connection was established or a connection attempt failed */
			BLEHR_LOG(INFO, "connection %s; status=%d\n",
					event->connect.status == 0 ? "established" : "failed",
					event->connect.status);

			if (event->connect.status != 0) {
				/* Connection failed; resume advertising */
				blehr_advertise();
			}
			break;

		case BLE_GAP_EVENT_DISCONNECT:
			BLEHR_LOG(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

			/* Connection terminated; resume advertising */
			blehr_advertise();
			break;

		case BLE_GAP_EVENT_ADV_COMPLETE:
			BLEHR_LOG(INFO, "adv complete\n");
			blehr_advertise();
			break;

		case BLE_GAP_EVENT_SUBSCRIBE:
			BLEHR_LOG(INFO, "subscribe event; cur_notify=%d\n value handle; "
					"val_handle=%d\n",
					event->subscribe.cur_notify,
					hrs_hrm_handle);
			if (event->subscribe.attr_handle == hrs_hrm_handle) {
				notify_state = event->subscribe.cur_notify;
				blehr_tx_hrate_reset();
			} else if (event->subscribe.attr_handle != hrs_hrm_handle) {
				notify_state = event->subscribe.cur_notify;
				blehr_tx_hrate_stop();
			}
			break;

		case BLE_GAP_EVENT_MTU:
			BLEHR_LOG(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
					event->mtu.conn_handle,
					event->mtu.value);
			break;

	}

	return 0;
}

	static void
blehr_on_sync(void)
{
	int rc;

	/* Use privacy */
	rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
	assert(rc == 0);

	/* Begin advertising */
	blehr_advertise();
}




/**
 * main
 *
 * The main task for the project. This function initializes the packages, calls
 * init_tasks to initialize additional tasks (and possibly other objects),
 * then starts serving events from default event queue.
 *
 * @return int NOTE: this function should never return!
 */

/**
 * SI1143 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
	int
stop_si1143_sensor(void)
{

	//this shuts off the green leds
	struct sensor_itf *itf;
	struct si1143 *si1143;
	struct os_dev *dev;
	//struct sensor *heart_sensor;
	//si1143->sensor = sensor_mgr_find_next_bydevname(MY_SENSOR_DEVICE_HEART, NULL);


	dev = (struct os_dev *) os_dev_open(MY_SENSOR_DEVICE_HEART, OS_TIMEOUT_NEVER, NULL);
	si1143=(struct si1143 *)dev;
	itf = SENSOR_GET_ITF(&(si1143->sensor));
	//	itf = SENSOR_GET_ITF(&(heart_sensor));

	si1143_enable(itf, 0);
	os_dev_close(dev);
	return 0;
}


	int
start_si1143_sensor(void)
{

	//this shuts off the green leds
	struct sensor_itf *itf;
	struct si1143 *si1143;
	struct os_dev *dev;
	dev = (struct os_dev *) os_dev_open("si1143_0", OS_TIMEOUT_NEVER, NULL);
	si1143=(struct si1143 *)dev;
	itf = SENSOR_GET_ITF(&(si1143->sensor));

	si1143_enable(itf, 1);
	os_dev_close(dev);
	return 0;
}


float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag) 
	/**
	 * \brief        Autocorrelation function
	 * \par          Details
	 *               Compute autocorrelation sequence's n_lag's element for a given series pn_x
	 *               Robert Fraczkiewicz, 12/21/2017
	 * \retval       Autocorrelation sum
	 */
{
	int16_t i, n_temp=n_size-n_lag;
	float sum=0.0,*pn_ptr;
	if(n_temp<=0) return sum;
	for (i=0,pn_ptr=pn_x; i<n_temp; ++i,++pn_ptr) {
		sum += (*pn_ptr)*(*(pn_ptr+n_lag));
	}
	return sum/n_temp;
}


/**
 * * \brief        Coefficient beta of linear regression 
 * * \par          Details
 * *               Compute directional coefficient, beta, of a linear regression of pn_x against mean-centered
 * *               point index values (0 to BUFFER_SIZE-1). xmean must equal to (BUFFER_SIZE-1)/2! sum_x2 is 
 * *               the sum of squares of the mean-centered index values. 
 * *               Robert Fraczkiewicz, 12/22/2017
 * * \retval       Beta
 * */

float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2)
{
	float x,beta,*pn_ptr;
	beta=0.0;
	for(x=-xmean,pn_ptr=pn_x;x<=xmean;++x,++pn_ptr)
		beta+=x*(*pn_ptr);
	return beta/sum_x2;
}



void rf_signal_periodicity(float *pn_x, int32_t n_size, int32_t *p_last_periodicity, int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio, float aut_lag0, float *ratio)
	/**
	 * \brief        Signal periodicity
	 * \par          Details
	 *               Finds periodicity of the IR signal which can be used to calculate heart rate.
	 *               Makes use of the autocorrelation function. If peak autocorrelation is less
	 *               than min_aut_ratio fraction of the autocorrelation at lag=0, then the input 
	 *               signal is insufficiently periodic and probably indicates motion artifacts.
	 *               Robert Fraczkiewicz, 01/07/2018
	 * \retval       Average distance between peaks
	 */
{
	int32_t n_lag;
	float aut,aut_left,aut_right,aut_save;
	bool left_limit_reached=false;
	// Start from the last periodicity computing the corresponding autocorrelation
	n_lag=*p_last_periodicity;
	aut_save=aut=rf_autocorrelation(pn_x, n_size, n_lag);
	// Is autocorrelation one lag to the left greater?
	aut_left=aut;
	do {
		aut=aut_left;
		n_lag--;
		aut_left=rf_autocorrelation(pn_x, n_size, n_lag);
	} while(aut_left>aut && n_lag>n_min_distance);
	// Restore lag of the highest aut
	if(n_lag==n_min_distance) {
		left_limit_reached=true;
		n_lag=*p_last_periodicity;
		aut=aut_save;
	} else n_lag++;
	if(n_lag==*p_last_periodicity) {
		// Trip to the left made no progress. Walk to the right.
		aut_right=aut;
		do {
			aut=aut_right;
			n_lag++;
			aut_right=rf_autocorrelation(pn_x, n_size, n_lag);
		} while(aut_right>aut && n_lag<n_max_distance);
		// Restore lag of the highest aut
		if(n_lag==n_max_distance) n_lag=0; // Indicates failure
		else n_lag--;
		if(n_lag==*p_last_periodicity && left_limit_reached) n_lag=0; // Indicates failure
	}
	*ratio=aut/aut_lag0;
	if(*ratio < min_aut_ratio) n_lag=0; // Indicates failure
	*p_last_periodicity=n_lag;
}




float rf_rms(float *pn_x, int32_t n_size, float *sumsq) 
	/**
	 * \brief        Root-mean-square variation 
	 * \par          Details
	 *               Compute root-mean-square variation for a given series pn_x
	 *               Robert Fraczkiewicz, 12/25/2017
	 * \retval       RMS value and raw sum of squares
	 */
{
	int16_t i;
	float r,*pn_ptr;
	(*sumsq)=0.0;
	for (i=0,pn_ptr=pn_x; i<n_size; ++i,++pn_ptr) {
		r=(*pn_ptr);
		(*sumsq) += r*r;
	}
	(*sumsq)/=n_size; // This corresponds to autocorrelation at lag=0
	return sqrt(*sumsq);
}



//static int read_heartratemeter(struct sensor* sensor, void *arg, void *databuf, sensor_type_t type);

	int
read_light_data(struct sensor* sensor, void *arg, void *databuf, sensor_type_t type)
{
	/*
	 * uint16_t sld_full, sld_if uint32_t sld_lux
	 */
	//	uint32_t test;
	struct sensor_light_data *sld;
	long gemiddelde, jemiddelde;
	unsigned int positie;
	float an_x[MAX_SAMPLES], *ptr_x, ratio;
	float beta_light,x,  f_light_sumsq;
	//	int plot[MAX_SAMPLES];
	static int32_t n_last_peak_interval=INIT_INTERVAL;
	char tmpstr[13];
	//	uint32_t pn_heart_rate;


	//	console_printf("read light intensity\n");
	if (!databuf) {
		return SYS_EINVAL;
	}
	sld = (struct sensor_light_data *)databuf;
	if (sld->sld_lux_is_valid){
		//		console_printf("light = %lu\n", (unsigned long)sld->sld_lux); 
		beats[teller]=(long)sld->sld_lux/100; //divide by 100 cause values are really big -- overflow?
	}
	teller++;
	if (teller > MAX_SAMPLES){ 
		teller=0;
		gemiddelde=0;
		//remove DC component
		for (positie=0;positie < MAX_SAMPLES; ++positie)
			gemiddelde += beats[positie]; 
		jemiddelde=(long)(gemiddelde/100); //somehow dividing by MAX_SAMPLES does not work

		for (positie=0, ptr_x=an_x;positie < MAX_SAMPLES; ++positie, ++ptr_x)
		{
			an_x[positie] = beats[positie] - jemiddelde;
			//*ptr_x = beats[positie] - jemiddelde;
			//			console_printf("value = %ld   %ld   %ld\n", (beats[positie]-jemiddelde), beats[positie], jemiddelde); 
		}
		//remove linear trend (baseline leveling)

		beta_light = rf_linear_regression_beta(an_x, mean_X, sum_X2);
		for (positie=0, x=-mean_X, ptr_x=an_x;positie < MAX_SAMPLES; ++positie, ++x, ++ptr_x)
		{
		console_printf("an_x = %s  \n", sensor_ftostr(an_x[positie], tmpstr, 13));	
		console_printf("beta_light = %s \n",  sensor_ftostr((beta_light), tmpstr, 13));	
			*ptr_x -= beta_light*x;
	                //	console_printf("value = %f %f \n", beta_light*x, an_x[positie]); 
			//	plot[positie] = 
		}

		//f_x_ac = rf_rms(an_x, MAX_SAMPLES, &f_light_sumsq); 
		rf_rms(an_x, MAX_SAMPLES, &f_light_sumsq); 
		//f_x_ac = 0; //avoid compiler warning jj variable used in spo2 calculation

		rf_signal_periodicity(an_x, MAX_SAMPLES, &n_last_peak_interval, LOWEST_PERIOD, HIGHEST_PERIOD, min_autocorrelation_ratio, f_light_sumsq, &ratio);
		heartrate = 3; // initialise
		if(n_last_peak_interval !=0 )
			heartrate = (uint8_t)(FS60/n_last_peak_interval); 
		else
			heartrate = 2; //unable to calculate because signal is aperiodic




		console_printf("heartbeat = %d\n", heartrate); 


	}

	//	test = sad->sld_lux; 
	return 0;
}



	int
config_si1143_sensor(void)
{
#if MYNEWT_VAL(SI1143_ONB)
	//int rc;
	struct os_dev *dev;
	struct si1143_cfg cfg;

	dev = (struct os_dev *) os_dev_open("si1143_0", OS_TIMEOUT_NEVER, NULL);
	assert(dev != NULL);

	memset(&cfg, 0, sizeof(cfg));

	cfg.mask = SENSOR_TYPE_LIGHT;
	cfg.lum_addr = SI1143_ADDR;
	cfg.intensity_measurement = 99; //random number
	si1143_config((struct si1143 *)dev, &cfg);
	//    SYSINIT_PANIC_ASSERT(rc == 0); //jj this halts the system 
	//    assert(rc == 0);

	os_dev_close(dev);
#endif
	return 0;
}



/**
 * KX022 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 int
 config_kx022_sensor(void)
 {
#if MYNEWT_VAL(KX022_ONB)
int rc;
struct os_dev *dev;
struct kx022_cfg cfg;

dev = (struct os_dev *) os_dev_open("kx022_0", OS_TIMEOUT_NEVER, NULL);
assert(dev != NULL);

memset(&cfg, 0, sizeof(cfg));

cfg.lc_s_mask = SENSOR_TYPE_ACCELEROMETER;
cfg.rate = KX022_ACC_OUTPUT_RATE_100_HZ;
cfg.range = KX022_RANGE_4G;
cfg.acc_addr = KX022_ADDR_H;
// cfg.lc_pull_up_disc = 1;

rc = kx022_config((struct kx022 *)dev, &cfg);
SYSINIT_PANIC_ASSERT(rc == 0);

os_dev_close(dev);
#endif
return 0;
}
*/






	static void
timer_ev_cb(struct os_event *ev)
{


	assert(ev != NULL);

	//	console_printf("in timer call sensor read\n");
	//start_si1143_sensor();
	my_sensor = sensor_mgr_find_next_bydevname("si1143_0", NULL);
	sensor_read(my_sensor, SENSOR_TYPE_LIGHT, read_light_data, (void *)READ_CB, OS_TIMEOUT_NEVER); 
	//stop_si1143_sensor();
	//   sensor_read(my_sensor, SENSOR_TYPE_ACCELEROMETER, read_accelerometer,
	//   (void *)READ_CB, OS_TIMEOUT_NEVER);
	os_callout_reset(&sensor_callout, READ_SENSOR_INTERVAL);
	return;
}


	static void
init_timer(void)
{
	console_printf("init  timer \n");
	os_callout_init(&sensor_callout, os_eventq_dflt_get(),
			timer_ev_cb, NULL);

	os_callout_reset(&sensor_callout, READ_SENSOR_INTERVAL);
	return;
}




/*
 * main
 *
 * The main task for the project. This function initializes the packages,
 * then starts serving events from default event queue.
 *
 * @return int NOTE: this function should never return!
 */
	int
main(void)
{
	int rc;
	int nouse;
	char str[12];
	int menu=1;
	/* Initialize OS */
	//	time_t epoch;
	//	struct tm ts ;
	sysinit();
	init_timer();
	//after a powercut, dead battery, these sensors do not get initialised, that's why config_si1143_sensor appears here....
	nouse=config_si1143_sensor();
	start_si1143_sensor();
	//	nouse=config_kx022_sensor();
	nouse++;
	//	nouse=stop_si1143_sensor();
	//	nouse++;

	g_led_pin = LED_BLINK_PIN;
	g_front_button_pin = FRONT_BUTTON_PIN;
	g_side_button_pin = SIDE_BUTTON_PIN;

	hal_gpio_init_out(g_led_pin, 0);
	hal_gpio_init_in(g_front_button_pin,HAL_GPIO_PULL_NONE);
	hal_gpio_init_in(g_side_button_pin,HAL_GPIO_PULL_UP);

	//set time
	//
	struct os_timeval time_now = { __TIME_UNIX__, 0 }; 
	struct os_timezone my_timezone = { 60, 0 };
	os_settimeofday(&time_now,&my_timezone);
	//init oled
	ssd1306_Init();

	ssd1306_Fill(Black);
	ssd1306_SetCursor(0,0);
	ssd1306_WriteString("Setup",Font_7x10,White);
	ssd1306_SetCursor(0,11);
	ssd1306_WriteString("Done",Font_7x10,White);
	ssd1306_UpdateScreen();
	os_time_delay(OS_TICKS_PER_SEC/2);


	/* Initialize the blehr log */
	log_register("blehr_sens_log", &blehr_log, &log_console_handler, NULL,
			LOG_SYSLEVEL);

	/* Initialize the NimBLE host configuration */
	log_register("blehr_sens", &ble_hs_log, &log_console_handler, NULL,
			LOG_SYSLEVEL);
	ble_hs_cfg.sync_cb = blehr_on_sync;

	os_callout_init(&blehr_tx_timer, os_eventq_dflt_get(),
			blehr_tx_hrate, NULL);

	rc = gatt_svr_init();
	assert(rc == 0);

	/* Set the default device name */
	rc = ble_svc_gap_device_name_set(device_name);
	assert(rc == 0);

	//my_sensor = sensor_mgr_find_next_bydevname(MY_SENSOR_DEVICE_HEART, NULL);
	//my_sensor = sensor_mgr_find_next_bydevname("si1143_0", NULL);

	//sensor_read(my_sensor, SENSOR_TYPE_LIGHT, read_light_data, (void *)READ_CB, OS_TIMEOUT_NEVER); 

	/* As the last thing, process events from default event queue */
	while (1) {
		ssd1306_Fill(Black);
		//display heartrate
		//
		switch(menu){
			case 1:
			       {
				       sprintf(str, "%d", heartrate);
				       ssd1306_SetCursor(0,0);
				       ssd1306_WriteString(str,Font_7x10,White);
				       ssd1306_SetCursor(0,11);
				       ssd1306_WriteString("BPM",Font_7x10,White);
			       }	
			case 2:
			       {
				       ssd1306_Fill(Black);
				       //display menu
			       }	
			default :
			       {
				       //do something	
			       }

		}
		//display time
		//
		//		os_gettimeofday(&os_time,NULL);        
		//		epoch=os_time.tv_sec;
		//		ts = *localtime(&epoch);
		/* I get this nasty error when using strftime
		 *Error: /usr/lib/gcc/arm-none-eabi/5.4.1/../../../arm-none-eabi/lib/armv6-m/libg.a(lib_a-strtol.o): In function `strtol':
		 /build/newlib-XAuz1P/newlib-2.4.0.20160527/build/arm-none-eabi/armv6-m/newlib/libc/stdlib/../../../../../../newlib/libc/stdlib/strtol.c:222: multiple definition of `strtol'
		 /usr/src/myproj/bin/targets/id107_hr/app/libc/baselibc/libc_baselibc.a(strtol.o):/usr/src/myproj/repos/apache-mynewt-core/libc/baselibc/src/templates/strtox.c.template:12: first defined here
		 *
		 */



		//		strftime(str, sizeof(str), "%H:%M:%S", &ts);
		/*		ssd1306_SetCursor(0,11);
				ssd1306_WriteString(str,Font_7x10,White);   
				strftime(str, sizeof(str), "%d.%m.%y", &ts);   
				ssd1306_SetCursor(0,21);
				ssd1306_WriteString(str,Font_7x10,White);
				*/
		ssd1306_UpdateScreen();
		os_time_delay(OS_TICKS_PER_SEC/10);


		//read button
		//
		if (hal_gpio_read(g_front_button_pin) == 1)
		{
			/* Pulse the motor */
			hal_gpio_write(g_led_pin,1);
			os_time_delay(OS_TICKS_PER_SEC/20);
			hal_gpio_write(g_led_pin,0);
			if (menu==1) menu=2; //toggle menu
			else 
				menu=1;
		}

		if (hal_gpio_read(g_side_button_pin) == 0)
		{
			/* Pulse the motor */
			hal_gpio_write(g_led_pin,1);
			os_time_delay(OS_TICKS_PER_SEC/2);
			hal_gpio_write(g_led_pin,0);
		}


		os_eventq_run(os_eventq_dflt_get());
	}
	return 0;
}

