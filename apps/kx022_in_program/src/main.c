#include "sysinit/sysinit.h"
#include "console/console.h"
#include "os/os.h"
#include <defs/error.h>
#include <sensor/sensor.h>
#include <sensor/accel.h>
#include <kx022/kx022.h>



#define READ_SENSOR_INTERVAL (5 * OS_TICKS_PER_SEC)
#define MY_SENSOR_DEVICE "kx022_0"
#define MY_SENSOR_POLL_TIME 2000

//static struct os_callout sensor_callout;

static struct sensor *my_sensor;




#define LISTENER_CB 1
#define READ_CB 2

static int read_accelerometer(struct sensor* sensor, void *arg, void *databuf, sensor_type_t type);

	static int
read_accelerometer(struct sensor* sensor, void *arg, void *databuf, sensor_type_t type)
{

	char tmpstr[13];
	struct sensor_accel_data *sad;
	console_printf("read_accelerometer \n");

	if (!databuf) {
		return SYS_EINVAL;

	}
	sad = (struct sensor_accel_data *)databuf;

	if (!sad->sad_x_is_valid ||
			!sad->sad_y_is_valid ||
			!sad->sad_z_is_valid) {

		return SYS_EINVAL;
	}

	console_printf("%s: [ secs: %ld usecs: %d cputime: %u ]\n",
			((int)arg == LISTENER_CB) ? "LISTENER_CB" : "READ_CB",
			(long int)sensor->s_sts.st_ostv.tv_sec,
			(int)sensor->s_sts.st_ostv.tv_usec,
			(unsigned int)sensor->s_sts.st_cputime);

	console_printf("x = %s ", sensor_ftostr(sad->sad_x, tmpstr, 13));
	console_printf("y = %s ", sensor_ftostr(sad->sad_y, tmpstr, 13));
	console_printf("z = %s\n\n", sensor_ftostr(sad->sad_z, tmpstr, 13));
	return 0;
}

/*
	static int
sensors_test_config_kx022(void)
{
	int rc;
	struct os_dev *dev;
	struct kx022_cfg bcfg;

	dev = (struct os_dev *) os_dev_open("kx022_0", OS_TIMEOUT_NEVER, NULL);
	assert(dev != NULL);

memset(&bcfg, 0, sizeof(bcfg));
	bcfg.lc_s_mask = SENSOR_TYPE_ACCELEROMETER;
	bcfg.rate = KX022_ACC_OUTPUT_RATE_100_HZ;
	bcfg.range =  KX022_RANGE_2G;
	bcfg.acc_addr =  KX022_ADDR_H;

	rc = kx022_config((struct kx022 *) dev, &bcfg);

	os_dev_close(dev);
	return rc;
}

*/

/*

   static void
   timer_ev_cb(struct os_event *ev)
   {


   assert(ev != NULL);

   console_printf("in timer call sensor read\n");
   sensor_read(my_sensor, SENSOR_TYPE_ACCELEROMETER, read_accelerometer,
   (void *)READ_CB, OS_TIMEOUT_NEVER);
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

*/


/**
 * Depending on the type of package, there are different
 * compilation rules for this directory.  This comment applies
 * to packages of type "app."  For other types of packages,
 * please view the documentation at http://mynewt.apache.org/.
 *
 * Put source files in this directory.  All files that have a *.c
 * ending are recursively compiled in the src/ directory and its
 * descendants.  The exception here is the arch/ directory, which
 * is ignored in the default compilation.
 *
 * The arch/<your-arch>/ directories are manually added and
 * recursively compiled for all files that end with either *.c
 * or *.a.  Any directories in arch/ that don't match the
 * architecture being compiled are not compiled.
 *
 * Architecture is set by the BSP/MCU combination.
 */

	int
main(int argc, char **argv)
{
//	int rc;

	/* Perform some extra setup if we're running in the simulator. */
#ifdef ARCH_sim
	mcu_sim_parse_args(argc, argv);
#endif

	/* Initialize all packages. */
	sysinit();

	//	rc = sensor_set_poll_rate_ms(MY_SENSOR_DEVICE, MY_SENSOR_POLL_TIME);
	//		console_printf("set poll rate\n");
	//	assert(rc == 0);

	my_sensor = sensor_mgr_find_next_bydevname(MY_SENSOR_DEVICE, NULL);
	sensor_read(my_sensor, SENSOR_TYPE_ACCELEROMETER, read_accelerometer,
			(void *)READ_CB, OS_TIMEOUT_NEVER);
//	console_printf("find device by name\n");

//rc = sensors_test_config_kx022();
//    assert(rc == 0);

	sensor_read(my_sensor, SENSOR_TYPE_ACCELEROMETER, read_accelerometer,
			(void *)READ_CB, OS_TIMEOUT_NEVER);

	//	assert(my_sensor != NULL);
	//	rc = sensor_register_listener(my_sensor, &listener);
	//		console_printf("listener started\n");
	//	assert(rc == 0);

//	init_timer();
	/* As the last thing, process events from default event queue. */
	while (1) {
		//console_printf("test1\n");
		os_eventq_run(os_eventq_dflt_get());
		//console_printf("test2\n");

	}

	return 0;
}
