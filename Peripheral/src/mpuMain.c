/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      main.c
 *       @brief     Test app for eMPL using the Motion Driver DMP image.
 */

/* Includes ------------------------------------------------------------------*/
#include <mpuMain.h>
#include "stm32f4xx.h"
#include "stdio.h"

#include "cmsis_os.h"

#include "uart.h"
#include "i2c.h"
#include "gpio.h"
#include "rgbLed.h"
//#include "board-st_discovery.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "localization.h"
#include "packet.h"
/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)
#define PRINT_COMPASS_HR (0x400)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)  // Can be set between 4 Hz to 1KHz

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100) // Controls the Compass frequency
struct rx_s {
	unsigned char header[3];
	unsigned char cmd;
};
struct hal_s {
	unsigned char lp_accel_mode;
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned char motion_int_mode;
	unsigned long no_dmp_hz;
	unsigned long next_pedo_ms;
	unsigned long next_temp_ms;
	unsigned long next_compass_ms;
	unsigned int report;
	unsigned short dmp_features;
	struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
	signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */

/*						     ^ +Y board
 * 				 _____	     |
 * 				|	  |      |
 *				| UWB |    Z(.)- - -> +X board
 *	  |---------|	  |--------------|
 *	  |			 -----               |
 *	  |								 |
 *	  |								 |
 *	  |								 |
 *	  |								 |
 *	  |		MPU9250				     |
 *	  |		|----|					 |
 *	  |		|	 |					 |
 *	  |		|___.|					 |
 *    |--------------[ USB  ]--------|
 */
static struct platform_data_s gyro_pdata = { // NED
		.orientation = { 0, -1, 0,
						 -1, 0, 0,
						 0, 0, -1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = { // NED
		.orientation = { -1, 0, 0,
						 0, -1, 0,
						 0, 0, 1}


};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
		.orientation = {-1, 0, 0,
				0, 1, 0,
				0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
		.orientation = {-1, 0, 0,
				0,-1, 0,
				0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif


/* Private define ------------------------------------------------------------*/



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t mpuInitComplete = 0;

osThreadId mpuCompassTaskHandle;
osThreadId mpuTemperatureTaskHandle;
osThreadId mpuProcessInterruptTaskHandle;
osThreadId mpuNewDataTaskHandle;

SemaphoreHandle_t mpuInterruptSemHandle;
SemaphoreHandle_t mpuNewDataSemHandle;

unsigned long sensor_timestamp;

/* Private function prototypes -----------------------------------------------*/

int mpu_setup(void);
void mpu_prosess_interrupt(unsigned long sensor_timestamp);

void MpuCompassTask(void const * argument);
void MpuTemperatureTask(void const * argument);
void MpuProcessInterruptTask(void const * argument);
void MpuNewDataTask(void const * argument);


/* ---------------------------------------------------------------------------*/
/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void)
//void read_from_mpl(void)
{
	long msg, data[9];
	int8_t accuracy;
	unsigned long timestamp;
	float float_data[3] = {0};
	char outString[MAX_UART_STRING_LENGTH];

	if (hal.report & PRINT_QUAT)
	{
		if (inv_get_sensor_type_quat(data, &accuracy,
				(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}

	if (hal.report & PRINT_ACCEL) {
		if (inv_get_sensor_type_accel(data, &accuracy,
				(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_ACCEL, data);
//			readINS(data);

	}
	if (hal.report & PRINT_GYRO) {
		if (inv_get_sensor_type_gyro(data, &accuracy,
				(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_GYRO, data);
			//readINS(data);
	}
#ifdef COMPASS_ENABLED
	if (hal.report & PRINT_COMPASS) {
		if (inv_get_sensor_type_compass(data, &accuracy,
				(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_COMPASS, data);
			//readINS(data);
	}
#endif
if (hal.report & PRINT_EULER) {
	if (inv_get_sensor_type_euler(data, &accuracy,
			(inv_time_t*)&timestamp))
		eMPL_send_data(PACKET_DATA_EULER, data);
}
if (hal.report & PRINT_ROT_MAT) {
	if (inv_get_sensor_type_rot_mat(data, &accuracy,
			(inv_time_t*)&timestamp))
		eMPL_send_data(PACKET_DATA_ROT, data);
}
if (hal.report & PRINT_HEADING) {
	if (inv_get_sensor_type_heading(data, &accuracy,
			(inv_time_t*)&timestamp))
		eMPL_send_data(PACKET_DATA_HEADING, data);
}
if (hal.report & PRINT_LINEAR_ACCEL) {
	if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
		//MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
				//		float_data[0], float_data[1], float_data[2]);

		sprintf(outString, "Linear Accel: %7.5f %7.5f %7.5f\r\n",
				float_data[0], float_data[1], float_data[2]);
		uartWriteLineNoOS(outString);

	}
}
if (hal.report & PRINT_GRAVITY_VECTOR) {
	if (inv_get_sensor_type_gravity(float_data, &accuracy,
			(inv_time_t*)&timestamp))
		//MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
		//		float_data[0], float_data[1], float_data[2]);

		sprintf(outString, "Gravity Vector: %7.5f %7.5f %7.5f\r\n",
				float_data[0], float_data[1], float_data[2]);
	uartWriteLineNoOS(outString);

}
if (hal.report & PRINT_PEDO) {
	unsigned long timestamp;
	mpu_get_tick_count(&timestamp);
	if (timestamp > hal.next_pedo_ms) {
		hal.next_pedo_ms = timestamp + PEDO_READ_MS;
		unsigned long step_count, walk_time;
		dmp_get_pedometer_step_count(&step_count);
		dmp_get_pedometer_walk_time(&walk_time);
		MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
				walk_time);
	}
}
if (hal.report & PRINT_COMPASS_HR) {
	send_status_compass();
}

/* Whenever the MPL detects a change in motion state, the application can
 * be notified. For this example, we use an LED to represent the current
 * motion state.
 */
msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
		INV_MSG_NO_MOTION_EVENT);
if (msg) {
	if (msg & INV_MSG_MOTION_EVENT) {
		MPL_LOGI("Motion!\n");
		SetRGBLed(120, 0, 0, 100);
	} else if (msg & INV_MSG_NO_MOTION_EVENT) {
		MPL_LOGI("No motion!\n");
		SetRGBLed(0, 120, 0, 100);
	}
}
}

#ifdef COMPASS_ENABLED
void send_status_compass() {
	char outString[MAX_UART_STRING_LENGTH];
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	//MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
	//		data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	//MPL_LOGI("Accuracy= %d\r\n", accuracy);

	sprintf(outString, "Compass: %7.4f %7.4f %7.4f\r\n",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	uartWriteLineNoOS(outString);

	sprintf(outString, "Accuracy= %d\r\n", accuracy);
	uartWriteLineNoOS(outString);


}
#endif

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
	unsigned char mask = 0, lp_accel_was_on = 0;
	if (hal.sensors & ACCEL_ON)
		mask |= INV_XYZ_ACCEL;
	if (hal.sensors & GYRO_ON) {
		mask |= INV_XYZ_GYRO;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
#ifdef COMPASS_ENABLED
	if (hal.sensors & COMPASS_ON) {
		mask |= INV_XYZ_COMPASS;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
#endif
/* If you need a power transition, this function should be called with a
 * mask of the sensors still enabled. The driver turns off any sensors
 * excluded from this mask.
 */
	mpu_set_sensors(mask);
	mpu_configure_fifo(mask);
	if (lp_accel_was_on) {
		unsigned short rate;
		hal.lp_accel_mode = 0;
		/* Switching out of LP accel, notify MPL of new accel sampling rate. */
		mpu_get_sample_rate(&rate);
		inv_set_accel_sample_rate(1000000L / rate);
	}
}

static void tap_cb(unsigned char direction, unsigned char count)
{
	switch (direction) {
	case TAP_X_UP:
		MPL_LOGI("Tap X+ ");
		break;
	case TAP_X_DOWN:
		MPL_LOGI("Tap X- ");
		break;
	case TAP_Y_UP:
		MPL_LOGI("Tap Y+ ");
		break;
	case TAP_Y_DOWN:
		MPL_LOGI("Tap Y- ");
		break;
	case TAP_Z_UP:
		MPL_LOGI("Tap Z+ ");
		break;
	case TAP_Z_DOWN:
		MPL_LOGI("Tap Z- ");
		break;
	default:
		return;
	}
	MPL_LOGI("x%d\n", count);
	return;
}

static inline void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
	result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
	result = mpu_run_self_test(gyro, accel);
#endif
	if (result == 0x7) {
		MPL_LOGI("Passed!\n");
		MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
				accel[0]/65536.f,
				accel[1]/65536.f,
				accel[2]/65536.f);
		MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
				gyro[0]/65536.f,
				gyro[1]/65536.f,
				gyro[2]/65536.f);
		/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
		/*
		 * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
		 * instead of pushing the cal data to the MPL software library
		 */
		unsigned char i = 0;

		for(i = 0; i<3; i++) {
			gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
			accel[i] *= 2048.f; //convert to +-16G
			accel[i] = accel[i] >> 16;
			gyro[i] = (long)(gyro[i] >> 16);
		}

		mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
		mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
		mpu_set_accel_bias_6050_reg(accel);
#endif
#else
		/* Push the calibrated data to the MPL library.
		 *
		 * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
		unsigned short accel_sens;
		float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
	}
	else {
		if (!(result & 0x1))
			MPL_LOGE("Gyro failed.\n");
		if (!(result & 0x2))
			MPL_LOGE("Accel failed.\n");
		if (!(result & 0x4))
			MPL_LOGE("Compass failed.\n");
	}

}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
	//static signed BaseType_t xHigherPriorityTaskWoken;

	hal.new_gyro = 1;
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	/* Is it time for vATask() to run? */
	//xHigherPriorityTaskWoken = pdFALSE;

	if(mpuInitComplete)
	{
		xSemaphoreGiveFromISR(mpuInterruptSemHandle, NULL);
	}

}
/*******************************************************************************/

/**
 * @brief main entry point.
 * @par Parameters None
 * @retval void None
 * @par Required preconditions: None
 */

int mpu_setup(void)
{ 

	inv_error_t result;
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	unsigned long timestamp;
	struct int_param_s int_param;
#ifdef COMPASS_ENABLED
	unsigned short compass_fsr;
#endif

	//board_init();

	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	result = mpu_init(&int_param);
	if (result) {
		MPL_LOGE("Could not initialize gyro.\n");
	}


	/* If you're not using an MPU9150 AND you're not using DMP features, this
	 * function will place all slaves on the primary bus.
	 * mpu_set_bypass(1);
	 */

	result = inv_init_mpl();
	if (result) {
		MPL_LOGE("Could not initialize MPL.\n");
	}

	/* Compute 6-axis and 9-axis quaternions. */
	inv_enable_quaternion();
	inv_enable_9x_sensor_fusion();
	/* The MPL expects compass data at a constant rate (matching the rate
	 * passed to inv_set_compass_sample_rate). If this is an issue for your
	 * application, call this function, and the MPL will depend on the
	 * timestamps passed to inv_build_compass instead.
	 *
	 * inv_9x_fusion_use_timestamps(1);
	 */

	/* This function has been deprecated.
	 * inv_enable_no_gyro_fusion();
	 */

	/* Update gyro biases when not in motion.
	 * WARNING: These algorithms are mutually exclusive.
	 */
	inv_enable_fast_nomot();
	/* inv_enable_motion_no_motion(); */
	/* inv_set_no_motion_time(1000); */

	/* Update gyro biases when temperature changes. */
	inv_enable_gyro_tc();

	/* This algorithm updates the accel biases when in motion. A more accurate
	 * bias measurement can be made when running the self-test (see case 't' in
	 * handle_input), but this algorithm can be enabled if the self-test can't
	 * be executed in your application.
	 *
	 * inv_enable_in_use_auto_calibration();
	 */
#ifdef COMPASS_ENABLED
	/* Compass calibration algorithms. */
	inv_enable_vector_compass_cal();
	inv_enable_magnetic_disturbance();
#endif
	/* If you need to estimate your heading before the compass is calibrated,
	 * enable this algorithm. It becomes useless after a good figure-eight is
	 * detected, so we'll just leave it out to save memory.
	 * inv_enable_heading_from_gyro();
	 */

	/* Allows use of the MPL APIs in read_from_mpl. */
	inv_enable_eMPL_outputs();

	result = inv_start_mpl();
	if (result == INV_ERROR_NOT_AUTHORIZED) {
		while (1) {
			MPL_LOGE("Not authorized.\n");
		}
	}
	if (result) {
		MPL_LOGE("Could not start the MPL.\n");
	}

	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
#ifdef COMPASS_ENABLED
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
	/* Push both gyro and accel data into the FIFO. */
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
	/* The compass sampling rate can be less than the gyro/accel sampling rate.
	 * Use this function for proper power management.
	 */
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
	/* Read back configuration in case it was set improperly. */
	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
	mpu_get_compass_fsr(&compass_fsr);
#endif
	/* Sync driver configuration with MPL. */
	/* Sample rate expected in microseconds. */
	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
	/* The compass rate is independent of the gyro and accel rates. As long as
	 * inv_set_compass_sample_rate is called with the correct value, the 9-axis
	 * fusion algorithm's compass correction gain will work properly.
	 */
	inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
	/* Set chip-to-body orientation matrix.
	 * Set hardware units to dps/g's/degrees scaling factor.
	 */
	inv_set_gyro_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)gyro_fsr<<15);
	inv_set_accel_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
	inv_set_compass_orientation_and_scale(
			inv_orientation_matrix_to_scalar(compass_pdata.orientation),
			(long)compass_fsr<<15);
#endif
	/* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
	hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
	hal.sensors = ACCEL_ON | GYRO_ON;
#endif
	hal.dmp_on = 0;
	hal.report = 0; // Enagle Acc,Gyro and compass
	hal.rx.cmd = 0;
	hal.next_pedo_ms = 0;
	hal.next_compass_ms = 0;
	hal.next_temp_ms = 0;

	/* Compass reads are handled by scheduler. */
	mpu_get_tick_count(&timestamp);

	/* To initialize the DMP:
	 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
	 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
	 * 2. Push the gyro and accel orientation matrix to the DMP.
	 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
	 *    executed unless the corresponding feature is enabled.
	 * 4. Call dmp_enable_feature(mask) to enable different features.
	 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	 * 6. Call any feature-specific control functions.
	 *
	 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
	 * be called repeatedly to enable and disable the DMP at runtime.
	 *
	 * The following is a short summary of the features supported in the DMP
	 * image provided in inv_mpu_dmp_motion_driver.c:
	 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	 * 200Hz. Integrating the gyro data at higher rates reduces numerical
	 * errors (compared to integration on the MCU at a lower sampling rate).
	 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
	 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
	 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
	 * an event at the four orientations where the screen should rotate.
	 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
	 * no motion.
	 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
	 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
	 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
	 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
	 */
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	dmp_register_tap_cb(tap_cb);
	//dmp_register_android_orient_cb(android_orient_cb);
	/*
	 * Known Bug -
	 * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
	 * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
	 * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
	 * there will be a 25Hz interrupt from the MPU device.
	 *
	 * There is a known issue in which if you do not enable DMP_FEATURE_TAP
	 * then the interrupts will be at 200Hz even if fifo rate
	 * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
	 *
	 * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
	 */
//	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
//        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
//        DMP_FEATURE_GYRO_CAL;

	hal.dmp_features = DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL; // minimum features to work

	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	mpu_set_dmp_state(1);
	hal.dmp_on = 1;


	//hal.report = (PRINT_LINEAR_ACCEL | PRINT_GRAVITY_VECTOR | PRINT_COMPASS);
	hal.report = PRINT_ACCEL | PRINT_GYRO | PRINT_COMPASS; // enable readings from the three sensors

	//run_self_test();

#ifdef COMPASS_ENABLED
	osThreadDef(mpuCompassTask, MpuCompassTask, osPriorityNormal, 0, 64);
	mpuCompassTaskHandle = osThreadCreate(osThread(mpuCompassTask), &sensor_timestamp);
#endif

	osThreadDef(mpuTemperatureTask, MpuTemperatureTask, osPriorityNormal, 0, 64);
	mpuTemperatureTaskHandle = osThreadCreate(osThread(mpuTemperatureTask), &sensor_timestamp);

	osThreadDef(mpuNewDataTask, MpuNewDataTask, osPriorityNormal, 0, 256);
	mpuNewDataTaskHandle = osThreadCreate(osThread(mpuNewDataTask), NULL);


	mpuNewDataSemHandle = xSemaphoreCreateBinary();

	osThreadDef(mpuProcessInterruptTask, MpuProcessInterruptTask, osPriorityNormal, 0, 128);
	mpuProcessInterruptTaskHandle = osThreadCreate(osThread(mpuProcessInterruptTask), &sensor_timestamp);


	mpuInterruptSemHandle = xSemaphoreCreateBinary();

	mpuInitComplete = 1;

	return 0;
}


void mpu_prosess_interrupt(unsigned long sensor_timestamp){

	int new_data = 0;

	if (hal.new_gyro && hal.lp_accel_mode) {
		short accel_short[3];
		long accel[3];
		mpu_get_accel_reg(accel_short, &sensor_timestamp);
		accel[0] = (long)accel_short[0];
		accel[1] = (long)accel_short[1];
		accel[2] = (long)accel_short[2];
		inv_build_accel(accel, 0, sensor_timestamp);
		new_data = 1;
		hal.new_gyro = 0;
	} else if (hal.new_gyro && hal.dmp_on) {
		short gyro[3], accel_short[3], sensors;
		unsigned char more;
		long accel[3], quat[4];
		/* This function gets new data from the FIFO when the DMP is in
		 * use. The FIFO can contain any combination of gyro, accel,
		 * quaternion, and gesture data. The sensors parameter tells the
		 * caller which data fields were actually populated with new data.
		 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
		 * the FIFO isn't being filled with accel data.
		 * The driver parses the gesture data to determine if a gesture
		 * event has occurred; on an event, the application will be notified
		 * via a callback (assuming that a callback function was properly
		 * registered). The more parameter is non-zero if there are
		 * leftover packets in the FIFO.
		 */
		dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
		if (!more)
			hal.new_gyro = 0;
		if (sensors & INV_XYZ_GYRO) {
			/* Push the new data to the MPL. */
			inv_build_gyro(gyro, sensor_timestamp);
			new_data = 1;
			/*if (new_temp) {
	                    new_temp = 0;
	                     Temperature only used for gyro temp comp.
	                    mpu_get_temperature(&temperature, &sensor_timestamp);
	                    inv_build_temp(temperature, sensor_timestamp);
	                }*/
		}
		if (sensors & INV_XYZ_ACCEL) {
			accel[0] = (long)accel_short[0];
			accel[1] = (long)accel_short[1];
			accel[2] = (long)accel_short[2];
			inv_build_accel(accel, 0, sensor_timestamp);
			new_data = 1;
		}
		if (sensors & INV_WXYZ_QUAT) {
			inv_build_quat(quat, 0, sensor_timestamp);
			new_data = 1;
		}
	} else if (hal.new_gyro) {
		short gyro[3], accel_short[3];
		unsigned char sensors, more;
		long accel[3];
		/* This function gets new data from the FIFO. The FIFO can contain
		 * gyro, accel, both, or neither. The sensors parameter tells the
		 * caller which data fields were actually populated with new data.
		 * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
		 * being filled with accel data. The more parameter is non-zero if
		 * there are leftover packets in the FIFO. The HAL can use this
		 * information to increase the frequency at which this function is
		 * called.
		 */
		hal.new_gyro = 0;
		mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
				&sensors, &more);
		if (more)
			hal.new_gyro = 1;
		if (sensors & INV_XYZ_GYRO) {
			/* Push the new data to the MPL. */
			inv_build_gyro(gyro, sensor_timestamp);
			new_data = 1;
			/*if (new_temp) {
				new_temp = 0;
				 Temperature only used for gyro temp comp.
				mpu_get_temperature(&temperature, &sensor_timestamp);
				inv_build_temp(temperature, sensor_timestamp);
			}*/
		}
		if (sensors & INV_XYZ_ACCEL) {
			accel[0] = (long)accel_short[0];
			accel[1] = (long)accel_short[1];
			accel[2] = (long)accel_short[2];
			inv_build_accel(accel, 0, sensor_timestamp);
			new_data = 1;
		}
	}

	if (new_data) {
		xSemaphoreGive(mpuNewDataSemHandle);
	}


}

void MpuInitTask(void const * argument) {

	UNUSED(argument);

	mpu_setup();

	osThreadTerminate(NULL);

}

void MpuCompassTask(void const * argument) {

	unsigned long timestamp = (unsigned long) &argument;

#ifdef COMPASS_ENABLED
	/* We're not using a data ready interrupt for the compass, so we'll
	 * make our compass reads timer-based instead.
	 */

	while(1)
	{


		short compass_short[3];
		long compass[3];

		//SetRGBLed(GREENYELLOW,100);

		/* For any MPU device with an AKM on the auxiliary I2C bus, the raw
		 * magnetometer registers are copied to special gyro registers.
		 */
		if (!mpu_get_compass_reg(compass_short, &timestamp)) {
			compass[0] = (long)compass_short[0];
			compass[1] = (long)compass_short[1];
			compass[2] = (long)compass_short[2];
			/* NOTE: If using a third-party compass calibration library,
			 * pass in the compass data in uT * 2^16 and set the second
			 * parameter to INV_CALIBRATED | acc, where acc is the
			 * accuracy from 0 to 3.
			 */
			inv_build_compass(compass, 0, timestamp);
		}

		xSemaphoreGive(mpuNewDataSemHandle);

		osDelay(COMPASS_READ_MS);
	}



#endif



}

void MpuTemperatureTask(void const * argument) {

	/* Temperature data doesn't need to be read with every gyro sample.
	 * Let's make them timer-based like the compass reads.
	 */
	long temperature;

	unsigned long timestamp = (unsigned long) &argument;


	while(1)
	{
		//SetRGBLed(ORANGE,100);
		mpu_get_temperature(&temperature, &sensor_timestamp);
		inv_build_temp(temperature, timestamp);
		osDelay(TEMP_READ_MS);
	}



}

void MpuProcessInterruptTask(void const * argument) {

	unsigned long timestamp = (unsigned long) &argument;

	xSemaphoreTake(mpuInterruptSemHandle,0); // dont block

	while(1)
	{

		if(xSemaphoreTake(mpuInterruptSemHandle, ( TickType_t ) 10000) == pdTRUE)
		{
				mpu_prosess_interrupt(timestamp);
		}

		//osDelay(1);

	}

}

void MpuNewDataTask(void const * argument) {

	UNUSED(argument);
	SetRGBLed(YELLOW,100);

	xSemaphoreTake(mpuNewDataSemHandle,0); // dont block

	long data[9];
	int8_t accuracy;
	unsigned long timestamp;
//	double acc[4]={0},gyr[4]={0},cmp[4]={0},euler[4]={0};
	double acc[4]={0},cmp[4]={0};
	Ins_data dataqueue;

	while(1)
	{
		if(xSemaphoreTake(mpuNewDataSemHandle, ( TickType_t ) 10000) == pdTRUE)
		{
			inv_execute_on_data();

			/* This function reads bias-compensated sensor data and sensor
	 	 	 * fusion outputs from the MPL. The outputs are formatted as seen
	 	 	 * in eMPL_outputs.c. This function only needs to be called at the
	 	 	 * rate requested by the host.
	 	 	 */

			if (hal.report & PRINT_ACCEL)
			{
				if (inv_get_sensor_type_accel(data, &accuracy,(inv_time_t*)&timestamp))
				{
					convert_ins_data(data,acc);
					dataqueue.acceleration=&acc[0];
				}
			}

//			if (hal.report & PRINT_GYRO)
//			{
//				if (inv_get_sensor_type_gyro(data, &accuracy,(inv_time_t*)&timestamp))
//				{
//					convert_ins_data(data,gyr);
//					dataqueue.gyroscope=&gyr[0];
//				}
//			}

			#ifdef COMPASS_ENABLED
			if (hal.report & PRINT_COMPASS)
			{
				if (inv_get_sensor_type_compass(data, &accuracy,(inv_time_t*)&timestamp))
				{
					convert_ins_data(data,cmp);
					dataqueue.compass=&cmp[0];
				}
			}
			#endif

			osMessagePut(MsgIns,&dataqueue, osWaitForever);
		}
		osThreadYield();
	}

	osThreadTerminate(NULL);


}
