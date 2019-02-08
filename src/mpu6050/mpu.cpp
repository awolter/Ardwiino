#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define FSR 2000
//#define GYRO_SENS       ( 131.0f * 250.f / (float)FSR )
#define GYRO_SENS 16.375f
#define QUAT_SENS 1073741824.f // 2^30

#define EPSILON 0.0001f
#define PI_2 1.57079632679489661923f

struct s_mympu mympu;
struct s_quat last;
static int ret;
static short sensors;
static unsigned char fifoCount;
int mympu_open(unsigned int rate) {

  ret = mpu_init(NULL);
#ifdef MPU_DEBUG
  if (ret)
    return 10 + ret;
#endif

  ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#ifdef MPU_DEBUG
  if (ret)
    return 20 + ret;
#endif

  ret = mpu_set_gyro_fsr(FSR);
#ifdef MPU_DEBUG
  if (ret)
    return 30 + ret;
#endif

  ret = mpu_set_accel_fsr(2);
#ifdef MPU_DEBUG
  if (ret)
    return 40 + ret;
#endif

  mpu_get_power_state((unsigned char *)&ret);
#ifdef MPU_DEBUG
  if (!ret)
    return 50 + ret;
#endif

  ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#ifdef MPU_DEBUG
  if (ret)
    return 60 + ret;
#endif

  ret = dmp_load_motion_driver_firmware();
#ifdef MPU_DEBUG
  if (ret)
    return 80 + ret;
#endif

  ret = dmp_set_fifo_rate(rate);
#ifdef MPU_DEBUG
  if (ret)
    return 90 + ret;
#endif

  ret = mpu_set_dmp_state(1);
#ifdef MPU_DEBUG
  if (ret)
    return 100 + ret;
#endif

  ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT);
//	ret =
// dmp_enable_feature(DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL);
#ifdef MPU_DEBUG
  if (ret)
    return 110 + ret;
#endif

  return 0;
}

static inline float rad2deg(float rad) {
  // return (180.f/PI) * rad;
  return 57.2957795131f * rad;
}

static float test, sqy, sqz, sqw;
static void quaternionToEuler(const struct s_quat *q, float *x, float *y,
                              float *z) {
  // sqy = q->y * q->y;
  sqz = q->z * q->z;
  sqw = q->w * q->w;

  test = q->x * q->z - q->w * q->y;

  if (test > 0.5f - EPSILON) {
    *x = 2.f * atan2(q->y, q->w);
    // *y = PI_2;
    // *z = 0;
  } else if (test < -0.5f + EPSILON) {
    *x = -2.f * atan2(q->y, q->w);
    // *y = -PI_2;
    // *z = 0;
  } else {
    *x = atan2(2.f * (q->x * q->w + q->y * q->z), 1.f - 2.f * (sqz + sqw));
    // *y = asin(2.f * test);
    // *z = atan2(2.f * (q->x * q->y - q->z * q->w), 1.f - 2.f * (sqy + sqz));
  }
}

static inline float wrap_pi(float x) {
  return (x < -M_PI ? x + M_PI + M_PI : (x > M_PI ? x - M_PI : x));
}

int mympu_update() {
  if (last.w == I2Cdev::accelData._f.w && last.x == I2Cdev::accelData._f.x &&
      last.y == I2Cdev::accelData._f.y && last.z == I2Cdev::accelData._f.z) {
  } else {
    quaternionToEuler(&I2Cdev::accelData._f, &mympu.ypr[2], &mympu.ypr[1],
                      &mympu.ypr[0]);
    mympu.ypr[2] = wrap_pi(mympu.ypr[2]);
  }
  last.w = I2Cdev::accelData._f.w;
  last.x = I2Cdev::accelData._f.x;
  last.y = I2Cdev::accelData._f.y;
  last.z = I2Cdev::accelData._f.z;
  return 0;
}
