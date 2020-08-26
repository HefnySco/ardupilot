

#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/compass.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/camera.h>


WbNodeRef self_node;
double *linear_velocity;

void getInertia (const WbDeviceTag inertialUnit, const double coordinateSystem_NUE, char *buf);
void getLinearVelocity (WbNodeRef nodeRef, const double coordinateSystem_NUE, char * buf);
void getCompass (const WbDeviceTag compass, const double coordinateSystem_NUE, char *buf);
void getAcc (const WbDeviceTag accelerometer, const double coordinateSystem_NUE, char *buf);
void getGyro (const WbDeviceTag gyro, const double coordinateSystem_NUE, char *buf);
void getGPS (const WbDeviceTag gps, const double coordinateSystem_NUE, char *buf);
void getAllSensors (char *buf, const double coordinateSystem_NUE, WbDeviceTag gyro, WbDeviceTag accelerometer, WbDeviceTag compass, const WbDeviceTag gps, const WbDeviceTag inertial_unit);