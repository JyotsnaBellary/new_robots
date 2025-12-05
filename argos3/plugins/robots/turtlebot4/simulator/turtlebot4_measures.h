/**
 * @file <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef TURTLEBOT4_MEASURES_H
#define TURTLEBOT4_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>

using namespace argos;

extern UInt8 TURTLEBOT4_POWERON_LASERON;
extern const Real TURTLEBOT4_MASS;
extern const Real TURTLEBOT4_BASE_RADIUS;
extern const Real TURTLEBOT4_BASE_ELEVATION;
extern const Real TURTLEBOT4_BASE_HEIGHT;
extern const Real TURTLEBOT4_LOWER_BODY_HEIGHT;
extern const Real TURTLEBOT4_BASE_TOP;
extern const Real UPPER_BODY_RADIUS;

extern const Real TURTLEBOT4_COLUMN_RADIUS ;      
extern const Real TURTLEBOT4_COLUMN_HEIGHT;
extern const Real TURTLEBOT4_NUM_COLUMNS;
extern const Real TURTLEBOT4_UPPER_BODY_HEIGHT;

extern const Real TURTLEBOT4_WHEEL_RADIUS;
extern const Real TURTLEBOT4_WHEEL_DISTANCE;
extern const Real TURTLEBOT4_HALF_WHEEL_DISTANCE;

extern const Real TURTLEBOT4_IR_SENSOR_RING_ELEVATION;
extern const Real TURTLEBOT4_IR_SENSOR_RING_RADIUS;
extern const Real TURTLEBOT4_IR_SENSOR_RING_RANGE;

extern const CRadians TURTLEBOT4_LED_RING_START_ANGLE;
extern const Real TURTLEBOT4_LED_RING_RADIUS;
extern const Real TURTLEBOT4_LED_RING_ELEVATION;
extern const Real TURTLEBOT4_RAB_ELEVATION;

extern const Real TURTLEBOT4_ULTRASOUND_SENSORS_RING_ELEVATION;
extern const Real TURTLEBOT4_ULTRASOUND_SENSORS_RING_RADIUS;
extern const CRange<Real> TURTLEBOT4_ULTRASOUND_SENSORS_RING_RANGE;

extern const CVector2 TURTLEBOT4_IR_SENSORS_GROUND_OFFSET[4];

extern const CVector3 TURTLEBOT4_LEDS_OFFSET[3];

extern const Real TURTLEBOT4_LIDAR_ELEVATION;
extern const Real TURTLEBOT4_LIDAR_SENSORS_FAN_RADIUS;
extern const CRadians TURTLEBOT4_LIDAR_ANGLE_SPAN;
extern const CRange<Real> TURTLEBOT4_LIDAR_SENSORS_RING_RANGE;

extern const Real TURTLEBOT4_MAX_FORCE;
extern const Real TURTLEBOT4_MAX_TORQUE;
extern const Real OMNIDIRECTIONAL_CAMERA_ELEVATION;

#endif
