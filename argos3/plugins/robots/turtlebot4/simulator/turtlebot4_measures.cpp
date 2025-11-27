/**
 * @file <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "turtlebot4_measures.h"

/****************************************/
/****************************************/
// Measures from here:
// https://emanual.robotis.com/docs/en/platform/turtlebot4/features/

UInt8 TURTLEBOT4_POWERON_LASERON   = 3;
const Real TURTLEBOT4_MASS = 0.4f;
const Real TURTLEBOT4_BASE_RADIUS    = 0.169; //important used for collision
const Real TURTLEBOT4_BASE_ELEVATION = 0.045;
const Real TURTLEBOT4_BASE_HEIGHT    = 0.351;
const Real TURTLEBOT4_LOWER_BODY_HEIGHT    = 0.0934; // height of the lower body part
const Real TURTLEBOT4_UPPER_BODY_HEIGHT = TURTLEBOT4_BASE_HEIGHT - TURTLEBOT4_LOWER_BODY_HEIGHT;
const Real TURTLEBOT4_BASE_TOP       = TURTLEBOT4_BASE_ELEVATION + TURTLEBOT4_BASE_HEIGHT;
const Real TURTLEBOT4_COLUMN_RADIUS       = 0.01f;  // tubes ~1cm
const Real TURTLEBOT4_COLUMN_HEIGHT       = TURTLEBOT4_UPPER_BODY_HEIGHT;
const Real TURTLEBOT4_NUM_COLUMNS         = 3;

const Real TURTLEBOT4_WHEEL_RADIUS        = 0.036; //important
const Real TURTLEBOT4_WHEEL_DISTANCE      = 0.235;
const Real TURTLEBOT4_HALF_WHEEL_DISTANCE = TURTLEBOT4_WHEEL_DISTANCE * 0.5;
const Real UPPER_BODY_RADIUS = TURTLEBOT4_BASE_RADIUS * 0.95f;


// const CVector3 TURTLEBOT4_LEDS_OFFSET[3] = {
//    CVector3( 0.04,  0.025, TURTLEBOT4_BASE_TOP),
//    CVector3(-0.05,  0.000, TURTLEBOT4_BASE_TOP),
//    CVector3( 0.04, -0.025, TURTLEBOT4_BASE_TOP)
// };

// extern const Real TURTLEBOT4_IR_SENSORS_RING_ELEVATION;
// extern const Real TURTLEBOT4_IR_SENSORS_RING_RADIUS;
// extern const Real TURTLEBOT4_IR_SENSORS_RING_RANGE;

const Real TURTLEBOT4_LIDAR_CENTER_ELEVATION   = -0.01;  // Lidar hight is considered in the robot height
                                                         // Hence, negative value
const Real TURTLEBOT4_LIDAR_ELEVATION          = TURTLEBOT4_BASE_TOP + TURTLEBOT4_LIDAR_CENTER_ELEVATION;
const Real TURTLEBOT4_LIDAR_SENSORS_FAN_RADIUS = TURTLEBOT4_BASE_RADIUS;
const CRadians TURTLEBOT4_LIDAR_ANGLE_SPAN(ToRadians(CDegrees(360.0)));
const CRange<Real> TURTLEBOT4_LIDAR_SENSORS_RING_RANGE(0.150, 12.00);

const Real TURTLEBOT4_MAX_FORCE           = 2.0f; // Are these the right values?
const Real TURTLEBOT4_MAX_TORQUE          = 2.0f;

const CRadians TURTLEBOT4_LED_RING_START_ANGLE   = CRadians((ARGOS_PI / 8.0f) * 0.5f);
const Real TURTLEBOT4_LED_RING_RADIUS            = TURTLEBOT4_BASE_RADIUS + 0.007;
const Real TURTLEBOT4_LED_RING_ELEVATION         = 0.086f;
const Real TURTLEBOT4_RAB_ELEVATION              = TURTLEBOT4_LED_RING_ELEVATION;

const Real TURTLEBOT4_IR_SENSOR_RING_ELEVATION       = 0.06f; // is this correct?
const Real TURTLEBOT4_IR_SENSOR_RING_RADIUS          = TURTLEBOT4_BASE_RADIUS;
const CRadians TURTLEBOT4_IR_SENSOR_RING_START_ANGLE = CRadians((2 * ARGOS_PI / 8.0f) * 0.5f);
const Real TURTLEBOT4_IR_SENSOR_RING_RANGE           = 0.1f;

// Readings from here:
// https://emanual.robotis.com/docs/en/platform/turtlebot4/appendix_lds_01/

/****************************************/
/****************************************/
