/*
 * AUTHOR: Jyotsna Bellary <jyotsna.bellary@uni-konstanz.de>
 *
 * A test controller for testing the working of all newly added sensors to new turtlebot4.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * The controller uses the light sensors to detect light sources.
 * 
 * This controller is meant to be used with the XML files:
 *    experiments/turtlebot4_test.argos
 */

#ifndef TURTLEBOT4_TEST_H
#define TURTLEBOT4_TEST_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
// #include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
// #include <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_light_sensor.h>
#include <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_proximity_sensor.h>
#include <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_base_ground_sensor.h>
#include <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_lidar_sensor.h>
#include <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_colored_blob_omnidirectional_camera_sensor.h>
// #include <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_colored_blob_perspective_camera_default_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CTurtlebot4Test : public CCI_Controller {

public:

   /* Class constructor. */
   CTurtlebot4Test();

   /* Class destructor. */
   virtual ~CTurtlebot4Test() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><turtlebot4_test_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
   * This function logs the light sensor readings to the console.
   */
   void LogLightReadings() const;

   /*
   * This function logs the light sensor readings to the console.
   */
   void LogGroundSensorReadings() const;

   /*
   * This function logs the lidar sensor readings to the console.
   */
   void LogLidarSensorReadings() const;

   /*
   * This function logs the color of the light using Camera sensor to the console.
   */
   void LogLightUsingCameraSensorReadings() const;

   /*
    * Calculates the vector to the closest light.
    */
   // virtual CVector2 VectorToLight();

   /*
   * This function avoids obstacles using the proximity sensors.
   */
   void AvoidObstaclesWithProximitySensors();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;

   /* Pointer to the turtlebot4 proximity sensor */
   CCI_Turtlebot4ProximitySensor* m_pcProximity;

   /* Pointer to the turtlebot4 base ground sensor */
   CCI_Turtlebot4BaseGroundSensor* m_pcGround;

   /* Pointer to the new turtlebot4 light sensor*/
   // CCI_Turtlebot4LightSensor* m_pcLight;

   /* Pointer to the new turtlebot4 Lidar sensor*/
   CCI_Turtlebot4LIDARSensor* m_pcLidar;

   CCI_LEDsActuator* m_pcLEDs;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><turtlebot4_test_controller> section.
    */
   /* Wheel speed. */
   Real m_fWheelVelocity;

   /* Pointer to the omnidirectional camera sensor */
   CCI_Turtlebot4ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;

   // CCI_ColoredBlobPerspectiveCameraSensor* m_pcCamera;
   // CCI_LEDsActuator* m_pcLedAct;

};

#endif
