/*
 * AUTHOR: Jyotsna Bellary <jyotsna.bellary@uni-konstanz.de>
 *
 * A test controller for testing the working of all newly added sensors to testbot.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * The controller uses the light sensors to detect light sources.
 * 
 * This controller is meant to be used with the XML files:
 *    experiments/testbot_test.argos
 */

#ifndef TESTBOT_TEST_H
#define TESTBOT_TEST_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
// #include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
// #include <argos3/plugins/robots/testbot/control_interface/ci_testbot_light_sensor.h>
// #include <argos3/plugins/robots/testbot/control_interface/ci_testbot_proximity_sensor.h>
// #include <argos3/plugins/robots/testbot/control_interface/ci_testbot_base_ground_sensor.h>
// #include <argos3/plugins/robots/testbot/control_interface/ci_testbot_lidar_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CTestBotTest : public CCI_Controller {

public:

   /* Class constructor. */
   CTestBotTest();

   /* Class destructor. */
   virtual ~CTestBotTest() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><testbot_test_controller> section.
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
   // void LogLightReadings() const;

   // /*
   // * This function logs the light sensor readings to the console.
   // */
   // void LogGroundSensorReadings() const;

   // /*
   // * This function logs the lidar sensor readings to the console.
   // */
   // void LogLidarSensorReadings() const;

   // /*
   // * This function avoids obstacles using the proximity sensors.
   // */
   // void AvoidObstaclesWithProximitySensors();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

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

   /* Pointer to the e-puck proximity sensor */
   // CCI_TestBotProximitySensor* m_pcProximity;

   // /* Pointer to the e-puck base ground sensor */
   // CCI_TestBotBaseGroundSensor* m_pcGround;

   // /* Pointer to the testbot light sensor*/
   // CCI_TestBotLightSensor* m_pcLight;

   // /* Pointer to the testbot Lidar sensor*/
   // CCI_TestBotLIDARSensor* m_pcLidar;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><testbot_test_controller> section.
    */
   /* Wheel speed. */
   Real m_fWheelVelocity;

};

#endif
