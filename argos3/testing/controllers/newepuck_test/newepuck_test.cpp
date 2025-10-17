/* Include the controller definition */
#include "newepuck_test.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
using namespace std;
/****************************************/
/****************************************/

CNewEPuckTest::CNewEPuckTest() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcGround(NULL), 
   m_pcLight(NULL),
   m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CNewEPuckTest::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><newepuck_test><actuators> and
    * <controllers><newepuck_test><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */

   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_NewEPuckProximitySensor             >("newepuck_proximity"    );
   m_pcLight = GetSensor  <CCI_NewEPuckLightSensor>("newepuck_light");
   m_pcGround = GetSensor  <CCI_NewEPuckBaseGroundSensor>("newepuck_ground");
   m_pcLidar = GetSensor  <CCI_NewEPuckLIDARSensor    >("newepuck_lidar"  );

   const auto& tReadings = m_pcGround->GetReadings();
   
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CNewEPuckTest::LogLightReadings() const {
   static const char* kLabels[] = {"Front-Right", "Back-Right", "Back-Left", "Front-Left"};
   const auto& tReadings = m_pcLight->GetReadings();
   
   std::cout << "Light readings: ";
   for(size_t i = 0; i < tReadings.size(); ++i) {
      if(tReadings[i].Value > 0) {
         // guard against label/readings size mismatch
         const char* label = (i < 4 ? kLabels[i] : "Unknown");
         std::cout << label << ": " << tReadings[i].Value << " ";
      }
   }
   std::cout << std::endl;
}

/****************************************/
/****************************************/

void CNewEPuckTest::LogGroundSensorReadings() const {
    const auto& tGroundReads = m_pcGround->GetReadings();

    /* Get this robot's ID */
    const std::string& strId = GetId();

    /* Determine how many sensors are in "white" (close to 1.0) */
    size_t unWhiteCount = 0;
    for(size_t i = 0; i < tGroundReads.size(); ++i) {
        if(tGroundReads[i].Value > 0.8f) {   // 0.8 threshold for "white"
            ++unWhiteCount;
        }
    }

    /* Classify based on number of white sensors */
    std::string strZone;
    if(unWhiteCount == tGroundReads.size()) {
        strZone = "WHITE";
    } else {
        strZone = "GRAY";
    }

    /* Print results */
    std::cout << strId << " | ";
    std::cout << "| Zone: " << strZone << std::endl;
}

/****************************************/
/****************************************/

void CNewEPuckTest::LogLidarSensorReadings() const {
    const auto numReadings = m_pcLidar->GetNumReadings();
   argos::LOG << numReadings << " LIDAR readings: " << numReadings << std::endl;

}


/****************************************/
/****************************************/

void CNewEPuckTest::AvoidObstaclesWithProximitySensors() {
   const auto& readings = m_pcProximity->GetReadings();

   if(readings.empty()) {
      THROW_ARGOSEXCEPTION("Proximity sensor returned no readings");
   }

   // Safety check
   if(readings.size() < 8) {
      THROW_ARGOSEXCEPTION("Proximity sensor returned " << readings.size()
                           << " readings; expected at least 8");
   }

   /* Get the highest reading in front of the robot, which corresponds to the closest object */
   // Start with index 0
   Real fMaxReadVal = readings[0].Value;
   UInt32 unMaxReadIdx = 0;

   // Check indices 1, 7, and 6 (front left and right sensors)
   if(fMaxReadVal < readings[1].Value) {
      fMaxReadVal = readings[1].Value;
      unMaxReadIdx = 1;
   }
   if(fMaxReadVal < readings[7].Value) {
      fMaxReadVal = readings[7].Value;
      unMaxReadIdx = 7;
   }
   if(fMaxReadVal < readings[6].Value) {
      fMaxReadVal = readings[6].Value;
      unMaxReadIdx = 6;
   }

   /* Do we have an obstacle in front? */
   if(fMaxReadVal > 0.0f) {
     /* Yes, we do: avoid it */
     if(unMaxReadIdx == 0 || unMaxReadIdx == 1) {
       /* The obstacle is on the left, turn right */
       m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
     }
     else {
       /* The obstacle is on the left, turn right */
       m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
     }
   }
   else {
     /* No, we don't: go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }

}
/****************************************/
/****************************************/

void CNewEPuckTest::ControlStep() {

   // --- Obstacle Avoidance with proximity sensors --- 
   AvoidObstaclesWithProximitySensors();
   
   LogGroundSensorReadings();
   // --- Light sensor debug ---
   LogLightReadings();

   LogLidarSensorReadings();
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CNewEPuckTest, "newepuck_test_controller")
