/* Include the controller definition */
#include "turtlebot4_test.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
using namespace std;
/****************************************/
/****************************************/

CTurtlebot4Test::CTurtlebot4Test() :
   m_pcWheels(NULL),
   // m_pcProximity(NULL),
   m_pcGround(NULL), 
   m_pcCamera(NULL),
   // m_pcLight(NULL),
   m_fWheelVelocity(-2.5f) {}

/****************************************/
/****************************************/

void CTurtlebot4Test::Init(TConfigurationNode& t_node) {
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
    * file at the <controllers><turtlebot4_test><actuators> and
    * <controllers><turtlebot4_test><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */

   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_Turtlebot4ProximitySensor             >("turtlebot4_proximity"    );
   // m_pcLight = GetSensor  <CCI_Turtlebot4LightSensor>("turtlebot4_light");
   m_pcCamera = GetSensor  <CCI_Turtlebot4ColoredBlobOmnidirectionalCameraSensor>("turtlebot4_colored_blob_omnidirectional_camera");
   m_pcGround = GetSensor  <CCI_Turtlebot4BaseGroundSensor>("turtlebot4_ground");
   m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcLidar = GetSensor  <CCI_Turtlebot4LIDARSensor    >("turtlebot4_lidar"  );
   // m_pcCamera  = GetSensor  <CCI_ColoredBlobPerspectiveCameraSensor>("turtlebot4_colored_blob_perspective_camera");
   m_pcCamera->Enable();

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

// void CTurtlebot4Test::LogLightReadings() const {
//    static const char* kLabels[] = {"Front-Right", "Back-Right", "Back-Left", "Front-Left"};
//    const auto& tReadings = m_pcLight->GetReadings();
   
//    std::cout << "Light readings: ";
//    for(size_t i = 0; i < tReadings.size(); ++i) {
//       if(tReadings[i].Value > 0) {
//          // guard against label/readings size mismatch
//          const char* label = (i < 4 ? kLabels[i] : "Unknown");
//          std::cout << label << ": " << tReadings[i].Value << " ";
//       }
//    }
//    std::cout << std::endl;
// }

/****************************************/
/****************************************/

void CTurtlebot4Test::LogGroundSensorReadings() const {
    const auto& tGroundReads = m_pcGround->GetReadings();

    /* Get this robot's ID */
    const std::string& strId = GetId();

    /* Determine how many sensors are in "white" (close to 1.0) */
    size_t unWhiteCount = 0;
    cout << "Number of Ground Readings: " << tGroundReads.size() << endl;
    for(size_t i = 0; i < tGroundReads.size(); ++i) {
        if(tGroundReads[i].Value > 0.8f) {   // 0.8 threshold for "white"
            std::cout << "not white detected at sensor " << i << " with value " << tGroundReads[i].Value << std::endl;
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

void CTurtlebot4Test::LogLidarSensorReadings() const {
    const auto numReadings = m_pcLidar->GetNumReadings();
   argos::LOG << numReadings << " LIDAR readings: " << numReadings << std::endl;

}

/****************************************/
/****************************************/

void CTurtlebot4Test::LogLightUsingCameraSensorReadings() const {
    /* Perspective Camera */
   const CCI_Turtlebot4ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   LOG << CCI_Controller::GetId() << "> Camera: " << std::endl;
   LOG << "Number of blobs detected: " << sReadings.BlobList.size() << std::endl;
   LOG << "Counter: " << sReadings.Counter << std::endl;
   for (size_t i = 0; i < sReadings.BlobList.size(); i++) {
         CCI_Turtlebot4ColoredBlobOmnidirectionalCameraSensor::SBlob* sBlob = sReadings.BlobList[i];
      LOG << "Color = " << sBlob->Color << std::endl;
   }
}


/****************************************/
/****************************************/

void CTurtlebot4Test::AvoidObstaclesWithProximitySensors() {
   const auto& readings = m_pcProximity->GetReadings();
   std::cout << "Avoiding obstacles with proximity sensors..." << std::endl;
   if(readings.empty()) {
      THROW_ARGOSEXCEPTION("Proximity sensor returned no readings");
   }

   // Safety check
   if(readings.size() < 7) {
      THROW_ARGOSEXCEPTION("Proximity sensor returned " << readings.size()
                           << " readings; expected 7");
   }

   /* Get the highest reading in front of the robot, which corresponds to the closest object */
   // Start with index 0
   const std::string& strId = GetId();
   std::cout << strId << " | " << endl;

   Real IRvalue_0 = readings[0].Value;
   Real IRvalue_1 = readings[1].Value;
   Real IRvalue_2 = readings[2].Value;
   Real IRvalue_3 = readings[3].Value;
   Real IRvalue_4 = readings[4].Value;
   Real IRvalue_5 = readings[5].Value;
   Real IRvalue_6 = readings[6].Value;
   Real fMaxReadVal = 0.0f;
   UInt32 unMaxReadIdx = 0;

   argos::LOG << "IRvalue_0: " << IRvalue_0 << std::endl;
   argos::LOG << "IRvalue_1: " << IRvalue_1 << std::endl;
   argos::LOG << "IRvalue_2: " << IRvalue_2 << std::endl;
   argos::LOG << "IRvalue_3: " << IRvalue_3 << std::endl;
   argos::LOG << "IRvalue_4: " << IRvalue_4 << std::endl;
   argos::LOG << "IRvalue_5: " << IRvalue_5 << std::endl;
   argos::LOG << "IRvalue_6: " << IRvalue_6 << std::endl;

   // Check indices 1, 7, and 6 (front left and right sensors)
   // if(fMaxReadVal < IRvalue_2 || fMaxReadVal < IRvalue_3 || fMaxReadVal < IRvalue_4) {
   //    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);

   // }
   // if(fMaxReadVal < IRvalue_5 || fMaxReadVal < IRvalue_6) {
   //    fMaxReadVal = IRvalue_5;
   //    unMaxReadIdx = 5;
   // }
   // if(fMaxReadVal < IRvalue_0 || fMaxReadVal < IRvalue_1) {
   //    fMaxReadVal = IRvalue_0;
   //    unMaxReadIdx = 1;
   // }

   /* Do we have an obstacle in front? */
   if(IRvalue_2 > 0.0f || IRvalue_3 > 0.0f || IRvalue_4 > 0.0f || IRvalue_5 > 0.0f || IRvalue_6 > 0.0f) {
     /* Yes, we do: avoid it */
   //   if(unMaxReadIdx == 1 || unMaxReadIdx == 3) {
       /* The obstacle is straight, turn left */
       m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
   //   }
   //   else if (unMaxReadIdx == 5) {
   //     /* The obstacle is on the right, turn right */
   //     m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
   //   }
   }
   else {
     /* No, we don't: go straight */
       argos::LOG << "obj straight unMaxReadIdx set to both wheels: " << m_fWheelVelocity << std::endl;
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      // Real angularVel = m_pcWheels-
   }

}
/****************************************/
/****************************************/

void CTurtlebot4Test::ControlStep() {

   // --- Obstacle Avoidance with proximity sensors --- 
   AvoidObstaclesWithProximitySensors();
   
   LogGroundSensorReadings();
   // --- Light sensor debug ---
   // LogLightReadings();
   LogLightUsingCameraSensorReadings();

   // LogLidarSensorReadings();
}

/****************************************/
/****************************************/

void CTurtlebot4Test::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);

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
REGISTER_CONTROLLER(CTurtlebot4Test, "turtlebot4_test_controller")
