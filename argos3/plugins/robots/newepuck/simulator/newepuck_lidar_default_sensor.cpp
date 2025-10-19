/**
 * @file <argos3/plugins/robots/newepuck/simulator/newepuck_lidar_default_sensor.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

#include "newepuck_lidar_default_sensor.h"
// #include "newepuck_measures.h"

#include <cstring>

   /****************************************/
   /****************************************/
   // Lidar Specs:
   // https://emanual.robotis.com/assets/docs/LDS_Basic_Specification.pdf
   //
   /****************************************/
   /****************************************/
namespace argos {

   /****************************************/
   /****************************************/

   static UInt8 NEWEPUCK_POWERON_LASERON   = 3;
   const Real NEWEPUCK_BASE_ELEVATION = 0.009;
   const Real NEWEPUCK_BASE_RADIUS    = 0.08;
   const Real NEWEPUCK_BASE_HEIGHT    = 0.192;
   const Real NEWEPUCK_BASE_TOP       = NEWEPUCK_BASE_ELEVATION + NEWEPUCK_BASE_HEIGHT;
   const Real NEWEPUCK_LIDAR_CENTER_ELEVATION   = -0.01;  // Lidar hight is considered in the robot height

   const Real NEWEPUCK_LIDAR_ELEVATION = NEWEPUCK_BASE_TOP + NEWEPUCK_LIDAR_CENTER_ELEVATION;
   const Real NEWEPUCK_LIDAR_SENSORS_FAN_RADIUS = NEWEPUCK_BASE_RADIUS;

   const CRange<Real> NEWEPUCK_LIDAR_SENSORS_RING_RANGE (0.02, 1.0); // in meters
   const CRadians NEWEPUCK_LIDAR_ANGLE_SPAN(ToRadians(CDegrees(360.0)));


   /****************************************/
   /****************************************/

   CNewEPuckLIDARDefaultSensor::CNewEPuckLIDARDefaultSensor() :
      m_pnReadings(NULL),
      m_unNumReadings(1800),
      m_unPowerLaserState(NEWEPUCK_POWERON_LASERON),
      m_pcEmbodiedEntity(NULL),
      m_bShowRays(false),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()) {}

   /****************************************/
   /****************************************/

   CNewEPuckLIDARDefaultSensor::~CNewEPuckLIDARDefaultSensor() {
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcProximityEntity = &(c_entity.GetComponent<CProximitySensorEquippedEntity>("proximity_sensors[lidar]"));
         m_pcProximityEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the NewEPuck LIDAR default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_NewEPuckLIDARSensor::Init(t_tree);
         /* How many readings? */
         GetNodeAttributeOrDefault(t_tree, "num_readings", m_unNumReadings, m_unNumReadings);
         m_pcProximityEntity->AddSensorFan(
            CVector3(0.0, 0.0, NEWEPUCK_LIDAR_ELEVATION),
            NEWEPUCK_LIDAR_SENSORS_FAN_RADIUS + NEWEPUCK_LIDAR_SENSORS_RING_RANGE.GetMin(),
            -NEWEPUCK_LIDAR_ANGLE_SPAN * 0.5,
            NEWEPUCK_LIDAR_ANGLE_SPAN * 0.5,
            NEWEPUCK_LIDAR_SENSORS_FAN_RADIUS + NEWEPUCK_LIDAR_SENSORS_RING_RANGE.GetMax(),
            m_unNumReadings,
            m_pcEmbodiedEntity->GetOriginAnchor());
         m_pnReadings = new long[m_unNumReadings];
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise level */
         Real fNoiseLevel = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
         if(fNoiseLevel < 0.0f) {
            THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the proximity sensor");
         }
         else if(fNoiseLevel > 0.0f) {
            m_bAddNoise = true;
            m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
            m_pcRNG = CRandom::CreateRNG("argos");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default proximity sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::Update() {
      /* Nothing to do if sensor is deactivated */
      if(m_unPowerLaserState != NEWEPUCK_POWERON_LASERON)
         return;
      /* Ray used for scanning the environment for obstacles */
      CRay3 cScanningRay;
      CVector3 cRayStart, cRayEnd;
      /* Buffers to contain data about the intersection */
      SEmbodiedEntityIntersectionItem sIntersection;
      /* Go through the sensors */
      for(UInt32 i = 0; i < m_unNumReadings; ++i) {
         /* Compute ray for sensor i */
         cRayStart = m_pcProximityEntity->GetSensor(i).Offset;
         cRayStart.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
         cRayStart += m_pcProximityEntity->GetSensor(i).Anchor.Position;
         cRayEnd = m_pcProximityEntity->GetSensor(i).Offset;
         cRayEnd += m_pcProximityEntity->GetSensor(i).Direction;
         cRayEnd.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
         cRayEnd += m_pcProximityEntity->GetSensor(i).Anchor.Position;
         cScanningRay.Set(cRayStart,cRayEnd);
         /* Compute reading */
         /* Get the closest intersection */
         if(GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                     cScanningRay,
                                                     *m_pcEmbodiedEntity)) {
            /* There is an intersection */
            if(m_bShowRays) {
               m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                            sIntersection.TOnRay);
               m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
            }
            /* The actual reading is in cm */
            m_pnReadings[i] = cScanningRay.GetDistance(sIntersection.TOnRay) * 100;
         }
         else {
            /* No intersection */
            m_pnReadings[i] = 0;
            if(m_bShowRays) {
               m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
            }
         }
         /* Apply noise to the sensor */
         if(m_bAddNoise) {
            m_pnReadings[i] += m_pcRNG->Uniform(m_cNoiseRange);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::Reset() {
      memset(m_pnReadings, 0, m_unNumReadings * sizeof(long int));
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::Destroy() {
      delete[] m_pnReadings;
   }

   /****************************************/
   /****************************************/

   long CNewEPuckLIDARDefaultSensor::GetReading(UInt32 un_idx) const {
      return m_pnReadings[un_idx];
   }

   /****************************************/
   /****************************************/

   size_t CNewEPuckLIDARDefaultSensor::GetNumReadings() const {
      return m_unNumReadings;
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::PowerOn() {
      m_unPowerLaserState = m_unPowerLaserState | 0x1;
      m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::PowerOff() {
      m_unPowerLaserState = m_unPowerLaserState & 0xFE;
      m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::LaserOn() {
      m_unPowerLaserState = m_unPowerLaserState | 0x2;
      m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   void CNewEPuckLIDARDefaultSensor::LaserOff() {
      m_unPowerLaserState = m_unPowerLaserState & 0xFD;
      m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CNewEPuckLIDARDefaultSensor,
                   "newepuck_lidar", "default",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "The NewEPuck 3 LIDAR sensor.",
                   "This sensor accesses the NewEPuck 3 LIDAR sensor. The sensors return the\n"
                   "distance to nearby objects. In controllers, you must include the\n"
                   "ci_newepuck_lidar_sensor.h header.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to draw the rays shot by the LIDAR sensor in the OpenGL\n"
                   "visualization. This can be useful for sensor debugging but also to understand\n"
                   "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
                   "they are not obstructed and in purple when they are. In case a ray is\n"
                   "obstructed, a black dot is drawn where the intersection occurred.\n"
                   "To turn this functionality on, add the attribute \"show_rays\" as in this\n"
                   "example:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\"\n"
                   "               show_rays=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "It is possible to change the default number of readings to make computation\n"
                   "faster. The default number of readings is 682, but using the 'num_readings'\n"
                   "attribute you can change it to a different value:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\"\n"
                   "               num_readings=\"100\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "It is possible to add uniform noise to the sensors, thus matching the\n"
                   "characteristics of a real robot better. This can be done with the attribute\n"
                   "\"noise_level\", whose allowed range is in [-1,1] and is added to the calculated\n"
                   "reading. The final sensor reading is always normalized in the [0-1] range.\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\"\n"
                   "               noise_level=\"0.1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",
                   "Usable"
		  );

}
