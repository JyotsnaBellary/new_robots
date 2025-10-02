/**
 * @file <argos3/plugins/robots/newepuck/simulator/newepuck_lidar_sensor.cpp>
 *
 * @author Raffaele Todesco - <raffaele.todesco@ulb.be>
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

#include "newepuck_lidar_default_sensor.h"
#include <cstring>

namespace argos
{
    const Real NEWEPUCK_BASE_RADIUS = 0.08;
    const Real NEWEPUCK_BASE_ELEVATION = 0.009;
    const Real NEWEPUCK_BASE_HEIGHT = 0.192;
    const Real NEWEPUCK_BASE_TOP = NEWEPUCK_BASE_ELEVATION + NEWEPUCK_BASE_HEIGHT;

    const Real NEWEPUCK_LIDAR_CENTER_ELEVATION = -0.01; // Lidar hight is considered in the robot height

    static UInt8 NEWEPUCK_POWERON_LASERON = 3;
    const Real NEWEPUCK_LIDAR_ELEVATION = NEWEPUCK_BASE_TOP + NEWEPUCK_LIDAR_CENTER_ELEVATION;
    const Real NEWEPUCK_LIDAR_SENSORS_FAN_RADIUS = NEWEPUCK_BASE_RADIUS;
    const CRadians NEWEPUCK_LIDAR_ANGLE_SPAN(ToRadians(CDegrees(360.0)));
    const CRange<Real> NEWEPUCK_LIDAR_SENSORS_RING_RANGE(0.120, 3.500);

    CNewEPuckLidarDefaultSensor::
        CNewEPuckLidarDefaultSensor() : m_pnReadings(NULL),
                                        m_unNumReadings(1800),
                                        m_unPowerLaserState(NEWEPUCK_POWERON_LASERON),
                                        m_pcEmbodiedEntity(NULL),
                                        m_bShowRays(false),
                                        m_pcRNG(NULL),
                                        m_bAddNoise(false),
                                        m_cSpace(CSimulator::GetInstance().GetSpace()) {}

    /****************************************/
    /****************************************/

    CNewEPuckLidarDefaultSensor::~CNewEPuckLidarDefaultSensor()
    {
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::SetRobot(CComposableEntity &c_entity)
    {
        try
        {
            m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
            m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
            m_pcProximityEntity = &(c_entity.GetComponent<CProximitySensorEquippedEntity>("proximity_sensors[lidar]"));
            m_pcProximityEntity->Enable();
        }
        catch (CARGoSException &ex)
        {
            THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the new E-Puck LIDAR default sensor", ex);
        }
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::Init(TConfigurationNode &t_tree)
    {
        try
        {
            CCI_NewEPuckLidarSensor::Init(t_tree);
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
            if (fNoiseLevel < 0.0f)
            {
                THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the proximity sensor");
            }
            else if (fNoiseLevel > 0.0f)
            {
                m_bAddNoise = true;
                m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
                m_pcRNG = CRandom::CreateRNG("argos");
            }
        }
        catch (CARGoSException &ex)
        {
            THROW_ARGOSEXCEPTION_NESTED("Initialization error in default proximity sensor", ex);
        }
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::Update()
    {
        /* Nothing to do if sensor is deactivated */
        if (m_unPowerLaserState != NEWEPUCK_POWERON_LASERON)
            return;
        /* Ray used for scanning the environment for obstacles */
        CRay3 cScanningRay;
        CVector3 cRayStart, cRayEnd;
        /* Buffers to contain data about the intersection */
        SEmbodiedEntityIntersectionItem sIntersection;
        /* Go through the sensors */
        for (UInt32 i = 0; i < m_unNumReadings; ++i)
        {
            /* Compute ray for sensor i */
            cRayStart = m_pcProximityEntity->GetSensor(i).Offset;
            cRayStart.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
            cRayStart += m_pcProximityEntity->GetSensor(i).Anchor.Position;
            cRayEnd = m_pcProximityEntity->GetSensor(i).Offset;
            cRayEnd += m_pcProximityEntity->GetSensor(i).Direction;
            cRayEnd.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
            cRayEnd += m_pcProximityEntity->GetSensor(i).Anchor.Position;
            cScanningRay.Set(cRayStart, cRayEnd);
            /* Compute reading */
            /* Get the closest intersection */
            if (GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                         cScanningRay,
                                                         *m_pcEmbodiedEntity))
            {
                /* There is an intersection */
                if (m_bShowRays)
                {
                    m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                                 sIntersection.TOnRay);
                    m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
                }
                /* The actual reading is in cm */
                m_pnReadings[i] = cScanningRay.GetDistance(sIntersection.TOnRay) * 100;
            }
            else
            {
                /* No intersection */
                m_pnReadings[i] = 0;
                if (m_bShowRays)
                {
                    m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
                }
            }
            /* Apply noise to the sensor */
            if (m_bAddNoise)
            {
                m_pnReadings[i] += m_pcRNG->Uniform(m_cNoiseRange);
            }
        }
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::Reset()
    {
        memset(m_pnReadings, 0, m_unNumReadings * sizeof(long int));
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::Destroy()
    {
        delete[] m_pnReadings;
    }

    /****************************************/
    /****************************************/

    long CNewEPuckLidarDefaultSensor::GetReading(UInt32 un_idx) const
    {
        return m_pnReadings[un_idx];
    }

    /****************************************/
    /****************************************/

    size_t CNewEPuckLidarDefaultSensor::GetNumReadings() const
    {
        return m_unNumReadings;
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::PowerOn()
    {
        m_unPowerLaserState = m_unPowerLaserState | 0x1;
        m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::PowerOff()
    {
        m_unPowerLaserState = m_unPowerLaserState & 0xFE;
        m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::LaserOn()
    {
        m_unPowerLaserState = m_unPowerLaserState | 0x2;
        m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
    }

    /****************************************/
    /****************************************/

    void CNewEPuckLidarDefaultSensor::LaserOff()
    {
        m_unPowerLaserState = m_unPowerLaserState & 0xFD;
        m_pcProximityEntity->SetEnabled(m_unPowerLaserState == NEWEPUCK_POWERON_LASERON);
    }

    /****************************************/
    /****************************************/

    REGISTER_SENSOR(CNewEPuckLidarDefaultSensor,
                    "newepuck_lidar", "default",
                    "Carlo Pinciroli [ilpincy@gmail.com]",
                    "1.0",
                    "The New 3 LIDAR sensor.",
                    "This sensor accesses the E-Puck LIDAR sensor. The sensors return the\n"
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
                    "Usable");

}
// namespace argos
// {

//     /****************************************/
//     /****************************************/

//     static CRange<Real> UNIT(0.0f, 12.0f);

//     /****************************************/
//     /****************************************/

//     CNewEPuckLidarSensor::CNewEPuckLidarSensor() : m_pcEmbodiedEntity(NULL),
//                                          m_bShowRays(false),
//                                          m_pcRNG(NULL),
//                                          m_bAddNoise(false),
//                                          m_cSpace(CSimulator::GetInstance().GetSpace())
//     {
//     }

//     /****************************************/
//     /****************************************/

//     void CNewEPuckLidarSensor::Init(TConfigurationNode &t_tree)
//     {
//         try
//         {
//             CCI_NewEPuckLidarSensor::Init(t_tree);
//             /* Show rays? */
//             GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
//             /* Parse noise level */
//             Real fNoiseLevel = 0.0f;
//             GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
//             if (fNoiseLevel < 0.0f)
//             {
//                 THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level"
//                                      << " of the nwepuck proximity sensor");
//             }
//             else if (fNoiseLevel > 0.0f)
//             {
//                 m_bAddNoise = true;
//                 m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
//                 m_pcRNG = CRandom::CreateRNG("argos");
//             }
//             m_tReadings.resize(719); // will store only Lidar measurements
//         }
//         catch (CARGoSException &ex)
//         {
//             THROW_ARGOSEXCEPTION_NESTED("Initialization error in newepuck lidar sensor", ex);
//         }
//     }

//     /****************************************/
//     /****************************************/

//     void CNewEPuckLidarSensor::SetRobot(CComposableEntity &c_entity)
//     {
//         try
//         {
//             m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
//             m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
//             m_pcProximityEntity = &(c_entity.GetComponent<CProximitySensorEquippedEntity>("proximity_sensors"));
//             m_pcProximityEntity->Enable();
//         }
//         catch (CARGoSException &ex)
//         {
//             THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the newepuck proximity default sensor", ex);
//         }
//     }

//     /****************************************/
//     /****************************************/

//     void CNewEPuckLidarSensor::Update()
//     {
//         /* Ray used for scanning the environment for obstacles */
//         CRay3 cScanningRay;
//         CVector3 cRayStart, cRayEnd;
//         /* Buffers to contain data about the intersection */
//         SEmbodiedEntityIntersectionItem sIntersection;
//         /* Go through the sensors */
//         /* skip the first 8 that are the sensor ring */
//         UInt32 offset = 8;

//         for (UInt32 i = 0; i < m_tReadings.size(); ++i)
//         {
//             /* Compute ray for sensor i */
//             cRayStart = m_pcProximityEntity->GetSensor(i + offset).Offset;
//             cRayStart.Rotate(m_pcEmbodiedEntity->GetOriginAnchor().Orientation);
//             cRayStart += m_pcEmbodiedEntity->GetOriginAnchor().Position;
//             cRayEnd = m_pcProximityEntity->GetSensor(i + offset).Offset;
//             cRayEnd += m_pcProximityEntity->GetSensor(i + offset).Direction;
//             cRayEnd.Rotate(m_pcEmbodiedEntity->GetOriginAnchor().Orientation);
//             cRayEnd += m_pcEmbodiedEntity->GetOriginAnchor().Position;
//             cScanningRay.Set(cRayStart, cRayEnd);
//             /* Compute reading */
//             /* Get the closest intersection */
//             if (GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
//                                                          cScanningRay,
//                                                          *m_pcEmbodiedEntity))
//             {
//                 /* There is an intersection */
//                 if (m_bShowRays)
//                 {
//                     m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
//                                                                  sIntersection.TOnRay);
//                     m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
//                 }
//                 m_tReadings[i].Value = CalculateReading(cScanningRay.GetDistance(sIntersection.TOnRay));
//             }
//             else
//             {
//                 /* No intersection */
//                 m_tReadings[i].Value = 0.0f;
//                 if (m_bShowRays)
//                 {
//                     m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
//                 }
//             }
//             /* Apply noise to the sensor */
//             if (m_bAddNoise)
//             {
//                 m_tReadings[i].Value += m_pcRNG->Uniform(m_cNoiseRange);
//             }
//             /* Trunc the reading between 0 and 1 */
//             UNIT.TruncValue(m_tReadings[i].Value);
//         }
//     }

//     /****************************************/
//     /****************************************/

//     void CNewEPuckLidarSensor::Reset()
//     {
//         for (UInt32 i = 0; i < GetReadings().size(); ++i)
//         {
//             m_tReadings[i].Value = 0.0f;
//         }
//     }

//     /****************************************/
//     /****************************************/

//     Real CNewEPuckLidarSensor::CalculateReading(Real f_distance)
//     {

//         // value is :
//         // 0 if the sensor is too close to the object
//         // 12 if there are no objects in range
//         // the distance otherwise
//         // this is based on my quick empirical analysis of the sensor,
//         // these values should maybe be adapted
//         // especially the out of range value that I could not test and only deduce
//         Real value = 0.0f;
//         if (0.1 <= f_distance && f_distance <= 10) // range of ydlidar X4 : 0.1 - 10 meters
//         {
//             value = f_distance;
//         }
//         else
//         {
//             if (f_distance > 10)
//             {
//                 value = 12.0f;
//             }
//         }
//         return value;
//     }

//     /****************************************/
//     /****************************************/

//     REGISTER_SENSOR(CNewEPuckLidarSensor,
//                     "newepuck_lidar", "default",
//                     "Raffaele Todesco [raffaele.todesco@ulb.be]",
//                     "1.0",
//                     "The new lidar X4 sensor",
//                     "This sensor accesses a set of proximity sensors. The sensors all return a value\n"
//                     "between 0 and 12, where 12 means nothing within range and 0 means an external\n"
//                     "object is touching the sensor. Values between 0.1 and 10.0 are the distance between\n"
//                     "the occluding object and the sensor.\n"
//                     "For usage, refer to [ci_newepuck_lidar_sensor.h]\n\n"
//                     "REQUIRED XML CONFIGURATION\n\n"
//                     "   <controllers>\n"
//                     "      ...\n"
//                     "      <my_controller>\n"
//                     "         ...\n"
//                     "         <sensors>\n"
//                     "            ...\n"
//                     "            <newepuck_proximity implementation=\"default\"/>\n"
//                     "            ...\n"
//                     "         <sensors/>\n"
//                     "         ...\n"
//                     "      <my_controller/>\n"
//                     "      ...\n"
//                     "   <controllers>\n\n"
//                     "OPTIONAL XML CONFIGURATION\n\n"
//                     "It is possible to draw the rays shot by the proximity sensor in the OpenGL\n"
//                     "visualization. This can be useful for sensor debugging but also to understand\n"
//                     "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
//                     "they are not obstructed and in purple when they are. In case a ray is\n"
//                     "obstructed, a black dot is drawn where the intersection occurred.\n"
//                     "To turn this functionality on, add the attribute \"show_rays\".\n\n"
//                     "   <controllers>\n"
//                     "      ...\n"
//                     "      <my_controller>\n"
//                     "         ...\n"
//                     "         <sensors>\n"
//                     "            ...\n"
//                     "            <newepuck_proximity implementation=\"default\" show_rays=\"true\"/>\n"
//                     "            ...\n"
//                     "         <sensors/>\n"
//                     "         ...\n"
//                     "      <my_controller/>\n"
//                     "      ...\n"
//                     "   <controllers>\n\n"
//                     "It is possible to add uniform noise to the sensors, thus matching the\n"
//                     "characteristics of a real robot better. This can be done with the attribute\n"
//                     "\"noise_level\", whose allowed range is in [-1,1] and is added to the calculated\n"
//                     "reading..\n\n"
//                     "   <controllers>\n"
//                     "      ...\n"
//                     "      <my_controller>\n"
//                     "         ...\n"
//                     "         <sensors>\n"
//                     "            ...\n"
//                     "            <newepuck_proximity implementation=\"default\" noise_level=\"0.1\"/>\n"
//                     "            ...\n"
//                     "         <sensors/>\n"
//                     "         ...\n"
//                     "      <my_controller/>\n"
//                     "      ...\n"
//                     "   <controllers>\n",
//                     "Usable");
// }
