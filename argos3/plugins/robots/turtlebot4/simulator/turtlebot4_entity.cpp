/**
 * @file <turtlebot4/simulator/turtlebot4_entity.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "turtlebot4_entity.h"
#include "turtlebot4_measures.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/omnidirectional_camera_equipped_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos {


   CTurtlebot4Entity::CTurtlebot4Entity() :
      CComposableEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcGroundSensorEquippedEntity(nullptr),
      m_pcLEDEquippedEntity(nullptr),
      m_pcProximitySensorEquippedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcOmnidirectionalCameraEquippedEntity(nullptr)
      // m_pcPerspectiveCameraEquippedEntity(NULL)
      {
   }

   /****************************************/
   /****************************************/

   CTurtlebot4Entity::CTurtlebot4Entity(const std::string& str_id,
                              const std::string& str_controller_id,
                              const CVector3& c_position,
                              const CQuaternion& c_orientation,
                              Real f_rab_range,
                              size_t un_rab_data_size,
                              const std::string& str_bat_model,
                              const CRadians& c_omnicam_aperture,
                                  bool b_perspcam_front,
                              const CRadians& c_perspcam_aperture,
                              Real f_perspcam_focal_length,
                              Real f_perspcam_range) :
                              
      CComposableEntity(nullptr, str_id),
      m_pcControllableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcGroundSensorEquippedEntity(nullptr),
      m_pcLEDEquippedEntity(nullptr),
      m_pcProximitySensorEquippedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcOmnidirectionalCameraEquippedEntity(nullptr)
      // m_pcPerspectiveCameraEquippedEntity(nullptr)
       {
      try {
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);

         // /* LED equipped entity */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLEDRing(
            CVector3(0.0f, 0.0f, TURTLEBOT4_LED_RING_ELEVATION),
            TURTLEBOT4_LED_RING_RADIUS,
            TURTLEBOT4_LED_RING_START_ANGLE,
            8,
            m_pcEmbodiedEntity->GetOriginAnchor());

         /* LIDAR sensor equipped entity */
         m_pcLIDARSensorEquippedEntity =
            new CProximitySensorEquippedEntity(this,
                                               "lidar");
         AddComponent(*m_pcLIDARSensorEquippedEntity);

         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity =
            new CProximitySensorEquippedEntity(this,
                                               "proximity");
         AddComponent(*m_pcProximitySensorEquippedEntity);

         CRadians sensor_angle[7] = {
    -CRadians::PI / 2.75f,             // -65,3°
    -CRadians::PI / 4.736f,             // -38°
    -CRadians::PI / 9.0f,                   // -20° 
    -CRadians::PI / 60.0f,              // -3°
    CRadians::PI / 12.630f,              // +60°
    CRadians::PI / 5.294f,              // +60°
    CRadians::PI / 2.7565f             // +90° (rightmost)            // slight extra left bias if needed
         };

         CRadians cAngle;
         CVector3 cOff, cDir, c_center = CVector3(0.0f, 0.0f, TURTLEBOT4_IR_SENSOR_RING_ELEVATION);
         for(UInt32 i = 0; i < 7; ++i)
         {
            cAngle = sensor_angle[i];
            cAngle.SignedNormalize();
            cOff.Set(TURTLEBOT4_IR_SENSOR_RING_RADIUS, 0.0f, 0.0f);
            cOff.RotateZ(cAngle);
            cOff += c_center;
            cDir.Set(TURTLEBOT4_IR_SENSOR_RING_RANGE, 0.0f, 0.0f);
            cDir.RotateZ(cAngle);
            m_pcProximitySensorEquippedEntity->AddSensor(cOff, cDir, TURTLEBOT4_IR_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
         }

         /* Omnidirectional camera equipped entity */
         m_pcOmnidirectionalCameraEquippedEntity =
            new COmnidirectionalCameraEquippedEntity(this,
                                                     "omnidirectional_camera_0",
                                                     c_omnicam_aperture,
                                                     CVector3(0.0f,
                                                              0.0f,
                                                              OMNIDIRECTIONAL_CAMERA_ELEVATION));
         AddComponent(*m_pcOmnidirectionalCameraEquippedEntity);
         
         /* Perspective camera equipped entity */
         // CQuaternion cPerspCamOrient(CRadians::PI_OVER_TWO, CVector3::Y);
         // SAnchor& cPerspCamAnchor = m_pcEmbodiedEntity->AddAnchor("perspective_camera",
         //                                                          CVector3(0.0, 0.0, 0.0),
                                                                  // cPerspCamOrient);
         // m_pcPerspectiveCameraEquippedEntity =
         //    new CPerspectiveCameraEquippedEntity(this,
         //                                         "perspective_camera_0",
         //                                         c_perspcam_aperture,
         //                                         f_perspcam_focal_length,
         //                                         f_perspcam_range,
         //                                         640, 480,
         //                                         cPerspCamAnchor);
         // AddComponent(*m_pcPerspectiveCameraEquippedEntity);

         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
            new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.1425, 0.0268),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.1425, -0.0268),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0879, 0.109),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0879, -0.109),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());

         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);

         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4Entity::Init(TConfigurationNode& t_tree) {
      try {
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);
         /* LED equipped entity, with LEDs [0-11] and beacon [12] */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLEDRing(
            CVector3(0.0f, 0.0f, TURTLEBOT4_LED_RING_ELEVATION),
            TURTLEBOT4_LED_RING_RADIUS,
            TURTLEBOT4_LED_RING_START_ANGLE,
            8,
            m_pcEmbodiedEntity->GetOriginAnchor());
         
         /* LIDAR sensor equipped entity */
         m_pcLIDARSensorEquippedEntity =
            new CProximitySensorEquippedEntity(this,
                                               "lidar");
         AddComponent(*m_pcLIDARSensorEquippedEntity);
         
         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity =
            new CProximitySensorEquippedEntity(this,
                                               "proximity");
         AddComponent(*m_pcProximitySensorEquippedEntity);


	      CRadians sensor_angle[7] = {
    -CRadians::PI / 2.75f,             // -65,3°
    -CRadians::PI / 4.736f,             // -38°
    -CRadians::PI / 9.0f,                   // -20° 
    -CRadians::PI / 60.0f,              // -3°
    CRadians::PI / 12.630f,              // +60°
    CRadians::PI / 5.294f,              // +60°
    CRadians::PI / 2.7565f             // +90° (rightmost)            // slight extra left bias if needed
         };
         CRadians cAngle;
         CVector3 cOff, cDir, c_center = CVector3(0.0f, 0.0f, TURTLEBOT4_IR_SENSOR_RING_ELEVATION);
         for(UInt32 i = 0; i < 7; ++i)
         {
            cAngle = sensor_angle[i];
            cAngle.SignedNormalize();
            cOff.Set(TURTLEBOT4_IR_SENSOR_RING_RADIUS, 0.0f, 0.0f);
            cOff.RotateZ(cAngle);
            cOff += c_center;
            cDir.Set(TURTLEBOT4_IR_SENSOR_RING_RANGE, 0.0f, 0.0f);
            cDir.RotateZ(cAngle);
            m_pcProximitySensorEquippedEntity->AddSensor(cOff, cDir, TURTLEBOT4_IR_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
         }

         /* Omnidirectional camera equipped entity */
         CDegrees cAperture(70.0f);
         GetNodeAttributeOrDefault(t_tree, "omnidirectional_camera_aperture", cAperture, cAperture);
         m_pcOmnidirectionalCameraEquippedEntity =
            new COmnidirectionalCameraEquippedEntity(this,
                                                     "omnidirectional_camera_0",
                                                     ToRadians(cAperture),
                                                     CVector3(0.0f,
                                                              0.0f,
                                                              OMNIDIRECTIONAL_CAMERA_ELEVATION));
         AddComponent(*m_pcOmnidirectionalCameraEquippedEntity);

         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
            new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.1425, 0.0268),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.1425, -0.0268),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0879, 0.109),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0879, -0.109),
            CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
            m_pcEmbodiedEntity->GetOriginAnchor());
         
         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this);
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4Entity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4Entity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

#define UPDATE(COMPONENT) if(COMPONENT->IsEnabled()) COMPONENT->Update();

   void CTurtlebot4Entity::UpdateComponents() {
      UPDATE(m_pcLEDEquippedEntity);
      UPDATE(m_pcGroundSensorEquippedEntity);
      // UPDATE(m_pcPerspectiveCameraEquippedEntity)
   }

   /****************************************/
   /****************************************/
   
   REGISTER_ENTITY(CTurtlebot4Entity,
                   "turtlebot4",
                   "Jyotsna Bellary [jyotsnabellary@gmail.com]",
                   "1.0",
                   "The turtlebot4 robot.",
                   "The turtlebot4 is a open-hardware, extensible robot intended for education. In its\n"
                   "simplest form, it is a two-wheeled robot equipped with proximity sensors,\n"
                   "ground sensors, light sensors, a microphone, a frontal camera, and a ring of\n"
                   "red LEDs. More information is available at http://www.turtlebot4.org\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <turtlebot4 id=\"eb0\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,90,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </turtlebot4>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'body/position' attribute specifies the position of the pucktom point of the\n"
                   "turtlebot4 in the arena. When the robot is untranslated and unrotated, the\n"
                   "pucktom point is in the origin and it is defined as the middle point between\n"
                   "the two wheels on the XY plane and the lowest point of the robot on the Z\n"
                   "axis, that is the point where the wheels touch the floor. The attribute values\n"
                   "are in the X,Y,Z order.\n"
                   "The 'body/orientation' attribute specifies the orientation of the turtlebot4. All\n"
                   "rotations are performed with respect to the pucktom point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the robot is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller/config' attribute is used to assign a controller to the\n"
                   "turtlebot4. The value of the attribute must be set to the id of a previously\n"
                   "defined controller. Controllers are defined in the <controllers> XML subtree.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "You can set the emission range of the range-and-bearing system. By default, a\n"
                   "message sent by an turtlebot4 can be received up to 80cm. By using the 'rab_range'\n"
                   "attribute, you can change it to, i.e., 4m as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <turtlebot4 id=\"eb0\" rab_range=\"4\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,90,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </turtlebot4>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can also set the data sent at each time step through the range-and-bearing\n"
                   "system. By default, a message sent by an turtlebot4 is 2 bytes long. By using the\n"
                   "'rab_data_size' attribute, you can change it to, i.e., 20 bytes as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <turtlebot4 id=\"eb0\" rab_data_size=\"20\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,90,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </turtlebot4>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can also configure the battery of the robot. By default, the battery never\n"
                   "depletes. You can choose among several battery discharge models, such as\n"
                   "- time: the battery depletes by a fixed amount at each time step\n"
                   "- motion: the battery depletes according to how the robot moves\n"
                   "- time_motion: a combination of the above models.\n"
                   "You can define your own models too. Follow the examples in the file\n"
                   "argos3/src/plugins/simulator/entities/battery_equipped_entity.cpp.\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <turtlebot4 id=\"eb0\"\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery model=\"time\" factor=\"1e-5\"/>\n"
                   "    </turtlebot4>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <turtlebot4 id=\"eb0\"\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery model=\"motion\" pos_factor=\"1e-3\"\n"
                   "                              orient_factor=\"1e-3\"/>\n"
                   "    </turtlebot4>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <turtlebot4 id=\"eb0\"\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery model=\"time_motion\" time_factor=\"1e-5\"\n"
                   "                                   pos_factor=\"1e-3\"\n"
                   "                                   orient_factor=\"1e-3\"/>\n"
                   "    </turtlebot4>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "Finally, you can change the parameters of the camera. You can set its aperture,\n"
                   "focal length, and range with the attributes 'camera_aperture',\n"
                   "'camera_focal_length', and 'camera_range', respectively. The default values are:\n"
                   "30 degrees for aperture, 0.035 for focal length, and 2 meters for range. Check\n"
                   "the following example:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <eye-bot id=\"eb0\"\n"
                   "             camera_aperture=\"45\"\n"
                   "             camera_focal_length=\"0.07\"\n"
                   "             camera_range=\"10\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </eye-bot>\n"
                   "    ...\n"
                   "  </arena>\n\n",
                   "Under development"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CTurtlebot4Entity);

   /****************************************/
   /****************************************/

}
