/**
 * @file <turtlebot4/simulator/turtlebot4_entity.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef TURTLEBOT4_ENTITY_H
#define TURTLEBOT4_ENTITY_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CTurtlebot4Entity;
   class CGroundSensorEquippedEntity;
   class CLEDEquippedEntity;
   // class CLightSensorEquippedEntity;
   // class CPerspectiveCameraEquippedEntity;
   class CProximitySensorEquippedEntity;
   // class CQuadRotorEntity;
   // class CRABEquippedEntity;
   // class CBatteryEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>

namespace argos {

   class CTurtlebot4Entity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CTurtlebot4Entity();

      CTurtlebot4Entity(const std::string& str_id,
                   const std::string& str_controller_id,
                   const CVector3& c_position = CVector3(),
                   const CQuaternion& c_orientation = CQuaternion(),
                   Real f_rab_range = 0.8f,
                   size_t un_rab_data_size = 2,
                   const std::string& str_bat_model = ""
                  );
      
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();
      
      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CGroundSensorEquippedEntity& GetGroundSensorEquippedEntity() {
         return *m_pcGroundSensorEquippedEntity;
      }

      inline CLEDEquippedEntity& GetLEDEquippedEntity() {
         return *m_pcLEDEquippedEntity;
      }

      // inline CLightSensorEquippedEntity& GetLightSensorEquippedEntity() {
      //    return *m_pcLightSensorEquippedEntity;
      // }

      inline CProximitySensorEquippedEntity& GetLidarSensorEquippedEntity() {
         return *m_pcLIDARSensorEquippedEntity;
      }

      inline CProximitySensorEquippedEntity& GetProximitySensorEquippedEntity() {
         return *m_pcProximitySensorEquippedEntity;
      }

      // inline CRABEquippedEntity& GetRABEquippedEntity() {
      //    return *m_pcRABEquippedEntity;
      // }

      inline CWheeledEntity& GetWheeledEntity() {
         return *m_pcWheeledEntity;
      }

      // inline CBatteryEquippedEntity& GetBatterySensorEquippedEntity() {
      //     return *m_pcBatteryEquippedEntity;
      // }

      virtual std::string GetTypeDescription() const {
         return "turtlebot4";
      }

      // inline CQuadRotorEntity& GetQuadRotorEntity() {
      //    return *m_pcQuadRotorEntity;
      // }

   private:

      void SetLEDPosition();

   private:

      CControllableEntity*                   m_pcControllableEntity;
      CEmbodiedEntity*                       m_pcEmbodiedEntity;
      CGroundSensorEquippedEntity*           m_pcGroundSensorEquippedEntity;
      CLEDEquippedEntity*                    m_pcLEDEquippedEntity;
      // CLightSensorEquippedEntity*            m_pcLightSensorEquippedEntity;
      CProximitySensorEquippedEntity*        m_pcProximitySensorEquippedEntity;
      CProximitySensorEquippedEntity*        m_pcLIDARSensorEquippedEntity;
      // CRABEquippedEntity*                    m_pcRABEquippedEntity;
      CWheeledEntity*                        m_pcWheeledEntity;
      // CBatteryEquippedEntity*                m_pcBatteryEquippedEntity;
      // CQuadRotorEntity*                      m_pcQuadRotorEntity;
      // CPerspectiveCameraEquippedEntity*      m_pcPerspectiveCameraEquippedEntity;
   };

}

#endif
