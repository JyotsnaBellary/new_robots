#ifndef TURTLEBOT4_COLORED_BLOB_PERSPECTIVE_CAMERA_DEFAULT_SENSOR_H
#define TURTLEBOT4_COLORED_BLOB_PERSPECTIVE_CAMERA_DEFAULT_SENSOR_H

namespace argos {
   class CTurtlebot4ColoredBlobPerspectiveCameraDefaultSensor;
   class CPerspectiveCameraEquippedEntity;
   class CLEDEntity;
   class CControllableEntity;
   class CTurtlebot4PerspectiveCameraLEDCheckOperation;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>

namespace argos {

   class CTurtlebot4ColoredBlobPerspectiveCameraDefaultSensor : public CCI_ColoredBlobPerspectiveCameraSensor,
                                                      public CSimulatedSensor {

   public:

      CTurtlebot4ColoredBlobPerspectiveCameraDefaultSensor();

      virtual ~CTurtlebot4ColoredBlobPerspectiveCameraDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual void Enable();

      virtual void Disable();

      /**
       * Returns true if the rays must be shown in the GUI.
       * @return true if the rays must be shown in the GUI.
       */
      inline bool IsShowRays() {
         return m_bShowRays;
      }

      /**
       * Sets whether or not the rays must be shown in the GUI.
       * @param b_show_rays true if the rays must be shown, false otherwise
       */
      inline void SetShowRays(bool b_show_rays) {
         m_bShowRays = b_show_rays;
      }

   protected:

      CPerspectiveCameraEquippedEntity*    m_pcCamEntity;
      CControllableEntity*                 m_pcControllableEntity;
      CEmbodiedEntity*                     m_pcEmbodiedEntity;
      CPositionalIndex<CLEDEntity>*        m_pcLEDIndex;
      CPositionalIndex<CEmbodiedEntity>*   m_pcEmbodiedIndex;
      CTurtlebot4PerspectiveCameraLEDCheckOperation* m_pcOperation;
      bool                                 m_bShowRays;

   };
}

#endif
