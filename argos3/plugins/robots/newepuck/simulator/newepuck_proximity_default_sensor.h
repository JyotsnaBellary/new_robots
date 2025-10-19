/**
 * @file <argos3/plugins/robots/newepuck/simulator/newepuck_proximity_default_sensor.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef NEWEPUCK_PROXIMITY_DEFAULT_SENSOR_H
#define NEWEPUCK_PROXIMITY_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CNewEPuckProximityDefaultSensor;
}

#include <argos3/plugins/robots/newepuck/control_interface/ci_newepuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>

namespace argos {

   class CNewEPuckProximityDefaultSensor : public CCI_NewEPuckProximitySensor,
                                            public CSimulatedSensor {

   public:

      CNewEPuckProximityDefaultSensor();

      virtual ~CNewEPuckProximityDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   private:

      CProximityDefaultSensor* m_pcProximityImpl;

   };

}

#endif
