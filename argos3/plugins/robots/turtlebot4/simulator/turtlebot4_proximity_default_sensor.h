/**
 * @file <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_proximity_default_sensor.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef TURTLEBOT4_PROXIMITY_DEFAULT_SENSOR_H
#define TURTLEBOT4_PROXIMITY_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CTurtlebot4ProximityDefaultSensor;
}

#include <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_proximity_sensor.h>
#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>

namespace argos {

   class CTurtlebot4ProximityDefaultSensor : public CCI_Turtlebot4ProximitySensor,
                                            public CSimulatedSensor {

   public:

      CTurtlebot4ProximityDefaultSensor();

      virtual ~CTurtlebot4ProximityDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   private:

      CProximityDefaultSensor* m_pcProximityImpl;

   };

}

#endif
