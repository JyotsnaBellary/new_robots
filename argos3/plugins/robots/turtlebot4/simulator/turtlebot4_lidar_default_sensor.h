/**
 * @file <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_lidar_default_sensor.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef TURTLEBOT4_LIDAR_DEFAULT_SENSOR_H
#define TURTLEBOT4_LIDAR_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CTurtlebot4LIDARDefaultSensor;
}

#include <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_lidar_sensor.h>
#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>

namespace argos {

   class CTurtlebot4LIDARDefaultSensor : public CCI_Turtlebot4LIDARSensor,
                                        public CSimulatedSensor {

   public:

      CTurtlebot4LIDARDefaultSensor();

      virtual ~CTurtlebot4LIDARDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual long GetReading(UInt32 un_idx) const;

      virtual size_t GetNumReadings() const;

      virtual void PowerOn();

      virtual void PowerOff();

      virtual void LaserOn();

      virtual void LaserOff();

   private:

      /** Readings of the LIDAR sensor */
      long int* m_pnReadings;

      /** Number of readings of the LIDAR sensor */
      size_t m_unNumReadings;

      /** Power and Laser states */
      UInt8 m_unPowerLaserState;

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to proximity sensor equipped entity associated to this sensor */
      CProximitySensorEquippedEntity* m_pcProximityEntity;

      /** Reference to controllable entity associated to this sensor */
      CControllableEntity* m_pcControllableEntity;

      /** Flag to show rays in the simulator */
      bool m_bShowRays;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      /** Reference to the space */
      CSpace& m_cSpace;
   };

}

#endif
