/**
 * @file <argos3/plugins/robots/newepuck/control_interface/ci_newepuck_lidar_sensor.h>
 *
 * @brief This file provides the definition of the newepuck YD lidar X4
 * sensor.
 *
 * This sensor provides 719 measures of range detection, from -pi to pi.
 * @author Raffaele Todesco - <raffaele.todesco@ulb.be>
 */

#ifndef CCI_NEWEPUCK_LIDAR_SENSOR_H
#define CCI_NEWEPUCK_LIDAR_SENSOR_H

namespace argos {
  class CCI_NewEPuckLidarSensor;
}

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_sensor.h>
// #include <argos3/core/config.h>
// #include <vector>

namespace argos
{
  class CCI_NewEPuckLidarSensor : public CCI_Sensor
  {

    public:

      /**
       * Class Constructor
       */
    CCI_NewEPuckLidarSensor();

      /**
       * Class destructor
       */
      virtual ~CCI_NewEPuckLidarSensor() {}

      /**
       * Returns the readings of this sensor
       */
      virtual long GetReading(UInt32 un_idx) const = 0;

      /**
       * Returns the readings of this sensor
       */
      virtual size_t GetNumReadings() const = 0;

      /*
       * Switches the sensor power on.
       */
      virtual void PowerOn() = 0;

      /*
       * Switches the sensor power off.
       */
      virtual void PowerOff() = 0;

      /*
       * Switches the laser on.
       */
      virtual void LaserOn() = 0;

      /*
       * Switches the laser off.
       */
      virtual void LaserOff() = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   };

}

#endif