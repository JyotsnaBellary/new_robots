/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_turtlebot4_proximity_sensor.h>
 *
 * @author Danesh Tarapore <daneshtarapore@gmail.com>
 */

#ifndef CCI_TURTLEBOT4_PROXIMITY_SENSOR_H
#define CCI_TURTLEBOT4_PROXIMITY_SENSOR_H

namespace argos
{
   class CCI_Turtlebot4ProximitySensor;
}

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_sensor.h>

namespace argos
{

   class CCI_Turtlebot4ProximitySensor : public CCI_Sensor
   {

   public:
      /**
       * Class constructor
       */
      CCI_Turtlebot4ProximitySensor();

      /**
       * Class destructor
       */
      virtual ~CCI_Turtlebot4ProximitySensor() {}

      struct SReading
      {
         Real Value;
         CRadians Angle;

         SReading() : Value(0.0f) {}

         SReading(Real f_value,
                  const CRadians &c_angle) : Value(f_value),
                                             Angle(c_angle) {}
      };

      typedef std::vector<SReading> TReadings;

      /**
       * Returns the readings of this sensor
       */
      const TReadings &GetReadings() const;
      // {
      //    return m_tReadings;
      // }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State *pt_lua_state);

      virtual void ReadingsToLuaState(lua_State *pt_lua_state);
#endif

   protected:
      TReadings m_tReadings;
   };

   std::ostream& operator<<(std::ostream& c_os, const CCI_Turtlebot4ProximitySensor::SReading& s_reading);
   std::ostream& operator<<(std::ostream& c_os, const CCI_Turtlebot4ProximitySensor::TReadings& t_readings);

}

#endif
