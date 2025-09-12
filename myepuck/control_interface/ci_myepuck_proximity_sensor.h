/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_myepuck_proximity_sensor.h>
 *
 * @author Danesh Tarapore <daneshtarapore@gmail.com>
 */

#ifndef CCI_MYEPUCK_PROXIMITY_SENSOR_H
#define CCI_MYEPUCK_PROXIMITY_SENSOR_H
struct lua_State;
namespace argos {
   class CCI_MyEPuckProximitySensor;
}

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_MyEPuckProximitySensor : public CCI_Sensor {

   public:

      CCI_MyEPuckProximitySensor();
      virtual ~CCI_MyEPuckProximitySensor() {}

      struct SReading 
      {
         Real Value;
         CRadians Angle;

         SReading() :
            Value(0.0f) {}

         SReading(Real f_value,
                  const CRadians& c_angle) :
            Value(f_value),
            Angle(c_angle) {}
      };

      typedef std::vector<SReading> TReadings;



      inline const TReadings& GetReadings() const 
      {
         return m_tReadings;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      TReadings m_tReadings;
   };

}

#endif
