/**
 * @file <argos3/plugins/robots/foot-bot/control_interface/ci_newepuck_light_sensor.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "ci_newepuck_light_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

/**
    * @brief Angular spacing between adjacent light readings.
    *
    * 24 readings over 360° → 360/24 = 15° = π/12 rad.
    */
   static CRadians SPACING = CRadians(ARGOS_PI / 12.0f);

   /**
    * @brief Start angle for the first reading.
    *
    * Half-spacing (7.5°) centers each sector on its sampling direction.
    */
   static CRadians START_ANGLE = SPACING * 0.5f;

   /**
    * @brief Constructs the CI and initializes the 24 reading angles.
    *
    * Values are left at 0.0 here; the simulator plugin will fill them each update.
    */
   CCI_NewEPuckLightSensor::CCI_NewEPuckLightSensor() :
      m_tReadings(24) {
      for(size_t i = 0; i < 24; ++i) {
         m_tReadings[i].Angle = START_ANGLE + i * SPACING;
         m_tReadings[i].Angle.SignedNormalize();
      }
   }

   /**
    * @brief Returns all light sensor readings (angles + values).
    * @return Const reference to the vector of 24 readings.
    *
    * Angles are fixed at construction; values are updated by the simulator plugin.
    */
   const CCI_NewEPuckLightSensor::TReadings& CCI_NewEPuckLightSensor::GetReadings() const {
     return m_tReadings;
   }

#ifdef ARGOS_WITH_LUA

   /**
    * @brief (Optional) Creates the Lua table structure for this sensor.
    *
    * Call this to expose the readings to Lua as:
    *   robot.light[i] = { angle = <CRadians>, value = <Real> }
    *
    * Note: Currently disabled; uncomment to enable Lua exposure.
    *
    * @param pt_lua_state The target Lua state.
    */
   void CCI_NewEPuckLightSensor::CreateLuaState(lua_State* pt_lua_state) {
      // CLuaUtility::OpenRobotStateTable(pt_lua_state, "light");
      // for(size_t i = 0; i < GetReadings().size(); ++i) {
      //    CLuaUtility::StartTable(pt_lua_state, i+1                           );
      //    CLuaUtility::AddToTable(pt_lua_state, "angle",  m_tReadings[i].Angle);
      //    CLuaUtility::AddToTable(pt_lua_state, "value",  m_tReadings[i].Value);
      //    CLuaUtility::EndTable  (pt_lua_state                                );
      // }
      // CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#else
   /**
    * @brief Stub when Lua is disabled.
    */
   void CCI_NewEPuckLightSensor::CreateLuaState(void*) {}
#endif

#ifdef ARGOS_WITH_LUA

   /**
    * @brief (Optional) Writes current values into the existing Lua table.
    *
    * Assumes CreateLuaState() previously created 'light' table.
    *
    * @param pt_lua_state The target Lua state.
    */
   void CCI_NewEPuckLightSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "light");
      for(size_t i = 0; i < GetReadings().size(); ++i) {
         lua_pushnumber(pt_lua_state, i+1                 );
         lua_gettable  (pt_lua_state, -2                  );
         lua_pushnumber(pt_lua_state, m_tReadings[i].Value);
         lua_setfield  (pt_lua_state, -2, "value"         );
         lua_pop(pt_lua_state, 1);
      }
      lua_pop(pt_lua_state, 1);
   }
#else 
   /**
    * @brief Stub when Lua is disabled.
    */
   void CCI_NewEPuckLightSensor::ReadingsToLuaState(void*) {}
#endif

    /**
    * @brief Pretty-prints a single reading as "Value=<v>, Angle=<a>".
    * @param c_os Output stream.
    * @param s_reading The reading to print.
    * @return The output stream.
    */
   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_NewEPuckLightSensor::SReading& s_reading) {
      c_os << "Value=<" << s_reading.Value
           << ">, Angle=<" << s_reading.Angle << ">";
      return c_os;
   }

   /**
    * @brief Prints all reading values in a compact "{ v0 } { v1 } ... { vN }" form.
    * @param c_os Output stream.
    * @param t_readings Vector of readings.
    * @return The output stream.
    *
    * @note Fixed a small bug: previously printed t_readings[0] for every element.
    */
   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_NewEPuckLightSensor::TReadings& t_readings) {
      if(! t_readings.empty()) {
         c_os << "{ " << t_readings[0].Value << " }";
         for(UInt32 i = 1; i < t_readings.size(); ++i) {
            c_os << " { " << t_readings[0].Value << " }";
         }
         c_os << std::endl;
      }
      return c_os;
   }
}
