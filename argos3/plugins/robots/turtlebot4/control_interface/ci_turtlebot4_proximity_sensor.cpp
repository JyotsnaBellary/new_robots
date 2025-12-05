/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_turtlebot4_proximity_sensor.cpp>
 *
 * @author Jyotsna Bellary <jyotsnabellary@gmail.com>
 */

#include "ci_turtlebot4_proximity_sensor.h"
#include <argos3/core/utility/math/angles.h>

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos 
{

   /****************************************/
   /****************************************/

   static CRadians SPACING = CRadians(ARGOS_PI / 4.0f);
   static CRadians START_ANGLE = SPACING * 0.5f;

   /****************************************/
   /****************************************/

   CCI_Turtlebot4ProximitySensor::CCI_Turtlebot4ProximitySensor() :
      m_tReadings(7)
   {

       // Set the values for the proximity sensor angles
       m_tReadings[0].Angle = -CRadians::PI / 2.75f;
       m_tReadings[1].Angle = -CRadians::PI / 4.736f;
       m_tReadings[2].Angle = -CRadians::PI / 9.0f;
       m_tReadings[3].Angle = -CRadians::PI / 60.0f;
       m_tReadings[4].Angle =  CRadians::PI / 12.630f;
       m_tReadings[5].Angle =  CRadians::PI / 5.294f;
       m_tReadings[6].Angle =  CRadians::PI / 2.7565f;

       for(size_t i = 0; i < 7; ++i) {
           m_tReadings[i].Angle.SignedNormalize();
       }

   }

   /****************************************/
   /****************************************/

   const CCI_Turtlebot4ProximitySensor::TReadings& CCI_Turtlebot4ProximitySensor::GetReadings() const {
     return m_tReadings;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_Turtlebot4ProximitySensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "proximity");
      for(size_t i = 0; i < GetReadings().size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i+1                           );
         CLuaUtility::AddToTable(pt_lua_state, "angle",  m_tReadings[i].Angle);
         CLuaUtility::AddToTable(pt_lua_state, "value",  m_tReadings[i].Value);
         CLuaUtility::EndTable  (pt_lua_state                                );
      }
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_Turtlebot4ProximitySensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "proximity");
      for(size_t i = 0; i < GetReadings().size(); ++i) {
         lua_pushnumber(pt_lua_state, i+1                 );
         lua_gettable  (pt_lua_state, -2                  );
         lua_pushnumber(pt_lua_state, m_tReadings[i].Value);
         lua_setfield  (pt_lua_state, -2, "value"         );
         lua_pop(pt_lua_state, 1);
      }
      lua_pop(pt_lua_state, 1);
   }
#endif


   /****************************************/
   /****************************************/

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_Turtlebot4ProximitySensor::SReading& s_reading) {
      c_os << "Value=<" << s_reading.Value
           << ">, Angle=<" << s_reading.Angle << ">";
      return c_os;
   }

   /****************************************/
   /****************************************/

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_Turtlebot4ProximitySensor::TReadings& t_readings) {
      if(! t_readings.empty()) {
         c_os << "{ " << t_readings[0].Value << " }";
         for(UInt32 i = 1; i < t_readings.size(); ++i) {
            c_os << " { " << t_readings[0].Value << " }";
         }
         c_os << std::endl;
      }
      return c_os;
   }

   /****************************************/
   /****************************************/

}
