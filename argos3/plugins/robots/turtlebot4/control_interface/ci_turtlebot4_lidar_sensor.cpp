/**
 * @file <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_lidar_sensor.cpp>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */

#include "ci_turtlebot4_lidar_sensor.h"
#include <argos3/core/utility/math/angles.h>

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_Turtlebot4LIDARSensor::CCI_Turtlebot4LIDARSensor() {
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_Turtlebot4LIDARSensor::CreateLuaState(lua_State* pt_lua_state) {
      // CLuaUtility::OpenRobotStateTable(pt_lua_state, "lidar");
      // for(size_t i = 0; i < GetReadings().size(); ++i) {
      //    CLuaUtility::AddToTable(pt_lua_state, i+1);
      //    CLuaUtility::AddToTable(pt_lua_state, m_tReadings[i]);
      // }
      // CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_Turtlebot4LIDARSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      // lua_getfield(pt_lua_state, -1, "lidar");
      // for(size_t i = 0; i < GetReadings().size(); ++i) {
      //    lua_pushnumber(pt_lua_state, i+1                 );
      //    lua_gettable  (pt_lua_state, -2                  );
      //    lua_pushnumber(pt_lua_state, m_tReadings[i].Value);
      //    lua_setfield  (pt_lua_state, -2, "value"         );
      //    lua_pop(pt_lua_state, 1);
      // }
      // lua_pop(pt_lua_state, 1);
   }
#endif


   /****************************************/
   /****************************************/

}
