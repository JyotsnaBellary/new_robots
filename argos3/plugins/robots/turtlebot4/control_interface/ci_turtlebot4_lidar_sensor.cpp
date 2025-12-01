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
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_Turtlebot4LIDARSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
   }
#endif


   /****************************************/
   /****************************************/

}
