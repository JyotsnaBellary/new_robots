/**
 * @file <argos3/plugins/robots/newepuck/control_interface/ci_newepuck_lidar_sensor.cpp>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */

#include "ci_newepuck_lidar_sensor.h"
#include <argos3/core/utility/math/angles.h>

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_NewEPuckLIDARSensor::CCI_NewEPuckLIDARSensor() {
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_NewEPuckLIDARSensor::CreateLuaState(lua_State* pt_lua_state) {
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_NewEPuckLIDARSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
   }
#endif


   /****************************************/
   /****************************************/

}
