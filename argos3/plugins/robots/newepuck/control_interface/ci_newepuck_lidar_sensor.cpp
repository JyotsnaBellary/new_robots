/**
 * @file <argos3/plugins/robots/e-puck/control_interface/ci_newepuck_lidar_sensor.cpp>
 *
 * @author Raffaele Todesco- <raffaele.todesco@ulb.be>
 */

#include "ci_newepuck_lidar_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos
{
    const int CCI_NewEPuckLidarSensor::MEASURES_COUNT = 719;
    /****************************************/
    /****************************************/

    CCI_NewEPuckLidarSensor::CCI_NewEPuckLidarSensor() : m_tReadings(CCI_NewEPuckLidarSensor::MEASURES_COUNT)
    {
        // Set the values for the proximity sensor angles
        // goes from -pi to pi
        // angle_i = -pi + i * 2pi/719
        for (int i = 0; i < CCI_NewEPuckLidarSensor::MEASURES_COUNT; ++i)
        {
            m_tReadings[i].Angle = -CRadians::PI + i * 2 * CRadians::PI / CCI_NewEPuckLidarSensor::MEASURES_COUNT;
        }
    }

    /****************************************/
    /****************************************/

#ifdef ARGOS_WITH_LUA
    void CCI_NewEPuckLidarSensor::CreateLuaState(lua_State *pt_lua_state)
    {
        CLuaUtility::OpenRobotStateTable(pt_lua_state, "proximity");
        for (size_t i = 0; i < GetReadings().size(); ++i)
        {
            CLuaUtility::StartTable(pt_lua_state, i + 1);
            CLuaUtility::AddToTable(pt_lua_state, "angle", m_tReadings[i].Angle);
            CLuaUtility::AddToTable(pt_lua_state, "value", m_tReadings[i].Value);
            CLuaUtility::EndTable(pt_lua_state);
        }
        CLuaUtility::CloseRobotStateTable(pt_lua_state);
    }
#endif

    /****************************************/
    /****************************************/

#ifdef ARGOS_WITH_LUA
    void CCI_NewEPuckLidarSensor::ReadingsToLuaState(lua_State *pt_lua_state)
    {
        lua_getfield(pt_lua_state, -1, "proximity");
        for (size_t i = 0; i < GetReadings().size(); ++i)
        {
            lua_pushnumber(pt_lua_state, i + 1);
            lua_gettable(pt_lua_state, -2);
            lua_pushnumber(pt_lua_state, m_tReadings[i].Value);
            lua_setfield(pt_lua_state, -2, "value");
            lua_pop(pt_lua_state, 1);
        }
        lua_pop(pt_lua_state, 1);
    }
#endif

    /****************************************/
    /****************************************/

    std::ostream &operator<<(std::ostream &c_os,
                             const CCI_NewEPuckLidarSensor::SReading &s_reading)
    {
        c_os << "SReading("
             << s_reading.Value << ","
             << s_reading.Angle << ")";
        return c_os;
    }

    /****************************************/
    /****************************************/

    std::ostream &operator<<(std::ostream &c_os,
                             const CCI_NewEPuckLidarSensor::TReadings &t_readings)
    {
        if (!t_readings.empty())
        {
            c_os << t_readings[0];
            for (UInt16 i = 1; i < t_readings.size(); ++i)
            {
                c_os << " " << t_readings[i];
            }
        }
        return c_os;
    }

    /****************************************/
    /****************************************/
}
