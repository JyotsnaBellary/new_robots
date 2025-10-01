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
#include <argos3/core/config.h>
#include <vector>

struct lua_State;

namespace argos
{
  class CCI_NewEPuckLidarSensor : public CCI_Sensor
  {

    private:
      static const int MEASURES_COUNT;

    public:
      /**
         * Single sensor reading
         */
      struct SReading
      {
        /**
            * Value of the reading
            */
        Real Value;

        /**
            * Angle of the reading
            */
        CRadians Angle;

        /**
            * Empty constructor
            */
        SReading() : Value(0.0f) {}

        /**
            * Filled constructor
            *
            * @param f_value the value of the reading
            *
            * @param c_angle the angle of the reading
            */
        SReading(Real f_value, const CRadians &c_angle) : Value(f_value), Angle(c_angle) {}
      };

      /**
       * Sensor ring readings
       */
      typedef std::vector<SReading> TReadings;

  public:
    /**
       * Constructor
       */
    CCI_NewEPuckLidarSensor();

    /**
       * Destructor
       */
    virtual ~CCI_NewEPuckLidarSensor() {}

    /**
       * Returns the sensor readings as an array of ranges.
       * @return The sensor readings
       */
    inline const TReadings &GetReadings() const
    {
      return m_tReadings;
    }

    /**
       * Returns a single sensor reading.
       * @param i The id of the wanted sensor
       * @return The reading of the wanted sensor
       */
    inline const SReading &GetReading(UInt16 i) const
    {
      /* TODO: maybe an exception or smthg if i < 0 or i > 7 ? */
      return m_tReadings[i];
    }

#ifdef ARGOS_WITH_LUA
    /**
       *
       * @param pt_lua_state
       */
    virtual void CreateLuaState(lua_State* pt_lua_state);

    /**
       *
       * @param pt_lua_state
       */
    virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

  protected:
    /**
       * Store the readings
       */
    TReadings m_tReadings;
  };

  /**
    * Redefine the "<<" operator for one reading
    */
  std::ostream &operator<<(std::ostream &c_os,
                           const CCI_NewEPuckLidarSensor::SReading &s_reading);

  /**
    * Redefine the "<<" operator for multiple readings
    */
  std::ostream &operator<<(std::ostream &c_os,
                           const CCI_NewEPuckLidarSensor::TReadings &t_readings);
}
#endif
