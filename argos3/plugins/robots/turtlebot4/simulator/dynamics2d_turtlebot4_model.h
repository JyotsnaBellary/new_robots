/**
 * @file <turtlebot4/simulator/dynamics2d_turtlebot4_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_TURTLEBOT4_MODEL_H
#define DYNAMICS2D_TURTLEBOT4_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DTurtlebot4Model;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_entity.h>

namespace argos {

   class CDynamics2DTurtlebot4Model : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DTurtlebot4Model(CDynamics2DEngine& c_engine,
                              CTurtlebot4Entity& c_entity);
      virtual ~CDynamics2DTurtlebot4Model();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      
   private:

      CTurtlebot4Entity& m_cTurtlebot4Entity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

   };

}

#endif
