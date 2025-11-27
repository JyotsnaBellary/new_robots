/**
 * @file <testbot/simulator/dynamics2d_testbot_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_TESTBOT_MODEL_H
#define DYNAMICS2D_TESTBOT_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DTestBotModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <argos3/plugins/robots/testbot/simulator/testbot_entity.h>

namespace argos {

   class CDynamics2DTestBotModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DTestBotModel(CDynamics2DEngine& c_engine,
                              CTestBotEntity& c_entity);
      virtual ~CDynamics2DTestBotModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      
   private:

      CTestBotEntity& m_cTestBotEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

   };

}

#endif
