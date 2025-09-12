/**
 * @file <myepuck/simulator/dynamics2d_myepuck_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_MYEPUCK_MODEL_H
#define DYNAMICS2D_MYEPUCK_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DMyEPuckModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <myepuck/simulator/myepuck_entity.h>

namespace argos {

   class CDynamics2DMyEPuckModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DMyEPuckModel(CDynamics2DEngine& c_engine,
                              CMyEPuckEntity& c_entity);
      virtual ~CDynamics2DMyEPuckModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      
   private:

      CMyEPuckEntity& m_cMyEPuckEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

   };

}

#endif
