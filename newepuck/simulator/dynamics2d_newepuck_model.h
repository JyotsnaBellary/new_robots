/**
 * @file <newepuck/simulator/dynamics2d_newepuck_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_NEWEPUCK_MODEL_H
#define DYNAMICS2D_NEWEPUCK_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DNewEPuckModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <newepuck/simulator/newepuck_entity.h>

namespace argos {

   class CDynamics2DNewEPuckModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DNewEPuckModel(CDynamics2DEngine& c_engine,
                              CNewEPuckEntity& c_entity);
      virtual ~CDynamics2DNewEPuckModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      
   private:

      CNewEPuckEntity& m_cNewEPuckEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

   };

}

#endif
