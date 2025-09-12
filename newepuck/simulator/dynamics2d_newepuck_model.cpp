/**
 * @file <newepuck/simulator/dynamics2d_newepuck_model.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "dynamics2d_newepuck_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real NEWEPUCK_MASS                = 0.4f;

   static const Real NEWEPUCK_RADIUS              = 0.035f;
   static const Real NEWEPUCK_INTERWHEEL_DISTANCE = 0.053f;
   static const Real NEWEPUCK_HEIGHT              = 0.086f;

   static const Real NEWEPUCK_MAX_FORCE           = 1.5f;
   static const Real NEWEPUCK_MAX_TORQUE          = 1.5f;

   enum NEWEPUCK_WHEELS {
      NEWEPUCK_LEFT_WHEEL = 0,
      NEWEPUCK_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CDynamics2DNewEPuckModel::CDynamics2DNewEPuckModel(CDynamics2DEngine& c_engine,
                                                CNewEPuckEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cNewEPuckEntity(c_entity),
      m_cWheeledEntity(m_cNewEPuckEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      NEWEPUCK_MAX_FORCE,
                      NEWEPUCK_MAX_TORQUE,
                      NEWEPUCK_INTERWHEEL_DISTANCE,
                      c_entity.GetConfigurationNode()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Create the body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(NEWEPUCK_MASS,
                                  cpMomentForCircle(NEWEPUCK_MASS,
                                                    0.0f,
                                                    NEWEPUCK_RADIUS + NEWEPUCK_RADIUS,
                                                    cpvzero)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());
      /* Create the body shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpCircleShapeNew(ptBody,
                                          NEWEPUCK_RADIUS,
                                          cpvzero));
      ptShape->e = 0.0; // No elasticity
      ptShape->u = 0.7; // Lots of friction
      /* Constrain the actual base body to follow the diff steering control */
      m_cDiffSteering.AttachTo(ptBody);
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, NEWEPUCK_HEIGHT);
   }

   /****************************************/
   /****************************************/

   CDynamics2DNewEPuckModel::~CDynamics2DNewEPuckModel() {
      m_cDiffSteering.Detach();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DNewEPuckModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DNewEPuckModel::UpdateFromEntityStatus() {
      /* Do we want to move? */
      if((m_fCurrentWheelVelocity[NEWEPUCK_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocity[NEWEPUCK_RIGHT_WHEEL] != 0.0f)) {
         m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[NEWEPUCK_LEFT_WHEEL],
                                          m_fCurrentWheelVelocity[NEWEPUCK_RIGHT_WHEEL]);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_cDiffSteering.Reset();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CNewEPuckEntity, CDynamics2DNewEPuckModel);

   /****************************************/
   /****************************************/

}
