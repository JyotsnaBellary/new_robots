/**
 * @file <turtlebot4/simulator/dynamics2d_turtlebot4_model.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "dynamics2d_turtlebot4_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include "turtlebot4_measures.h"

namespace argos {

   enum TURTLEBOT4_WHEELS {
      TURTLEBOT4_LEFT_WHEEL = 0,
      TURTLEBOT4_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CDynamics2DTurtlebot4Model::CDynamics2DTurtlebot4Model(CDynamics2DEngine& c_engine,
                                                CTurtlebot4Entity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cTurtlebot4Entity(c_entity),
      m_cWheeledEntity(m_cTurtlebot4Entity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      TURTLEBOT4_MAX_FORCE,
                      TURTLEBOT4_MAX_TORQUE,
                      TURTLEBOT4_WHEEL_DISTANCE,
                      c_entity.GetConfigurationNode()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Create the body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(TURTLEBOT4_MASS,
                                  cpMomentForCircle(TURTLEBOT4_MASS,
                                                    0.0f,
                                                    TURTLEBOT4_BASE_RADIUS + TURTLEBOT4_BASE_RADIUS,
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
                                          TURTLEBOT4_BASE_RADIUS,
                                          cpvzero));
      ptShape->e = 0.0; // No elasticity
      ptShape->u = 0.7; // Lots of friction
      /* Constrain the actual base body to follow the diff steering control */
      m_cDiffSteering.AttachTo(ptBody);
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, TURTLEBOT4_BASE_TOP);
   }

   /****************************************/
   /****************************************/

   CDynamics2DTurtlebot4Model::~CDynamics2DTurtlebot4Model() {
      m_cDiffSteering.Detach();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DTurtlebot4Model::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DTurtlebot4Model::UpdateFromEntityStatus() {
      /* Do we want to move? */
      if((m_fCurrentWheelVelocity[TURTLEBOT4_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocity[TURTLEBOT4_RIGHT_WHEEL] != 0.0f)) {
         m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[TURTLEBOT4_LEFT_WHEEL],
                                          m_fCurrentWheelVelocity[TURTLEBOT4_RIGHT_WHEEL]);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_cDiffSteering.Reset();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CTurtlebot4Entity, CDynamics2DTurtlebot4Model);

   /****************************************/
   /****************************************/

}
