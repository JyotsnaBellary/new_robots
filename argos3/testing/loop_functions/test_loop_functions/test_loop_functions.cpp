#include "test_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
using namespace argos;
using namespace std;

/****************************************/
/****************************************/

CTestLoopFunctions::CTestLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

void CTestLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tForaging = GetNode(t_node, "testing");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();

      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CTestLoopFunctions::Reset() {}

void CTestLoopFunctions::Destroy() {}

/****************************************/
/****************************************/

CColor CTestLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() < -1.0f) {
      return CColor::GRAY50;
   }
   return CColor::WHITE;
}

REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "test_loop_functions")
