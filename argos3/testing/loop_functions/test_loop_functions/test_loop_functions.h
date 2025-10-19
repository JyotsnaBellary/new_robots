#ifndef TEST_LOOP_FUNCTIONS_H
#define TEST_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CTestLoopFunctions : public CLoopFunctions {

public:

   CTestLoopFunctions();
   virtual ~CTestLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

private:
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;
};

#endif
