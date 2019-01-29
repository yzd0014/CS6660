#pragma once
#include "Engine/GameCommon/GameObject.h"
namespace eae6320 {
	class Cloth : public eae6320::GameCommon::GameObject {
	public:
		Cloth(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			totalElapsedSimulationTime(0.0f)
		{
		}
		void EventTick(const float i_secondCountToIntegrate) override;
	private:
		float totalElapsedSimulationTime;
	};
}