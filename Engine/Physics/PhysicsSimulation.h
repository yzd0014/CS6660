#pragma once
#include "Engine/GameCommon/GameObject.h"
#include <vector>

namespace eae6320 {
	struct CollisionPair;
	
	namespace Physics {
		void MoveObjectsForward(std::vector<GameCommon::GameObject *> & o_allGameObjects, float timeSpan);
		void PhysicsUpdate(std::vector<GameCommon::GameObject *> & o_allGameObjects, float i_dt);
		void ResolveCollision(CollisionPair & o_earliestCollision);
	}
}
