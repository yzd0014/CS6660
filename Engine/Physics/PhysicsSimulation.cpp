#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/sVector.h"
#include "PhysicsSimulation.h"
#include "CollisionPair.h"
#include "CollisionDetection.h"
#include "CollisionDetection.h"

namespace eae6320 {
	namespace Physics {
		void MoveObjectsForward(std::vector<GameCommon::GameObject *> & o_allGameObjects, float timeSpan) {
			size_t numOfObjects = o_allGameObjects.size();
			for (size_t i = 0; i < numOfObjects; i++) {
				o_allGameObjects[i]->m_State.UpdatePosition(timeSpan);
			}
		}
		
		void ResolveCollision(CollisionPair & o_earliestCollision) {
			Math::sVector normal_A;
			Math::sVector normal_B;

			normal_A = o_earliestCollision.collisionNormal4A;
			normal_B = -1 * normal_A;

			Math::sVector v_reflectedA = abs(Math::Dot(-2 * o_earliestCollision.collisionObjects[0]->velocity, normal_A)) * normal_A - (-1 * o_earliestCollision.collisionObjects[0]->velocity);
			o_earliestCollision.collisionObjects[0]->velocity = v_reflectedA;

			Math::sVector v_reflectedB = abs(Math::Dot(-2 * o_earliestCollision.collisionObjects[1]->velocity, normal_B)) * normal_B - (-1 * o_earliestCollision.collisionObjects[1]->velocity);
			o_earliestCollision.collisionObjects[1]->velocity = v_reflectedB;

		}
		void PhysicsUpdate(std::vector<GameCommon::GameObject *> & o_allGameObjects, float i_dt) {
			float frameTime = i_dt;
			
			while (frameTime > 0) {
				CollisionPair earliestCollision;
				if (FindEarliestCollision(o_allGameObjects, frameTime, earliestCollision)) {
					MoveObjectsForward(o_allGameObjects, earliestCollision.collisionTime);
					ResolveCollision(earliestCollision);
					frameTime = frameTime - earliestCollision.collisionTime;
				}
				else {
					MoveObjectsForward(o_allGameObjects, earliestCollision.collisionTime);
					frameTime = 0;
				}
			}

			size_t numOfObjects = o_allGameObjects.size();
			for (size_t i = 0; i < numOfObjects; i++) {
				o_allGameObjects[i]->m_State.UpdateVelocity(i_dt);
				//o_allGameObjects[i]->m_State.UpdateOrientation(i_dt);
				//handle rotation collision		
				if (FindRotationCollision(o_allGameObjects, i_dt, i) == false) {
					o_allGameObjects[i]->m_State.UpdateOrientation(i_dt);
				}
			}
		}
		
	}
}