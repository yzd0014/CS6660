#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
namespace eae6320 {
	class Cloth : public eae6320::GameCommon::GameObject {
	public:
		Cloth(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			totalElapsedSimulationTime(0.0f)
		{
			Mesh* clothMesh = Mesh::s_manager.Get(i_Mesh);
			
			//rotate cloth to make it parallel to ground
			Math::cMatrix_transformation rotMatrix(Math::cQuaternion(Math::ConvertDegreesToRadians(-90), Math::sVector(1, 0, 0)), Math::sVector(0.0f, 0.0f, 0.0f));
			for (uint16_t i = 0; i < clothMesh->GetVerticesCount(); i++) {
				Math::sVector oldPos, newPos;
				oldPos.x = clothMesh->m_pVertexDataInRAM[i].x;
				oldPos.y = clothMesh->m_pVertexDataInRAM[i].y;
				oldPos.z = clothMesh->m_pVertexDataInRAM[i].z;
				newPos = rotMatrix * oldPos;

				clothMesh->m_pVertexDataInRAM[i].x = newPos.x;
				clothMesh->m_pVertexDataInRAM[i].y = newPos.y;
				clothMesh->m_pVertexDataInRAM[i].z = newPos.z;
			}
			clothMesh->updateVertexBuffer = true;
			
			lastFramePos = new Math::sVector[clothMesh->GetVerticesCount()];
			for (int i = 0; i < clothMesh->GetVerticesCount(); i++) {
				lastFramePos[i] = Math::sVector(clothMesh->m_pVertexDataInRAM[i].x, clothMesh->m_pVertexDataInRAM[i].y, clothMesh->m_pVertexDataInRAM[i].z);
			}
			verticeCount = clothMesh->GetVerticesCount();
			clothResolution = (int)sqrt(verticeCount) - 1;

			//fixedPos[0] = Math::sVector(clothMesh->m_pVertexDataInRAM[0].x, clothMesh->m_pVertexDataInRAM[0].y, clothMesh->m_pVertexDataInRAM[0].z);
			//fixedPos[1] = Math::sVector(clothMesh->m_pVertexDataInRAM[clothResolution].x, clothMesh->m_pVertexDataInRAM[clothResolution].y, clothMesh->m_pVertexDataInRAM[clothResolution].z);
			for (int i = 0; i < clothResolution+1; i++) {
				fixedPos[i].x = clothMesh->m_pVertexDataInRAM[i].x;
				fixedPos[i].y = clothMesh->m_pVertexDataInRAM[i].y;
				fixedPos[i].z = clothMesh->m_pVertexDataInRAM[i].z;
			}
		}
		void EventTick(const float i_secondCountToIntegrate) override;
		~Cloth() {
			delete[] lastFramePos;
		}
		Math::sVector* lastFramePos;
		Math::sVector fixedPos[11];
	private:
		float totalElapsedSimulationTime;
		int verticeCount;
		int clothResolution;
	};
}