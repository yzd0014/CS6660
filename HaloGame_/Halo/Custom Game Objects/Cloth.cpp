#include "Cloth.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine//Math/Functions.h"
void eae6320::Cloth::EventTick(const float i_secondCountToIntegrate) {
	float k = 2000;
	int clothResolution = 10;
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	Math::sVector* displacement = new Math::sVector[clothMesh->GetVerticesCount()];

	Math::sVector leftPos;
	leftPos.x = clothMesh->m_pVertexDataInRAM[0].x;
	leftPos.y = clothMesh->m_pVertexDataInRAM[0].y;
	leftPos.z = clothMesh->m_pVertexDataInRAM[0].z;
	Math::sVector rightPos;
	rightPos.x = clothMesh->m_pVertexDataInRAM[clothResolution].x;
	rightPos.y = clothMesh->m_pVertexDataInRAM[clothResolution].y;
	rightPos.z = clothMesh->m_pVertexDataInRAM[clothResolution].z;

	for (int16_t i = 0; i < clothMesh->GetVerticesCount(); i++) {
		int16_t up, down, left, right;
		up = i - (clothResolution + 1);
		down = i + (clothResolution + 1);
		left = i - 1;
		right = i + 1;

		Math::sVector accel(0.0f, 0.0f, 0.0f);
		if (up >= 0) {//accumlate from up
			Math::sVector distance;
			distance.x = clothMesh->m_pVertexDataInRAM[up].x - clothMesh->m_pVertexDataInRAM[i].x;
			distance.y = clothMesh->m_pVertexDataInRAM[up].y - clothMesh->m_pVertexDataInRAM[i].y;
			distance.z = clothMesh->m_pVertexDataInRAM[up].z - clothMesh->m_pVertexDataInRAM[i].z;

			accel = accel + k * (distance.GetLength() - 1) * distance.GetNormalized();
		}
		if (down <= (clothResolution + 1) * (clothResolution + 1) - 1) {//accumlate from down
			Math::sVector distance;
			distance.x = clothMesh->m_pVertexDataInRAM[down].x - clothMesh->m_pVertexDataInRAM[i].x;
			distance.y = clothMesh->m_pVertexDataInRAM[down].y - clothMesh->m_pVertexDataInRAM[i].y;
			distance.z = clothMesh->m_pVertexDataInRAM[down].z - clothMesh->m_pVertexDataInRAM[i].z;

			accel = accel + k * (distance.GetLength() - 1) * distance.GetNormalized();
		}
		if (i % (clothResolution + 1) > 0) {//accumlate from left 
			Math::sVector distance;
			distance.x = clothMesh->m_pVertexDataInRAM[left].x - clothMesh->m_pVertexDataInRAM[i].x;
			distance.y = clothMesh->m_pVertexDataInRAM[left].y - clothMesh->m_pVertexDataInRAM[i].y;
			distance.z = clothMesh->m_pVertexDataInRAM[left].z - clothMesh->m_pVertexDataInRAM[i].z;
			//Math::sVector tempAccel = k * (distance.GetLength() - 1) * distance.GetNormalized();
			accel = accel + k * (distance.GetLength() - 1) * distance.GetNormalized();
			//UserOutput::DebugPrint("acceleration: %f", tempAccel.GetLength());
		}
		if (i % (clothResolution + 1) < clothResolution) {//accumlate from right
			Math::sVector distance;
			distance.x = clothMesh->m_pVertexDataInRAM[right].x - clothMesh->m_pVertexDataInRAM[i].x;
			distance.y = clothMesh->m_pVertexDataInRAM[right].y - clothMesh->m_pVertexDataInRAM[i].y;
			distance.z = clothMesh->m_pVertexDataInRAM[right].z - clothMesh->m_pVertexDataInRAM[i].z;

			accel = accel + k * (distance.GetLength() - 1) * distance.GetNormalized();
		}
		
		Math::sVector gravity(0.0f, -9.81f, 0.0f);
		accel = accel + gravity;
		/*
		if (Math::floatEqual(accel.GetLength(), 0.0f)) {
			accel = Math::sVector(0.0f, 0.0f, 0.0f);
		}
		*/
		//UserOutput::DebugPrint("acceleration %d: %f", i, accel.GetLength());
		//update velocity
		verticeVelocity[i] = verticeVelocity[i] + i_secondCountToIntegrate * accel;
		//cache displacement
		displacement[i] = i_secondCountToIntegrate * verticeVelocity[i];
	}
	for (uint16_t i = 0; i < clothMesh->GetVerticesCount(); i++) {
		clothMesh->m_pVertexDataInRAM[i].x = clothMesh->m_pVertexDataInRAM[i].x + displacement[i].x;
		clothMesh->m_pVertexDataInRAM[i].y = clothMesh->m_pVertexDataInRAM[i].y + displacement[i].y;
		clothMesh->m_pVertexDataInRAM[i].z = clothMesh->m_pVertexDataInRAM[i].z + displacement[i].z;
	}
	delete[] displacement;
	clothMesh->m_pVertexDataInRAM[0].x = leftPos.x;
	clothMesh->m_pVertexDataInRAM[0].y = leftPos.y;
	clothMesh->m_pVertexDataInRAM[0].z = leftPos.z;

	clothMesh->m_pVertexDataInRAM[clothResolution].x = rightPos.x;
	clothMesh->m_pVertexDataInRAM[clothResolution].y = rightPos.y;
	clothMesh->m_pVertexDataInRAM[clothResolution].z = rightPos.z;

	Mesh::s_manager.Get(GetMesh())->updateVertexBuffer = true;
	
}