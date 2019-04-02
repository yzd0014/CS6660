#include "Cloth.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine//Math/Functions.h"


void eae6320::Cloth::EventTick(const float i_secondCountToIntegrate) {
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	for (int i = 0; i < verticeCount; i++) {
		//get y
		y(0, i) = 2 * clothMesh->m_pVertexDataInRAM[i].x - lastFramePos[i].x;
		y(1, i) = 2 * clothMesh->m_pVertexDataInRAM[i].y - lastFramePos[i].y;
		y(2, i) = 2 * clothMesh->m_pVertexDataInRAM[i].z - lastFramePos[i].z;

		lastFramePos[i].x = clothMesh->m_pVertexDataInRAM[i].x;
		lastFramePos[i].y = clothMesh->m_pVertexDataInRAM[i].y;
		lastFramePos[i].z = clothMesh->m_pVertexDataInRAM[i].z;
	}
	
	//projective dynamics
	x = y;
	for (int k = 0; k < 10; k++) {
		for (int i = 0; i < edgeCount; i++) {
			Math::sVector restVec;
			restVec.x = (float)(x * A[i])(0, 0);
			restVec.y = (float)(x * A[i])(1, 0);
			restVec.z = (float)(x * A[i])(2, 0);
			restVec.Normalize();
			d(0, i) = restVec.x;
			d(1, i) = restVec.y;
			d(2, i) = restVec.z;
		}
		
		x = (d * J + (1 / pow(h, 2)) *y*M - gxm)*matInverse;
	}

	for (int i = 0; i < verticeCount; i++) {
		clothMesh->m_pVertexDataInRAM[i].x = (float)x(0, i);
		clothMesh->m_pVertexDataInRAM[i].y = (float)x(1, i);
		clothMesh->m_pVertexDataInRAM[i].z = (float)x(2, i);
	}
	//UserOutput::DebugPrint("%f, %f, %f", clothMesh->m_pVertexDataInRAM[6].y, clothMesh->m_pVertexDataInRAM[7].y, clothMesh->m_pVertexDataInRAM[8].y);
	
	for (int i = 0; i < clothResolution + 1; i+= clothResolution) {
		clothMesh->m_pVertexDataInRAM[i].x = fixedPos[i].x;
		clothMesh->m_pVertexDataInRAM[i].y = fixedPos[i].y;
		clothMesh->m_pVertexDataInRAM[i].z = fixedPos[i].z;
	}
	
	Mesh::s_manager.Get(GetMesh())->updateVertexBuffer = true;
	
}