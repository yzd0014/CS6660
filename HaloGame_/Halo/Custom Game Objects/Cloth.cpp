#include "Cloth.h"
void eae6320::Cloth::EventTick(const float i_secondCountToIntegrate) {
	totalElapsedSimulationTime = totalElapsedSimulationTime + i_secondCountToIntegrate;
	m_State.position.z = 5 * sin(totalElapsedSimulationTime);
	/*
	if (totalElapsedSimulationTime > 2.0f) {
		//float test = Mesh::s_manager.Get(GetMesh())->m_pVertexDataInRAM[110].y;
		Mesh::s_manager.Get(GetMesh())->m_pVertexDataInRAM[110].y = -6;
		Mesh::s_manager.Get(GetMesh())->updateVertexBuffer = true;
	}
	*/
}