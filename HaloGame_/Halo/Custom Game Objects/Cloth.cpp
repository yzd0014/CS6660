#include "Cloth.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine//Math/Functions.h"
#include "Eigen/Dense"

using namespace Eigen;

void eae6320::Cloth::EventTick(const float i_secondCountToIntegrate) {
	float k = 5000;
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	int edgeCount = (2 * clothResolution + 1)*clothResolution + clothResolution;
	/*
	Math::sVector topPos[11];
	for (int i = 0; i < 11; i++) {
		topPos[i].x = clothMesh->m_pVertexDataInRAM[i].x;
		topPos[i].y = clothMesh->m_pVertexDataInRAM[i].y;
		topPos[i].z = clothMesh->m_pVertexDataInRAM[i].z;
	}
	*/
	MatrixXd m(1, verticeCount);
	MatrixXd M(verticeCount, verticeCount);
	M.setZero();
	for (int i = 0; i < verticeCount; i++) {
		m(0, i) = 1;
		M(i, i) = m(0, i);
	}

	MatrixXd x(3, verticeCount);
	MatrixXd* A = new MatrixXd[edgeCount];
	MatrixXd L(verticeCount, verticeCount);
	L.setZero();
	MatrixXd J(edgeCount, verticeCount);
	J.setZero();
	for (int i = 0; i < edgeCount; i++) {
		//generate A
		int row = i / (2 * clothResolution + 1);
		int	remander = i % (2 * clothResolution + 1);
		if (remander < clothResolution) {
			int vertexIndex = row * (clothResolution + 1) + remander;
			A[i].resize(verticeCount, 1);
			for (int j = 0; j < verticeCount; j++) {
				A[i](j, 0) = 0;
			}
			A[i](vertexIndex, 0) = -1;
			A[i](vertexIndex + 1, 0) = 1;
		}
		else {
			int vertexIndex = row * (clothResolution + 1) + remander - clothResolution;
			A[i].resize(verticeCount, 1);
			for (int j = 0; j < verticeCount; j++) {
				A[i](j, 0) = 0;
			}
			A[i](vertexIndex, 0) = 1;
			A[i](vertexIndex + clothResolution + 1, 0) = -1;
		}

		//generate J from S
		MatrixXd Si;
		Si.resize(edgeCount, 1);
		Si.setZero();
		Si(i, 0) = 1;
		J = J + k * Si * A[i].transpose();

		//get L
		L = L + k * A[i] * A[i].transpose();
	}
	Vector3d g(0.0f, 1.0f, 0.0f);
	MatrixXd d(3, edgeCount);
	MatrixXd y(3, verticeCount);
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
	for (int k = 0; k < 2; k++) {
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
		
		x = (d * J + (1 / pow(i_secondCountToIntegrate, 2))*y*M - g * m)*((1 / pow(i_secondCountToIntegrate, 2))*M + L).inverse();
	}
	delete[] A;

	for (int i = 0; i < verticeCount; i++) {
		clothMesh->m_pVertexDataInRAM[i].x = (float)x(0, i);
		clothMesh->m_pVertexDataInRAM[i].y = (float)x(1, i);
		clothMesh->m_pVertexDataInRAM[i].z = (float)x(2, i);
	}
	//UserOutput::DebugPrint("%f, %f, %f", clothMesh->m_pVertexDataInRAM[6].y, clothMesh->m_pVertexDataInRAM[7].y, clothMesh->m_pVertexDataInRAM[8].y);
	/*
	for (int i = 0; i < 11; i++) {
		clothMesh->m_pVertexDataInRAM[i].x = topPos[i].x;
		clothMesh->m_pVertexDataInRAM[i].y = topPos[i].y;
		clothMesh->m_pVertexDataInRAM[i].z = topPos[i].z;
	}
	*/
	
	clothMesh->m_pVertexDataInRAM[0].x = fixedPos[0].x;
	clothMesh->m_pVertexDataInRAM[0].y = fixedPos[0].y;
	clothMesh->m_pVertexDataInRAM[0].z = fixedPos[0].z;

	clothMesh->m_pVertexDataInRAM[clothResolution].x = fixedPos[1].x;
	clothMesh->m_pVertexDataInRAM[clothResolution].y = fixedPos[1].y;
	clothMesh->m_pVertexDataInRAM[clothResolution].z = fixedPos[1].z;
	
	Mesh::s_manager.Get(GetMesh())->updateVertexBuffer = true;
	
}