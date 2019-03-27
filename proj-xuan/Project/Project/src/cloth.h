#ifndef _CLOTH_H_INCLUDED_
#define _CLOTH_H_INCLUDED_
#endif

#include "cyPoint.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define K_SPRING 100
#define H 0.033
#define G 1

class Cloth {
public:
	cy::Point3f* ver;				// current vertices of the (res+1)x(res+1) cloth	
	cy::Point3f* ver_origin;		// initial vertices 
	cy::Point3f* ver_new;			// uodated vertices 
	cy::Point3f* f;
	cy::Point3f* vel;

	float* m;

	float cwidth, cheight;		// cloth width and height
	int res;					// resolution, determines the number of grid per row/col


	Cloth() {};
	cy::Point3f* getX(int i, int j) { return (ver + (res + 1)*i + j); }
	cy::Point3f* getX_orig(int i, int j) { return (ver_origin + (res + 1)*i + j); }
	cy::Point3f* getX_new(int i, int j) { return (ver_new + (res + 1)*i + j); }
	cy::Point3f* getF(int i, int j) { return (f + (res + 1)*i + j); }
	cy::Point3f* getV(int i, int j) { return (vel + (res + 1)*i + j); }
	float* getM(int i, int j) { return (m + (res + 1)*i + j); }

	class Pair {
	public: 
		int index_i;
		int index_j;
		Pair(int a, int b) { index_i = a; index_j = b; }
	};

	float restEdgeLen(int a, int b, int a2, int b2) {
		return (*getX_orig(a, b) - *getX_orig(a2, b2)).Length();
	}

	float curEdgeLen(int a, int b, int a2, int b2, cy::Point3f* dir) {
		cy::Point3f edge = (*getX(a, b) - *getX(a2, b2));
		float len = edge.Length();
		*dir = edge / len;
		return len;
	}



	void init(int res_in, float w, float h) {
		res = res_in;
		ver = new cyPoint3f[(res+1)*(res+1)];
		ver_origin = new cyPoint3f[(res + 1)*(res + 1)];
		ver_new = new cyPoint3f[(res + 1)*(res + 1)];

		m = new float[(res + 1)*(res + 1)];
		f = new cyPoint3f[(res + 1)*(res + 1)];
		vel = new cyPoint3f[(res + 1)*(res + 1)];


		float gridSize_w = w / res;
		float gridSize_h = h / res;
		for(int i=0;i<res+1;i++){
			for (int j = 0; j < res + 1; j++) {
				ver[i*(res+1) + j].x = -w / 2.0f + j * gridSize_w;
				ver[i*(res+1) + j].z = -h / 2.0f + i * gridSize_h;
				ver[i*(res+1) + j].y = 0;
				ver_origin[i*(res + 1) + j] = ver[i*(res + 1) + j];
				m[i*(res + 1) + j] = 1;
				vel[i*(res + 1) + j] = cy::Point3f(0,0,0); 
			}
		}
	};

	void move(float t) {
		float vel = sin(t);
		for (int i = 0; i < (res + 1);i++ ) 
			for (int j = 0; j < (res + 1); j++) 
				getX(i,j)->x +=  0.01*vel;
		
	}

	void getNeighbors(std::vector<Pair>* neighbors, int i, int j) {
		
		if (i > 0) neighbors->push_back(Pair(i-1, j));
		if (j > 0) neighbors->push_back(Pair(i, j-1));
		if (j < res) neighbors->push_back(Pair(i, j+1));
		if (i < res) neighbors->push_back(Pair(i+1, j));
		
	}

	void computeForces() {
		for (int i = 0; i < res+1; i++) {
			for (int j = 0; j < (res + 1); j++) {
				std::vector<Pair> neighbors;
				getNeighbors(&neighbors, i, j);

				// clear forces
				*getF(i, j) = cy::Point3f(0, 0, 0); 
				for (int k = 0; k < neighbors.size(); k++) {
					Pair theNeighborV = neighbors[k];
					cy::Point3f dir;
					float rest_len = restEdgeLen(theNeighborV.index_i, theNeighborV.index_j, i,j);
					float cur_len = curEdgeLen(theNeighborV.index_i, theNeighborV.index_j, i, j, &dir);
					//std::cout << i << " " << j << ":" << dir.x << " " << dir.y << " " << dir.z << std::endl;
					*getF(i, j) += K_SPRING *(cur_len - rest_len)*dir;
				}

				// add gravity
				*getF(i, j) += G * (*getM(i,j)) * cy::Point3f(0,-1,0);
			}
		}
	}

	void computeNextState_exp() {
		for (int i = 0; i < res + 1; i++) {
			for (int j = 0; j < (res + 1); j++) {
				// clear ver_new
				*getX_new(i, j) = cy::Point3f(0, 0, 0);

				cy::Point3f acceleration = *getF(i, j) / (*getM(i, j));
				*getX_new(i, j) = *getX(i, j) + *getV(i, j) * H;
				*getV(i, j) += acceleration * H;

			}
		}

	}
	void computeNextState_smp() {
		for (int i = 0; i < res + 1; i++) {
			for (int j = 0; j < (res + 1); j++) {
				// clear ver_new
				*getX_new(i, j) = cy::Point3f(0, 0, 0);

				cy::Point3f acceleration = *getF(i, j) / (*getM(i, j));
				*getV(i, j) += acceleration * H;
				*getX_new(i, j) = *getX(i, j) + *getV(i, j) * H;
			}
		}

	}
	void incrementStep() {
		for (int i = 0; i < res + 1; i++) {
			for (int j = 0; j < (res + 1); j++) {
				if ((i == res)) {}
				else if ((j == 0) && (i == res)) {}
				else
					*getX(i, j) = *getX_new(i, j);
			}
		}
	}

	void fill_v_array(float* v_array) {	// fill the draw array with vertices in triangles
		int value_per_grid = 3 * 2 * 3;
		for (int j = 0; j < res; j++) {
			for (int i = 0; i < res; i++) {
				v_array[j*value_per_grid*res+ i * value_per_grid] = ver[j*(res + 1)+i + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 1] = ver[j*(res + 1) + i + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 2] = ver[j*(res + 1) + i + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 3] = ver[j*(res + 1) + i].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 4] = ver[j*(res + 1) + i].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 5] = ver[j*(res + 1) + i].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 6] = ver[j*(res + 1) + i + res + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 7] = ver[j*(res + 1) + i + res + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 8] = ver[j*(res + 1) + i + res + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 9] = ver[j*(res + 1) + i + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 10] = ver[j*(res + 1) + i + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 11] = ver[j*(res + 1) + i + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 12] = ver[j*(res + 1) + i + res + 2].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 13] = ver[j*(res + 1) + i + res + 2].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 14] = ver[j*(res + 1) + i + res + 2].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 15] = ver[j*(res + 1) + i + res + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 16] = ver[j*(res + 1) + i + res + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 17] = ver[j*(res + 1) + i + res + 1].z;
			}
		}
	};

};