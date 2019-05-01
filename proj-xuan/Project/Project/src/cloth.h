#ifndef _CLOTH_H_INCLUDED_
#define _CLOTH_H_INCLUDED_
#endif

#include "cyPoint.h"
#include <Eigen/Dense>

#define _USE_MATH_DEFINES
#include <math.h>

#define K_SPRING 100
#define H 0.033
#define GRAV 1

class Cloth {
public:
	cy::Point3f* x;				// current vertices of the (res+1)x(res+1) cloth	
	cy::Point3f* x_origin;		// initial vertices 
	cy::Point3f* x_new;			// updated vertices 
	cy::Point3f* x_last;		// last step vertices 
	cy::Point3f* f;
	cy::Point3f* vel;

	float* m;

	float cwidth, cheight;		// cloth width and height
	int res;					// resolution, determines the number of grid per row/col
	int numv, nume;

	Eigen::MatrixXf L;
	Eigen::MatrixXf J;
	Eigen::MatrixXf d;

	Eigen::MatrixXf Dm;
	Eigen::MatrixXf G;
	float Wt;


	Cloth() {};
	cy::Point3f* getX(int i, int j) { return (x + (res + 1)*i + j); }
	cy::Point3f* getX_orig(int i, int j) { return (x_origin + (res + 1)*i + j); }
	cy::Point3f* getX_new(int i, int j) { return (x_new + (res + 1)*i + j); }
	cy::Point3f* getX_last(int i, int j) { return (x_last + (res + 1)*i + j); }
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

	void getEdgeByIndex(Pair &e, int e_i) {
		assert(e_i < nume);
		if (e_i < (nume / 2)) {
			// adjacent edges by row
			int row = e_i / res;
			int col = e_i % res;
			e.index_i = (res + 1)*row + col;
			e.index_j = e.index_i + 1;
		}else {
			// adjacent edges by col
			e_i -= nume / 2;
			int col = e_i / res;
			int row = e_i % res;
			e.index_i = (res + 1)*row + col;
			e.index_j = (res + 1)*(row+1) + col;
		}
	}

	void toRowColIndex(int p, int &i, int &j) {
		i = p / (res + 1);
		j = p % (res + 1);
	}


	void init(int res_in, float w, float h) {
		res = res_in;
		x = new cyPoint3f[(res+1)*(res+1)];
		x_origin = new cyPoint3f[(res + 1)*(res + 1)];
		x_new = new cyPoint3f[(res + 1)*(res + 1)];
		x_last = new cyPoint3f[(res + 1)*(res + 1)];

		m = new float[(res + 1)*(res + 1)];
		f = new cyPoint3f[(res + 1)*(res + 1)];
		vel = new cyPoint3f[(res + 1)*(res + 1)];

		numv = (res + 1)*(res + 1);
		nume = res * (res+1) * 2;

		float gridSize_w = w / res;
		float gridSize_h = h / res;
		for(int i=0;i<res+1;i++){
			for (int j = 0; j < res + 1; j++) {
				x[i*(res+1) + j].x = -w / 2.0f + j * gridSize_w;
				x[i*(res+1) + j].z = -h / 2.0f + i * gridSize_h;
				x[i*(res+1) + j].y = 0;
				x_origin[i*(res + 1) + j] = x[i*(res + 1) + j];
				x_last[i*(res + 1) + j] = x[i*(res + 1) + j];
				m[i*(res + 1) + j] = 1;
				vel[i*(res + 1) + j] = cy::Point3f(0,0,0); 
				//std::cout << i*(res + 1) + j <<" ";
			}
			//std::cout<<std::endl;
		}

		initEdgeConnectionMat();
	};

	void initEdgeConnectionMat() {
		L.resize(numv, numv);
		J.resize(nume, numv);
		d.resize(3, nume);

		L.setZero();
		J.setZero();
		d.setZero();

		for (int i = 0; i < nume; i++) {
			Eigen::MatrixXf Ai, Si;
			Ai.resize(numv, 1);
			Si.resize(nume, 1);
			Ai.setZero();
			Si.setZero();
			Pair tempP(-1, -1);
			getEdgeByIndex(tempP, i);
			Ai(tempP.index_i, 0) = 1;
			Ai(tempP.index_j, 0) = -1;
			Si(i, 0) = 1;
			L += K_SPRING * Ai*Ai.transpose();
			J += K_SPRING * Si*Ai.transpose();
		}
	}

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
				*getF(i, j) += GRAV * (*getM(i,j)) * cy::Point3f(0,-1,0);
			}
		}
	}

	void implicitInt() {
		Eigen::MatrixXf X, Y, M, D, g, m_vec;
		X.resize(3,numv);
		Y.resizeLike(X);
		M.resize(numv, numv);
		g.resize(3, 1);
		m_vec.resize(1,numv);

		X.setZero();
		Y.setZero();
		M.setZero();
		g(0, 0) = 0;
		g(1, 0) = GRAV;
		g(2, 0) = 0;

		// construct X Y M
		for (int i = 0; i < numv; i++) {
			X(0,i) = x[i].x;
			X(1,i) = x[i].y;
			X(2,i) = x[i].z;

			Y(0,i) = 2 * x[i].x - x_last[i].x;
			Y(1,i) = 2 * x[i].y - x_last[i].y;
			Y(2,i) = 2 * x[i].z - x_last[i].z;

			M(i, i) = m[i];
			m_vec(0,i) = m[i];
		}

		

		for (int k = 0; k < 10; k++) {
			// construct d
			for (int i = 0; i < nume; i++) {
				Pair tempP(-1, -1);
				getEdgeByIndex(tempP, i);
				cy::Point3f dir;
				cy::Point3f p1 = x[tempP.index_i];
				cy::Point3f p2 = x[tempP.index_j];

				float r = (x_origin[tempP.index_i] - x_origin[tempP.index_j]).Length();
				dir = (p1 - p2) / (p1 - p2).Length();
				d(0, i) = r * dir.x;
				d(1, i) = r * dir.y;
				d(2, i) = r * dir.z;
			}

			X = (d*J + Y * M*(1 / (H*H)) - g * m_vec) * (M*(1 / (H*H)) + L).inverse();
		}
		for (int i = 0; i < numv; i++) {
			x_new[i].x = X(0, i);
			x_new[i].y = X(1, i);
			x_new[i].z = X(2, i);
		}
		//for (int i = 0; i < numv; i++) {
		//	std::cout <<"("<< X(0, i) << " " << (1, i) << " " << (2, i) << " )";
		//}
	}

	void computeNextState_exp() {
		for (int i = 0; i < res + 1; i++) {
			for (int j = 0; j < (res + 1); j++) {
				// clear x_new
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
				// clear x_new
				*getX_new(i, j) = cy::Point3f(0, 0, 0);

				cy::Point3f acceleration = *getF(i, j) / (*getM(i, j));
				*getV(i, j) += acceleration * H;
				*getX_new(i, j) = *getX(i, j) + *getV(i, j) * H;
			}
		}

	}

	void explicitInt() {
		computeForces();
		computeNextState_smp();
	}

	void incrementStep() {
		for (int i = 0; i < res + 1; i++) {
			for (int j = 0; j < (res + 1); j++) {
				if ((i == res)) {}
				else if ((j == 0) && (i == res)) {}
				else {
					*getX_last(i, j) = *getX(i, j);
					*getX(i, j) = *getX_new(i, j);
				}
			}
		}
	}

	void fill_v_array(float* v_array) {	// fill the draw array with xtices in triangles
		int value_per_grid = 3 * 2 * 3;
		for (int j = 0; j < res; j++) {
			for (int i = 0; i < res; i++) {
				v_array[j*value_per_grid*res+ i * value_per_grid] = x[j*(res + 1)+i + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 1] = x[j*(res + 1) + i + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 2] = x[j*(res + 1) + i + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 3] = x[j*(res + 1) + i].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 4] = x[j*(res + 1) + i].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 5] = x[j*(res + 1) + i].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 6] = x[j*(res + 1) + i + res + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 7] = x[j*(res + 1) + i + res + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 8] = x[j*(res + 1) + i + res + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 9] = x[j*(res + 1) + i + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 10] = x[j*(res + 1) + i + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 11] = x[j*(res + 1) + i + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 12] = x[j*(res + 1) + i + res + 2].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 13] = x[j*(res + 1) + i + res + 2].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 14] = x[j*(res + 1) + i + res + 2].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 15] = x[j*(res + 1) + i + res + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 16] = x[j*(res + 1) + i + res + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 17] = x[j*(res + 1) + i + res + 1].z;
			}
		}
	};

	////// tet

	void pushXYZ(float* v_array, int v_index, cyPoint3f a) {
		v_array[v_index] = a.x;
		v_array[v_index+1] = a.y;
		v_array[v_index+2] = a.z;
	}

	void initTet() {
		x = new cyPoint3f[4];
		x_origin = new cyPoint3f[4];
		x_new = new cyPoint3f[4];
		x_last = new cyPoint3f[4];
		
		x[0] = cyPoint3f(-1, -1, -1);
		x[1] = cyPoint3f(1, -1, -1);
		x[2] = cyPoint3f(0, 1, -1);
		x[3] = cyPoint3f(0, 0, 1);

		for (int i = 0; i < 4; i++) {
			x_last[i] = x[i];
		}

		initTetMat();
	}


	void initTetMat() {

		Dm.resize(3,3);
		L.resize(4,3);
		cyPoint3f e[3];

		e[0] = x[0] - x[3];
		e[1] = x[1] - x[3];
		e[2] = x[2] - x[3];
		Dm(0, 0) = e[0].x; Dm(0, 1) = e[1].x; Dm(0, 2) = e[2].x;
		Dm(1, 0) = e[0].y; Dm(1, 1) = e[1].y; Dm(1, 2) = e[2].y;
		Dm(2, 0) = e[0].z; Dm(2, 1) = e[1].z; Dm(2, 2) = e[2].z;

		L.setZero();
		L(0, 0) = 1;
		L(1, 1) = 1;
		L(2, 2) = 1;
		L(3, 0) = -1;
		L(3, 1) = -1;
		L(3, 2) = -1;

		Wt = abs(Dm.determinant() / 6.0);

		G = L * Dm.inverse();

		std::cout <<"\n"<< "L" << std::endl;
		std::cout << L << std::endl;
		std::cout << "\n" << "Dm" << std::endl;
		std::cout << Dm << std::endl;
		std::cout << "\n" << "G" << std::endl;
		std::cout << G << std::endl;
		std::cout << "\n" << "Wt " <<Wt<< std::endl;
	}

	void initTetSetFar() {
		x[3].z += 1;
	}

	void integrateTet() {
		Eigen::MatrixXf X, Y, I;
		X.resize(3, 4);
		Y.resizeLike(X);
		I.resize(4, 4);
		X.setZero();
		Y.setZero();
		I.setIdentity();

		Eigen::MatrixXf m_vec,g;
		m_vec.resize(1, 4);
		g.resize(3, 1);
		g(0, 0) = 0;
		g(2, 0) = GRAV;
		g(1, 0) = 0;
		// construct X Y M
		for (int i = 0; i < 4; i++) {
			X(0, i) = x[i].x;
			X(1, i) = x[i].y;
			X(2, i) = x[i].z;

			Y(0, i) = 2 * x[i].x - x_last[i].x;
			Y(1, i) = 2 * x[i].y - x_last[i].y;
			Y(2, i) = 2 * x[i].z - x_last[i].z;

			m_vec(0, i) = Wt / 4;
		}
		//Wt = 1000;
		//for (int k = 0; k < 10; k++) {
			// construct d
			Eigen::MatrixXf E;
			E.resize(3, 3);
			E(0, 0) = X(0, 0) - X(0, 3); E(0, 1) = X(0, 1) - X(0, 3); E(0, 2) = X(0, 2) - X(0, 3);
			E(1, 0) = X(1, 0) - X(1, 3); E(1, 1) = X(1, 1) - X(1, 3); E(1, 2) = X(1, 2) - X(1, 3);
			E(2, 0) = X(2, 0) - X(2, 3); E(2, 1) = X(2, 1) - X(2, 3); E(2, 2) = X(2, 2) - X(2, 3);
			float d = abs(E.determinant() - Wt);
			X = (Y * I / (H * H) + G.transpose()*Wt - g* m_vec)*(I / (H*H) + Wt * G*G.transpose()).inverse();
		//}

		for (int i = 0; i < 4; i++) {
			x_new[i].x = X(0, i);
			x_new[i].y = X(1, i);
			x_new[i].z = X(2, i);
		}
	}

	void incrementStepTet() {
		for (int i = 0; i <3 ; i++) {
			x_last[i] = x[i];
			x[i] = x_new[i];			
		}
	}

	void fill_v_arrayTet(float* v_array, float* c_array) {
		pushXYZ(v_array, 0, x[0]);
		pushXYZ(v_array, 3, x[1]);
		pushXYZ(v_array, 6, x[2]);

		pushXYZ(v_array, 9, x[0]);
		pushXYZ(v_array, 12, x[1]);
		pushXYZ(v_array, 15, x[3]);

		pushXYZ(v_array, 18, x[0]);
		pushXYZ(v_array, 21, x[2]);
		pushXYZ(v_array, 24, x[3]);

		pushXYZ(v_array, 27, x[1]);
		pushXYZ(v_array, 30, x[2]);
		pushXYZ(v_array, 33, x[3]);


		pushXYZ(c_array, 0, cyPoint3f(1, 1, 1));
		pushXYZ(c_array, 3, cyPoint3f(1, 1, 1));
		pushXYZ(c_array, 6, cyPoint3f(1, 1, 1));

		pushXYZ(c_array, 9, cyPoint3f(1, 1, 0));
		pushXYZ(c_array, 12, cyPoint3f(1, 1, 0));
		pushXYZ(c_array, 15, cyPoint3f(1, 1, 0));
		
		pushXYZ(c_array, 18, cyPoint3f(0, 1, 0));
		pushXYZ(c_array, 21, cyPoint3f(0, 1, 0));
		pushXYZ(c_array, 24, cyPoint3f(0, 1, 0));

		pushXYZ(c_array, 27, cyPoint3f(0, 1, 1));
		pushXYZ(c_array, 30, cyPoint3f(0, 1, 1));
		pushXYZ(c_array, 33, cyPoint3f(0, 1, 1));

	}

};