#ifndef _CLOTH_H_INCLUDED_
#define _CLOTH_H_INCLUDED_
#endif

#include "cyPoint.h"

class Cloth {
public:
	cy::Point3f* v;				// vertices of the (res+1)x(res+1) cloth	
	float cwidth, cheight;		// cloth width and height
	int res;					// resolution, determines the number of grid per row/col

	Cloth() {};
	void init(int res_in, float w, float h) {
		res = res_in;
		v = new cyPoint3f[(res+1)*(res+1)];
		float gridSize_w = w / res;
		float gridSize_h = h / res;
		for(int i=0;i<res+1;i++){
			for (int j = 0; j < res + 1; j++) {
				v[i*(res+1) + j].x = -w / 2.0f + j * gridSize_w;
				v[i*(res+1) + j].y = -h / 2.0f + i * gridSize_h;
				v[i*(res+1) + j].z = 0;
			}
		}
	};

	void fill_v_array(float* v_array) {	// fill the draw array with vertices in triangles
		int value_per_grid = 3 * 2 * 3;
		for (int j = 0; j < res; j++) {
			for (int i = 0; i < res; i++) {
				v_array[j*value_per_grid*res+ i * value_per_grid] = v[j*(res + 1)+i + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 1] = v[j*(res + 1) + i + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 2] = v[j*(res + 1) + i + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 3] = v[j*(res + 1) + i].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 4] = v[j*(res + 1) + i].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 5] = v[j*(res + 1) + i].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 6] = v[j*(res + 1) + i + res + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 7] = v[j*(res + 1) + i + res + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 8] = v[j*(res + 1) + i + res + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 9] = v[j*(res + 1) + i + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 10] = v[j*(res + 1) + i + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 11] = v[j*(res + 1) + i + 1].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 12] = v[j*(res + 1) + i + res + 2].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 13] = v[j*(res + 1) + i + res + 2].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 14] = v[j*(res + 1) + i + res + 2].z;

				v_array[j*value_per_grid*res + i * value_per_grid + 15] = v[j*(res + 1) + i + res + 1].x;
				v_array[j*value_per_grid*res + i * value_per_grid + 16] = v[j*(res + 1) + i + res + 1].y;
				v_array[j*value_per_grid*res + i * value_per_grid + 17] = v[j*(res + 1) + i + res + 1].z;
			}
		}
	};

};