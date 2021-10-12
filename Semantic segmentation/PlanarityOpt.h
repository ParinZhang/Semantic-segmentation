#pragma once
#include "types.h"
#include "util.h"

//**Method 1 : Move SearchBox to generate patial planarity *//

/*Calculate the coffients of the plane*/
void CalPlaneCoffis(
	MeshT& mesh,
	VertexHandles& v_h_s,
	Eigen::Vector4d* coffis
);


void Project2XYPlane(
	MeshT& mesh,
	SegVertexHandles& seg_vertex_handles,
	std::vector<MyTraits::Point>* TriCentroids
);

//**Method 1 : Move SearchBox to generate patial planarity *//

