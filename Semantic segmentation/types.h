#pragma once
#include <set>
#include <vector>
#include <limits>
#include <Eigen/dense>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// OPEN MESH //
struct MyTraits : public OpenMesh::DefaultTraits
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
	typedef OpenMesh::Vec2f TexCoord2D;
};

using MeshT = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
using MeshP = OpenMesh::PolyMesh_ArrayKernelT<MyTraits>;
// OPEN MESH //




// SEGMENT //
using FaceHandles = std::vector<OpenMesh::SmartFaceHandle>;
using VertexHandles = std::vector<OpenMesh::SmartVertexHandle>;
using SegFaceHandles = std::vector<std::vector<OpenMesh::SmartFaceHandle>>;
using SegVertexHandles = std::vector<std::vector<OpenMesh::SmartVertexHandle>>;
using SegMedianNormals = std::vector<Eigen::Vector3d>;
// SEGMENT //


//FACET ELEVATION//
using Face = OpenMesh::SmartFaceHandle;





//FACET ELEVATION//


//FACET PLANARITY//
typedef double                      FT;
typedef CGAL::Simple_cartesian<FT>  K;
typedef K::Plane_3                  Plane;
typedef K::Point_3                  Point;
typedef K::Triangle_3               Triangle;
//FACET PLANARITY//

//GRAPH CUT
struct node_data
{
	node_data() {
		labels[0] = 0; costs[0] = -1;
		labels[1] = 1; costs[1] = -1;
		labels[2] = 2; costs[2] = -1;
		labels[3] = 3; costs[3] = -1;

	}
    int SegId;
	int labels[4];
	float costs[4];
};

struct edge_data 
{
	int SegId;
	std::vector<int> neighbor_segIds;
	std::vector<double> edge_cost;
	int count;
};


//GRAPH CUT




