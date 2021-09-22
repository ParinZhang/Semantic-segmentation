#pragma once
#include <cmath>

#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include "util.h"



/** Obtain the centroid of triangle */
void ObtainTriCentroid(
	const MeshT& mesh,
	//const SegFaceHandles& seg_face_handles,
	std::vector<MyTraits::Point>* TriCentroids
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
	);

/** Obtain the average of the length of edges*/
double  ObtainAveEdge(const MeshT& mesh);

/** Obtain the neiborhood of the triangle in K rings*/
std::set<Face> GetKRingFaces(const MeshT& mesh, const int k, const Face& f);



/** Obain Facet Elevation   */
void  TriElevation(
	 MeshT& mesh,
	std::vector<double>* TrisElevation);
void  TriElevation2(MeshT& mesh,
	std::vector<double>* TrisElevation);
void TriElevation3(MeshT& mesh,
	std::vector<double>* TrisElevation);


/** Obain Facet Horizontality */
void TriHorizontality(
	 MeshT& mesh,
	std::vector<double>* TrisHorizontality);

/** Obain Facet Horizontality */

/******* Obain Facet Planarity ******/
void SegPlanarity(
	MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	std::vector<double>* SegPlanarity);

void DoPlanarity3(
	MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	std::vector<double>* inliners_ratios
);

void GetInliersLSP(
	Eigen::Vector4d coffi,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::IndicesPtr inliers,
	double distance_thres
);
//******* Obain Facet Planarity *******/

//Obtain Facet Planarity
void FacetGeo(MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	std::vector<double> TrisGeo,
	std::vector<double>* FacetsGeo,
	std::vector<double>* SegGeo
);

/**  Obatain SegDensity of each segment  */
//Obatain SegDensity of each triangle
void ObtainTriSegDensity(MeshT& mesh,
	std::vector<double>* tri_seg_density);
void ObtainFacetSegDensity(
	MeshT& mesh,
	SegFaceHandles seg_face_handles,
	std::vector<double>* facet_seg_density
);


//Write mesh
void WriteMesh(
	MeshT& mesh,
	std::string& out_path,
	std::vector<double>& segs_geo);