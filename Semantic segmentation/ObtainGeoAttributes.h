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
/** Obain Facet Elevation */
void TriHorizontality(
	 MeshT& mesh,
	std::vector<double>* TrisHorizontality);

//version 2
void TriPlanarity(
	MeshT& mesh,
	std::vector<double>* TrisPlanarity);
//Obtain Facet Planarity
void FacetGeo(MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	std::vector<double> TrisGeo,
	std::vector<double>* FacetsGeo,
	std::vector<double>* SegGeo
);
//Write mesh
void WriteMesh(
	MeshT& mesh,
	std::string& out_path,
	std::vector<double>& segs_geo);