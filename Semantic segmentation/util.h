#pragma once
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "types.h"
#include "timer.h"

// IO //
/** Read mesh */
void ReadMesh(const std::string& path, MeshT* mesh);

/** Read label vector (first element is segment number) */
template <typename T>
std::vector<T> ReadLabels(const std::string& filename)
{
	std::ifstream in(filename.c_str(), std::ios::binary);
	in.seekg(0, in.end);
	const size_t filesize = in.tellg();
	in.seekg(0, in.beg);

	const size_t num_elements = filesize / sizeof(T);
	std::vector<T> vector(num_elements);
	in.read(reinterpret_cast<char*>(&vector[0]), num_elements * sizeof(T));

	in.close();
	return vector;
}
// IO //


// SEGMENT //
/** Obtain face handles and vertex handles */
void ObtainSegHandles(
	MeshT& mesh,
	const std::string& label_name,
	SegFaceHandles* seg_face_handle,
	SegVertexHandles* seg_vertex_handle);


/** Obtain segment area */
void ObtainSegAreas(
	const MeshT& mesh,
	const SegFaceHandles& seg_face_handles,
	std::vector<double>* seg_areas);

/** Obtain segment normals*/
void ObtainSegNormals(
	const MeshT& mesh,
	const SegFaceHandles& seg_face_handles,
	SegMedianNormals* seg_median_normals);

/**Cal COS value of angel of segments */
float CalSegAngle(SegMedianNormals& seg_median_normals, size_t seg_id_i, size_t seg_id_j);

/**Obtain Seg Topology Map*/
void GetSegTopologyMap(
	MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	Eigen::ArrayXXf* topology_map
);