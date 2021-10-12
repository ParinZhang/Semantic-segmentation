#pragma once
#include "types.h"
#include "util.h"
#include "GCoptimization.h"
class GraphCut
{
public:
	GraphCut();
	~GraphCut();

	/**Obtain Node Data Cost */
	void ObtainNodeData(
		SegFaceHandles& seg_face_handles,
		std::vector<double>& SegsElevation,
		std::vector<double>& SegsHorizontality,
		std::vector<double>& SegsPlanarity,
		std::vector<double>& NormSegsGreenness,
		//std::vector<double>& SegsDensity,
		std::vector<double>& segs_area,
		GCoptimizationGeneralGraph* gc);


	/**Obtain Edge Cost*/
	void ObtainEdgeData(MeshT& mesh,
		SegFaceHandles& seg_face_handles,
		const std::string& label_path,
		GCoptimizationGeneralGraph* gc);

	/**Obtain label*/
	void Run(MeshT& mesh,
		SegFaceHandles& seg_face_handles,
		const std::string& label_path, 
		std::vector<double>& SegsElevation,
	    std::vector<double>& SegsHorizontality,
		std::vector<double>& SegsPlanarity,
		std::vector<double>& NormSegsGreenness,
		//std::vector<double>& SegsDensity,
		std::vector<double>& segs_area,
		std::vector <size_t>*resultLabelArray);
	

	void ModifyInitLabel(
		std::vector <size_t>& seg_init_labels,
		Eigen::ArrayXXf& topology_map
	);
    /** Export mesh after GraphCut*/
	void WriteGCMesh(
		MeshT& mesh,
		std::string& export_path,
		SegFaceHandles seg_face_handles,
		std::vector <size_t> seg_labels
	);
}; 