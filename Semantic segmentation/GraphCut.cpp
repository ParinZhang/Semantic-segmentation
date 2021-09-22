#include "GraphCut.h"

GraphCut::GraphCut()
{

}
GraphCut::~GraphCut()
{

}



void GraphCut::ObtainNodeData(
	SegFaceHandles& seg_face_handles,
	std::vector<double>& SegsElevation,
	std::vector<double>& SegsHorizontality,
	std::vector<double>& SegsPlanarity,
	std::vector<double>& NormSegsGreenness,
	//std::vector<double>& SegsDensity,
	std::vector<double>& segs_area,
	GCoptimizationGeneralGraph* gc)
{
	for(size_t i=0;i<seg_face_handles.size();i++)
	{
		
		node_data n_a;
		auto seg_id = i;
		n_a.costs[0] = segs_area[seg_id] * (1 - SegsHorizontality[seg_id] * (1 - SegsElevation[seg_id]) * (1 - NormSegsGreenness[seg_id])*SegsPlanarity[seg_id]);//Ground
		n_a.costs[1] = segs_area[seg_id] * (1 - SegsHorizontality[seg_id] * NormSegsGreenness[seg_id]*(1 - SegsPlanarity[seg_id]));//Tree
		n_a.costs[2] = segs_area[seg_id] * (1 - (1 - SegsHorizontality[seg_id]) * (1 - NormSegsGreenness[seg_id])*SegsPlanarity[seg_id]);//Facade
		n_a.costs[3] = segs_area[seg_id] * (1 -  SegsHorizontality[seg_id] * SegsElevation[seg_id] * (1 - NormSegsGreenness[seg_id])* SegsPlanarity[seg_id]);//Roof


		gc->setDataCost(seg_id, n_a.labels[0], n_a.costs[0]);
		gc->setDataCost(seg_id, n_a.labels[1], n_a.costs[1]);
		gc->setDataCost(seg_id, n_a.labels[2], n_a.costs[2]);
		gc->setDataCost(seg_id, n_a.labels[3], n_a.costs[3]);

		std::vector<double> vec{ n_a.costs[0] ,n_a.costs[1] ,n_a.costs[2],n_a.costs[3] };
		std::cout << min_element(vec.begin(),vec.end())- vec.begin() << std::endl;

	}

}


void GraphCut::ObtainEdgeData(MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	const std::string& label_path,
	GCoptimizationGeneralGraph* gc) 
{

	//GraphCut::AddSegIDToFace(mesh, label_path);
	auto f_seg = OpenMesh::getProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");

	SegMedianNormals seg_median_normals;
	ObtainSegNormals(mesh, seg_face_handles, &seg_median_normals);

	for (auto s_f_hs : seg_face_handles)
	{
		size_t cur_sid = f_seg[s_f_hs.front()];
		std::set<size_t> neighbor_seg_ids;
		std::vector<float> inter_length;
		inter_length.resize(seg_face_handles.size(), 0.f);

		for (auto s_f_h : s_f_hs)
		{
			auto seg_id_i = f_seg[s_f_h];
			for (auto h_e_h : mesh.fh_range(s_f_h))
			{
				int hid = h_e_h.idx();
				auto opp_halfedge = mesh.opposite_halfedge_handle(h_e_h);
				auto f_h = mesh.face_handle(opp_halfedge);
				if (f_h.idx() == -1)
				{
					continue;
				}
				auto seg_id_j = f_seg[f_h];
				if (seg_id_i != seg_id_j)
				{
					neighbor_seg_ids.insert(seg_id_j);
					inter_length[seg_id_j] += mesh.calc_edge_length(h_e_h);
				}
			}
		}
		for (const auto& neighbor_id : neighbor_seg_ids)
		{

			auto cos_angle = CalSegAngle(seg_median_normals, cur_sid, neighbor_id);
			float e_cost = inter_length[neighbor_id] * cos_angle;
			if (cur_sid < neighbor_id)
			{
				gc->setNeighbors(cur_sid, neighbor_id, inter_length[neighbor_id] * cos_angle);
			}
		}
	}

}

void GraphCut::Run(
	MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	const std::string& label_path,
	std::vector<double>& SegsElevation,
	std::vector<double>& SegsHorizontality,
	std::vector<double>& SegsPlanarity,
	std::vector<double>& NormSegsGreenness,
	//std::vector<double>& SegsDensity,
	std::vector<double>& segs_area,
	std::vector <size_t>* seg_labels)
{
	size_t num_sites = seg_face_handles.size();

	GCoptimizationGeneralGraph* gc = new GCoptimizationGeneralGraph(num_sites, 4);;
	ObtainNodeData(seg_face_handles, SegsElevation, SegsHorizontality, SegsPlanarity, NormSegsGreenness,segs_area, gc);
	ObtainEdgeData(mesh, seg_face_handles, label_path, gc);
	for (int l1 = 0; l1 < 4; l1++)
		for (int l2 = 0; l2 < 4; l2++)
		{
			int cost = l1 == l2 ? 0 : 1;
			gc->setSmoothCost(l1, l2, cost);
		}

	gc->expansion(2);

	seg_labels->reserve(num_sites);

	for (size_t i = 0; i < num_sites; i++)
	{
		seg_labels->emplace_back(gc->whatLabel(i));

	}

}

//Export the result of GraphCut
void GraphCut::WriteGCMesh(
	MeshT& mesh,
	std::string& export_path,
	SegFaceHandles seg_face_handles,
	std::vector <size_t> resultLabelArray) 
{
	auto f_seg = OpenMesh::getProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");
	for (auto s_f_h : seg_face_handles)
	{
		auto f_label= resultLabelArray[f_seg[s_f_h.front()]];
		for (auto f_hs : s_f_h)
		{
			MeshT::Color f_color;
			switch (f_label)
			{
			   case 0: {
				f_color[0] = 220;
				f_color[1] = 220;
				f_color[2] = 220;
				break;
			   }
			   case 1: {
				f_color[0] = 0;
				f_color[1] = 255;
				f_color[2] = 0;
				break;
			   }
			   case 2: {
				f_color[0] = 205;
				f_color[1] = 85;
				f_color[2] = 85;
				break;
			   }
			   case 3: {
				f_color[0] = 123;
				f_color[1] = 104;
				f_color[2] = 238;
				break;
			   }
					 break;

			}

			mesh.set_color(f_hs, f_color);
            
		}
	}
	OpenMesh::IO::Options wopt;
	wopt += OpenMesh::IO::Options::FaceColor;

	if (!OpenMesh::IO::write_mesh(mesh, export_path, wopt)) 
	{
	
		std::cout << " Fail ! !" << std::endl;
	
	}


}



