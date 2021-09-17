#include "util.h"


// IO //
/** Read mesh */
void ReadMesh(const std::string& path, MeshT* mesh)
{  

	mesh->request_vertex_texcoords2D();
	mesh->request_face_texture_index();
	mesh->request_halfedge_texcoords2D();
	OpenMesh::IO::Options ropt;
	//ropt += OpenMesh::IO::Options::VertexTexCoord;
	
	ropt += OpenMesh::IO::Options::FaceTexCoord;
	if (!OpenMesh::IO::read_mesh(*mesh, path, ropt))
	{
		std::cout << "Fail to read mesh!" << std::endl;
	}
	
	mesh->request_face_normals();
	mesh->request_vertex_normals();
	mesh->update_normals();
	
	
}
// IO //


// SEGMENT //
/** Obtain face handles and vertex handles */
void ObtainSegHandles(
	MeshT& mesh,
	const std::string& label_name,
	SegFaceHandles* seg_face_handles,
	SegVertexHandles* seg_vertex_handles)
{
	// The first element in label.dat is the number of segments
	std::vector<size_t> face_seg_ids = ReadLabels<size_t>(label_name);
	size_t num_segments = face_seg_ids.front();

	//Add new property
	auto f_seg = OpenMesh::getOrMakeProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");
	auto a = mesh.n_faces();
	for (auto& f_h : mesh.faces()) {
		f_seg[f_h] = face_seg_ids[f_h.idx()+1 ];
	}
	
	// Obtain face handles per segment
	seg_face_handles->resize(num_segments);
	for (size_t face_id = 0; face_id < face_seg_ids.size() - 1; face_id++)
	{
		OpenMesh::SmartFaceHandle face_handle = static_cast<OpenMesh::SmartFaceHandle>(face_id);
		seg_face_handles->at(face_seg_ids[face_id + 1]).emplace_back(face_handle);
	}

	// Obtain vertex handles per segment
	seg_vertex_handles->resize(num_segments);
	for (auto seg_id = 0; seg_id < num_segments; seg_id++)
	{
		std::set<OpenMesh::SmartVertexHandle> set_vhandles;
		for (const auto& f_h : seg_face_handles->at(seg_id))
		{
			for (const auto& v_h : mesh.fv_range(f_h))
			{
				set_vhandles.insert(v_h);
			}
		}
		seg_vertex_handles->at(seg_id).assign(set_vhandles.begin(), set_vhandles.end());
	}
}



/** Obtain segment area */
void ObtainSegAreas(
	const MeshT& mesh,
	const SegFaceHandles& seg_face_handles,
	std::vector<double>* seg_areas)
{
	seg_areas->reserve(seg_face_handles.size());

	for (const auto& face_handles : seg_face_handles)
	{
		double seg_area = 0.0;
		for (const auto& f_h : face_handles)
		{
			seg_area += mesh.calc_face_area(f_h);
		}
		seg_areas->emplace_back(seg_area);
	}
} 

void ObtainSegNormals(
	const MeshT& mesh,
	const SegFaceHandles& seg_face_handles,
	SegMedianNormals* seg_median_normals)
{
	seg_median_normals->reserve(seg_face_handles.size());

	for (const auto& face_handles : seg_face_handles)
	{
		std::vector<double> norm_set_x;
		std::vector<double> norm_set_y;
		std::vector<double> norm_set_z;

		norm_set_x.reserve(face_handles.size());
		norm_set_y.reserve(face_handles.size());
		norm_set_z.reserve(face_handles.size());

		for (const auto& f_h : face_handles)
		{
			norm_set_x.emplace_back(mesh.normal(f_h)[0]);
			norm_set_y.emplace_back(mesh.normal(f_h)[1]);
			norm_set_z.emplace_back(mesh.normal(f_h)[2]);
		}

		double median_norm_x, median_norm_y, median_norm_z;
		std::nth_element(norm_set_x.begin(), norm_set_x.begin() + norm_set_x.size() / 2, norm_set_x.end());
		std::nth_element(norm_set_y.begin(), norm_set_y.begin() + norm_set_y.size() / 2, norm_set_y.end());
		std::nth_element(norm_set_z.begin(), norm_set_z.begin() + norm_set_z.size() / 2, norm_set_z.end());

		median_norm_x = norm_set_x[norm_set_x.size() / 2];
		median_norm_y = norm_set_y[norm_set_y.size() / 2];
		median_norm_z = norm_set_z[norm_set_z.size() / 2];

		seg_median_normals->emplace_back(Eigen::Vector3d(median_norm_x, median_norm_y, median_norm_z));
	}
}

 float CalSegAngle(SegMedianNormals& seg_median_normals,size_t seg_id_i,size_t seg_id_j) 
 {
	 auto n1 = static_cast<Eigen::Vector3d>(seg_median_normals[seg_id_i]);
	 auto n2 = static_cast<Eigen::Vector3d>(seg_median_normals[seg_id_j]);
	 return n1.dot(n2) ;
 }



// SEGMENT //





