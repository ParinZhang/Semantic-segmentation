#include "ObtainGeoAttributes.h"

// Obtain TriCentroid
void ObtainTriCentroid(
	const MeshT& mesh,
	//const SegFaceHandles& seg_face_handles,
	std::vector<MyTraits::Point>* TriCentroids
)
{
	TriCentroids->reserve(mesh.n_faces());
	for (const auto& face_handles : mesh.faces()) 
	{
			auto pt = mesh.calc_centroid(face_handles);
			TriCentroids->emplace_back(pt);
	}

}

//Calculate the AveEdge
double  ObtainAveEdge(const MeshT& mesh)
{
	double sum_edge_length = 0;
	for (auto e : mesh.edges())
	{
		auto e_length=mesh.calc_edge_length(e);
		sum_edge_length += mesh.calc_edge_length(e);
		//sum_edge_length += mesh.calc_edge_length(e);

	}

	double ave_edge_length = sum_edge_length / mesh.n_edges();
	return ave_edge_length;
}

std::set<Face> GetKRingFaces(const MeshT& mesh, const int k, const Face& f)
{
	std::set<Face> neighbors, new_neighbors, add1_neighbors, add2_neighbors, last_neighbors;
	std::set<Face> f_mark;

	neighbors.insert(f);
	new_neighbors.insert(f);

	last_neighbors = new_neighbors;
	int i = 0,if_break=0;
	for (;;) 
	{   
		//std::cout << i << std::endl;
		if (if_break == 0) {
			last_neighbors = new_neighbors;
			new_neighbors.clear();
			for (const auto& neighbor : last_neighbors)
			{
				if (i < k)
				{
					for (const auto ff_h : mesh.ff_range(neighbor))
					{
						new_neighbors.insert(ff_h);
					}

					if (i == 0) {
						new_neighbors.erase(f);

					}
					neighbors.insert(new_neighbors.begin(), new_neighbors.end());
					
					//new_neighbors.clear();				
				}
				else
				{
					if_break = 1;
					break;
				}
			
			}
		}
		else { break; }
		i++;
	}
		
	
	//neighbors.erase(f);

	return neighbors;
	//new_neighbors.insert(f);
	//add1_neighbors.insert(f);





};

//Version-2 Obtain Facet Elevation
void  TriElevation2(MeshT& mesh,
	std::vector<double>* TrisElevation) 
{
	
	std::vector<MyTraits::Point> TriCentroids;
	ObtainTriCentroid(mesh, &TriCentroids);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(TriCentroids.size());
	for (std::size_t i = 0; i < cloud->size(); ++i)
	{
		(*cloud)[i].x = TriCentroids[i][0];
		(*cloud)[i].y = TriCentroids[i][1];
		(*cloud)[i].z = TriCentroids[i][2];
	}


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 36.8;
	
	for (auto centroid : TriCentroids) {
		pcl::PointXYZ searchPoint;

		searchPoint.x = centroid[0];
		searchPoint.y = centroid[1];
		searchPoint.z = centroid[2];
		
		kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	

		std::vector<double> tri_height;
		for (const auto& point_idx:pointIdxRadiusSearch) 
		{
			auto cloud_Radius = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			cloud_Radius->emplace_back((cloud->points[point_idx]));
		    
		}

		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(*cloud, minPt, maxPt);
		auto height_min = minPt.z;
		auto height_max = maxPt.z;

		double elevation_face = sqrt((searchPoint.z -height_min) / (height_max - height_min));
		TrisElevation->emplace_back(elevation_face);
		int a = 0;
		
	}
	
}


//Version-3 Obtain Facet Elevation
void  TriElevation3(MeshT& mesh,
	std::vector<double>* TrisElevation) 
{
	std::vector<std::set <Face>> ring_3_faces;
	for (const auto& f : mesh.faces()) 
	{    
		std::set <Face> ring_3_face;
		for (const auto& f_1f : mesh.ff_range(f)) 
		{
			for (const auto& f_2f : mesh.ff_range(f_1f))
			{
				for (const auto& f_3f : mesh.ff_range(f_2f))
				{
					ring_3_face.insert(f_3f);
				}
			}
	
		}
		ring_3_faces.emplace_back(ring_3_face);
	}


	auto ave_edge = ObtainAveEdge(mesh);
	auto cir_num = int(round(36.375 / ave_edge));
	std::vector<std::set <Face>> neibor;
	for (const auto& f : mesh.faces())
	{   
		TimeCount ring("3_ring");
		std::set <Face> init_faces = ring_3_faces[f.idx()];
		for (int i = 0; i < int(round(cir_num/3)); i++)
		{    
			std::set<Face> next_faces;
			for (const auto& f_r: init_faces)
			{
				next_faces.insert(ring_3_faces[f_r.idx()].begin(), ring_3_faces[f_r.idx()].end());
				 
			}
			init_faces = next_faces;
			if (i != int(round(cir_num / 3)) - 1)
			{
				next_faces.clear();
			}

			else
			{
				neibor.emplace_back(next_faces);
			}
		}
		ring.end();
	}
}

// Obtain Facet Elevation
void  TriElevation(
	 MeshT& mesh,
	std::vector<double>* TrisElevation) {
	 std::vector<MyTraits::Point> TriCentroids;
	 //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	 ObtainTriCentroid(mesh,  &TriCentroids);

	 auto ave_edge = ObtainAveEdge(mesh);
	 auto cir_num = int(round(36.375 / ave_edge));
	 //int cir_num = 10;
	 
	 for (auto f : mesh.faces()) 
	 {
		 TimeCount FN("Find neighbor");
		 std::vector<double> neighbor_elevation;
		 TimeCount GKRF("GetKRingFaces");
		 auto neighbor_faces = GetKRingFaces(mesh, cir_num, f);
		 GKRF.end();
		 for (auto const& n_f : neighbor_faces) 
		 {
			auto pt_height= TriCentroids[n_f.idx()][2];
			neighbor_elevation.emplace_back(pt_height);
		 }

		 double elevation_max = *max_element(neighbor_elevation.begin(),neighbor_elevation.end());
		 double elevation_min = *min_element(neighbor_elevation.begin(), neighbor_elevation.end());
		 double elevation_tri = TriCentroids[f.idx()][2];

		 double face_elevation = sqrt((elevation_tri - elevation_min) / (elevation_max - elevation_min));

		 TrisElevation->emplace_back(face_elevation);
		 FN.end();
		
	 }
	 

	




	};

/** Obtain Facet  horizontality */
void TriHorizontality(
	  MeshT& mesh,
	std::vector<double>* TrisHorizontality){ 
	TrisHorizontality->reserve(mesh.n_faces());
	Eigen::Vector3d norm_z(0, 0, 1);
	mesh.request_vertex_normals();
	mesh.request_face_normals();
	for (auto f_h : mesh.faces()) 
	{
		Eigen::Vector3d norm_i(mesh.normal(f_h)[0], mesh.normal(f_h)[1], mesh.normal(f_h)[2]);
		double Horizontality = fabs(norm_i.dot(norm_z));
		TrisHorizontality->emplace_back(Horizontality);
	
	}


}  

/**Obtain Segment Planarity*/
void SegPlanarity(
	MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	std::vector<double>* SegPlanarity) 
{   
	std::vector<MyTraits::Point> TriCentroids;
	ObtainTriCentroid(mesh, &TriCentroids);

	//auto find_faceid = OpenMesh::PropertyManager<OpenMesh::FaceHandle, size_t>(mesh,"f_seg");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(TriCentroids.size());
	for (std::size_t i = 0; i < cloud->size(); ++i)
	{    
	
		(*cloud)[i].x = TriCentroids[i][0];
		(*cloud)[i].y = TriCentroids[i][1];
		(*cloud)[i].z = TriCentroids[i][2];
	}


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	float radius = 8;

	SegPlanarity->reserve(seg_face_handles.size());
	auto f_seg = OpenMesh::getProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");
	for (const auto& s_f_h : seg_face_handles)
	{   
		std::vector<double> fittings;
		for (const auto& face : s_f_h)
		{
			pcl::PointXYZ searchPoint;
			searchPoint.x = TriCentroids[face.idx()][0]; searchPoint.y = TriCentroids[face.idx()][1]; searchPoint.z = TriCentroids[face.idx()][2];
			std::vector<int> point_idx_radius_search;
			std::vector<float> point_radius_squared_distance;
			kdtree.radiusSearch(searchPoint, radius, point_idx_radius_search, point_radius_squared_distance);
		    
			std::set<Point> seg_points;
			for (const auto idx : point_idx_radius_search)
			{
				auto tri_centroid = TriCentroids[idx];
				Point point(tri_centroid[0], tri_centroid[1], tri_centroid[2]);
			    seg_points.insert(point);

			
			}
			Plane plane;
			auto fitting = linear_least_squares_fitting_3(seg_points.begin(), seg_points.end(), plane, CGAL::Dimension_tag<0>());
			fittings.emplace_back(fitting);
		}
		
		double sum = 0.0;
		for (const auto& fitting : fittings)
		{

			sum += fitting;
		}
		auto seg_planary = sum / s_f_h.size();
	
		SegPlanarity->emplace_back(seg_planary);
	}
	
}



//Calculate segment attribute based on the triangle attribute
void FacetGeo(MeshT &mesh,
	SegFaceHandles& seg_face_handles, 
	std::vector<double> TrisGeo,
	std::vector<double>* FacetsGeo,
	std::vector<double>* SegGeo

) {
	FacetsGeo->resize(mesh.n_faces());
	for (auto& f_hs : seg_face_handles) 
	{   
		//std::cout << "/////" << std::endl;
		double attri_mul_area =0;
		double seg_area = 0;
		for (auto& f_h: f_hs)
		{
			attri_mul_area += TrisGeo[f_h.idx()] * (mesh.calc_face_area(f_h));
			//std::cout << sum_planarity << std::endl;
			seg_area += mesh.calc_face_area(f_h);
		    
		}
		for (auto& f_h : f_hs)
		{
			FacetsGeo->at(f_h.idx())= attri_mul_area / seg_area;

		}
		SegGeo->emplace_back(attri_mul_area / seg_area);

	}
}

//Obtain SegDensity
void ObtainTriSegDensity(MeshT& mesh,
	std::vector<double>* tri_seg_density) 
{
	std::vector<MyTraits::Point> TriCentroids;
	ObtainTriCentroid(mesh, &TriCentroids);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(TriCentroids.size());
	for (std::size_t i = 0; i < cloud->size(); ++i)
	{
		(*cloud)[i].x = TriCentroids[i][0];
		(*cloud)[i].y = TriCentroids[i][1];
		(*cloud)[i].z = TriCentroids[i][2];
	}


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 10;

	for (auto centroid : TriCentroids) {
		pcl::PointXYZ searchPoint;

		searchPoint.x = centroid[0];
		searchPoint.y = centroid[1];
		searchPoint.z = centroid[2];

		kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

		std::vector<Face> mesh_faces;
		auto p_manager = OpenMesh::getProperty<OpenMesh::FaceHandle,size_t>(mesh,"f_seg");
		for (auto face : mesh.faces())
		{
			mesh_faces.emplace_back(face);

		}

		std::set<int>neibor_seg_ids;
		for (const auto& point_idx : pointIdxRadiusSearch)
		{
			auto neibor_segid=p_manager(mesh_faces[point_idx]);
			neibor_seg_ids.insert(neibor_segid);

		}
		tri_seg_density->emplace_back(neibor_seg_ids.size());		
	}
}

void ObtainFacetSegDensity(
	MeshT& mesh,
	SegFaceHandles seg_face_handles,
	std::vector<double>* seg_density_normalize
) 
{   
	
     std::vector<double> tri_seg_density;
	ObtainTriSegDensity(mesh, &tri_seg_density);
	
	auto max_density = *max_element(tri_seg_density.begin(), tri_seg_density.end());
	auto min_density = *min_element(tri_seg_density.begin(), tri_seg_density.end());

	std::vector<double> norm_seg_density;
	for (const auto& t_s_d : tri_seg_density)
	{    
		auto n_s_d = (t_s_d - min_density) / (max_density- min_density);
		norm_seg_density.emplace_back(n_s_d);
	}

	auto mesh_property_manager = OpenMesh::getProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");
	std::vector<double> facet_seg_density;
	for (const auto& s_f_h : seg_face_handles)
	{   
		double sum = 0.0;
		for (const auto& f_h : s_f_h) 
		{
			auto f_id = mesh_property_manager(f_h);
			sum += norm_seg_density[f_id];   
		}
		double f_s_d = sum / s_f_h.size();
		seg_density_normalize->emplace_back(f_s_d);
	}

	
}




//Write mesh
void WriteMesh(
	MeshT& mesh,
	std::string& out_path,
	std::vector<double>& segs_geo) {
	mesh.request_face_colors();
	auto f_seg = OpenMesh::getProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");
	double tris_geometry_max = *std::max_element(segs_geo.begin(), segs_geo.end());
	double tris_geometry_min = *std::min_element(segs_geo.begin(), segs_geo.end());

	for (auto f_h : mesh.faces())
	{    
		auto p = f_seg[f_h];
		MeshT::Color f_color;
		auto robust_tris_geomrtry = segs_geo[f_seg[f_h]];
	
			f_color[0] = 222 * (1 - robust_tris_geomrtry);
			f_color[1] = 222 * (1 - robust_tris_geomrtry);
			f_color[2] = 222;

			mesh.set_color(f_h, f_color);

	}
	OpenMesh::IO::Options wopt;
	wopt += OpenMesh::IO::Options::FaceColor;

	if (!OpenMesh::IO::write_mesh(mesh, out_path, wopt))
	{

		std::cout << "Fail ! !" << std::endl;
	}

}



