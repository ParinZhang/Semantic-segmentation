#include "ObtainGeoAttributes.h"

// Obtain TriCentroid
void ObtainTriCentroid(
	const MeshT& mesh,
	//const SegFaceHandles& seg_face_handles,
	std::vector<MyTraits::Point>* TriCentroids
)
{
	TriCentroids->reserve(mesh.n_faces());
	for (const auto& face_handle : mesh.faces()) 
	{
			auto pt = mesh.calc_centroid(face_handle);
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

//** Elevation    *//

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
//** Elevation     *//

//**Horizontality**//
/* Obtain Facet  horizontality */
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
//** Horizontality**//

//** Planarity**//
/**Obtain Segment Planarity*/
/*
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

*/

void DoPlanarity3(
	MeshT& mesh,
	SegFaceHandles& seg_face_handles,
	std::vector<double>* inliners_ratios
)
{
	std::vector<MyTraits::Point> tri_centroids;
	ObtainTriCentroid(mesh, &tri_centroids);
	for (const auto& s_f_h : seg_face_handles)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& f_h : s_f_h)
		{
			int face_id = f_h.idx();
			pcl::PointXYZ p;
			p.x = tri_centroids[face_id][0];
			p.y = tri_centroids[face_id][1];
			p.z = tri_centroids[face_id][2];
			cloud->emplace_back(p);

		}
		double meanX = 0, meanY = 0, meanZ = 0;
		double meanXX = 0, meanYY = 0, meanZZ = 0;
		double meanXY = 0, meanXZ = 0, meanYZ = 0;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			meanX += cloud->points[i].x;
			meanY += cloud->points[i].y;
			meanZ += cloud->points[i].z;

			meanXX += cloud->points[i].x * cloud->points[i].x;
			meanYY += cloud->points[i].y * cloud->points[i].y;
			meanZZ += cloud->points[i].z * cloud->points[i].z;

			meanXY += cloud->points[i].x * cloud->points[i].y;
			meanXZ += cloud->points[i].x * cloud->points[i].z;
			meanYZ += cloud->points[i].y * cloud->points[i].z;
		}
		meanX /= cloud->points.size();
		meanY /= cloud->points.size();
		meanZ /= cloud->points.size();
		meanXX /= cloud->points.size();
		meanYY /= cloud->points.size();
		meanZZ /= cloud->points.size();
		meanXY /= cloud->points.size();
		meanXZ /= cloud->points.size();
		meanYZ /= cloud->points.size();

		/* eigenvector */
		Eigen::Matrix3d m;
		m(0, 0) = meanXX - meanX * meanX; m(0, 1) = meanXY - meanX * meanY; m(0, 2) = meanXZ - meanX * meanZ;
		m(1, 0) = meanXY - meanX * meanY; m(1, 1) = meanYY - meanY * meanY; m(1, 2) = meanYZ - meanY * meanZ;
		m(2, 0) = meanXZ - meanX * meanZ; m(2, 1) = meanYZ - meanY * meanZ; m(2, 2) = meanZZ - meanZ * meanZ;
		Eigen::EigenSolver<Eigen::Matrix3d> PlMat(m * cloud->points.size());
		Eigen::Matrix3d eigenvalue = PlMat.pseudoEigenvalueMatrix();
		Eigen::Matrix3d eigenvector = PlMat.pseudoEigenvectors();

		/* the eigenvector corresponding to the minimum eigenvalue */
		double v1 = eigenvalue(0, 0), v2 = eigenvalue(1, 1), v3 = eigenvalue(2, 2);
		int minNumber = 0;
		if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3)))	minNumber = 1;
		if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2)))	minNumber = 2;
		double a = eigenvector(0, minNumber), b = eigenvector(1, minNumber), c = eigenvector(2, minNumber), d = -(a * meanX + b * meanY + c * meanZ);
		Eigen::Vector4d coffi(a, b, c, d);
		if (c < 0)
		{
			a *= -1.0;
			b *= -1.0;
			c *= -1.0;
			d *= -1.0;
		}

		pcl::IndicesPtr inliers(new std::vector <int>);
		double distance_thres = 0.3;
		GetInliersLSP(coffi, cloud, inliers, distance_thres);


		MeshT mesh1; MeshT mesh2;
		//ExportRansac2(mesh1, mesh2, inliers, cloud);

		double inliners_ratio = (double)(*inliers).size() / cloud->points.size();
		inliners_ratios->emplace_back(inliners_ratio);
	}

}

void GetInliersLSP(
	Eigen::Vector4d coffi,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::IndicesPtr inliers,
	double distance_thres
)
{
	double coffi_length = sqrt(coffi[0] * coffi[0] + coffi[1] * coffi[1] + coffi[2] * coffi[2]);
	int i = 0;
	for (const auto& pt : cloud->points)
	{
		double distance = abs(coffi[0] * pt.x + coffi[1] * pt.y + coffi[2] * pt.z + coffi[3]) / coffi_length;
		if (distance < distance_thres)
		{
			inliers->emplace_back(i);

		}
		i++;
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

//** Planarity**//

//** Density **//
/* Obtain Triangle SegDensity */
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

	for (auto centroid : TriCentroids) 
	{
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

//
void ObtainFacetSegDensity(
	MeshT& mesh,
	SegFaceHandles seg_face_handles,
	std::vector<double>* norm_ave_density)
{   
	
    std::vector<double> tri_seg_density;
	ObtainTriSegDensity(mesh, &tri_seg_density);
	
	
	std::vector<double> facets_seg_density;
	for (const auto& t_s_d : tri_seg_density)
	{   
		double f_s_d = 0;
		f_s_d += t_s_d;
		//average
		auto f_s_d = f_s_d / (double)tri_seg_density.size(); 
		//medium												
		/* 
		sort(tri_seg_density.begin(),tri_seg_density.end());
		auto f_s_d = f_s_d / (double)tri_seg_density.size(); 
		*/
		facets_seg_density.emplace_back(f_s_d);
	}

	//average
	double ave_density;
	ave_density= std::accumulate(facets_seg_density.begin(), facets_seg_density.end(), 0.);
	//std::vector<double> norm_ave_density;
	for (const auto& f_s_d : facets_seg_density)
	{   
		if ((f_s_d / ave_density) < 1)
		{
			norm_ave_density->emplace_back(f_s_d / ave_density);
		}else
		{
			norm_ave_density->emplace_back(1.0);
		}
		
	}

	//medium
	double medium_density;
	medium_density = facets_seg_density[facets_seg_density.size()/2];
	std::vector<double> norm_med_density;
	for (const auto& f_s_d : facets_seg_density)
	{
		if ((f_s_d / medium_density) < 1)
		{
			norm_med_density.emplace_back(f_s_d / medium_density);
		}
		else
		{
			norm_med_density.emplace_back(1.0);
		}
	}

	
}

//** Density **//



//** Write mesh **//
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
	
			f_color[0] = 255 * (1 - robust_tris_geomrtry);
			f_color[1] = 255 * (1 - robust_tris_geomrtry);
			f_color[2] = 0;

			mesh.set_color(f_h, f_color);

	}
	OpenMesh::IO::Options wopt;
	wopt += OpenMesh::IO::Options::FaceColor;

	if (!OpenMesh::IO::write_mesh(mesh, out_path, wopt))
	{

		std::cout << "Fail ! !" << std::endl;
	}

}
//** Write mesh **//



