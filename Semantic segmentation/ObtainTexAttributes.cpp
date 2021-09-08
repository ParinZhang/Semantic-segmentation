#include "ObtainTexAttributes.h"
#include <opencv2/core/core.hpp>
#include <array>

ObtainTexAttri::ObtainTexAttri() {
};

ObtainTexAttri::~ObtainTexAttri() {
};


void ObtainTexAttri::FaceTexCoords(MeshT& mesh, std::vector<std::array<Eigen::Vector2f, 3>>* face_vertex_coord)
{
    for (auto f_h : mesh.faces())
    {   
        std::array<Eigen::Vector2f, 3> face_tex_coord;
        int i = 0;
        for (auto h_h : mesh.fh_range(f_h))
        {
            Eigen::Vector2f tex_coord(mesh.texcoord2D(h_h)[0], 1-mesh.texcoord2D(h_h)[1]);
            face_tex_coord[i] = tex_coord;
            i++;
        }
        face_vertex_coord->emplace_back(face_tex_coord);
    }
}

void ObtainTexAttri::FaceTexPath(MeshT& mesh, std::map<size_t,std::string>* face_tex_path,std::map<int,std::string>* mp)
{
    OpenMesh::MPropHandleT<std::map<int, std::string>> mph;
    mesh.get_property_handle(mph, "TextureMapping");
    *mp = mesh.property(mph);

    for (auto f_h : mesh.faces()) 
    {
        MeshT::TextureIndex tex_idx = mesh.texture_index(f_h);
        std::string tex_path = mp->at(tex_idx);
        face_tex_path->insert(std::pair<size_t, std::string>(f_h.idx(), tex_path));
    }

}

void ObtainTexAttri::ReadImgs(const std::map<int, std::string>& mp, std::map<int, cv::Mat>* all_src)
{
    for (auto& it : mp) 
    {
        auto img_path = it.second;
        cv::Mat src = cv::imread(img_path);
        all_src->insert({ it.first , src });

    }

}

void ObtainTexAttri::ObtainTriGreenness(
    const cv::Mat& src,
    const std::array<Eigen::Vector2f, 3>& face_tex_coords,
    std::vector<double>* greenness)
{    
    const auto& pt1 = face_tex_coords[0];
    const auto& pt2 = face_tex_coords[1];
    const auto& pt3 = face_tex_coords[2];

    int img_height = src.rows;
    int img_width = src.cols;

    std::array<cv::Point2i, 3> pixel_pts;
    pixel_pts[0] = cv::Point2i(round(pt1[0] * img_width), round(pt1[1] * img_height));
    pixel_pts[1] = cv::Point2i(round(pt2[0] * img_width), round(pt2[1] * img_height));
    pixel_pts[2] = cv::Point2i(round(pt3[0] * img_width), round(pt3[1] * img_height));

    int max_x = 0;
    int max_y = 0;
    int min_x = img_width;
    int min_y = img_height;

    for (const auto& pt : pixel_pts)
    {
        if (pt.x > max_x) max_x = pt.x;
        if (pt.y > max_y) max_y = pt.y;
        if (pt.x < min_x) min_x = pt.x;
        if (pt.y < min_y) min_y = pt.y;
    }

    std::vector<int> blue;
    std::vector<int> green;
    std::vector<int> red;

    int sum_blue = 0;
    int sum_green = 0;
    int sum_red = 0;

    for (int Py = min_y; Py < max_y; ++Py)
    {   
        for (int Px = min_x; Px < max_x; ++Px)
        {
                int Xv1 = pixel_pts[0].x, Yv1 = pixel_pts[0].y;
                int Xv2 = pixel_pts[1].x, Yv2 = pixel_pts[1].y;
                int Xv3 = pixel_pts[2].x, Yv3 = pixel_pts[2].y;
               
                float A = (Yv2 - Yv3) * (Px - Xv3) + (Xv3 - Xv2) * (Py - Yv3);
                float B = (Yv3 - Yv1) * (Px - Xv3) + (Xv1 - Xv3) * (Py - Yv3);
                float C = (Yv2 - Yv3) * (Xv1 - Xv3) + (Xv3 - Xv2) * (Yv1 - Yv3);

                float Wv1 = A / C;
                float Wv2 = B / C;
                float Wv3 = 1 - Wv1 - Wv2;

                if (Wv1 >= 0 && Wv2 >= 0 && Wv3 >= 0) 
                {   
                    int v_blue = src.at<cv::Vec3b>(Py, Px)[0];
                    int v_green = src.at<cv::Vec3b>(Py, Px)[1];
                    int v_red = src.at<cv::Vec3b>(Py, Px)[2];

                    sum_blue += v_blue;
                    sum_green += v_green;
                    sum_red += v_red;

                    blue.emplace_back(v_blue);
                    green.emplace_back(v_green);
                    red.emplace_back(v_red);
                }
        }
    }

    if (blue.size() > 0)
    {
        auto face_blue = sum_blue / blue.size();
        auto face_green = sum_green / green.size();
        auto face_red = sum_red / red.size();
        double face_greenness = face_green - 0.39 * face_red - 0.61 * face_blue;
        greenness->emplace_back(face_greenness);
    }
    else
    {
        greenness->emplace_back(-255);
    }


    //std::cout<<"***"<<std::endl;

} 

void ObtainTexAttri::NormalizeSegGreenness(std::vector<double>& seg_greenness, std::vector<double>* norm_segs_greenness)
{
    auto max_seg_greenness = *max_element(seg_greenness.begin(), seg_greenness.end());
    auto min_seg_greenness = *min_element(seg_greenness.begin(), seg_greenness.end());

       //auto max_seg_greenness = 255;
    //auto min_seg_greenness = -255;
    for (auto s_g : seg_greenness) 
    {
       double norm_seg_greenness = sqrt((s_g - min_seg_greenness) / (max_seg_greenness - min_seg_greenness));
       norm_segs_greenness->emplace_back(norm_seg_greenness);
    }

}




void ObtainTexAttri::Run(MeshT& mesh, SegFaceHandles& seg_face_handles, std::string out_path, std::vector<double>* norm_segs_geo )
{
    std::vector<std::array<Eigen::Vector2f, 3>> face_vertex_coord;
    FaceTexCoords(mesh, &face_vertex_coord);

    //FaceTexPath(mesh, &face_tex_path, &mp);
    
    OpenMesh::MPropHandleT<std::map<int, std::string>> mph;
    mesh.get_property_handle(mph, "TextureMapping");
    std::map<int, std::string> mp = mesh.property(mph);

    std::vector<double> greeness;
    std::map<int, cv::Mat> all_src;
    ReadImgs(mp, &all_src);
    
    for (size_t fid = 0; fid < face_vertex_coord.size(); fid++)
    {
        OpenMesh::SmartFaceHandle f_h(fid, &mesh);
        MeshT::TextureIndex tex_idx = mesh.texture_index(f_h);
        cv::Mat src = all_src.at(tex_idx);

        const auto& face_tex_coords = face_vertex_coord[fid];

        ObtainTriGreenness(src, face_tex_coords, &greeness);
    }

    std::vector<double> facets_geo;
    std::vector<double> segs_geo;
    FacetGeo(mesh, seg_face_handles, greeness, &facets_geo, &segs_geo);

    
    NormalizeSegGreenness(segs_geo,norm_segs_geo);



    WriteMesh(mesh, out_path, *norm_segs_geo);
    int a = 0;


}