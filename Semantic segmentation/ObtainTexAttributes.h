#pragma once
#include "types.h"
#include "ObtainGeoAttributes.h"

class ObtainTexAttri {
      public:
          ObtainTexAttri();
		  ~ObtainTexAttri();

          /** Obtain texture 2Dcoord of vertex in each face  */
          void FaceTexCoords(MeshT& mesh, std::vector<std::array<Eigen::Vector2f, 3>>* face_vertex_coord);

          /**Obtain texture img path of each face */
          void FaceTexPath(MeshT& mesh, std::map<size_t, std::string>* face_tex_path, std::map<int, std::string>* mp);

          /** read all imgs through map*/
          void ReadImgs(const std::map<int, std::string>& mp, std::map<int, cv::Mat>* all_src);
          
          /** Obtain the greeness tri-face   */
          void ObtainTriGreenness(
              const cv::Mat& src,
              const std::array<Eigen::Vector2f, 3>& face_tex_coords,
              std::vector<double>* greenness);
          
          void ChangeSegGreenness(std::vector<double>& seg_greenness, std::vector<double>* change_segs_greenness);
          
          /** final step */
          //void Run(MeshT& mesh, SegFaceHandles& seg_face_handles, std::string out_path);
          void Run(MeshT& mesh, SegFaceHandles& seg_face_handles, std::string out_path, std::vector<double>* norm_segs_geo);
          





};

