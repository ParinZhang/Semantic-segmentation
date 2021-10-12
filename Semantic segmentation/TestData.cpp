#include "types.h"


//Export data into txt//
void ExportData(std::string export_path, std::vector<double> segs_attributes)
{
  
    std::ofstream myfile;
    
    myfile.open(export_path, std::ios::out);
    if (myfile.is_open())
    {
        for (auto const seg_attributes:segs_attributes)
        {
            myfile << seg_attributes << std::endl;
        }

    }
    myfile.close();
    
}
//Export data into txt//

//Find segment id//
void GetSegId(size_t faceid, MeshT& mesh) 
{
    int i = 0;
    auto f_seg = OpenMesh::getProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");
    for (const auto face_handle : mesh.faces())
    {
        if (i == faceid) 
        {
            std::cout << faceid << "is" << f_seg(face_handle) << std::endl;
        }
        i++;
    }

}
//Find segment id//

