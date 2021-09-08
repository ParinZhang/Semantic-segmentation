#include "util.h"
#include "ObtainGeoAttributes.h"
#include "ObtainTexAttributes.h"
#include "GraphCut.h"

#include <fstream> 
#include <iostream>
using namespace std;

int main(int argc, char** argv)
{   
    //Read data path
    std::string mesh_path = "data/seg_material_test20210908.obj";
    std::string label_path = "data/label.dat";
   

    //Export data path
    std::string planarity_out_path = "data/Planarity.off";
    std::string elevation_out_path = "data/Elevation.off";
    std::string horizontality_out_path = "data/Horizontality.off";
    std::string greenness_out_path = "data/Greenness.off";
    std::string  export_path = "data/GraphCut.off";
    std::string path = "./result.txt";

    ofstream out_file;
    out_file.open (path, ios::out | ios::app);
   
    
 
    //Read mesh
    MeshT mesh;
    ReadMesh(mesh_path, &mesh);
    SegFaceHandles seg_face_handles;
    SegVertexHandles seg_vertex_handles;
    ObtainSegHandles(mesh, label_path, &seg_face_handles, &seg_vertex_handles);
    std::vector<double> seg_areas;
    ObtainSegAreas(mesh, seg_face_handles, &seg_areas);
    
    auto f_seg = OpenMesh::getProperty<OpenMesh::FaceHandle, size_t>(mesh, "f_seg");
    for (auto f : mesh.faces()) 
    {
        if (f.idx() == 20095)
        {
            std::cout << f_seg(f) << std::endl;
        }
        else if (f.idx() == 14225)
        {
            std::cout << f_seg(f) << std::endl;
        }else if (f.idx() == 12877)
        {
            std::cout << f_seg(f) << std::endl;
        }else if (f.idx() == 12745)
        {
            std::cout << f_seg(f) << std::endl;
        }
    
    }
    
    //Obtain Elevation
    std::vector<double> TrisElevation;
    std::vector<double> FacetsElevation;
    std::vector<double> SegsElevation;
     TriElevation2(mesh,  &TrisElevation);
    FacetGeo(mesh, seg_face_handles, TrisElevation, &FacetsElevation,&SegsElevation);
    WriteMesh(mesh, elevation_out_path, FacetsElevation);
   
    out_file << "Elevation " << endl;
    for (auto e : SegsElevation)
    {
        out_file << e << endl;
    }
    

    //Obtain Horizontality 
    std::vector<double> TrisHorizontality;
    std::vector<double> FacetsHorizontality;
    std::vector<double> SegsHorizontality;
    TriHorizontality(mesh, &TrisHorizontality);
    FacetGeo(mesh, seg_face_handles, TrisHorizontality,&FacetsHorizontality,&SegsHorizontality);
    WriteMesh(mesh, horizontality_out_path, SegsHorizontality);


    out_file << "Horizontality " << endl;
    for (auto e : SegsHorizontality)
    {
        out_file << e << endl;
    }
    //Obtain Planarity
    std::vector<double> TrisPlanarity;
    std::vector<double> FacetsPlanarity;
    std::vector<double> SegsPlanarity;
    TriPlanarity(mesh,&TrisPlanarity);
    FacetGeo(mesh, seg_face_handles, TrisPlanarity, &FacetsPlanarity,&SegsPlanarity);
    WriteMesh(mesh, planarity_out_path, SegsPlanarity);
  
    
    out_file << "Planarity " << endl;
    for (auto e : SegsPlanarity)
    {
        out_file << e << endl;
    }


    //Obtain Greenness

    std::vector<double> NormSegsGreenness;
    ObtainTexAttri OTA;
    OTA.Run(mesh, seg_face_handles,  greenness_out_path, &NormSegsGreenness);

    out_file << "NormSegsGreenness "<< endl;
    for (auto e : NormSegsGreenness)
    {
        out_file << e<<endl;
    }

    //GraphCut
    
    std::vector <size_t> resultLabelArray;
    GraphCut GC;
    GC.Run(mesh, seg_face_handles, label_path, SegsElevation, SegsHorizontality, SegsPlanarity, NormSegsGreenness, seg_areas, &resultLabelArray);
    GC.WriteGCMesh(mesh, export_path, seg_face_handles, resultLabelArray);
    



   
 
 
}

