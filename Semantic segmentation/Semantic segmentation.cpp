#include "util.h"
#include "ObtainGeoAttributes.h"
#include "ObtainTexAttributes.h"
#include "GraphCut.h"

#include <fstream> 
#include <iostream>

#include "TestData.h"
using namespace std;

int main(int argc, char** argv)
{   
    //Read data path
    std::string mesh_path = "data/subTile_1984_2693.obj";
    std::string label_path = "data/label.dat";
   

    //Export data path
    std::string planarity_out_path = "data/Planarity.off";
    std::string elevation_out_path = "data/Elevation.off";
    std::string horizontality_out_path = "data/Horizontality.off";
    std::string greenness_out_path = "data/Greenness.off";
    std::string density_out_path = "data/Density.off";
    std::string export_path = "data/GraphCut.off";
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
    Eigen::ArrayXXf topology_map = Eigen::ArrayXXf::Zero(seg_face_handles.size(), seg_face_handles.size());
    GetSegTopologyMap(mesh, seg_face_handles,  &topology_map);
    GetSegId(94819, mesh);


    //Obtain Greenness
    std::vector<double> norm_segs_greenness;
    ObtainTexAttri OTA;
    OTA.Run(mesh, seg_face_handles, greenness_out_path, &norm_segs_greenness);

    //Obtain Density
    std::vector<double> norm_segs_density;
    ObtainFacetSegDensity(mesh, seg_face_handles, &norm_segs_density);

 
   //Obtain Planarity
    std::vector<double> SegsPlanarity;
   //SegPlanarity(mesh, seg_face_handles, &SegsPlanarity);
    DoPlanarity3(mesh, seg_face_handles, &SegsPlanarity);
    WriteMesh(mesh, planarity_out_path, SegsPlanarity);
    ExportData("Planarity.txt", SegsPlanarity);

    //Obtain Elevation
    std::vector<double> TrisElevation;
    std::vector<double> FacetsElevation;
    std::vector<double> SegsElevation;
    TriElevation2(mesh,  &TrisElevation);
    FacetGeo(mesh, seg_face_handles, TrisElevation, &FacetsElevation,&SegsElevation);
    WriteMesh(mesh, elevation_out_path, FacetsElevation);
    
    ExportData("Elevation.txt", SegsElevation);

    //Obtain Horizontality 
    std::vector<double> TrisHorizontality;
    std::vector<double> FacetsHorizontality;
    std::vector<double> SegsHorizontality;
    TriHorizontality(mesh, &TrisHorizontality);
    FacetGeo(mesh, seg_face_handles, TrisHorizontality,&FacetsHorizontality,&SegsHorizontality);
    WriteMesh(mesh, horizontality_out_path, SegsHorizontality);
    ExportData("Horizontality.txt", SegsHorizontality);


    //GraphCut
    std::vector <size_t> resultLabelArray;
    GraphCut GC;
    GC.Run(mesh, seg_face_handles, label_path, SegsElevation, SegsHorizontality, SegsPlanarity, norm_segs_greenness, seg_areas, &resultLabelArray);

    GC.WriteGCMesh(mesh, export_path, seg_face_handles, resultLabelArray);

    GC.ModifyInitLabel(resultLabelArray, topology_map);
    GC.WriteGCMesh(mesh, export_path, seg_face_handles, resultLabelArray);
    int y = 0;
}

