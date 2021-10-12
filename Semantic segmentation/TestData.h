#pragma once
#include "types.h"

//Export data into txt//
void ExportData(std::string export_path, std::vector<double> segs_attributes);
//Export data into txt//

//Find segment id//
void GetSegId(size_t faceid, MeshT& mesh);
//Find segment id//
