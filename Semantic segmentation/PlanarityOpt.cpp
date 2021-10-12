#include "PlanarityOpt.h"

void CalPlaneCoffis(
	MeshT& mesh,
	VertexHandles& v_h_s,
	Eigen::Vector4d* coffis
) {
	double meanX = 0, meanY = 0, meanZ = 0;
	double meanXX = 0, meanYY = 0, meanZZ = 0;
	double meanXY = 0, meanXZ = 0, meanYZ = 0;
	for (const auto& v_h : v_h_s)
	{
		meanX += mesh.point(v_h)[0];
		meanY += mesh.point(v_h)[1];
		meanZ += mesh.point(v_h)[2];

		meanXX += mesh.point(v_h)[0] * mesh.point(v_h)[0];
		meanYY += mesh.point(v_h)[1] * mesh.point(v_h)[1];
		meanZZ += mesh.point(v_h)[2] * mesh.point(v_h)[2];

		meanXY += mesh.point(v_h)[0] * mesh.point(v_h)[1];
		meanXZ += mesh.point(v_h)[0] * mesh.point(v_h)[2];
		meanYZ += mesh.point(v_h)[1] * mesh.point(v_h)[2];

	}
	meanX /= v_h_s.size();
	meanY /= v_h_s.size();
	meanZ /= v_h_s.size();
	meanXX /= v_h_s.size();
	meanYY /= v_h_s.size();
	meanZZ /= v_h_s.size();
	meanXY /= v_h_s.size();
	meanXZ /= v_h_s.size();
	meanYZ /= v_h_s.size();

	/* eigenvector */
	Eigen::Matrix3d m;
	m(0, 0) = meanXX - meanX * meanX; m(0, 1) = meanXY - meanX * meanY; m(0, 2) = meanXZ - meanX * meanZ;
	m(1, 0) = meanXY - meanX * meanY; m(1, 1) = meanYY - meanY * meanY; m(1, 2) = meanYZ - meanY * meanZ;
	m(2, 0) = meanXZ - meanX * meanZ; m(2, 1) = meanYZ - meanY * meanZ; m(2, 2) = meanZZ - meanZ * meanZ;
	Eigen::EigenSolver<Eigen::Matrix3d> PlMat(m * v_h_s.size());
	Eigen::Matrix3d eigenvalue = PlMat.pseudoEigenvalueMatrix();
	Eigen::Matrix3d eigenvector = PlMat.pseudoEigenvectors();

	/* the eigenvector corresponding to the minimum eigenvalue */
	double v1 = eigenvalue(0, 0), v2 = eigenvalue(1, 1), v3 = eigenvalue(2, 2);
	int minNumber = 0;
	if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3)))	minNumber = 1;
	if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2)))	minNumber = 2;
	double a = eigenvector(0, minNumber), b = eigenvector(1, minNumber), c = eigenvector(2, minNumber), d = -(a * meanX + b * meanY + c * meanZ);
	Eigen::Vector4d final_coffi(a, b, c, d);
	if (c < 0)
	{
		a *= -1.0;
		b *= -1.0;
		c *= -1.0;
		d *= -1.0;
	}
	*coffis = final_coffi;
}

void Project2XYPlane(
	MeshT& mesh,
    VertexHandles& seg_vertex_handles,
	Eigen::Vector4d coffis)
{   
	for (auto const &v_h: seg_vertex_handles)
	{   
		double point_x = mesh.point(v_h)[0];
		double point_y = mesh.point(v_h)[1];
		double point_z = mesh.point(v_h)[2];

		double denominator = (coffis[0] * coffis[0] + coffis[1] * coffis[1] + coffis[2] * coffis[2]);
		double project_point_x = ((coffis[1] * coffis[1]+ coffis[2] * coffis[2]) * point_x- coffis[0]*(coffis[2]* point_x+)) / denominator;
	}




};
