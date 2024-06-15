#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>

namespace CForge {
using namespace Eigen;

class EigenMesh {
public:
	EigenMesh() = default;
	EigenMesh(const CForge::T3DMesh<float>& inMesh);
	MatrixXd& getDV();
	MatrixXi& getDF();
private:
	MatrixXd m_DV;
	MatrixXi m_DF;
};

}//CForge
