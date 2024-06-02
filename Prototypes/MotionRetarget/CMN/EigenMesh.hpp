#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>

namespace CForge {

class EigenMesh {
public:
	EigenMesh(const CForge::T3DMesh<float>& inMesh);
	Eigen::MatrixXd& getDV();
	Eigen::MatrixXi& getDF();
private:
	Eigen::MatrixXd m_DV;
	Eigen::MatrixXi m_DF;
};

}//CForge
