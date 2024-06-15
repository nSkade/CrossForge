#include "EigenMesh.hpp"

namespace CForge {
using namespace Eigen;

EigenMesh::EigenMesh(const CForge::T3DMesh<float>& inMesh) { 
	uint32_t faceCount = 0;
	for (uint32_t i = 0; i < inMesh.submeshCount(); i++) {
		faceCount += inMesh.getSubmesh(i)->Faces.size();
	}
	m_DV = MatrixXd(inMesh.vertexCount(), 3);
	m_DF = MatrixXi(faceCount, 3);

	for (uint32_t i = 0; i < inMesh.vertexCount(); ++i) {
		m_DV.row(i) = inMesh.vertex(i).cast<double>();
	}

	std::vector<std::unordered_map<uint32_t,bool>> submeshTriangles; // contains Triangle IDs
	Index startIndex = 0;
	for (uint32_t i = 0; i < inMesh.submeshCount(); i++) {
		const T3DMesh<float>::Submesh* submesh = inMesh.getSubmesh(i);
		const std::vector<T3DMesh<float>::Face>& faces = submesh->Faces;
		
		for (uint32_t j = 0; j < faces.size(); j++) {
			Vector3i subMVert(faces[j].Vertices[0], faces[j].Vertices[1], faces[j].Vertices[2]);
			m_DF.row(startIndex + j) = subMVert;
		} // for every face of a submesh
		startIndex += submesh->Faces.size();
	} // for all submeshes
}//EigenMesh

MatrixXd& EigenMesh::getDV() {
	return m_DV;
}
MatrixXi& EigenMesh::getDF() {
	return m_DF;
}

}//CForge
