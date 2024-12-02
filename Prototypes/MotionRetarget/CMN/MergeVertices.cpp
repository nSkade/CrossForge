#include "MergeVertices.hpp"

namespace CForge {

uint32_t getMatchingVertex(uint32_t RedundantVertexID, std::vector<std::pair<uint32_t, uint32_t>> *pRedundantVertices) {
	uint32_t Rval = 0;

	for (auto [original,copy] : (*pRedundantVertices)) {
		if (copy == RedundantVertexID) {
			Rval = original;
			break;
		}
	}
	return Rval;
}//getMatchingVertex

std::vector<uint32_t> mergeRedundantVertices(T3DMesh<float>* pMesh) {
	float Epsilon = /**/std::numeric_limits<float>::min();//*/0.00025f;//TODO

	std::vector<std::pair<uint32_t, uint32_t>> RedundantVertices; // <first occurence, copy>

	std::vector<uint32_t> vertCorr; // old to new vertex correspondence
	
	std::vector<bool> IsRedundant;
	std::vector <uint32_t> VertexMapping;
	for (uint32_t i = 0; i < pMesh->vertexCount(); ++i) {
		IsRedundant.push_back(false);
		VertexMapping.push_back(i);
		vertCorr.push_back(i);
	}

	// find redundant vertices
	for (uint32_t i = 0; i < pMesh->vertexCount(); ++i) {
		if (IsRedundant[i])
			continue;
		auto v1 = pMesh->vertex(i);

		for (uint32_t k = i+1; k < pMesh->vertexCount(); ++k) {
			auto v2 = pMesh->vertex(k);

			if ((v2 - v1).dot(v2 - v1) < Epsilon) {
				RedundantVertices.push_back(std::pair<uint32_t, uint32_t>(i,k));
				IsRedundant[k] = true;
				vertCorr[k] = i;
			}
		}//for[all remaining vertices]
	}//for[all vertices]

	printf("Found %d double vertices                             \n", uint32_t(RedundantVertices.size()));

	// rebuild vertices, normals, tangents
	std::vector<Eigen::Vector3f> Vertices;
	for (uint32_t i = 0; i < pMesh->vertexCount(); ++i) {
		VertexMapping[i] = Vertices.size();
		if (!IsRedundant[i])
			Vertices.push_back(pMesh->vertex(i));
	}

	for (uint32_t i=0;i<vertCorr.size();++i)
		vertCorr[i] = VertexMapping[vertCorr[i]];

	std::vector<Eigen::Vector3f> Normals;
	for (uint32_t i = 0; i < pMesh->normalCount(); ++i) {
		if (!IsRedundant[i]) Normals.push_back(pMesh->normal(i));
	}

	std::vector<Eigen::Vector3f> Tangents;
	for (uint32_t i = 0; i < pMesh->tangentCount(); ++i) {
		if (!IsRedundant[i]) Tangents.push_back(pMesh->tangent(i));
	}

	// replace indices in faces
	for (uint32_t i = 0; i < pMesh->submeshCount(); ++i) {
		auto* pM = pMesh->getSubmesh(i);
		for (auto& f : pM->Faces) {
			if (IsRedundant[f.Vertices[0]]) f.Vertices[0] = getMatchingVertex(f.Vertices[0], &RedundantVertices);
			if (IsRedundant[f.Vertices[1]]) f.Vertices[1] = getMatchingVertex(f.Vertices[1], &RedundantVertices);
			if (IsRedundant[f.Vertices[2]]) f.Vertices[2] = getMatchingVertex(f.Vertices[2], &RedundantVertices);

			// and remap
			f.Vertices[0] = VertexMapping[f.Vertices[0]];
			f.Vertices[1] = VertexMapping[f.Vertices[1]];
			f.Vertices[2] = VertexMapping[f.Vertices[2]];

		}//for[faces of submesh]
	}//for[submeshes]

	// replace vertex weights
	for (uint32_t i = 0; i < pMesh->boneCount(); ++i) {
		auto* pBone = pMesh->getBone(i);

		// collect data
		std::vector<int32_t> Influences;
		std::vector<float> Weights;

		for (uint32_t k = 0; k < pBone->VertexInfluences.size(); ++k) {
			uint32_t ID = pBone->VertexInfluences[k];
			if (IsRedundant[ID]) pBone->VertexInfluences[k] = getMatchingVertex(ID, &RedundantVertices);

			pBone->VertexInfluences[k] = VertexMapping[pBone->VertexInfluences[k]];
		}
	}//for[all bones]

	// replace mesh data
	if (Vertices.size() > 0) pMesh->vertices(&Vertices);
	if (Normals.size() > 0) pMesh->normals(&Normals);
	if (Tangents.size() > 0) pMesh->tangents(&Tangents);

	return vertCorr;
}//mergeDoubleVertices

}//CForge
