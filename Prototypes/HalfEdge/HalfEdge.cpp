#include <crossforge/AssetIO/SAssetIO.h>
#include "HalfEdge.h"
#include <crossforge/Utility/CForgeUtility.h>
#include <crossforge/Math/CForgeMath.h>

using namespace CForge;
using namespace std;
using namespace Eigen;

// this is taken from: 

namespace CForge {

		HalfEdgeSetup::HalfEdgeSetup(){
			// empty
		}

		void HalfEdgeSetup::buildHEDataStructure(std::vector<Eigen::Vector3f> Positions, std::vector<Eigen::Vector3i> Faces) {
		// clean up old data
		clearHEDataStructure();

		// every position translates to one Vertex
		for (uint32_t i = 0; i < Positions.size(); ++i) {
			Vertex* pV = new Vertex();	// always create the structures using new
			pV->Position = Positions[i];
			pV->ID = int32_t(m_Vertices.size()); // you can create unique IDs always like this
			m_Vertices.push_back(pV); // simple push back to store the newly created data
		}//for[positions]

		// create the rest of the data structure ...

		//from every face create polygon
		for (uint32_t i = 0; i < Faces.size(); i++) {
			Polygon* pP = new Polygon();

			HalfEdge* hP0 = new HalfEdge();
			hP0->pPolygon = pP; 
			hP0->ID = i * 3 + 0;
			hP0->pOrigin = m_Vertices[Faces[i][0]];
			m_Vertices[Faces[i][0]]->Outgoing.push_back(hP0);
		
			HalfEdge* hP1 = new HalfEdge();
			hP1->pPolygon = pP;
			hP1->ID = i * 3 + 1;
			hP1->pOrigin = m_Vertices[Faces[i][1]];
			m_Vertices[Faces[i][1]]->Outgoing.push_back(hP1);

			HalfEdge* hP2 = new HalfEdge();
			hP2->pPolygon = pP;
			hP2->ID = i * 3 + 2;
			hP2->pOrigin = m_Vertices[Faces[i][2]];
			m_Vertices[Faces[i][2]]->Outgoing.push_back(hP2);

			hP0->pNext = hP1;
			hP1->pNext = hP2;
			hP2->pNext = hP0;

			m_HalfEdges.push_back(hP0);
			m_HalfEdges.push_back(hP1);
			m_HalfEdges.push_back(hP2);

			pP->ID = i;
			m_Polygons.push_back(pP);
			m_Polygons[i]->pEdge = hP0;
		}

		for (uint32_t i = 0; i < m_HalfEdges.size(); i++) { // id has to be id of the next 
			for (uint32_t j = 0; j < m_HalfEdges.size(); j++) {
				bool fst = m_HalfEdges[i]->pOrigin->ID == m_HalfEdges[j]->pNext->pOrigin->ID;
				bool snd = m_HalfEdges[j]->pOrigin->ID == m_HalfEdges[i]->pNext->pOrigin->ID;
				if (fst && snd) {
					m_HalfEdges[i]->pSibling = m_HalfEdges[j];
					m_HalfEdges[j]->pSibling = m_HalfEdges[i];
				}
			}
		}
	}//buildHEDataStructure

	void HalfEdgeSetup::initHEDataStructure(T3DMesh<float>* pMesh) {
		if (nullptr == pMesh) throw NullpointerExcept("pMesh");
		if (pMesh->vertexCount() == 0) throw CForgeExcept("Mesh contains no vertex data!");

		std::vector<Vector3f> Positions;
		std::vector<Vector3i> Faces;

		for (uint32_t i = 0; i < pMesh->vertexCount(); ++i) Positions.push_back(pMesh->vertex(i));
		for (uint32_t i = 0; i < pMesh->submeshCount(); ++i) {
			T3DMesh<float>::Submesh* pSub = pMesh->getSubmesh(i);
			for (auto f : pSub->Faces) {
				Vector3i F;
				F[0] = f.Vertices[0];
				F[1] = f.Vertices[1];
				F[2] = f.Vertices[2];
				Faces.push_back(F);
			}
		}//for[submeshes]

		printf("Creating half edge data structure for model (%d vertices) ... ", uint32_t(Positions.size()));
		uint64_t Start = CForgeUtility::timestamp();
		buildHEDataStructure(Positions, Faces);
		printf(" finished in %d ms\n", uint32_t(CForgeUtility::timestamp() - Start));
	}//initHEDataStructure

	void HalfEdgeSetup::clear(void) {
	}//clear

	void HalfEdgeSetup::clearHEDataStructure(void) {
		// clean up old data
		
		for (auto& i : m_Polygons) {
			if (nullptr != i) delete i;
		}
		for (auto& i : m_HalfEdges) {
			if (nullptr != i) delete i;
		}
		for (auto& i : m_Vertices) {
			if (nullptr != i) delete i;
		}
		m_Vertices.clear();
		m_Polygons.clear();
		m_HalfEdges.clear();
	}//clearHEDataStructure

	HalfEdgeSetup::Vertex* HalfEdgeSetup::vertex(uint32_t Index) {
		if (Index >= m_Vertices.size()) throw IndexOutOfBoundsExcept("Index");
		return m_Vertices[Index];
	}//vertex

	HalfEdgeSetup::HalfEdge* HalfEdgeSetup::halfEdge(uint32_t Index) {
		if (Index >= m_HalfEdges.size()) throw IndexOutOfBoundsExcept("Index");
		return m_HalfEdges[Index];
	}//halfEdge

	HalfEdgeSetup::Polygon* HalfEdgeSetup::polygon(uint32_t Index) {
		if (Index >= m_Polygons.size()) throw IndexOutOfBoundsExcept("Index");
		return m_Polygons[Index];
	}//polygon

	uint32_t HalfEdgeSetup::vertexCount(void)const {
		return m_Vertices.size();
	}//vertexCount

	uint32_t HalfEdgeSetup::halfEdgeCount(void)const {
		return m_HalfEdges.size();
	}//halfEdgeCount

	uint32_t HalfEdgeSetup::polygonCount(void)const {
		return m_Polygons.size();
	}//polygonCount

	std::vector<HalfEdgeSetup::Vertex*> HalfEdgeSetup::getAjacentVertices(int index){
		std::vector<Vertex*> ajacentVertices;

		for(int i = 0; i < m_Vertices[index]->Outgoing.size(); i++){
			ajacentVertices.push_back(m_Vertices[index]->Outgoing[i]->pNext->pOrigin);
		}
		
		return ajacentVertices;
	}

}//name space