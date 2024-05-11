/*****************************************************************************\
*                                                                           *
* File(s): HalfEdgeSetup.h and HalfEdgeSetup.cpp                            *
*                                                                           *
* Content:                                                                  *
*                                                  						 	*
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* FreeBSD License without any warranty or guaranty to work properly.        *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>
#include <crossforge/Utility/CForgeUtility.h>
#include <crossforge/Math/CForgeMath.h>

namespace CForge {
	/**
	* \brief Base implementation of the half edge data structure exercise.
	* \ingroup HalfEdge
	*/
	class HalfEdgeSetup {
	public:
		struct Vertex;
		struct Polygon;

		/**
		* \brief Definition for the HalfEdge data structure
		*/
		struct HalfEdge {
			int32_t ID;			///< Internal index
			HalfEdge* pNext;	///< Next half edge in the polygon
			HalfEdge* pSibling;	///< Partner half edge

			Vertex* pOrigin;	///< Vertex where the half edge originates
			Polygon* pPolygon;	///< Polygon the half edge belongs to

			/**
			* \brief Constructor
			*/
			HalfEdge(void) {
				pNext = nullptr;
				pSibling = nullptr;
				pOrigin = nullptr;
				pPolygon = nullptr;
			}//Constructor

			/**
			* \brief Destructor
			*/
			~HalfEdge(void) {
				if (nullptr != pSibling) pSibling->pSibling = nullptr;
				if (nullptr != pOrigin) pOrigin->remove(this);
			}//Destructor

			/**
			* \brief Identifies previous half edge of the polygon and returns it.
			* 
			* \return Previous half edge.
			*/
			HalfEdge* previous(void) {
				HalfEdge* pRval = nullptr;
				if (nullptr != pNext) pRval = pNext->pNext;
				return pRval;
			}//previous

			/**
			* \brief Returns next half edge of the polygon.
			* 
			* \return Next half edge.
			*/
			HalfEdge* next(void) {
				return pNext;
			}//next

			/**
			* \brief Returns partner half edge.
			* 
			* \return Partner half edge.
			*/
			HalfEdge* sibling(void) {
				return pSibling;
			}//sibling

			/**
			* \brief Determines whether the edge is a border edge.
			* 
			* \return True if border edge, false otherwise.
			*/
			bool borderEdge(void)const {
				return (pSibling == nullptr);
			}//borderEdge

		};//HEEdge

		/**
		* \brief Definition for the vertex type.
		*/
		struct Vertex {
			int32_t ID;				///< Internal index.
			std::vector<HalfEdge*> Outgoing; ///< All outgoing half edges.
			Eigen::Vector3f Position; ///< 3D position.
			
			/**
			* \brief Constructor
			*/
			Vertex(void) {
				ID = -1;
				Position = Eigen::Vector3f::Zero();
				Outgoing.clear();
			}//Constructor

			/**
			* \brief Inserts a new outgoing half edge. Revokes duplicates.
			* 
			* If the specified half edge is already known to the vertex, it will not be inserted.
			* \param[in] pHE Half edge to insert. 
			*/
			void insert(HalfEdge* pHE) {
				// do we already have thin one?
				for (auto i : Outgoing) {
					if (i == pHE) return;
				}

				int32_t Index = -1;
				for (uint32_t i = 0; i < Outgoing.size(); ++i) {
					if (nullptr == Outgoing[i]) {
						Index = i;
						break;
					}
				}//for[outgoing edges]

				if (Index == -1) Outgoing.push_back(pHE);
				else Outgoing[Index] = pHE;

			}//insert

			/**
			* \brief Removes a half edge from the outgoing half edges.
			* 
			* \param[in] pHE Half edge to remove.
			*/
			void remove(HalfEdge* pHE) {
				for (auto &i : Outgoing) {
					if (i == pHE) i = nullptr;
				}//for[outgoing edges]
			}//remove

			/**
			* \brief Returns whether the vertex is isolated (not belonging to any half edge).
			* 
			* \return True if isolated, false otherwise.
			*/
			bool isolated(void) {
				bool Rval = true;
				for (auto i : Outgoing) {
					if (nullptr != i) Rval = false;
				}
				return Rval;
			}//isolated
			
		};//HEVertex

		/**
		* \brief Structure definition for the polygon.
		*/
		struct Polygon {
			int32_t ID;		///< Index
			HalfEdge* pEdge;///< One half edge of the polgyon.

			/**
			* \brief Constructor
			*/
			Polygon(void) {
				ID = -1;
				pEdge = nullptr;
			}

			/**
			* \brief Determines whether this is a border polygon.
			* 
			* \return True if border polygon, false otherwise.
			*/
			bool borderPolygon(void)const {
				bool Rval = false;
				if(pEdge->pSibling == nullptr) Rval = true;
				if (pEdge->next()->pSibling == nullptr) Rval = true;
				if (pEdge->previous()->pSibling == nullptr) Rval = true;
				return Rval;
			}//borderPolygon

			/**
			* \brief Computes whether the triangle is obtuse (one corner with angle > 90�)
			* return True if obtuse, false otherwise.
			*/
			bool obtuse(void)const {
				Eigen::Vector3f P[3];
				P[0] = pEdge->pOrigin->Position;
				P[1] = pEdge->next()->pOrigin->Position;
				P[2] = pEdge->previous()->pOrigin->Position;

				Eigen::Vector3f E1 = (P[1] - P[0]).normalized();
				Eigen::Vector3f E2 = (P[2] - P[0]).normalized();
				if (E1.dot(E2) <= 0.0f) return true;

				E1 = (P[2] - P[1]).normalized();
				E2 = (P[0] - P[1]).normalized();
				if (E1.dot(E2) <= 0.0f) return true;

				E1 = (P[0] - P[2]).normalized();
				E2 = (P[1] - P[2]).normalized();
				if (E1.dot(E2) <= 0.0f) return true;

				return false;
			}//obtuse

		};//HEPolygon

		/**
		* \brief Constructor
		*/
		HalfEdgeSetup(void);

		/**
		* \brief Destructor
		*/
		~HalfEdgeSetup(void);

        /**
		* \brief Initializes the half edge data structure.
		* 
		* \param[in] Positions Position data.
		* \param[in] Faces Polygon data.
		*/
		void buildHEDataStructure(std::vector<Eigen::Vector3f> Positions, std::vector<Eigen::Vector3i> Faces);

		/**
		* \brief Clear method.
		*/
		void clear(void);

		/**
		* \brief Initializes the half edge data structure from a mesh.
		* \param[in] pMesh Triangle mesh. Has to contain valid data.
		*/
		void initHEDataStructure(CForge::T3DMesh<float>* pMesh);

		/**
		* \brief Clears the half edge data structure by deleting all data.
		*/
		void clearHEDataStructure(void);

		/**
		* \brief Vertex getter.
		* \param[in] Index Index \f$ \in \f$ [0, vertexCount-1]
		* \return Vertex object.
		*/
		Vertex* vertex(uint32_t Index);

		/**
		* \brief Half edge getter.
		* \param[in] Index Index \f$ \in \f$ [0, halfEdgeCount-1]
		* \return Half edge object.
		*/
		HalfEdge *halfEdge(uint32_t Index);

		/**
		* \brief Polygon getter.
		* \param[in] Index \f$ \in \f$ [0, polygonCount-1]
		* \return Polygon object.
		*/
		Polygon* polygon(uint32_t Index);

		/**
		* \brief Returns number of available vertexes.
		* \return Number of available vertexes.
		*/
		uint32_t vertexCount(void)const;

		/**
		* \brief Returns number of available half edges.
		* \return Number of available half edges.
		*/
		uint32_t halfEdgeCount(void)const;

		/**
		* \brief Returns number of available polygons.
		* \return Number of available polygons.
		*/
		uint32_t polygonCount(void)const;

		/**
		* \brief Returns all ajacent Verticies of a given Vertex
		* \param[in] pVertex Vertex to get the ajacent Verticies from
		* \return Vector of ajacent Verticies 
		*/
		std::vector<Vertex*> getAjacentVertices(int index);


	protected:

		std::vector<HalfEdge*> m_HalfEdges;	///< Vector that holds the half edge objects.
		std::vector<Vertex*> m_Vertices;	///< Vector that holds the vertex objects.
		std::vector<Polygon*> m_Polygons;	///< Vector that holds the polygon objects.

		int32_t m_LoadedModel;		///< Defines which model to load. Can be cycled through using the keyboard.
		
	};//HalfEdgeSetup

}//name space
