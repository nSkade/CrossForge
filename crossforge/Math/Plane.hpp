/*****************************************************************************\
*                                                                           *
* File(s): Plane.hpp                                                        *
*                                                                           *
* Content: Well, a plane in Hesse normal form.                              *
*                                                                           *
*                                                                           *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_PLANE_HPP__
#define __CFORGE_PLANE_HPP__

#include "../Core/CForgeObject.h"

namespace CForge {

	/**
	* \brief A plane in Hesse normal form. \f$ <x,n> - d = 0 \f$
	* \ingroup Math
	*/
	class Plane {
	public:
		/**
		* \brief Constructor
		*/
		Plane(void) {
			m_Normal = Eigen::Vector3f::Zero();
			m_Distance = 0.0f;
		}//Constructor

		/**
		* \brief Destructor.
		*/
		~Plane(void) {
			clear();
		}//Destructor

		/**
		* \brief Initialization method.
		* 
		* \param[in] Distance Value d, i.e. shortest distance from the origin to the plane.
		* \param[in] Normal The Normal of the plane. Will be normalized.
		*/
		void init(float Distance, Eigen::Vector3f Normal) {
			m_Normal = Normal.normalized();
			m_Distance = Distance;
		}//initialize

		/**
		* \brief Initialization method.
		* 
		* \param[in] OnPoint An arbitrary point that lies on the plane.
		* \param[in9 Normal The Normal of the plane. Will be normalized.
		*/
		void init(Eigen::Vector3f OnPoint, Eigen::Vector3f Normal) {
			m_Normal = Normal.normalized();
			m_Distance = OnPoint.dot(m_Normal);
		}//initialize

		/**
		* \brief Clear method.
		*/
		void clear(void) {
			m_Normal = Eigen::Vector3f::Zero();
			m_Distance = 0.0f;
		}//clear

		/**
		* \brief Getter for the normal.
		* 
		* \return Normal of the plane..
		*/
		const Eigen::Vector3f normal(void)const {
			return m_Normal;
		}//normal

		/**
		* \brief Getter for the distance.
		* 
		* \return Distance value.
		*/
		const float distance(void)const {
			return m_Distance;
		}//distance

		/**
		* \brief Computes the signed distance to check whether a point is if front or behind the plane.
		* 
		* \param[in] Point to check.
		* \return Signed distance. Negative values for points behind the plane, positive for those in front, and 0 for those on the plane.
		*/
		const float signedDistance(Eigen::Vector3f Point)const {
			return m_Normal.dot(Point) - m_Distance;
		}//distance

	protected:
		Eigen::Vector3f m_Normal;	///< Normal of the plane.
		float m_Distance;			///< Shortest distance to the plane.
	};//Plane

}//name space

#endif 