/*****************************************************************************\
*                                                                           *
* File(s): Box.hpp                                                          *
*                                                                           *
* Content: It's a box.                                                      *
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
#ifndef __CFORGE_BOX_HPP__
#define __CFORGE_BOX_HPP__

#include "../Core/CForgeObject.h"

namespace CForge {
	/**
	* \brief Data structure of an axis aligned box.
	* \ingroup Math
	*/
	class Box {
	public:
		/**
		* \brief Constructor
		*/
		Box(void) {
			m_Min = Eigen::Vector3f::Zero();
			m_Max = Eigen::Vector3f::Zero();
		}//Constructor

		/**
		* \brief Destructor
		*/
		~Box(void) {
			clear();
		}//Destructor

		/**
		* \brief Initialize method.
		* 
		* \param[in] Min Point with minimal values.
		* \param[in] Max Point with maximal values.
		*/
		void init(Eigen::Vector3f Min, Eigen::Vector3f Max) {
			m_Min = Min;
			m_Max = Max;
		}//initialize

		/**
		* \brief Clear method.
		*/
		void clear(void) {
			m_Min = Eigen::Vector3f::Zero();
			m_Max = Eigen::Vector3f::Zero();
		}//clear

		/**
		* \brief Getter for minimal values of the box.
		* 
		* \return Minimal values of the box.
		*/
		const Eigen::Vector3f min(void)const {
			return m_Min;
		}//min

		/**
		* \brief Getter for maximal values of the box.
		* 
		* \return Maximal values of the box.
		*/
		const Eigen::Vector3f max(void)const {
			return m_Max;
		}//max

		/**
		* \brief Setter of minimal values.
		* 
		* \param[in] Min New minimal values.
		*/
		void min(const Eigen::Vector3f Min) {
			m_Min = Min;
		}//min

		/**
		* \brief Setter of maximal values.
		* 
		* \param[in] Max New maximal values.
		*/
		void max(const Eigen::Vector3f Max) {
			m_Max = Max;
		}//max

		/**
		* \brief Computes diagonal of the box from min to max.
		* 
		* \return Diagonal vector.
		*/
		const Eigen::Vector3f diagonal(void)const {
			return (m_Max - m_Min);
		}//diagonal

		/**
		* \brief Computes center of mass of the box.
		* 
		* \return Center of mass.
		*/
		const Eigen::Vector3f center(void)const {
			return m_Min + 0.5f * diagonal();
		}//center

	protected:
		Eigen::Vector3f m_Min;	///< Minimal values.
		Eigen::Vector3f m_Max;	///< Maximal values.
	};//Box

}//name space


#endif 