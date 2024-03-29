/*****************************************************************************\
*                                                                           *
* File(s): Rectangle.hpp                                                    *
*                                                                           *
* Content: A 2D rectangle                                                   *
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
#ifndef __CFORGE_RECTANGLE_HPP__
#define __CFORGE_RECTANGLE_HPP__

#include "../Core/CForgeObject.h"

namespace CForge {

	/**
	* \brief A two dimensional rectangle
	* \ingroup Math
	*/
	class Rectangle : public CForgeObject {
	public:
		/**
		* \brief Constructor.
		*/
		Rectangle(void): CForgeObject("Rectangle") {

		}//Constructor

		/**
		* \brief Destructor.
		*/
		~Rectangle(void) {
			clear();
		}//Destructor

		/**
		* \brief Initialization method.
		* 
		* \param[in] PosX X position.
		* \param[in] PosY Y position.
		* \param[in9 Width The Width.
		* \param[in] Height The height.
		*/
		void init(float PosX, float PosY, float Width, float Height) {
			position(Eigen::Vector2f(PosX, PosY));
			size(Eigen::Vector2f(Width, Height));
		}//initialize

		/**
		* \brief Initialization method.
		* 
		* \param[in] Position The position.
		* \param[in] Size Width and height.
		*/
		void init(const Eigen::Vector2f Position, const Eigen::Vector2f Size) {
			position(Position);
			size(Size);
		}//initialize

		/**
		* \brief Clear method.
		*/
		void clear() {
			m_Position = Eigen::Vector2f(0.0f, 0.0f);
			m_Size = Eigen::Vector2f(0.0f, 0.0f);
		}

		/**
		* \brief Setter for the position.
		* 
		* \param[in] Position The new position.
		*/
		void position(const Eigen::Vector2f Position) {
			m_Position = Position;
		}//position

		/**
		* \brief Setter for the size.
		* 
		* \param[in] Size The new size.
		*/
		void size(const Eigen::Vector2f Size) {
			m_Size = Size;
		}//size

		/**
		* \brief Getter for the position
		* 
		* \return Position.
		*/
		const Eigen::Vector2f position(void)const {
			return m_Position;
		}//position


		/**
		* \brief Getter for the size.
		* 
		* \return Size.
		*/
		const Eigen::Vector2f size(void)const {
			return m_Size;
		}//size

		/**
		* \brief Getter for the width.
		* 
		* \return Width.
		*/
		const float width(void)const {
			return m_Size.x();
		}//width

		/**
		* \brief Getter for the height.
		* 
		* \return Height.
		*/
		const float height(void)const {
			return m_Size.y();
		}//height

		/**
		* \brief Computes whether a point is inside the rectangle.
		* 
		* \param[in] Point The point to check.
		* \return True if point is inside or on the boundary, false otherwise.
		*/
		bool isPointInside(const Eigen::Vector2f Point) const {
			return (
				(Point.x() >= m_Position.x()) &&
				(Point.y() >= m_Position.y()) &&
				(Point.x() <= m_Position.x() + m_Size.x()) &&
				(Point.y() <= m_Position.y() + m_Size.y())
				);

		}//isPointInside

	protected:
		Eigen::Vector2f m_Position;	///< The position.
		Eigen::Vector2f m_Size;		///< Width and height.
	};//Rectangle

}//name space

#endif