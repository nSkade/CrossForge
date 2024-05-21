/*****************************************************************************\
*                                                                           *
* File(s): T2DImage.hpp                                                     *
*                                                                           *
* Content: Template class that stores a 2D image.   *
*          .                                         *
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
#ifndef __CFORGE_T2DIMAGE_HPP__
#define __CFORGE_T2DIMAGE_HPP__

#include "../Core/CForgeObject.h"

namespace CForge {

	/**
	* \brief Template class that stores a 2D image and implements some very basic image manipulation techniques.
	* \ingroup AssetIO
	* 
	* \todo Add compressed formats (DDS)
	* \todo Implement conversion to grayscale image.
	* \todo Change internal data handling to container.
	* \todo Change pass by pointer to pass by reference, where applicable.
	* \todo Implement copy operator.
	* \todo Implement operator(x,y)
	*/
	template <typename T>
	class T2DImage {
	public:
		/**
		* \brief Enumerate for available color spaces.
		*/
		enum ColorSpace : int8_t {
			COLORSPACE_UNKNOWN = -1,	///< Default value.
			COLORSPACE_GRAYSCALE = 0,	///< Grayscale.
			COLORSPACE_RGB,				///< Red-Green-Blue (RGB).
			COLORSPACE_RGBA,			///< Red-green-blue-alpha (RGBA).
		};//ColorSpace

		/**
		* \brief Constructor.
		* 
		* \param[in] Width The width of the image.
		* \param[in] Height The height of the image.
		* \param[in] CS Color space.
		* \param[in] pData Optional initial data.
		*/
		T2DImage(uint32_t Width = 0, uint32_t Height = 0, ColorSpace CS = COLORSPACE_UNKNOWN, const T* pData = nullptr) {
			m_Width = 0;
			m_Height = 0;
			m_pData = nullptr;
			m_ColorSpace = COLORSPACE_UNKNOWN;
			init(Width, Height, CS, pData);
		}//Constructor

		/**
		* \brief Destructor.
		*/
		~T2DImage(void) {
			clear();
		}//Destructor

		/**
		* \brief Initialization method.
		* 
		* \param[in] Width The width of the image.
		* \param[in] Height The height of the image.
		* \param[in] CS Color space.
		* \param[in] pData Optional initial data.
		*/
		void init(uint32_t Width, uint32_t Height, ColorSpace CS, const T* pData) {
			clear();

			if (Width != 0 && Height != 0) {
				m_Width = Width;
				m_Height = Height;
				m_ColorSpace = CS;
				m_pData = new T[pixelCount()*componentsPerPixel()];
				if (nullptr == m_pData) {
					clear();
					throw OutOfMemoryExcept("m_pData");
				}
				if (nullptr != pData) memcpy(m_pData, pData, size());
			}//if[initializing]

		}//initialize

		/**
		* \brief Clear method.
		*/
		void clear(void) {
			if(nullptr != m_pData)	delete[] m_pData;
			m_pData = nullptr;
			m_Width = 0;
			m_Height = 0;
			m_ColorSpace = COLORSPACE_UNKNOWN;
		}//clear

		/**
		* \brief Returns the number of components per pixel.
		* 
		* \return Number of components per pixel.
		*/
		uint8_t componentsPerPixel(void)const {
			uint8_t Rval = 0;
			switch (m_ColorSpace) {
			case COLORSPACE_GRAYSCALE: Rval = 1; break;
			case COLORSPACE_RGB: Rval = 3; break;
			case COLORSPACE_RGBA: Rval = 4; break;
			default: {
				Rval = 0;
			}break;
			}//switch[ColorSpace]

			return Rval;
		}//componentsPerPixel

		/**
		* \brief Returns number of pixels of the image, i.e. width * height.
		* 
		* \return Number of pixels the image has.
		*/
		uint32_t pixelCount(void)const {
			return m_Width * m_Height;
		}//pixelCount

		/**
		* \brief Getter for the width.
		* 
		* \return Width of the image.
		*/
		uint32_t width(void)const {
			return m_Width;
		}//width

		/**
		* \brief Getter for the height.
		* 
		* \return Height of the image.
		*/
		uint32_t height(void)const {
			return m_Height;
		}//height

		/**
		* \brief Getter for the size of the image.
		* 
		* \return Image size in bytes.
		*/
		uint32_t size(void) {
			return pixelCount() * componentsPerPixel() * sizeof(T);
		}//size

		/**
		* \brief Returns data pointer to the pixel at index (x,y).
		* 
		* \param[in] x Index x.
		* \param[in] y Index y.
		* \return Data pointer to pixel at index (x,y).
		*/
		const T* pixel(uint32_t x, uint32_t y) const{
			return &m_pData[index(x,y)];
		}//pixel

		/**
		* \brief Returns data pointer to the pixel at index (x,y).
		*
		* \param[in] x Index x.
		* \param[in] y Index y.
		* \return Data pointer to pixel at index (x,y).
		*/
		T* pixel(uint32_t x, uint32_t y) {
			return &m_pData[index(x, y)];
		}//pixel

		/**
		* \brief Access operator to a pixel at index (x,y).
		* 
		* \param[in] x Index x.
		* \param[in] y Index y.
		* \return Data pointer to pixel at index (x,y).
		*/
		T* operator()(int x, int y) {
			return pixel(x, y);
		}

		/**
		* \brief Access operator to a pixel at index (x,y).
		*
		* \param[in] x Index x.
		* \param[in] y Index y.
		* \return Data pointer to pixel at index (x,y).
		*/
		const T* operator()(int32_t x, int32_t y) const{
			return pixel(x, y);
		}

		/**
		* \brief Getter for the image data.
		* 
		* \return Pointer to the internal pixel data.
		*/
		const T* data(void)const {
			return m_pData;
		}//data

		/**
		* \brief Getter for the image data.
		*
		* \return Pointer to the internal pixel data.
		*/
		T* data(void) {
			return m_pData;
		}//data

		/**
		* \brief Getter for the color space.
		* 
		* \return Color space.
		*/
		ColorSpace colorSpace(void)const {
			return m_ColorSpace;
		}//colorSpace

		/**
		* \brief Flips the rows of the image.
		*/
		void flipRows(void) {
			T* pNewData = new T[m_Width * m_Height * componentsPerPixel()];
			uint32_t RowSize = m_Width * componentsPerPixel();
			for (uint32_t i = 0; i < m_Height; ++i) {
				uint32_t IndexOrig = i * RowSize;
				uint32_t IndexNew = (m_Height - i - 1) * RowSize;
				memcpy(&pNewData[IndexNew], &m_pData[IndexOrig], RowSize * sizeof(T));
			}
			delete[] m_pData;
			m_pData = pNewData;
		}//flipRows

		/**
		* \brief Rotates the image by 90 degree.
		*/
		void rotate90(void) {
			T* pNewData = new T[m_Width * m_Height * componentsPerPixel()];

			for (uint32_t r = 0; r < m_Width; ++r) {
				for (uint32_t c = 0; c < m_Height; ++c) {
					for (uint8_t p = 0; p < componentsPerPixel(); ++p)
						pNewData[(c * m_Height + (m_Height - r - 1))*componentsPerPixel() + p] = m_pData[(r * m_Width + c)*componentsPerPixel() + p];
				}
			}

			delete[] m_pData;
			m_pData = pNewData;

			std::swap(m_Width, m_Height);
		}//rotate90

		/**
		* \brief Rotates the image by 180 degree.
		*/
		void rotate180(void) {
			rotate90();
			rotate90();
		}//rotate180

		/**
		* \brief Rotates the image by 270 degree.
		*/
		void rotate270(void) {
			rotate90();
			rotate90();
			rotate90();
		}//rotate270

	protected:

		/**
		* \brief Computes the internal data index from pixel coordinates.
		* 
		* \param[in] x Coordinate x.
		* \param[in] y Coordinate y.
		* \return Internal data index.
		*/
		uint32_t index(uint32_t x, uint32_t y) const {
			if (x >= m_Width || y >= m_Height) throw IndexOutOfBoundsExcept("x or y");
			return (y * m_Width + x) * componentsPerPixel();
		}//index

		uint32_t m_Width;		///< Width of the image.
		uint32_t m_Height;		///< Height of the image.
		T* m_pData;				///< Internal data storage.
		ColorSpace m_ColorSpace;///< Color space.
	};//T2DImage

}//name space

#endif