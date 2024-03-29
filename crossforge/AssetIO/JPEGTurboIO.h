/*****************************************************************************\
*                                                                           *
* File(s): JPEGTurboIO.h and JPEGTurboIO.cpp                                *
*                                                                           *
* Content: Import/Export class for jpeg images using libjpeg-turbo          *
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
#ifndef __CFORGE_JPEGTURBOIO_H__
#define __CFORGE_JPEGTURBOIO_H__

#include <crossforge/AssetIO/I2DImageIO.h>

namespace CForge {

	/**
	* \brief Import/Export plug-in for jpeg images using libjpeg-turbo library.
	* \ingroup AssetIO
	*/
	class JPEGTurboIO : public I2DImageIO {
	public:
		
		/**
		* \brief Constructor
		*/
		JPEGTurboIO();

		/**
		* \brief Destructor
		*/
		~JPEGTurboIO();

		/**
		* \brief Initialization method.
		*/
		void init();

		/**
		* \brief Clear method.
		*/
		void clear();

		/**
		* \brief Load an image resource into the image data structure.
		*
		* \param[in] Filepath Path to the image file.
		* \param[out] pImgData Data structure where the image data will be written to.
		*/
		void load(const std::string Filepath, T2DImage<uint8_t>* pImgData) override;

		/**
		* \brief Convert image data from memory to the image data structure.
		*
		* \param[in] pBuffer Image data as array.
		* \param[in] BufferLength Size in bytes of the buffer.
		* \param[out] pImgData Data structure where the image data will be written to.
		*/
		void load(const uint8_t* pBuffer, const uint32_t BufferLength, T2DImage<uint8_t>* pImgData) override;

		/**
		* \brief Stores image data to a file resource.
		*
		* \param[in] Filepath Path to the file including filename and extension.
		* \param[in] pImgData Image data.
		*/
		void store(const std::string Filepath, const T2DImage<uint8_t>* pImgData) override;

		/**
		* \brief Checks whether a plugin accepts a certain operation for a specific file type.
		*
		* \param[in] Filepath Path to the image resource.
		* \param[in] Op Operation to check
		* \return True if operation is supported, false otherwise.
		*/
		bool accepted(const std::string Filepath, Operation Op) override;

		/**
		* \brief Plug-in's release method.
		*/
		void release(void) override;

	protected:
		/**
		* \brief Retrieves jpeg's pixel format from color space enum.
		* 
		* \param[in] CS Color space to find pixel format for.
		* \return Libjpeg's respective pixel format.
		*/
		int32_t pixelFormat(const T2DImage<uint8_t>::ColorSpace CS);

		/**
		* \brief Retries the color space enum from libjpeg's pixel format.
		* 
		* \param[in] CS Libjpeg's color space.
		* \return Color space enum value.
		*/
		T2DImage<uint8_t>::ColorSpace pixelFormat(int32_t CS);
	};//JPEGTurboIO

}//name-space

#endif 