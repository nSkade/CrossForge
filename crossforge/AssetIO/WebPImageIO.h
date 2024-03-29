/*****************************************************************************\
*                                                                           *
* File(s): WebPImageIO.h and WebPImageIO.cpp                                *
*                                                                           *
* Content: Import/Export class using libwebp.                               *
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
#ifndef __CFORGE_WEBPIMAGEIO_H__
#define __CFORGE_WEBPIMAGEIO_H__

#include "I2DImageIO.h"

namespace CForge {
	/**
	* \brief Import/Export plug-in for images using the libwebp library.
	* \ingroup AssetIO
	*/
	class WebPImageIO: public I2DImageIO {
	public:
		/**
		* \brief Constructor
		*/
		WebPImageIO(void);

		/**
		* \brief Destructor
		*/
		~WebPImageIO(void);

		/**
		* \brief Initialization method.
		*/
		void init(void);

		/**
		* \brief Clear method.
		*/
		void clear(void);

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

	};//WebPImageIO

}//name space


#endif 