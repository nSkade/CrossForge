/*****************************************************************************\
*                                                                           *
* File(s): I2DImageIO.h and I2DImageIO.cpp                                  *
*                                                                           *
* Content: Interface definition for image import/export plug-ins.            *
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
#ifndef __CFORGE_I2DIMAGEIO_H__
#define __CFORGE_I2DIMAGEIO_H__

#include "../Core/CForgeObject.h"
#include "T2DImage.hpp"


namespace CForge {

	/**
	* \brief Interface definition for image import/export plug-ins 
	* \ingroup AssetIO
	* 
	* \todo Change pass by pointer to pass by reference, where applicable.
	*/
	class CFORGE_API I2DImageIO: public CForgeObject {
	public:
		/**
		* \brief Allowed operations.
		*/
		enum Operation : uint8_t {
			OP_LOAD = 0,	///< Loading.
			OP_STORE,		///< Storing.
		};

		/**
		* \brief Load an image resource into the image data structure.
		* 
		* \param[in] Filepath Path to the image file.
		* \param[out] pImgData Data structure where the image data will be written to.
		*/
		virtual void load(const std::string Filepath, T2DImage<uint8_t>* pImgData) = 0;

		/**
		* \brief Convert image data from memory to the image data structure.
		* 
		* \param[in] pBuffer Image data as array.
		* \param[in] BufferLength Size in bytes of the buffer.
		* \param[out] pImgData Data structure where the image data will be written to.
		*/
		virtual void load(const uint8_t* pBuffer, const uint32_t BufferLength, T2DImage<uint8_t>* pImgData) = 0;

		/**
		* \brief Stores image data to a file resource.
		* 
		* \param[in] Filepath Path to the file including filename and extension.
		* \param[in] pImgData Image data.
		*/
		virtual void store(const std::string Filepath, const T2DImage<uint8_t>* pImgData) = 0;

		/**
		* \brief Checks whether a plugin accepts a certain operation for a specific file type.
		* 
		* \param[in] Filepath Path to the image resource.
		* \param[in] Op Operation to check
		* \return True if operation is supported, false otherwise.
		*/
		virtual bool accepted(const std::string Filepath, Operation Op) = 0;

		/**
		* \brief Plug-in's release method.
		*/
		virtual void release(void) = 0;

		/**
		* \brief Getter for the plug-in's name.
		* 
		* \return Name of the plug-in.
		*/
		virtual std::string pluginName(void)const;

	protected:
		/**
		* \brief Constructor
		* 
		* \param[in] ClassName Name of the deriving class.
		*/
		I2DImageIO(const std::string ClassName);	

		/**
		* \brief Destructor.
		*/
		virtual ~I2DImageIO(void);

		std::string m_PluginName;	///< Name of the plug-in.
	private:

	};//I2DImageIO

}//name space

#endif