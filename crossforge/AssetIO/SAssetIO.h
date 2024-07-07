/*****************************************************************************\
*                                                                           *
* File(s): SAssetIO.h and SAssetIO.cpp                                      *
*                                                                           *
* Content: Singleton class that handles import and export of resources.     *
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
#ifndef __CFORGE_SASSETIO_H__
#define __CFORGE_SASSETIO_H__

#include "I3DMeshIO.h"
#include "I2DImageIO.h"

namespace CForge {

	/**
	* \brief This singleton class handles import and export of resources such as images and 3d models.
	*
	* \todo Design and implement mesh file format for crossforge
	* \todo Implement TImagePyramid (handles image pyramid with different resolutions)
	* \todo implement DDS import/export
	* \todo Add json support.
	* \todo Write documentation concept page on import and export of data.
	* \todo Create test scene to check whether import/export of 3D objects work correctly.
	* \todo Extend load from memory method so other types of files than jpeg can be used.
	* \todo Write methods so store to memory are supported.
	* \todo Add methods that use container (std::vector<uint8_t> instead of pointer.
	* \todo Change internal handling of data to smart pointers.
	* \todo Change pass by pointer to pass by reference, where applicable.
	*/
	class CFORGE_API SAssetIO: public CForgeObject {
	public:
		/**
		* \brief Singleton's instantiation method.
		* 
		* \return Instance of the class.
		*/
		static SAssetIO* instance(void);

		/**
		* \brief Singleton's release method.
		*/
		void release(void);

		/**
		* \brief Getter for number of active instances.
		* 
		* \return Number of active instances.
		*/
		static int32_t instaceCount();

		/**
		* \brief Stores the specified data structure at the specified URI.
		*
		* \param[in] Filepath URI where the data will be located.
		* \param[in] pMesh 3D mesh data that will be stored.
		*/
		static void store(const std::string Filepath, const T3DMesh<float>* pMesh);

		/**
		* \brief Parse the file and store in the T3DMesh structure.
		*
		* \param[in] Filepath URI to the resource.
		* \param[out] pMesh Data structure where the data will be stored to.
		*/
		static void load(const std::string Filepath, T3DMesh<float>* pMesh);

		/**
		* \brief Stores image data to a file resource.
		*
		* \param[in] Filepath Path to the file including filename and extension.
		* \param[in] pImgData Image data.
		*/
		static void store(const std::string Filepath, const T2DImage<uint8_t>* pImage);

		/**
		* \brief Load an image resource into the image data structure.
		*
		* \param[in] Filepath Path to the image file.
		* \param[out] pImgData Data structure where the image data will be written to.
		*/
		static void load(const std::string Filepath, T2DImage<uint8_t>* pImage);

		/**
		* \brief Convert image data from memory to the image data structure.
		*
		* \param[in] pBuffer Image data as array.
		* \param[in] BufferLength Size in bytes of the buffer.
		* \param[out] pImgData Data structure where the image data will be written to.
		* \warning Currently assumes that data stream is of type jpeg.
		*/
		static void load(const uint8_t* pBuffer, uint32_t BufferLength, T2DImage<uint8_t>* pImage);

		/**
		* \brief Reads a text file and returns it's contents.
		* 
		* \param[in] Filepath Path to the resource file.
		* \return Content of the file.
		*/
		static std::string readTextFile(const std::string Filepath);

		//TODO(skade) brief + move in cpp
		static bool accepted(const std::string Filepath, I3DMeshIO::Operation op) {
			for (auto p : instance()->m_ModelIOPlugins)
				if (p.pInstance->accepted(Filepath,op))
					return true;
			return false;
		}
		static bool accepted(const std::string Filepath, I2DImageIO::Operation op) {
			for (auto p : instance()->m_ImageIOPlugins)
				if (p.pInstance->accepted(Filepath,op))
					return true;
			return false;
		}

	protected:
		/**
		* \brief Constructor
		*/
		SAssetIO(void);

		/**
		* \brief Destructor
		*/
		~SAssetIO(void);

		/**
		* \brief Initialization method.
		*/
		void init(void);

		/**
		* \brief Clear method.
		*/
		void clear(void);

		/**
		* \brief Stores the specified data structure at the specified URI.
		*
		* \param[in] Filepath URI where the data will be located.
		* \param[in] pMesh 3D mesh data that will be stored.
		*/
		void storeModel(const std::string Filepath, const T3DMesh<float>* pMesh);

		/**
		* \brief Parse the file and store in the T3DMesh structure.
		*
		* \param[in] Filepath URI to the resource.
		* \param[out] pMesh Data structure where the data will be stored to.
		*/
		void loadModel(const std::string Filepath, T3DMesh<float>* pMesh);
		
		/**
		* \brief Stores image data to a file resource.
		*
		* \param[in] Filepath Path to the file including filename and extension.
		* \param[in] pImgData Image data.
		*/
		void storeImage(const std::string Filepath, const T2DImage<uint8_t>* pImage);

		/**
		* \brief Load an image resource into the image data structure.
		*
		* \param[in] Filepath Path to the image file.
		* \param[out] pImgData Data structure where the image data will be written to.
		*/
		void loadImage(const std::string Filepath, T2DImage<uint8_t>* pImage);

		/**
		* \brief Convert image data from memory to the image data structure.
		*
		* \param[in] pBuffer Image data as array.
		* \param[in] BufferLength Size in bytes of the buffer.
		* \param[out] pImgData Data structure where the image data will be written to.
		* \warning Currently assumes data stream is of type jpeg.
		*/
		void loadImage(const uint8_t* pBuffer, uint32_t BufferLength, T2DImage<uint8_t>* pImage);

	private:
		/**
		* \brief Structure that holds a model import/export plug-in instance.
		*/
		struct ModelIOPlugin {
			I3DMeshIO* pInstance;	///< Instance of the plug-in.
			std::string Name;		///< Name of the plug-in.

			/**
			* \brief Constructor
			*/
			ModelIOPlugin(void) {
				pInstance = nullptr;
				Name = "";
			}

			/**
			* \brief Destructor
			*/
			~ModelIOPlugin() {
				//if (nullptr != pInstance) pInstance->release();
				pInstance = nullptr;
				Name = "";
			}
		};

		/**
		* \brief Structure that holds an image import/export plug-in instance.
		*/
		struct ImageIOPlugin {
			I2DImageIO* pInstance;	///< instance of the plug-in.
			std::string Name;		///< Name of the plug-in.

			/**
			* \brief Constructor.
			*/
			ImageIOPlugin(void) {
				pInstance = nullptr;
				Name = "";
			}

			/**
			* \brief Destructor.
			*/
			~ImageIOPlugin() {
				//if (nullptr != pInstance) pInstance->release();
				pInstance = nullptr;
				Name = "";
			}
		};

		std::vector<ModelIOPlugin> m_ModelIOPlugins;	///< Available mesh import/export plug-ins.
		std::vector<ImageIOPlugin> m_ImageIOPlugins;	///< Available image import/export plug-ins.

		static SAssetIO* m_pInstance;		///< Singleton's unique instance.
		static uint32_t m_InstanceCount;	///< Singleton's instance count.
	};//SAssetIO

	typedef SAssetIO AssetIO;

}//name space


#endif