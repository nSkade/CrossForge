/*****************************************************************************\
*                                                                           *
* File(s): I3DMeshIO.h                                                      *
*                                                                           *
* Content: Base interface of the 3D mesh import/export classes.             *
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
#ifndef __CFORGE_I3DMESHIO_H__
#define __CFORGE_I3DMESHIO_H__

#include "../Core/CForgeObject.h"
#include "T3DMesh.hpp"

namespace CForge {

	/**
	* \brief Interface definition for mesh import/export plug-ins
	* \ingroup AssetIO
	* 
	* \todo Change pass by pointer to pass by reference.
	*/
	class CFORGE_API I3DMeshIO: public CForgeObject {
	public:
		/**
		* \brief Available file operations.
		*/
		enum Operation: uint8_t {
			OP_LOAD = 0,	///< Loading.
			OP_STORE,		///< Storing.
		};

		/**
		* \brief Parse the file and store in the T3DMesh structure.
		* 
		* \param[in] Filepath URI to the resource.
		* \param[out] pMesh Data structure where the data will be stored to.
		*/
		virtual void load(const std::string Filepath, T3DMesh<float>* pMesh) = 0;

		/**
		* \brief Stores the specified data structure at the specified URI.
		* 
		* \param[in] Filepath URI where the data will be located.
		* \param[in] pMesh 3D mesh data that will be stored.
		*/
		virtual void store(const std::string Filepath, const T3DMesh<float>* pMesh) = 0;

		/**
		* \brief Release method.
		*/
		virtual void release(void) = 0;

		/**
		* \brief Returns whether the plug-in accepts a file for a certain operation.
		*
		* \param[in] Filepath URI to the resource.
		* \param[in] Op Operation to check.
		* \return Whether the plug-in can process the specified file.
		*/
		virtual bool accepted(const std::string Filepath, Operation Op) = 0;

		/**
		* \brief Returns the verbose name of the plug-in.
		*/
		virtual std::string pluginName(void)const;


	protected:
		/**
		* \brief Constructor
		* 
		* \param[in] ClassName Class name of the deriving class.
		*/
		I3DMeshIO(const std::string ClassName);

		/**
		* \brief Destructor.
		*/
		virtual ~I3DMeshIO(void);

		std::string m_PluginName;		///< Name of the plug-in.
	};//I3DMeshIO

}//name space


#endif