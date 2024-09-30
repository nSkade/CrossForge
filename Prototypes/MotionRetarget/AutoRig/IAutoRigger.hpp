#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>

namespace CForge {

class IAutoRigger {
public:
	virtual void rig(T3DMesh<float>* mesh) = 0;
};

}//CForge
