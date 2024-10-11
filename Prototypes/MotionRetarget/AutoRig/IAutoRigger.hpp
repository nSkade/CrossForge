#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>

namespace CForge {

template<typename AROptions>
class IAutoRigger {
public:
	virtual void rig(T3DMesh<float>* mesh, AROptions options) = 0;
};

}//CForge
