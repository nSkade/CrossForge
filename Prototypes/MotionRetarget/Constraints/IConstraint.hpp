
#include <Prototypes/MotionRetarget/IKController.h>

namespace CForge {

//TODO(skade) different IK methods probably need different Constraint Implementations for the same constraint Type.
//            Abstract interface, and automatically chose correct constraint implementation depending on solver.

/**
 * @brief Interface for Joint Constraints, applied during IK.
*/
class IConstraint {
	virtual void apply(IKController::IKSegment* seg, SkeletalAnimationController::SkeletalJoint* cJoint);
};//IConstraint

}//CForge
