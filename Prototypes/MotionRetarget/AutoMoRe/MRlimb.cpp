#include "MRlimb.hpp"

#include "Prototypes/MotionRetarget/CMN/MRMutil.hpp"

#include <iostream>

namespace CForge {
using namespace Eigen;

void MRlimb::initialize(std::shared_ptr<CharEntity> source, std::shared_ptr<CharEntity> target, std::vector<int> corr) {
	reset();

	m_sCE = source;
	m_tCE = target;
	m_ikcorr = corr;
	m_active = true;

	// assign limb scaling values
	auto& sCtrl = source->controller;
	auto& tCtrl = target->controller;

	for (int it = 0; it < m_ikcorr.size();++it) {
		int is = m_ikcorr[it];
		if (is==-1)
			continue;
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];
		
		m_scale_limbs.push_back(1.);
	}

	for (int it = 0; it < tCtrl->m_ikArmature.m_jointChains.size(); ++it) {
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];
		float tarLen = 0.;
		for (int i = 0; i < ct.joints.size(); ++i)
			tarLen += ct.joints[i]->LocalPosition.norm();
		m_tar_limbLen.push_back(tarLen);
	}
	for (int is = 0; is < sCtrl->m_ikArmature.m_jointChains.size(); ++is) {
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		float srcLen = 0.;
		for (int i = 0; i < cs.joints.size(); ++i)
			srcLen += cs.joints[i]->LocalPosition.norm();
		m_src_limbLen.push_back(srcLen);
	}

	m_src_rootPos = sCtrl->getRoot()->LocalPosition;
	m_tar_rootPos = tCtrl->getRoot()->LocalPosition;

	// reinitialize targets
	source->autoCreateTargets();
	target->autoCreateTargets();

	// create copyies of source iktargets for target char
	m_targets.clear();
	for (int it = 0; it < m_ikcorr.size();++it) {
		int is = m_ikcorr[it];
		if (is==-1)
			continue;
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];

		//TODOff(skade) auto create targets if they dont exist
		std::shared_ptr<IKTarget> t =
			std::make_shared<IKTarget>(*sCtrl->m_ikArmature.m_jointChains[is].target.lock().get());

		Vector3f pos; Quaternionf rot; Vector3f scale;
		target->sgn.buildTansformation(&pos,&rot,&scale);
		t->m_sgnT = CForgeMath::translationMatrix(pos) * CForgeMath::rotationMatrix(rot) * CForgeMath::scaleMatrix(scale);

		m_targets.emplace_back(t);
		ct.target = t;
	}
};
std::vector<int> MRlimb::jointIndexingFunc(int tarIdx, IKChain& cs, IKChain& ct) {
	auto source = m_sCE.lock();
	auto target = m_tCE.lock();
	if (!source || !target) {
		m_active = false;
		return std::vector<int>();
	}
	auto& sCtrl = source->controller;
	auto& tCtrl = target->controller;
#if 0 // by distance
	float closestDist = std::numeric_limits<float>::max();
	int closestIdx = -1;
	SkeletalAnimationController::SkeletalJoint* jt = ct.joints[tarIdx];
	for (int i=0;i<cs.joints.size();++i) {
		auto* js = cs.joints[i];
		float dist = (tCtrl->m_IKJoints[jt].posGlobal-sCtrl->m_IKJoints[js].posGlobal).norm(); //TODO(skade) from current pose, need rest pose global pos instead
		if (dist < closestDist) {
			closestDist = dist;
			closestIdx = i;
		}
	}
	return closestIdx;
#endif
#if 0 // by distribution
	return std::floor((float(tarIdx)/ct.joints.size())*cs.joints.size());
#endif
#if 0 // by index
	if (tarIdx < cs.joints.size())
		return {tarIdx};
#endif
#if 1 // smart mapping by assigning priority to close nodes in the relative length chains
	//TODOff(skade) cache these results in init func
	if (cs.joints.size() == ct.joints.size())
		return {tarIdx}; // identical matching possible
	else {
		std::vector<float> srcCumulative, tarCumulative;
		float totalLenS = 0.f, totalLenT = 0.f;

		for (int i = 0; i < cs.joints.size(); ++i) {
			totalLenS += cs.joints[i]->LocalPosition.norm();
			srcCumulative.push_back(totalLenS);
		}

		for (int i = 0; i < ct.joints.size(); ++i) {
			totalLenT += ct.joints[i]->LocalPosition.norm();
			tarCumulative.push_back(totalLenT);
		}

		// Normalize cumulative lengths
		std::vector<float> srcNorm, tarNorm;
		for (float len : srcCumulative) {
			srcNorm.push_back((totalLenS > 0) ? (len / totalLenS) : 0.f);
		}
		for (float len : tarCumulative) {
			tarNorm.push_back((totalLenT > 0) ? (len / totalLenT) : 0.f);
		}

		const int numTarNodes = tarNorm.size();
		const int numSrcNodes = srcNorm.size();
		std::vector<std::vector<int>> finalMap(numTarNodes);
		using NodeDistPair = std::pair<int, float>;
		///**
		// * @brief Compares two NodeDistPair objects based on their distance.
		// * * This is used for sorting.
		// */
		auto compareDistPairs = [](const NodeDistPair& a, const NodeDistPair& b) -> bool {
			return a.second < b.second;
		};
		// Step 1: Calculate and sort distances for each target node.
		// idxDist[i] will store a sorted list of pairs {source_index, distance}
		// for the target node i.
		std::vector<std::vector<NodeDistPair>> idxDist(numTarNodes);
		for (int i = 0; i < numTarNodes; ++i) {
			for (int j = 0; j < numSrcNodes; ++j) {
				float distance = std::abs(tarNorm[i] - srcNorm[j]);
				idxDist[i].push_back({j, distance});
			}
			// Sort each inner vector to easily find the closest source nodes.
			std::sort(idxDist[i].begin(), idxDist[i].end(), compareDistPairs);
		}

		// Step 2: Perform a greedy one-to-one mapping to handle the most confident
		// assignments first. This prevents a source node from being used for multiple
		// target nodes in this initial pass.
		std::vector<int> nodeMapping(numTarNodes, -1);
		std::vector<bool> usedSrcNodes(numSrcNodes, false);

		// Create a flattened list of all possible mappings sorted by distance.
		std::vector<std::pair<float, std::pair<int, int>>> sortedPairs;
		for (int i = 0; i < numTarNodes; ++i) {
			for (const auto& pair : idxDist[i]) {
				sortedPairs.push_back({pair.second, {i, pair.first}});
			}
		}
		std::sort(sortedPairs.begin(), sortedPairs.end());

		for (const auto& pair : sortedPairs) {
			int tarIdx = pair.second.first;
			int srcIdx = pair.second.second;

			if (nodeMapping[tarIdx] == -1 && !usedSrcNodes[srcIdx]) {
				nodeMapping[tarIdx] = srcIdx;
				usedSrcNodes[srcIdx] = true;
			}
		}

		// Step 3: Final mapping, including handling the surjective case (many-to-one).
		// Initialize the final map with the one-to-one assignments from the greedy step.
		//std::vector<std::vector<int>> finalMap(numTarNodes);
		for (int i = 0; i < numTarNodes; ++i) {
			if (nodeMapping[i] != -1) {
				finalMap[i].push_back(nodeMapping[i]);
			}
		}

		// Assign any unmapped source nodes to their closest mapped target node.
		for (int j = 0; j < numSrcNodes; ++j) {
			if (!usedSrcNodes[j]) {
				float minDistance = std::numeric_limits<float>::max();
				int bestTarIdx = -1;

				for (int i = 0; i < numTarNodes; ++i) {
					// Only consider already mapped target nodes.
					if (nodeMapping[i] != -1) {
						float distance = std::abs(tarNorm[i] - srcNorm[j]);
						if (distance < minDistance) {
							minDistance = distance;
							bestTarIdx = i;
						}
					}
				}
				// If a suitable target was found, add the unmapped source node to it.
				if (bestTarIdx != -1) {
					finalMap[bestTarIdx].push_back(j);
				}
			}
		} // reult in finalMap

		return finalMap[tarIdx];
	}

#endif
	return std::vector<int>();
}
void MRlimb::update() {
	auto source = m_sCE.lock();
	auto target = m_tCE.lock();
	if (!source || !target)
		m_active = false;
	if (!m_active) {
		m_targets.clear();
		return;
	}

	auto& sCtrl = source->controller;
	auto& tCtrl = target->controller;

	for (int it = 0; it < m_ikcorr.size();++it) {
		int is = m_ikcorr[it];
		if (is==-1)
			continue;
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];
		//ct.target = cs.target; // old way, assign other char entity target 
		
		// update target position
		{ // rescale limb target positions
			float scale = m_tar_limbLen[it]/m_src_limbLen[is];
			scale = CForgeMath::lerp(1.f,scale,m_scale_limbs[it]);

			//TODO(skade) srp needs offset of parent chain transform
			// root pos of chain
			Vector3f srp = sCtrl->m_IKJoints[cs.joints.back()].posGlobal;
			Vector3f sdir = cs.target.lock()->pos - srp;

			//TODO(skade) append limb dir to last frame not ideal
			ct.target.lock()->pos = tCtrl->m_IKJoints[ct.joints.back()].posGlobal + sdir*scale;
		}

//TODO(skade) look for reusable code
////		// get parent rot of root joint for reference
////		auto csRoot = cs.joints.back();
////		Quaternionf rootGlobRot = Quaternionf::Identity();
////		rootGlobRot = sCtrl->m_IKJoints[csRoot].rotGlobal;
////		//TODO(skade) parent of root?
////		//if (csRoot->Parent != -1)
////		//	rootGlobRot = sCtrl->m_IKJoints[sCtrl->getBone(csRoot->Parent)].rotGlobal;
////		Quaternionf parRot = rootGlobRot;
////		
////		// imitate joint angles, start from root of chain
//		for (int i=ct.joints.size()-1; i >= 0; --i) {
//			// joint, apply angle to
//			SkeletalAnimationController::SkeletalJoint* jt = ct.joints[i];
//
//			// find closest source joint to imitate angle from
//			int matchIdx = jointIndexingFunc(i,cs,ct);
//			if (matchIdx != -1) {
//				 // source joint
//				SkeletalAnimationController::SkeletalJoint* js = cs.joints[matchIdx];
//#if 1
//				jt->LocalRotation = js->LocalRotation;
//				jt->OffsetMatrix = js->OffsetMatrix;
////				// relative source rot to root
////				Quaternionf locRot = sCtrl->m_IKJoints[js].rotGlobal;
////				if (js->Parent != -1)
////					locRot = locRot * sCtrl->m_IKJoints[sCtrl->getBone(js->Parent)].rotGlobal.inverse();
////				locRot.normalize();
////				//parRot = locRot * parRot;
////				//parRot.normalize();
////				
////				//jt->LocalRotation = r * parRot.inverse();
////				jt->LocalRotation = locRot * jt->OffsetMatrix.block<3,3>(0,0).inverse();
////				jt->LocalRotation.normalize();
//#endif
//			}
//		}
	}

	// create map of joints which chains they contain //TODOff(skade) only compute once
	std::map<SkeletalAnimationController::SkeletalJoint*,std::vector<IKChain*>> jointToChain;
	auto& chains = tCtrl->m_ikArmature.m_jointChains;
	for (uint32_t i = 0; i < chains.size(); ++i)
		for (auto j : chains[i].joints)
			jointToChain[j].push_back(&chains[i]);

	std::function<void(SkeletalAnimationController::SkeletalJoint* j, Matrix4f parentT)> imitate;

	//TODO(skade) might be redundant
	sCtrl->forwardKinematics();

	imitate = [&](SkeletalAnimationController::SkeletalJoint* jt, Matrix4f parentT) {
		IKChain* ct = nullptr;
		if (jointToChain[jt].size() > 0)
			ct = jointToChain[jt][0]; //TODOff(skade) multiple chains?
		bool noMatch = true;
		if (ct) {
			//TODO(skade) find corresponding retarget chain
			int is = 0;
			for (int it = 0; it < m_ikcorr.size();++it)
				if (&tCtrl->m_ikArmature.m_jointChains[it] == ct)
					is = m_ikcorr[it];
			if (is!=-1) {
				IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];

				int i = std::distance(ct->joints.begin(),std::find(ct->joints.begin(),ct->joints.end(),jt));
				std::vector<int> matchIdx = jointIndexingFunc(i,cs,*ct);
				//TODO(skade) check why i need to do this here
				std::reverse(matchIdx.begin(), matchIdx.end());

				if (m_showMatchedJoints) {//TODO(skade) debug visualize joint matching
					Vector4f col = Vector4f::Zero(); {
						int isc = jt->ID;
						col = Vector4f(
							float(isc & 1),
							float(isc >> 1 & 1),
							float(isc >> 2 & 1),
							1.f);
					}
					if (auto& jp = tCtrl->getJointPickable(jt).lock()) {

						jp->colorSelect = col;
						jp->m_highlight = true;
						jp->colorOverride = true;
					}
					for (auto& idx : matchIdx) {
						if (auto& jp = sCtrl->getJointPickable(cs.joints[idx]).lock()) {
							jp->colorSelect = col;
							jp->m_highlight = true;
							jp->colorOverride = true;
						}
					}
				}

				if (matchIdx.size()) {
					// old parent joint retrival
					SkeletalAnimationController::SkeletalJoint* js = cs.joints[matchIdx[matchIdx.size()-1]];
					Eigen::Matrix4f jsT = CForgeMath::translationMatrix(js->LocalPosition)
										* CForgeMath::rotationMatrix(js->LocalRotation)
										* CForgeMath::scaleMatrix(js->LocalScale);
					Eigen::Matrix4f parentS = Matrix4f::Identity();
					{
						auto* jsc = js;
						Matrix4f adjS = Matrix4f::Identity();
						while (jsc->Parent != -1) {
							jsc = sCtrl->getBone(jsc->Parent);
							Eigen::Matrix4f jscT = CForgeMath::translationMatrix(jsc->LocalPosition)
												* CForgeMath::rotationMatrix(jsc->LocalRotation)
												* CForgeMath::scaleMatrix(jsc->LocalScale);
							parentS = jscT * parentS;

							////TODO(skade) adj
							//// with adjustment
							//parentS = jscT * adjS * parentS;
							//Matrix4f adjS = js->OffsetMatrix.inverse() * adjS;
						}
					}
					
					//Matrix4f t = jt->OffsetMatrix * parentT.inverse() * parentS * js->OffsetMatrix.inverse() * jsT;
					//Matrix4f t = jt->OffsetMatrix * parentT.inverse() * parentS * js->OffsetMatrix.inverse() * jsT;

					// new local transform
					Matrix4f t = Matrix4f::Identity();

					// parent target
					//if (jt->Parent != -1 && js->Parent != -1) {
					//	auto* jtp = tCtrl->getBone(jt->Parent);
					//	auto* jsp = sCtrl->getBone(js->Parent);

						// current global transform of retargeted parent
						//Matrix4f parentGlobal =  parentT * jtp->OffsetMatrix;
						//Matrix4f parentGlobalS = parentS * jsp->OffsetMatrix;

						// allign next transform so global transform of target and source are identical
						//t = jsT;

						////TODO(skade) adj
						//// get parent relative joint change of restpose
						//Matrix4f adjS = js->OffsetMatrix.inverse() * jsp->OffsetMatrix;
						//Matrix4f adjT = jt->OffsetMatrix.inverse() * jtp->OffsetMatrix;
						//adjS.block<3,1>(0,3) = Vector3f::Zero();
						//adjT.block<3,1>(0,3) = Vector3f::Zero();
						//t = adjT.inverse() * parentT.inverse() * parentS * adjS
						//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();
						
						// correct but doesnt account for rest pose differences
						//t = parentT.inverse() * js->SkinningMatrix * jt->OffsetMatrix.inverse();
						//t = parentT.inverse() * parentS * jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();

						//t = parentT.inverse() * parentS
						//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();

						//TODO(skade) optionally insert multiple source joints
						//if (js->Parent != -1) {
							Quaternionf combinedSourceRot = sCtrl->m_IKJoints[js].rotGlobal;
							for (int j=0;j<matchIdx.size()-1;++j) {
								//combinedSourceRot = cs.joints[j]->LocalRotation * combinedSourceRot;
								combinedSourceRot =  combinedSourceRot * cs.joints[j]->LocalRotation; //TODO(skade) this order correct?
								//* Quaternionf(cs.joints[j]->OffsetMatrix.block<3,3>(0,0))
								//* Quaternionf(cs.joints[j-1]->OffsetMatrix.block<3,3>(0,0).inverse());
							}

							t = parentT.inverse() //* parentS
								//* jsT
								//* CForgeMath::rotationMatrix(sCtrl->m_IKJoints[sCtrl->getBone(js->Parent)].rotGlobal).inverse()
								* CForgeMath::rotationMatrix(combinedSourceRot)
								* js->OffsetMatrix
								* jt->OffsetMatrix.inverse();
						//}
								//* (CForgeMath::translationMatrix(sCtrl->m_IKJoints[js].posGlobal)

						//parentS = parentS * js->OffsetMatrix * jsT;

						// correct
						parentT = parentT * t;
						// but also means:
						//parentT = parentS
						//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();

						////TODO(skade) adj
						//parentT = parentS * adjS
						//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();
					//}
					
					{ // set local rotation of joint
						Vector3f p,s; Quaternionf r;
						MRMutil::deconstructMatrix(t,&p,&r,&s);
						//jt->LocalPosition = p;
						jt->LocalRotation = r;
						//jt->LocalScale = s;
					}
					noMatch = false;
				} // if (matchIdx.size())
			} // if (is!=-1)
		} // if (ct)
		if (noMatch) {
			Eigen::Matrix4f jtT = CForgeMath::translationMatrix(jt->LocalPosition)
			                    * CForgeMath::rotationMatrix(jt->LocalRotation)
			                    * CForgeMath::scaleMatrix(jt->LocalScale);
			
			////TODO(skade) adj
			//Matrix4f adjT = Matrix4f::Identity();
			//if (jt->Parent != -1) {
			//	auto* jtp = tCtrl->getBone(jt->Parent);
			//	Matrix4f adjT = jt->OffsetMatrix.inverse() * jtp->OffsetMatrix;
			//}
			//parentT = parentT * jtT * adjT;

			parentT = parentT * jtT;
		}

		for (auto child : jt->Children) {
			imitate(tCtrl->getBone(child), parentT);
		}
	};

	if (m_imitiateAngle)
		imitate(tCtrl->getRoot(), Eigen::Matrix4f::Identity());

	// copy root position
	for (int i=0;i< tCtrl->boneCount();++i) {
		auto jt = tCtrl->getBone(i);
		if (jt->Parent == -1) {
			auto jt = tCtrl->getBone(i);
			for (int j=0;j< sCtrl->boneCount();++j) {
				auto js = sCtrl->getBone(j);
				if (js->Parent == -1) {
					if (m_copy_rootPos) {
						float scale = m_tar_rootPos.norm()/m_src_rootPos.norm();
						scale = CForgeMath::lerp(1.f,scale,m_scale_rootPos);
						jt->LocalPosition = js->LocalPosition * scale;
					}
					if (m_copy_rootRot)
						jt->LocalRotation = Quaternionf(js->LocalRotation.toRotationMatrix() * js->OffsetMatrix.block<3,3>(0,0) * jt->OffsetMatrix.inverse().block<3,3>(0,0));
					break;
				}
			}
			break;
		}
	}
	tCtrl->forwardKinematics();
};
void MRlimb::reset() {
	m_scale_limbs.clear();
	m_tar_limbLen.clear();
	m_src_limbLen.clear();

	m_ikcorr.clear();
	m_sCE.reset();
	m_tCE.reset();
	m_active = false;
}

void MRlimb::bakingInit() {
	m_isBaking = true;
	m_bakingInit = false;
}
void MRlimb::bakingUpdate(float FPS) {
	auto& sce = m_sCE.lock();
	auto& tce = m_tCE.lock();
	if (!sce || !tce || !sce->pAnimCurr) {
		if (!sce->pAnimCurr)
			std::cerr << "error baking: source has no animation selected";
		m_isBaking = false;
		return;
	}

	static T3DMesh<float>::SkeletalAnimation* newAnim = nullptr;

	static bool finished = true;
	static float t = 0.;

	if (!m_bakingInit) {
		finished = false;
		// create new pAnim for source
		sce->pAnimCurr->t = 0.;
		auto* sourceAnim = sce->controller->animation(sce->pAnimCurr->AnimationID);

		//create new animation container to write keyframes back to
		newAnim = new T3DMesh<float>::SkeletalAnimation();
		newAnim->Duration = sce->pAnimCurr->Duration;
		newAnim->Name = sourceAnim->Name;
		newAnim->SamplesPerSecond = sce->pAnimCurr->SamplesPerSecond;
		tce->mesh.addSkeletalAnimation(newAnim,false);
		m_bakingInit = true;

		for (int i=0;i<tce->mesh.boneCount();++i) {
			auto b = tce->mesh.getBone(i);
			T3DMesh<float>::BoneKeyframes* nkf = new T3DMesh<float>::BoneKeyframes();
			nkf->BoneID = b->ID;
			nkf->BoneName = b->Name;
			nkf->ID = i;
			newAnim->Keyframes.push_back(nkf);
		}
	}
	assert(newAnim);

	{// source do normal playback but snap to keyframes instead of interpolating
		auto* pA = sce->pAnimCurr;
		int animRotSize = sce->controller->animation(pA->AnimationID)->Keyframes[0]->Rotations.size()-1;
		float animTime = sce->controller->animation(pA->AnimationID)->Keyframes[0]->Timestamps.back();

		t += 1./FPS * pA->Speed; // make sure no individual keyframes are skipped here
		sce->animFrameCurr = t / animTime * animRotSize;

		// clamp timing back to keyframe so it ggets correctly displayed
		pA->t = float(sce->animFrameCurr) / (animRotSize) * animTime; //TODO(skade) make pose configurable, see set and get on sequencer

		if (sce->animFrameCurr > animRotSize)
			finished = true; //baking completed
		sce->actor->update();
	}
	sce->controller->forwardKinematics();
	tce->controller->forwardKinematics();

	update();
	// target complete ik
	if (tce->m_IKCupdate)
		tce->controller->update(60.0f / FPS);
	tce->actor->update();

	if (newAnim->Keyframes[0]->Timestamps.size() == 0 || sce->pAnimCurr->t > newAnim->Keyframes[0]->Timestamps.back()) {
		// read back target keyframe data into container
		for (int i=0;i<tce->controller->boneCount(); ++i) {
			auto* b = tce->controller->getBone(i);

			//TODO(skade) assumes that mesh and controller bones are the same
			newAnim->Keyframes[i]->Positions.push_back(b->LocalPosition);
			newAnim->Keyframes[i]->Rotations.push_back(b->LocalRotation);
			newAnim->Keyframes[i]->Scalings.push_back(b->LocalScale);
			newAnim->Keyframes[i]->Timestamps.push_back(sce->pAnimCurr->t);
		}
	}

	if (finished) {
		t = 0.;

		tce->controller->addAnimationData(newAnim);
		m_isBaking = false;
		newAnim = nullptr;
	}
}

}//CForge
