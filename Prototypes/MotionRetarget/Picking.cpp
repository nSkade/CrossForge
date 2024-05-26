
void MotionRetargetingScene::rayCast(Eigen::Vector3f* ro, Eigen::Vector3f* rd) {
	Vector4f Viewport = Vector4f(0.0f, 0.0f, float(m_RenderWin.width()), float(m_RenderWin.height())); 
	Vector2f CursorPos = Vector2f(m_RenderWin.mouse()->position().x(), Viewport(3) - m_RenderWin.mouse()->position().y());
	Matrix4f View = m_Cam.cameraMatrix();
	Matrix4f Projection = m_Cam.projectionMatrix();
	
	igl::unproject_ray(CursorPos, View, Projection, Viewport, *ro, *rd);
	rd->normalize();
}

//TODO(skade) implement target bone
void MotionRetargetingScene::pickTarget() {
	Eigen::Vector3f RayOrigin, RayDirection;
	rayCast(&RayOrigin, &RayDirection);

	for (int32_t i = 0; i < m_EndEffectors.size(); ++i) {
		auto EndEffector = m_EndEffectors[i];
		auto& TargetTransforms = m_TargetTransformSGNs.at(EndEffector.segmentName);

		//TODO(skade) adapt to new structure
		int32_t AABBIndex = EndEffector.jointIK->pEndEffectorData->TargetPosGlobal.cols()-1;
		Vector3f AABBPos = TargetTransforms[0]->translation(); // get end-effector pos

		AlignedBox3f TranslatedAABB = AlignedBox3f(m_TargetMarkerAABB.min() + AABBPos, m_TargetMarkerAABB.max() + AABBPos);

		//TODOf(skade) implement mesh intersection test
		//igl::ray_mesh_intersect()
		//TODOf(skade) check for distance in case on screenspace overlap of multiple aabb

		const float T0 = m_Cam.nearPlane();
		const float T1 = m_Cam.farPlane();
		float TMin, TMax; // minimum and maximum of interval of overlap within [T0, T1] -> not actually used here, but required by function
		if (igl::ray_box_intersect(RayOrigin, RayDirection, TranslatedAABB, T0, T1, TMin, TMax)) {
			m_SelectedEffectorTarget = i;
			m_LastSelectedEffectorTarget = i;

			Vector3f DragPlaneNormal = m_Cam.dir();
			float IntersectionDist = (AABBPos - RayOrigin).dot(DragPlaneNormal) / RayDirection.dot(DragPlaneNormal);
			m_DragStart = RayOrigin + (RayDirection * IntersectionDist);
			m_guizmoMat = CForgeMath::translationMatrix(AABBPos);
			return;
		}
	}
	
	// Only deselect when last pick was nothing.
	if (m_SelectedEffectorTarget == -1)
		m_LastSelectedEffectorTarget = -1;
	m_SelectedEffectorTarget = -1; // assume nothing will be picked
}//pickTarget

void MotionRetargetingScene::dragTarget(int target) {
	if (target == -1)
		return;

	IKController::SkeletalEndEffector EndEffector = m_EndEffectors[target];
	std::vector<SGNTransformation*>& TargetTransforms = m_TargetTransformSGNs.at(EndEffector.segmentName);

	// apply translation to target points in character controller
	m_IKController->translateTarget(EndEffector.segmentName, m_guizmoMat.block<3,1>(0,3));
	
	//Eigen::Matrix3Xf targetPoints = m_IKController->getTargetPoints(pEndEffector->segmentName);
	Eigen::Matrix3Xf targetPoints = m_IKController->getTargetPoints(EndEffector.segmentName); //TODO(skade) remove

	// apply translation to target marker transformation SGNs
	for (int32_t i = 0; i < TargetTransforms.size(); ++i) {
		if (TargetTransforms[i] == nullptr)
			continue;
		TargetTransforms[i]->translation(targetPoints.col(i));
	}
}//dragTarget

