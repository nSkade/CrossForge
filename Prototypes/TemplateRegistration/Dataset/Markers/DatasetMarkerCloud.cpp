#include "DatasetMarkerCloud.h"

namespace TempReg {

	DatasetMarkerCloud::DatasetMarkerCloud() {

	}//Constructor

	DatasetMarkerCloud::~DatasetMarkerCloud() {

	}//Destructor

	void DatasetMarkerCloud::init(Vector3f Scale) {
		m_MarkerScale = Scale;
		m_CloudRoot.init(nullptr);
		m_MarkerInstances.reserve(256);
	}//init

	void DatasetMarkerCloud::clear(void) {
		m_CloudRoot.clear();
		for (auto& Instance : m_MarkerInstances) Instance.clear();
		m_MarkerInstances.clear();
		m_FreeMarkerInstances.clear();
		m_MarkerIDLookup.clear();
	}//clear

	void DatasetMarkerCloud::addMarkerInstance(size_t PointID, const Vector3f MarkerPos, CForge::StaticActor* pActor) {
		if (pActor == nullptr) throw NullpointerExcept("pMarkerActor");
		if (m_MarkerIDLookup.count(PointID) > 0) return; // there already exists a marker for this PointID, nothing to do

		int64_t MarkerID = -1;
		if (!m_FreeMarkerInstances.empty()) {
			MarkerID = *(m_FreeMarkerInstances.begin());
			m_FreeMarkerInstances.erase(m_FreeMarkerInstances.begin());
		}
		else {
			MarkerID = m_MarkerInstances.size();
			m_MarkerInstances.push_back(DatasetMarkerInstance());
		}

		m_MarkerInstances[MarkerID].init(&m_CloudRoot, pActor, MarkerPos, m_MarkerScale);
		m_MarkerIDLookup.emplace(PointID, MarkerID);
	}//addMarkerInstance

	void DatasetMarkerCloud::removeMarkerInstance(size_t PointID) {
		auto MarkerID = m_MarkerIDLookup.find(PointID);
		if (MarkerID == m_MarkerIDLookup.end()) return; // nothing to do

		m_MarkerInstances[MarkerID->second].clear();
		m_FreeMarkerInstances.insert(MarkerID->second);
		m_MarkerIDLookup.erase(PointID);
	}//removeMarkerInstance

	void DatasetMarkerCloud::addToSceneGraph(CForge::ISceneGraphNode* pParent) {
		if (pParent == nullptr) throw NullpointerExcept("pParent");
		pParent->addChild(&m_CloudRoot);
	}//addToSceneGraph

	void DatasetMarkerCloud::removeFromSceneGraph(void) {
		if (m_CloudRoot.parent() == nullptr) return; //nothing to do
		m_CloudRoot.parent()->removeChild(&m_CloudRoot);
	}//removeFromSceneGraph

	void DatasetMarkerCloud::show(bool Show) {
		m_CloudRoot.enable(true, Show);
	}//show

	void DatasetMarkerCloud::markerPosition(size_t PointID,  Vector3f Position) {
		auto MarkerID = m_MarkerIDLookup.find(PointID);
		if (MarkerID == m_MarkerIDLookup.end()) throw CForgeExcept("Could not find marker for specified PointID");
		m_MarkerInstances[MarkerID->second].translation(Position);
	}//markerPosition

	void DatasetMarkerCloud::markerActor(size_t PointID, CForge::StaticActor* pActor) {
		if (pActor == nullptr) throw NullpointerExcept("pActor");
		auto MarkerID = m_MarkerIDLookup.find(PointID);
		if (MarkerID == m_MarkerIDLookup.end()) throw CForgeExcept("Could not find marker for specified PointID");
		m_MarkerInstances[MarkerID->second].actor(pActor);
	}//markerActor

	const CForge::IRenderableActor* DatasetMarkerCloud::markerActor(size_t PointID) const {
		auto MarkerID = m_MarkerIDLookup.find(PointID);
		if (MarkerID == m_MarkerIDLookup.end()) throw CForgeExcept("Could not find marker for specified PointID");
		return m_MarkerInstances[MarkerID->second].actor();
	}//markerActor
}