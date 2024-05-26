#pragma once

namespace CForge {

class IKSegment {
public:
private:
	IKSegment* m_pParent;
	std::vector<IKSegment*> m_pChilds;
};

class IKChain {
public:
private:
	IKSegment* m_pRoot;
	std::vector<IKSegment*> m_pSegments;
};

}//CForge

