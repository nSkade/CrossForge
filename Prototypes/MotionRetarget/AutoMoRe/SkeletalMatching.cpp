#include "SkeletalMatching.hpp"

#include <hungarian.h>
#include <iostream>

namespace CForge {

void SkeletalMatcher::skelMatch(IKController* csc, IKController* ctc) {

	std::vector<int> ret;
	for (auto& jct : ctc->m_ikArmature.m_jointChains) {
		float bestScore = 0.;
		int bestIdx = 0;
		for (int i = 0; i < csc->m_ikArmature.m_jointChains.size(); ++i) {
			auto& jcs = csc->m_ikArmature.m_jointChains[i];
			
			float score = calculateScore(csc,jcs,ctc,jct);

			if (score > bestScore) {
				bestScore = score;
				bestIdx = i;
			}
		}
		ret.push_back(bestIdx);
	}
	
	m_corr = ret;
};

/**
 * @brief uses hungarian algorithm to optimal match chains globally
*/
void SkeletalMatcher::skelMatchInj(IKController* csc, IKController* ctc) {
	const int num_csc_chains = csc->m_ikArmature.m_jointChains.size();
	const int num_ctc_chains = ctc->m_ikArmature.m_jointChains.size();

	int rows = num_csc_chains;
	int cols = num_ctc_chains;

	// The C library requires a C-style 2D array of integers
	int** cost_matrix = new int*[rows];
	for (int i = 0; i < rows; ++i) {
		cost_matrix[i] = new int[cols];
	}

	// Step 1: Find the maximum possible score to normalize costs
	double max_score = -std::numeric_limits<double>::infinity();
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			double score = calculateScore(csc, csc->m_ikArmature.m_jointChains[i], ctc, ctc->m_ikArmature.m_jointChains[j]);
			if (score > max_score) {
				max_score = score;
			}
		}
	}

	// Step 2: Populate the cost matrix. Cost = (max_score - score) * scale.
	// Scaling is necessary to convert doubles to integers without losing precision.
	const double scale = 10000.0;
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			double score = calculateScore(csc, csc->m_ikArmature.m_jointChains[i], ctc, ctc->m_ikArmature.m_jointChains[j]);
			cost_matrix[i][j] = static_cast<int>((max_score - score) * scale);
		}
	}

#if 0	// --- DEBUGGING: Print the assignment matrix ---
	std::cout << "Generated Cost Matrix:" << std::endl;
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			std::cout << cost_matrix[i][j] << "\t";
		}
		std::cout << std::endl;
	}
	std::cout << "----------------------" << std::endl;
#endif	// --- END DEBUGGING ---

	// Step 3: Initialize the hungarian problem with the cost matrix
	hungarian_problem_t p;
	hungarian_init(&p, cost_matrix, rows, cols, HUNGARIAN_MODE_MINIMIZE_COST);

	// Step 4: Solve the problem
	hungarian_solve(&p);

#if 0	// --- DEBUGGING: Print the assignment matrix ---
	std::cout << "Assignment Matrix:" << std::endl;
	hungarian_print_assignment(&p);
	std::cout << "--------------------" << std::endl;
#endif	// --- END DEBUGGING ---

	// Step 5: Extract the results and clean up
	m_corr.clear();
	m_corr.resize(cols, -1);

	for (int i = 0; i < p.num_rows; ++i) {
		int ctc_match_index = -1;
		for (int j = 0; j < p.num_cols; ++j) {
			if (p.assignment[i][j] == 1) { // The library uses 1 to denote an assignment
				ctc_match_index = j;
				break;
			}
		}
		if (i < num_csc_chains && ctc_match_index < num_ctc_chains) {
			m_corr[ctc_match_index] = i;
		}
	}

	// Free the library's allocated memory
	hungarian_free(&p);

	// Free the temporary cost matrix
	for (int i = 0; i < rows; ++i) {
		delete[] cost_matrix[i];
	}
	delete[] cost_matrix;
}
// Helper function to calculate the score between two joint chains
double SkeletalMatcher::calculateScore(IKController* csc, const IKChain& jcs, IKController* ctc, const IKChain& jct) {
	// Positional scores
	double rootPos = 1.0 / (1.0 + (csc->m_IKJoints[jcs.joints.back()].posGlobal - ctc->m_IKJoints[jct.joints.back()].posGlobal).norm());
	double eefPos = 1.0 / (1.0 + (csc->m_IKJoints[jcs.joints.front()].posGlobal - ctc->m_IKJoints[jct.joints.front()].posGlobal).norm());

	// Directional score
	Vector3f dirT = ctc->m_IKJoints[jct.joints.front()].posGlobal - ctc->m_IKJoints[jct.joints.back()].posGlobal;
	Vector3f dirS = csc->m_IKJoints[jcs.joints.front()].posGlobal - csc->m_IKJoints[jcs.joints.back()].posGlobal;
	double dotS = dirT.dot(dirS);

	// Mean position score
	Vector3f csmp = Vector3f::Zero();
	Vector3f ctmp = Vector3f::Zero();
	for (auto joint_idx : jcs.joints)
		csmp += csc->m_IKJoints[joint_idx].posGlobal;
	csmp /= jcs.joints.size();
	for (auto joint_idx : jct.joints)
		ctmp += ctc->m_IKJoints[joint_idx].posGlobal;
	ctmp /= jct.joints.size();
	double meanPos = 1.0 / (1.0 + (csmp - ctmp).norm());

	// Final weighted score
	double score = m_wRootPos * rootPos
				 + m_wEefPos * eefPos
				 + m_wDir * dotS
				 + m_wMeanPos * meanPos;

	return score;
}

}//CForge

