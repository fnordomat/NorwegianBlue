#include "DFA.h++"

template<>
boost::optional<DFA<char>::state_t> DFA<char>::succ(state_t q, char c) const {
    const auto transitionEntry = mDelta.find(q);
    if (transitionEntry == mDelta.end()) {
        return boost::none;
    }

    const auto& transitionMap = transitionEntry->second;
    const auto it_successor = transitionMap.find(c);
    if (it_successor == transitionMap.cend())
    {
        return boost::none;
    }
    return it_successor->second;
}

template<>
bool DFA<char>::hasFiniteLanguage() const {
	std::vector<Matrix> powersums;
	precomputeMatrixPowerSumsUpto(mNumQ, getIdentityMatrix(),
								  getNumericMatrix(), powersums);
	
	Vector loopingStates(mNumQ);
	loopingStates.fill(0);
	
	for (size_t i = 0; i < mNumQ; ++i) {
		if (powersums[mNumQ](i,i) - 1 > 0) loopingStates[0] = 1;
	}
	
	Matrix initiallyReachableStates =
		powersums[mNumQ] * getNumericVectorQi();
	
	loopingStates = loopingStates.array() * initiallyReachableStates.array();
	
	Matrix relevantPaths = getNumericVectorQf() * powersums[mNumQ] * loopingStates;
	
	return relevantPaths(0,0) == 0;
}

void precomputeMatrixPowersUpto(size_t max, const Matrix& I,
						  const Matrix& M,
						  std::vector<Matrix>& powers) {
	if (!powers.size()) { powers.push_back(I); }
    for (size_t n = powers.size(); n <= max; ++n) {
        Matrix& P = powers.back();
        powers.emplace_back(P * M);
    }
}

void precomputeMatrixPowerSumsUpto(size_t max, const Matrix& I,
							 const Matrix& M,
							 std::vector<Matrix>& powerSums) {
	if (!powerSums.size()) { powerSums.push_back(I); }
    for (size_t n = powerSums.size(); n <= max; ++n) {
        Matrix& P = powerSums.back();
        powerSums.emplace_back(P * M + I);
    }
}
