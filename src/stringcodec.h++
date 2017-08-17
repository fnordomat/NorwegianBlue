#pragma once

#include "DFA.h++"
#include "declarations.h++"
#include <string>

namespace Norwegian {

class IStringCodec {
public:
	virtual boost::optional<integer> decode(const std::string& input) = 0;
	virtual std::string encode(const integer& number) = 0;
};

class DFAStringCodec : public IStringCodec {

public:

	DFAStringCodec(std::shared_ptr<DFA<char> > dfa);

	void precomputePowersUpto(size_t max);
	void precomputePowerSumsUpto(size_t max);
	virtual boost::optional<integer> decode(const std::string& input) override;
	virtual std::string encode(const integer& number) override;

private:

	std::shared_ptr<DFA<char> > mDFA;
	std::vector<Matrix> mPowers;    //!< Precomputed powers of mDFA.mDelta()
	std::vector<RowVector> mQfTimesPowers; //!< Precomputed Qf * powers

	std::vector<Matrix> mPowerSums; //!< Precomputed too
	std::vector<integer> mCounts;
};

} // namespace
