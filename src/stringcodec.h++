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

    DFAStringCodec(std::shared_ptr<DFA<char>> dfa);

    virtual boost::optional<integer> decode(const std::string& input) override;
    virtual std::string encode(const integer& number) override;

private:

    void precomputePowersUpto(size_t max);
    void precomputeSparsePowersUpto(size_t max);
    void precomputePowerSumsUpto(size_t max);

    std::shared_ptr<DFA<char>> mDFA;
    std::vector<RowVector> mQfTimesPowers; //!< Precomputed Qf * powers
    std::vector<SMatrix> mSparsePowers;    //!< Precomputed powers of mDFA.mDelta()

    std::vector<Matrix> mPowerSums; //!< Precomputed sums of powers of mDFA.mDelta() from 1 to n
    std::vector<integer> mCounts;   //!< Counts of shorter words, precomputed together with mPowerSums

    const SMatrix& sparsePower(size_t n);

};

} // namespace
