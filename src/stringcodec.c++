#include "stringcodec.h++"

namespace Norwegian {

DFAStringCodec::DFAStringCodec(std::shared_ptr<DFA<char> > dfa)
    : mDFA {dfa}
, mSparsePowers {mDFA->getSparseIdentityMatrix()}
, mPowerSums {mDFA->getIdentityMatrix()} { }

void DFAStringCodec::precomputePowersUpto(size_t max) {
    precomputeSparseMatrixPowersUpto(
           max, mDFA->getSparseIdentityMatrix(),
           mDFA->getSparseMatrix(), mSparsePowers);
    for (size_t n = mQfTimesPowers.size(); n <= max; ++n) {
        mQfTimesPowers.emplace_back(mDFA->getNumericVectorQf() * sparsePower(n));
    }
}

void DFAStringCodec::precomputePowerSumsUpto(size_t max) {
    precomputeMatrixPowerSumsUpto(
        max, mDFA->getIdentityMatrix(),
        mDFA->getNumericMatrix(), mPowerSums);
    for (size_t n = mCounts.size(); n <= max; ++n) {
        mCounts.emplace_back((mDFA->getNumericVectorQf() * mPowerSums[n] * mDFA->getNumericVectorQi())(0,0));
    }
}

boost::optional<integer> DFAStringCodec::decode(const std::string& input) {

    precomputePowersUpto(input.length());

    if (input.length() == 0) {
        if (mDFA->hasEpsilon()) {
            return boost::make_optional(integer {0});
        } else {
            return boost::none;
        }
    }

    auto trace = mDFA->read_infra(0, input.begin(), input.end());

    /* String did not parse */
    if (trace.size() < input.length()) {
        return boost::none;
    }

    integer lindex = 0;

    /* Count lexicographically inferior words within L(A,n)  */
    for (auto it = trace.begin(); it < trace.end(); ++it) {
        Vector Q_infra = it->second;
        integer infra = (mQfTimesPowers[trace.end() - it - 1] * Q_infra)(0,0);
        lindex += infra;
    }

    /* Count shorter words */
    precomputePowerSumsUpto(input.length() - 1);
    lindex += mCounts[input.length() - 1];

    return boost::make_optional(lindex);
}

std::string DFAStringCodec::encode(const integer& targetNumber) {

    const auto& alphabet = mDFA->getAlphabet();

    if (targetNumber == 0 && mDFA->hasEpsilon()) {
        return "";
    }

    integer cumulative_shorter =
        /* initialize to # of lower bound */
        (mDFA->getNumericVectorQf() * mDFA->getNumericVectorQi())(0,0);
    integer speculation = 0;
    size_t lower_bound = 0; // |w|-1 lower bound
    size_t upper_bound = 1; // |w|-1 upper bound (speculative)

    /* Open-ended binary search */
    while (lower_bound < upper_bound) {

        precomputePowerSumsUpto(upper_bound);

        /* transition matrix for words of length <=upper_bound */
        speculation = mCounts[upper_bound];

        /* targetNumber = the expected number of lower-ranking strings,
           as counting starts from 0, obviously. */
        if (speculation > targetNumber) {
            if (upper_bound == lower_bound + 1) {
                if (speculation == targetNumber) {
                    cumulative_shorter = speculation;
                    lower_bound = upper_bound;
                }
                /* done, in either case */
                break;
            } else {
                /* shrink interval, don't do anything else */
                upper_bound -= (upper_bound - lower_bound) >> 1;
            }
        } else {
            /* we have a new lower bound */
            lower_bound = upper_bound;
            cumulative_shorter = speculation;
            upper_bound *= 2;
        }
    }

    /* At this point we know how long the string is going to be */
    size_t length = lower_bound + 1;
    precomputePowersUpto(length);

    std::string result;
    result.resize(length);

    uint32_t q = mDFA->getQi(), qNext = q;

    /* Step through the string, constructing it */
    for (size_t k = 0; k < length; ++k) {

        integer infra_contrib = 0, additional = 0, speculation = 0;
        boost::optional<DFA<char>::state_t> maybe_q;

        for (auto it = alphabet.begin(); it != alphabet.end(); ++it) {
            char a = *it;

            maybe_q = mDFA->succ(q, a);

            if (maybe_q.is_initialized()) {
                qNext = maybe_q.get();
                speculation += mQfTimesPowers[length - k - 1](qNext);
            } else {
                continue;
            }

            auto itLookahead = it;
            itLookahead++;
            if (itLookahead == alphabet.cend()) {
                result[k] = a;
                q         = qNext;
                infra_contrib += additional;
                break;
            }

            if (cumulative_shorter + speculation <= targetNumber) {
                additional = speculation;
                continue;
            }
            else {
                result[k] = a;
                q         = qNext;
                infra_contrib += additional;
                break; /* break out of alphabet */
            }
        } // alphabet

        cumulative_shorter += infra_contrib;
    }

    result[lower_bound+1] = '\0';

    return result;
}

const SMatrix& DFAStringCodec::sparsePower(size_t n) {
	return mSparsePowers[n];
}

} // namespace
