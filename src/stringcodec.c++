#include "stringcodec.h++"

namespace Norwegian {

DFAStringCodec::DFAStringCodec(std::shared_ptr<DFA<char> > dfa)
    : mDFA{dfa}
    , mPowers{mDFA->getIdentityMatrix()}
    , mPowerSums{mDFA->getIdentityMatrix()} { }

void DFAStringCodec::precomputePowersUpto(size_t max) {
	precomputeMatrixPowersUpto(
	   max, mDFA->getIdentityMatrix(),
	   mDFA->getNumericMatrix(), mPowers);
}

void DFAStringCodec::precomputePowerSumsUpto(size_t max) {
	precomputeMatrixPowerSumsUpto(
        max, mDFA->getIdentityMatrix(),
		mDFA->getNumericMatrix(), mPowerSums);
}

boost::optional<integer> DFAStringCodec::decode(const std::string& input) {

    precomputePowersUpto(input.length());

    if (input.length() == 0) {
        if ((mDFA->getNumericVectorQf() * mDFA->getNumericVectorQi())(0,0) > 0) {
            return boost::make_optional(integer{0});
        } else {
            return boost::none;
        }
    }

    auto trace = mDFA->read_infra(0, input.begin(), input.end());

    /* String did not parse */
    if (trace.size() < input.length()) {
        return boost::none;
    }

    Matrix M = mDFA->getNumericMatrix();

    integer lindex = 0;

    /* Count lexicographically inferior words within L(A,n)  */
    for (auto it = trace.begin(); it < trace.end(); ++it) {
        Vector Q_infra = it->second;
        integer infra = (mDFA->getNumericVectorQf() * mPowers[trace.end() - it - 1] * Q_infra)(0,0);
        lindex += infra;
    }

    /* Count shorter words */
    precomputePowerSumsUpto(input.length() - 1);
    lindex +=
        (mDFA->getNumericVectorQf() * mPowerSums[input.length() - 1] * mDFA->getNumericVectorQi())(0,0);

    return boost::make_optional(lindex);
}

std::string DFAStringCodec::encode(const integer& targetNumber) {

    auto alphabet = mDFA->getAlphabet();

    if (targetNumber == 0 && (mDFA->getNumericVectorQf() * mDFA->getNumericVectorQi())(0,0) > 0) {
        return "";
    }

    integer cumulative_shorter =
        /* initialize to # of lower bound */
        (mDFA->getNumericVectorQf() * mDFA->getNumericVectorQi())(0,0),
        speculation = 0;
    size_t lower_bound = 0; // |w|-1 lower bound
    size_t upper_bound = 1; // |w|-1 upper bound (speculative)

    Matrix p2 = mDFA->getIdentityMatrix();

    /* Open-ended binary search */
    while (lower_bound < upper_bound) {

        precomputePowerSumsUpto(upper_bound);

        /* transition matrix for words of length <=upper_bound */
        p2 = mPowerSums[upper_bound];

        speculation =
            (mDFA->getNumericVectorQf() * p2 * mDFA->getNumericVectorQi())(0,0);

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
                upper_bound -= (upper_bound - lower_bound)/2;
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

    Vector qNextInfra(mDFA->numStates());

    std::string result;
    result.resize(length);

    uint32_t q = mDFA->getQi(), qNext = q;

    /* Step through the string, constructing it */
    for (size_t k = 0; k < length; ++k) {

        qNextInfra.fill(0);

        integer infra_contrib = 0, additional = 0, speculation = 0;

        for (auto it = alphabet.begin(); it != alphabet.end(); ++it) {
            char a = *it;

            auto maybe_q = mDFA->succ(q, a);

            if (maybe_q.is_initialized()) {
                qNext = maybe_q.get();
                qNextInfra[qNext] += 1;
            } else {
				continue;
            }

            speculation = mDFA->getNumericVectorQf() * mPowers[length - k - 1] * qNextInfra;

			auto itLookahead = it; itLookahead++;
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


} // namespace
