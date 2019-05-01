#include "DFA.h++"

#ifdef DEBUG_STDOUT
#include <iostream>
#endif

using state_t = DFA<char>::state_t;

template<>
std::tuple<double, double, double> DFA<char>::computeMDL(
    const std::vector<std::basic_string<char>>& data,
    double dataWeight
) const
{
    double modelBits = 0;
    const double logalph = std::log2(256);
    modelBits += 2 * std::log2(mNumQ);
    modelBits += std::log2(mNumQ);
    modelBits += std::log2(mQf.size());
    modelBits += mQf.size() * std::log2(mNumQ);
    modelBits += std::log2(mDelta.size());
    for (const auto& kv : mDelta) {
        modelBits += std::get<1>(kv).size() * (std::log2(mNumQ) + logalph + std::log2(mNumQ));
    }
    
    double dataBits = 0;
    for (const auto& str : data) {
        double strBits = 0;
        
        state_t q = mQi;
        for ( const auto c : str ) {
            auto maybe_q_next = succ(q, c);
            if (!maybe_q_next.is_initialized()) {
                strBits = std::numeric_limits<double>::max();
                break;
            }
            strBits += std::log2(mDelta.find(q)->second.size()
                               + (mQf.find(q) != mQf.cend() ? 1.0f : 0.0f));
            q = maybe_q_next.get();
        }
        
        dataBits += strBits;
    }

    return std::make_tuple(modelBits + dataWeight * dataBits, modelBits, dataBits);
}

template class DFA<char>;

