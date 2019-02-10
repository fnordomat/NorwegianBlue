#pragma once

#include "declarations.h++"

#include "DFA.h++"
#include <list>
#include <memory>
#include <tuple>

#include <iostream> // DEBUG

namespace Norwegian {

template<class Char>
class IDFALearner {
public:
    virtual void positiveExample(const std::string& input) = 0;
    virtual std::shared_ptr<DFA<Char>> makeDFA() = 0;
};

template<class Char>
class Trie {
public:
    /**
     * @pre (str, len) is a valid string (len bytes of allocated memory at str)
     * @return number of states added, final state added?
     */
    std::pair<int, bool> insert(const Char* str, size_t len) {
        if (0 == len) {
            if (mOccupied) {
                return std::make_pair(0, true);
            } else {
                mOccupied = true;
                mNumFinalStates++;
                return std::make_pair(0, true);
            }
        }
        else
        {
            auto it = mNext.lower_bound(str[0]);
            if (it->first == str[0])
            {
                auto ret = it->second.insert(str+1, len-1);
                mNumStates += ret.first;
                if (ret.second)
                    mNumFinalStates++;
                return std::make_pair(ret.first, ret.second);
            }
            else
            {
                Trie<Char> sub;
                sub.insert(str+1, len-1);
                it = mNext.emplace_hint(it, std::make_pair(str[0], sub));
                mNumStates += len;
                mNumFinalStates++;
                return std::make_pair(len, true);
            }
        }
    }
    
    size_t getNumStates() const {
        return mNumStates;
    }
    
    size_t getNumFinalStates() const {
        return mNumFinalStates;
    }
    
    /**
     * @param offset = state number offset (used internally)
     */
    std::list<std::tuple<int, Char, int>> getTransitionTuples(int offset = 0) const {
        std::list<std::tuple<int, Char, int>> result;
        int accum = offset + 1;
        for (const auto& kv : mNext) {
            result.push_back(std::make_tuple(offset, kv.first, accum));
            result.splice(result.end(), kv.second.getTransitionTuples(accum));
            accum += kv.second.getNumStates();
        }
        return result;
    }
    
    /**
     * @param offset = state number offset (used internally)
     */
    std::list<int> getFinalStates(int offset = 0) const {
        std::list<int> result;
        if (mOccupied) { result.push_back(offset); }
        int accum = offset + 1;
        for (const auto& kv : mNext) {
            result.splice(result.end(), kv.second.getFinalStates(accum));
            accum += kv.second.getNumStates();
        }
        return result;
    }
    
private:
    int mNumStates = 1; //!< redundant, precomputed, updated by insert()
    int mNumFinalStates = 0;
    bool mOccupied = false;
    std::map<Char, Trie<Char>> mNext;
};


template<class Char>
class DFALearner : public IDFALearner<Char> {

public:
    
    void setDataWeight(double kappa_) { kappa = kappa_; }
    
    virtual void positiveExample(const std::string& input) override {
        mExamples.push_back(input);
    }
    virtual std::shared_ptr<DFA<Char>> makeDFA() override;
    
private:
    std::vector<std::string> mExamples;
    double kappa = 1.0;
};

} // namespace
