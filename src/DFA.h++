#pragma once

#include "declarations.h++"
#include "aux.h++"

#include <set>
#include <map>
#include <vector>
#include <memory>
#include <unordered_map>
#include <iterator>
#include <tuple>

#include <boost/optional.hpp>
#include <Eigen/Core>
#include <gmpxx.h>

#include <string>
#include <boost/format.hpp>

#include <json/json.h>

#ifdef DEBUG_STDOUT
#include <iostream>
#endif

template<class Char>
class DFA {

public:

    using state_t = uint32_t;

    //! has to have ordered traversal!
    using succmap_t = std::map<Char, state_t>;
    
    using transition_t = std::tuple<state_t,Char,state_t>;
    
    std::string serialize() const;
    
    // double
    std::tuple<double, double, double>
    computeMDL(const std::vector<std::basic_string<Char>>& data, double dataWeight = 1.0) const;
    
    template<class II> boost::optional<state_t> read(state_t q, II begin, II end) const;

    boost::optional<state_t> read(state_t q, const std::vector<Char>&& word) const {
        return read(q, word.begin(), word.end());
    }

    boost::optional<state_t> read(state_t q, const std::string& word) const {
        return read(q, word.begin(), word.end());
    }
    
    template<class II> std::vector<std::pair<state_t, Vector>> read_infra(state_t q, II begin, II end) const;

    boost::optional<state_t> succ(state_t q, Char c) const;
    
    /**
     * @pre !mergeSeed.empty()
     */
    void mergeStates(const std::set<state_t>& mergeSeed);
    
    bool hasFiniteLanguage() const;
    
    /**
     * @brief Implies hasFiniteLanguage.
     */
    bool hasSparseLanguage() const;
    
    uint32_t numStates() const {
        return mNumQ;
    }
    
    succmap_t getOutEdges(state_t q) const {
        auto it = mDelta.find(q);
        if (it == mDelta.cend()) return {};
        return std::get<1>(*it);
    }
    bool isFinal(state_t q) const {
        return mQf.find(q) != mQf.cend();
    }

    DFA() : mNumQ(1), mQi(0), mDelta{}, mQf{} {};
    
    /**
     * @pre  numstates > 0 (otherwise, initialState makes no sense)
     * @post Constructors guarantee initialisation of matrix representation
     */
    DFA(state_t numstates,
        state_t initialState,
        std::initializer_list<transition_t> transitions,
        std::initializer_list<state_t> finalState)
    : mNumQ {numstates}
    , mQi {initialState}
    , mDelta {}
    , mQf {finalState}
    {
        for (const auto& tr : transitions) {
            addTransition(tr);
            mAlphabet.insert(std::get<1>(tr));
        }
        updatePrecomputedProperties();
    }

    template<class II1, class II2>
    DFA(state_t numstates, state_t initialState,
        II1 transitionsBegin, II1 transitionsEnd,
        II2 finalBegin, II2 finalEnd)
        : mNumQ {numstates}
    , mQi {initialState}
    , mDelta {}
    , mQf {}
    {
        for (II1 trIt = transitionsBegin; trIt != transitionsEnd; ++trIt) {
            addTransition(*trIt);
            mAlphabet.insert(std::get<1>(*trIt));
        }
        for (II2 fIt = finalBegin; fIt != finalEnd; ++fIt) {
            mQf.insert(*fIt);
        }
#ifdef DEBUG_STDOUT
        std::cout << boost::format("Made DFA with %d states, %d symbols, %d transitions and %d final states") % numstates % mAlphabet.size() % mDelta.size() % mQf.size() << std::endl;
#endif
        updatePrecomputedProperties();
    }

    DFA(const DFA& other)
      : mNumQ {other.mNumQ}
      , mQi {other.mQi}
      , mDelta {other.mDelta}
      , mQf {other.mQf}
    {}
    
    const std::set<char> getAlphabet() const {
        return mAlphabet;
    }

    Matrix getIdentityMatrix() const;

    SMatrix getSparseIdentityMatrix() const;

    /**
     * @brief Interpret mDelta as a transition matrix A, that
     * can be multiplied with a state column vector: \[A \cdot Q\]
     *
     * @note Not valid before updatePrecomputedPoperties() / updateNumericMatrix().
     */
    const Matrix& getNumericMatrix() const;

    /**
     * @note Not valid before updatePrecomputedPoperties() / updateSparseMatrix().
     */
    const SMatrix& getSparseMatrix() const {
        return mSparseMatrix;
    }

    void updatePrecomputedProperties() {
        updateNumericMatrix();
        updateNumericVectorQf();
        updateNumericVectorQi();
        updateSparseMatrix();
        updateHasEpsilon();
    }

    void updateSparseMatrix();

    void updateHasEpsilon();

    void updateNumericMatrix();

    void updateNumericVectorQf();

    void updateNumericVectorQi();

    /**
     * @note Not valid before updatePrecomputedPoperties().
     * @return true iff L(A) contains the emptyword.
     */
    bool hasEpsilon() const {
        return mHasEpsilon;
    }

    /**
     * @note Not valid before updatePrecomputedPoperties() / updateNumericVectorQf().
     */
    const RowVector& getNumericVectorQf() const {
        return mNumericVectorQf;
    }

    /**
     * @note Not valid before updatePrecomputedPoperties() / updateNumericVectorQi().
     */
    const Vector& getNumericVectorQi() const {
        return mNumericVectorQi;
    }

    state_t getQi() const {
        return mQi;
    }

    const std::set<state_t>& getQf() const {
        return mQf;
    }

    std::string toString() const;
    
    bool isDead(state_t q) const;
    
    /**
     * @return res[q] > 0 iff reachable
     */
    const Vector computeReachability(const std::set<state_t>& from) const {
        Vector fromVec(mNumQ);
        fromVec.fill(0);
        for (auto q : from) { fromVec[q] = 1; }
    
        std::vector<Matrix> powersums;
        
        precomputeMatrixPowerSumsUpto(
            mNumQ, getIdentityMatrix(),
            getNumericMatrix(), powersums);
        
        Vector reachableVec;
    
        reachableVec = powersums[mNumQ] * fromVec;
        
        return reachableVec;
    }
    
private:

    void addTransition(const transition_t& tr) {
        mDelta[std::get<0>(tr)][std::get<1>(tr)] = std::get<2>(tr);
    }

    state_t mNumQ;
    state_t mQi;
    std::unordered_map<state_t, succmap_t> mDelta;
    std::set<state_t> mQf;
    std::set<char> mAlphabet;

    // Precomputed properties:
    Matrix    mNumericMatrix;
    SMatrix   mSparseMatrix;
    RowVector mNumericVectorQf;
    Vector    mNumericVectorQi;
    bool      mHasEpsilon;
};

void precomputeMatrixPowersUpto(size_t max, const Matrix& I, const Matrix& M, std::vector<Matrix>& powers);

void precomputeSparseMatrixPowersUpto(size_t max, const SMatrix& I, const SMatrix& M, std::vector<SMatrix>& powers);

template<class MatrixT>
void precomputeMatrixPowerSumsUpto(size_t max, const MatrixT& I, const MatrixT& M, std::vector<MatrixT>& powerSums);
