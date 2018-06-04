#pragma once

#include "declarations.h++"
#include "aux.h++"

#include <set>
#include <map>
#include <vector>
#include <memory>
#include <unordered_map>
#include <iterator>

#include <boost/optional.hpp>
#include <Eigen/Core>
#include <gmpxx.h>

#include <string>
#include <boost/format.hpp>

#ifdef DEBUG_STDOUT
#include <iostream>
#endif

template<class Char>
class DFA {

public:

    using state_t = uint32_t;

    template<class II>
    boost::optional<state_t>
    read(state_t q, II begin, II end) const {

        state_t q_next;

        for (; begin!=end; ++begin) {

            const auto transitionEntry = mDelta.find(q);
            if (transitionEntry == mDelta.end()) {
                return boost::none;
            }

            const auto it_successor = transitionEntry->second.find(*begin);
            if (it_successor == transitionEntry->second.end()) {
                return boost::none;
            }

            q_next = it_successor->second;

            state_t qtmp;
            qtmp = q;
            q = q_next;
            q_next = qtmp;
        }

        return boost::make_optional(q);
    }

    boost::optional<state_t> read(state_t q,
                                  const std::vector<Char>&& word) const {
        return read(q, word.begin(), word.end());
    }

    boost::optional<state_t> read(state_t q, const std::string& word) const {
        return read(q, word.begin(), word.end());
    }

    template<class II>
    std::vector<std::pair<state_t, Vector>>
    read_infra(state_t q, II begin, II end) const {

        std::vector<std::pair<state_t, Vector>> result;

        state_t q_next, qtmp;
        Vector infra(mNumQ);

        for (; begin!=end; ++begin) {

            /* Is the state present at all as LHS of the Delta map? */
            const auto transitionEntry = mDelta.find(q);
            if (transitionEntry == mDelta.end()) {
                return {};
            }

            /* Is there a transition with the correct letter? */
            const auto& transitionMap = transitionEntry->second;
            const auto it_successor = transitionMap.find(*begin);
            if (it_successor == transitionMap.end()) {
                return {};
            }

            q_next = it_successor->second;

            infra.fill(0);
            /* An ordering of Sigma is implicit in the map structure */
            for (auto it = transitionMap.begin();
                    it != it_successor; ++it) {
                infra(it->second)+=1;
            }

            result.emplace_back(std::make_pair(q_next, infra));

            qtmp = q;
            q = q_next;
            q_next = qtmp;
        }

        /* Word not accepted, trace invalid */
        if (mQf.find(q) == mQf.cend()) {
            return {};
        }

        return result;
    }

    boost::optional<state_t> succ(state_t q, Char c) const;
    
    bool hasFiniteLanguage() const;
    
    /**
     * @brief Implies hasFiniteLanguage.
     */
    bool hasSparseLanguage() const;
    
    uint32_t numStates() const {
        return mNumQ;
    }

    DFA(state_t numstates,
        state_t initialState,
        std::initializer_list<std::tuple<state_t,Char,state_t>> transitions,
        std::initializer_list<state_t> finalState
       )
        : mNumQ {numstates}
    , mQi {initialState}
    , mDelta {}
    , mQf {finalState}
    {
        for (const auto& tr : transitions) {
            addTransition(tr);
            mAlphabet.insert(std::get<1>(tr));
        }
        updatePrecomputedPoperties();
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
        updatePrecomputedPoperties();
    }

    const std::set<char> getAlphabet() const {
        return mAlphabet;
    }

    Matrix getIdentityMatrix() const;

    SMatrix getSparseIdentityMatrix() const;

    /**
     * @brief Interpret mDelta as a transition matrix A, that
     * can be multiplied with a state column vector: \[A \cdot Q\]
     *
     * @note Not valid before update.
     */
    const Matrix& getNumericMatrix() const;

    /**
     * @note Not valid before update.
     */
    const SMatrix& getSparseMatrix() const {
        return mSparseMatrix;
    }

    void updatePrecomputedPoperties() {
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
     * @note Not valid before update.
     * @return true iff L(A) contains the emptyword.
     */
    bool hasEpsilon() const {
        return mHasEpsilon;
    }

    /**
     * @note Not valid before update.
     */
    const RowVector& getNumericVectorQf() const {
        return mNumericVectorQf;
    }

    /**
     * @note Not valid before update.
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

    DFA& operator=(const DFA& other) = delete;

    std::string toString() const;

private:

    void addTransition(const std::tuple<state_t,Char,state_t>& tr) {
        mDelta[std::get<0>(tr)][std::get<1>(tr)] = std::get<2>(tr);
    }

    const state_t mNumQ;
    const state_t mQi;
    //! has to have ordered traversal!
    using succmap_t = std::map<Char, state_t>;
    std::unordered_map<state_t, succmap_t> mDelta;
    std::set<state_t> mQf;
    std::set<char> mAlphabet;

    // Precomputed properties:
    Matrix mNumericMatrix;
    SMatrix mSparseMatrix;
    RowVector mNumericVectorQf;
    Vector mNumericVectorQi;
    bool mHasEpsilon;
};

void precomputeMatrixPowersUpto(size_t max, const Matrix& I, const Matrix& M,
                                std::vector<Matrix>& powers);

void precomputeSparseMatrixPowersUpto(size_t max, const SMatrix& I, const SMatrix& M, std::vector<SMatrix>& powers);

template<class MatrixT>
void precomputeMatrixPowerSumsUpto(size_t max, const MatrixT& I, const MatrixT& M, std::vector<MatrixT>& powerSums);
