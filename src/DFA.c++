#include "DFA.h++"

#ifdef DEBUG_STDOUT
#include <iostream>
#endif

using state_t = DFA<char>::state_t;

template<class Char>
// TODO re-implement using matrices
bool DFA<Char>::isDead(DFA<Char>::state_t q) const {
    std::set<state_t> reachable = {};
    std::set<state_t> next = {q};
    while (!next.empty()) {
        
        state_t q_ = *next.begin();
        next.erase(next.begin());
        
        if (isFinal(q_)) return false;
        
        reachable.insert(q_);
        
        auto dit = mDelta.find(q_);
        if (dit == mDelta.cend()) continue;
        
        for (const auto aq : std::get<1>(*dit)) {
            if (reachable.find(std::get<1>(aq)) == reachable.cend()) {
                next.insert(std::get<1>(aq));
            }
        }
    }
    return true;
}

template<class Char>
void DFA<Char>::mergeStates(const std::set<state_t>& mergeSeed) {
    
#if MERGE_DEBUG == 1
    std::cout << ">>> mergeStates called" << std::endl;
#endif
    
    state_t maxNew = 0; // number of Q~ states - 1
    
    using oldstate_t = state_t;
    using newstate_t = state_t;
    
    std::vector<int> assignment(mNumQ);
    std::map<newstate_t, std::set<oldstate_t>> inverseAssignment;
    
    // Assignment initialised to -1 = itself
    for (state_t q = 0; q < mNumQ ; ++q) {
        assignment[q] = -1;
    }

#if MERGE_DEBUG == 1
    std::cout << ">>> mergeStates updating(1)" << std::endl;
#endif
    
    updatePrecomputedProperties();
    
    // // std::cout << ">>> mergeStates reachability" << std::endl;
    // Vector reachability = computeReachability(mergeSeed);
    
#if MERGE_DEBUG == 1
    std::cout << ">>> mergeStates randomize" << std::endl;
#endif    
    
    // New state maxNew initialised with Mergeseed
    for (auto q : mergeSeed) {
        assignment[q] = maxNew;
        inverseAssignment[maxNew].insert(q);
    }
    
    std::deque<newstate_t> toInvestigate = {maxNew};
    std::map<newstate_t, std::map<Char, newstate_t>> newDelta;
    
    struct SetCompare {
        bool operator()(const std::set<state_t>& s1,
                        const std::set<state_t>& s2) const {
             
            auto it1 = s1.cbegin();
            auto it2 = s2.cbegin();
            
            for (;(it1 != s1.cend() && it2 != s2.cend()); ++it1, ++it2) {
                if ((*it1) < (*it2)) {
                    return true;
                } else if ((*it1) > (*it2)) {
                    return false;
                }
            }
            
            if (it1 == s1.cend() && it2 != s2.cend()) return true;
            
            return false;
        }
    };
    
    std::set<std::set<oldstate_t>, SetCompare> investigated;
    
#if MERGE_DEBUG == 1
    std::cout << ">>> mergeStates investigating" << std::endl;
#endif
    
    while (!toInvestigate.empty()) {
        
        newstate_t p = toInvestigate.front();
        toInvestigate.pop_front();

#if MERGE_DEBUG == 1
        std::cout << ">>> mergeStates investigating TO_INVESTIGATE: " << p << std::endl;
#endif
        
        // The state has ceased to exist, it is an ex-state
        if (inverseAssignment.find(p) == inverseAssignment.cend()) {
            continue;
        }

        const auto set = inverseAssignment.at(p);
        const auto queried = investigated.find(set);
        if (queried != investigated.cend()) {
            continue;
        }
        
        investigated.insert(set);
        
        // Collect successor states using the symbols on all outgoing edges
        std::map<Char, std::set<newstate_t>> macroSuccessors;
        
        std::vector<state_t> toDelete;
        
        if (inverseAssignment.find(p) == inverseAssignment.cend()) {
            continue;
        }
        
        // Go through the constituent states of p
        for (oldstate_t q : inverseAssignment[p]) {

#if MERGE_DEBUG == 1            
            std::cout << ">>> mergeStates investigating constituent " << q << std::endl;
#endif
            
            auto it = mDelta.find(q);
            if (it == mDelta.cend()) continue;
            
            // Go through all outgoing edges and collect ....
            for (const auto& aq : it->second) {
        
                Char letter = std::get<0>(aq);

#if MERGE_DEBUG == 1                
                std::cout << ">>> mergeStates investigating letter " << letter << " leading to Qnew " << std::get<1>(aq) << std::endl;
#endif                
                
                // For each letter b = std::get<0>(aq),
                // the successor oldstates go into the same equivalence class
                
                int image = assignment.at(std::get<1>(aq));
                
                switch (image) {
                case -1:
                    maxNew ++; // Newly added Q~ state
                    macroSuccessors[letter].insert(maxNew);

#if MERGE_DEBUG == 1                    
                    std::cout << ">>> mergeStates inserting (maxNew++) " << maxNew << std::endl;
#endif
                    assignment[std::get<1>(aq)] = maxNew;
                    inverseAssignment[maxNew] = {std::get<1>(aq)};
                    break;
                default:
                    // known Q~ state
#if MERGE_DEBUG == 1                    
                    std::cout << ">>> mergeStates inserting (known image) " << image << std::endl;
#endif
                    macroSuccessors[letter].insert(image);
                }
            }
        }

#if MERGE_DEBUG == 1
        std::cout << ">>> mergeStates analyzing macroSuccessors" << std::endl;
#endif
            
        for (const auto& macroSuccessor : macroSuccessors) {

            if (std::get<1>(macroSuccessor).size() == 0) {
                continue;
            }
            
            // Merge to the lowest-numbered new state, say.
            auto first = std::get<1>(macroSuccessor).cbegin();
            auto it = first;
            it++;
            
            // If the rest is empty, there's nothing to be merged.
            for (; it != std::get<1>(macroSuccessor).cend(); ++it) {
                
                for (const auto oldq : inverseAssignment.at(*it)) {
                    assignment[oldq] = *first;
                    inverseAssignment[*first].insert(oldq);
                }
                
                bool has_p_loop = false; // via loop
                
                for (const auto qQ : inverseAssignment) {
                    if (qQ.first == *first) {
                        continue;
                    }
                    
                    for (const auto oldq : inverseAssignment.at(*it)) {
                    
                        if (qQ.second.find(oldq) != qQ.second.cend()) {
                            inverseAssignment[*first].insert(
                                qQ.second.cbegin(), qQ.second.cend());
            
                            if (qQ.first == p) { has_p_loop = true; }
                            for (const auto q : qQ.second) {
                                assignment[q] = *first;
                            }
                            toDelete.push_back(qQ.first);
                        }
                    }
                }
                
                if (has_p_loop) { toInvestigate.push_back(p); }
            }

            
#if MERGE_DEBUG == 1
            std::cout << ">>> mergeStates analyzing, first=" << *first << std::endl;
#endif
            
            if (investigated.find(inverseAssignment.at(*first))
                == investigated.cend()) {
                toInvestigate.push_back(*first);
                
            } else {
                // already investigated
            }
        }

#if MERGE_DEBUG == 1        
        std::cout << ">>> mergeStates erasing obsoletes" << std::endl;
#endif        
    
        for (const auto state : toDelete) {
            inverseAssignment.erase(state);
        }
    }
    
    
#if MERGE_DEBUG == 1
    std::cout << ">>> mergeStates unreachables" << std::endl;
#endif
    
    for (state_t q = 0; q < mNumQ; ++q) {
        if (assignment[q] == -1) {
            assignment[q] = maxNew;
            inverseAssignment[maxNew] = {q};
            maxNew ++; // Newly added Q~ state
        }
    }
    
#if MERGE_DEBUG == 1
    std::cout << ">>> mergeStates renumbering" << std::endl;
#endif
    
    // change all the state indices according to computed assignment
    // renumbering ("normalizing") to remove gaps in new state numbers.
    int newIndexMax = 0;
    std::map<int, int> newIndexLookup;
    
    for (const auto kv : inverseAssignment) {
        newIndexLookup[kv.first] = newIndexMax ++;
    }
    
#if MERGE_DEBUG == 1
    for (int q : assignment) {
        std::cout << q << " ";
    }
#endif
    
    mNumQ = newIndexMax;
    
    mQi   = newIndexLookup.at(assignment.at(mQi));
    decltype(mQf) mQfOld;
    mQfOld.swap(mQf);
    for (auto qf : mQfOld) {
        mQf.insert(newIndexLookup.at(assignment.at(qf)));
    }
    
    decltype(mDelta) mDeltaOld;
    mDelta.swap(mDeltaOld);
    for (auto paq : mDeltaOld) {
        auto& newDeltaEntry = mDelta[newIndexLookup.at(assignment.at(std::get<0>(paq)))];
        for (auto aq : std::get<1>(paq)) {
            newDeltaEntry[std::get<0>(aq)] = newIndexLookup.at(assignment.at(std::get<1>(aq)));
        }
    }

#if MERGE_DEBUG == 1    
    std::cout << ">>> mergeStates updating" << std::endl;
#endif    
    
    // TODO smarted management of matrix pre-computation
    updatePrecomputedProperties();
    
#if MERGE_DEBUG == 1    
    std::cout << ">>> mergeStates done" << std::endl;
#endif
}

template<class Char>
std::string DFA<Char>::serialize() const {
    Json::StreamWriterBuilder builder;
    
    builder.settings_["indentation"] = "  ";
    Json::Value root;
    root["mNumQ"] = mNumQ;
    root["mQi"] = mQi;
    Json::Value mQfArray;
    for (const auto q : mQf) {
        mQfArray.append(q);
    }
    root["mQf"] = mQfArray;
    
    Json::Value mDeltaArray;
    for (const auto& kv : mDelta) {
        for (const auto& kv2 : std::get<1>(kv)) {
            Json::Value tupleArray;
            tupleArray.append(std::get<0>(kv));
            tupleArray.append(std::get<0>(kv2));
            tupleArray.append(std::get<1>(kv2));
            mDeltaArray.append(tupleArray);
        }
    }
    root["mDelta"] = mDeltaArray;
    
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ostringstream oss;
    int rc = writer->write(root, &oss);
    if (rc || !oss.good()) { throw std::runtime_error("Json writer error"); }

    return oss.str();
}

template<class Char>
template<class II>
boost::optional<state_t>
DFA<Char>::read(state_t q, II begin, II end) const {
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

template<class Char>
template<class II>
std::vector<std::pair<state_t, Vector>>
DFA<Char>::read_infra(state_t q, II begin, II end) const {

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

template<class Char>
const Matrix& DFA<Char>::getNumericMatrix() const {
    return mNumericMatrix;
}

template<class Char>
Matrix DFA<Char>::getIdentityMatrix() const {
    Matrix I = Matrix::Identity(mNumQ, mNumQ);
    return I;
}

template<class Char>
SMatrix DFA<Char>::getSparseIdentityMatrix() const {
    Matrix I = Matrix::Identity(mNumQ, mNumQ);
    return I.sparseView();
}

template<class Char>
void DFA<Char>::updateNumericMatrix() {
    mNumericMatrix = Matrix(mNumQ, mNumQ);
    Matrix& A = mNumericMatrix;

    A.fill(0);

    for (const auto& col: mDelta) {
        for (const auto& row: col.second) {
            ++A(row.second, col.first);
        }
    }
}

template<class Char>
void DFA<Char>::updateSparseMatrix() {
    mSparseMatrix = getNumericMatrix().sparseView();
}

template<class Char>
void DFA<Char>::updateHasEpsilon() {
    mHasEpsilon = (getNumericVectorQf() * getNumericVectorQi())(0,0) > 0;
}

template<class Char>
void DFA<Char>::updateNumericVectorQf() {
    mNumericVectorQf = RowVector(mNumQ);
    RowVector& F = mNumericVectorQf;
    F.fill(0);

    for (const state_t& q: mQf) {
        ++F(q);
    }
}

template<class Char>
void DFA<Char>::updateNumericVectorQi() {
    mNumericVectorQi = Vector(mNumQ);
    Vector& I = mNumericVectorQi;
    I.fill(0);
    I(mQi) = 1;
}

template<class Char>
boost::optional<state_t> DFA<Char>::succ(state_t q, Char c) const {
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
    return boost::make_optional(it_successor->second);
}

template<class Char>
bool DFA<Char>::hasFiniteLanguage() const {
    std::vector<Matrix> powersums;
    
    precomputeMatrixPowerSumsUpto(
        mNumQ, getIdentityMatrix(),
        getNumericMatrix(), powersums);

    Vector loopingStates(mNumQ);
    loopingStates.fill(0);

    for (size_t i = 0; i < mNumQ; ++i) {
        if (powersums[mNumQ](i,i) - 1 > 0) loopingStates[i] = 1;
    }

    Matrix initiallyReachableStates =
        powersums[mNumQ] * getNumericVectorQi();

    loopingStates = loopingStates.array() * initiallyReachableStates.array();

    Matrix relevantPaths = getNumericVectorQf() * powersums[mNumQ] * loopingStates;

    return relevantPaths(0,0) == 0;
}

template<class Char>
bool DFA<Char>::hasSparseLanguage() const {
    
    // Idea: the language is "sparse" (has O(n) words of length n)
    // iff there are no distinct loops
    // q0 -> [q1 -> q1] -> qf - a pair of paths p1,p2 from q1 to q1
    // with |p1| = |p2| and p1 != p2 must not exist.

    // Loops can have any period from 1 to n, therefore a common power
    // of p1, p2 is guaranteed to be found within lcm(1...n) if at all.
    // This will show up as a diagonal entry >1 in M^{lcm(1...n)}.

    // This suggests the following (probably stupid) algorithm to decide sparseness:
    
    // Computation similar to n# (OEIS A034386) "Primorial 2. definition"
    mpz_class lcm = mpz_class(1);
    mpz_class counter = mpz_class(1);
    for (size_t i = 2; i <= mNumQ; ++i) {
        counter += mpz_class(1);
        mpz_class gcd = euclid(lcm, counter);
        lcm *= counter / gcd;
    }

    TMatrix thePower = TMatrix::Identity(mNumQ, mNumQ);
    TMatrix TI = TMatrix::Identity(mNumQ, mNumQ);

    mpz_class c    = lcm;
    mpz_class crev = 0;
    size_t trailing = 0;

    while (c > 0 && c % 2 == 0) {
        c >>= 1;
        crev <<= 1;
        ++trailing;
    }
    while (c > 0) {
        if (c % 2 == 0) {
            c >>= 1;
            crev <<= 1;
        } else {
            c >>= 1;
            crev <<= 1;
            crev += 1;
        }
    }

    Matrix NM = getNumericMatrix();
    TMatrix M = NM.cast<Ternary>();

    TRowVector Qf = mNumericVectorQf.cast<Ternary>();
    TVector Qi = mNumericVectorQi.cast<Ternary>();

    while (crev >= 1) {
        if (crev % 2 == 0) {
            crev /= 2;
            thePower *= thePower;
        } else {
            crev -= 1;
            thePower *= M;
        }
    }
    
    for (size_t i = 0 ; i < trailing ; i++) {
        thePower *= thePower;
    }
    
    // If the automaton were guaranteed to be minimal, this could be simplified.

    TVector biLoopingStates(mNumQ);

    for (size_t i = 0 ; i < mNumQ ; i++) {
        if (thePower(i,i) == Ternary(Ternary::T::Many)) {
            biLoopingStates[i] = Ternary(1);
        } else {
            biLoopingStates[i] = Ternary(0);
        }
    }

    std::vector<TMatrix> powerSums;
    precomputeMatrixPowerSumsUpto<TMatrix>(
      mNumQ, TI, M,
      powerSums);

    TVector v1 = powerSums[mNumQ] * Qi;
    for (size_t i = 0 ; i < mNumQ ; ++i) {
        if (v1[i] > Ternary(0) && biLoopingStates[i] > Ternary(0)) {
            v1[i] = Ternary(1);
        } else {
            v1[i] = Ternary(0);
        }
    }

    Ternary result = Qf * v1;
    return !(Ternary(0) < result);
}

template<>
std::string DFA<char>::toString() const {
    std::stringstream ss;
    ss << "DFA(" << mNumQ << "," << mQi << ",{";
    for (const auto& qf : mQf) {
        ss << qf << ",";
    }
    ss << "}) with" << std::endl;
    for (const auto& x : mDelta) {
        for (const auto& y: x.second) {
            ss << boost::format("  %d %c %d") % x.first % y.first % y.second << std::endl;
        }
    }
    return ss.str();
}

void precomputeMatrixPowersUpto(size_t max, const Matrix& I,
                                const Matrix& M,
                                std::vector<Matrix>& powers) {
    if (!powers.size()) {
        powers.push_back(I);
    }

    for (size_t n = powers.size(); n <= max; ++n) {
        Matrix& P = powers.back();
        powers.emplace_back(P * M);
    }
}

void precomputeSparseMatrixPowersUpto
  (size_t max, const SMatrix& I,
   const SMatrix& M,
   std::vector<SMatrix>& powers) {

    if (!powers.size()) {
        powers.push_back(I);
    }

    for (size_t n = powers.size(); n <= max; ++n) {
        SMatrix& P = powers.back();
        powers.emplace_back(P * M);
    }
}

template<class MatrixT>
void precomputeMatrixPowerSumsUpto(
    size_t max, const MatrixT& I,
    const MatrixT& M,
    std::vector<MatrixT>& powerSums) {

    if (!powerSums.size()) {
        powerSums.push_back(I);
    }

    for (size_t n = powerSums.size(); n <= max; ++n) {
        MatrixT& P = powerSums.back();
        powerSums.emplace_back(P * M + I);
    }
}

template class DFA<char>;

template boost::optional<unsigned int> DFA<char>::read<std::string::const_iterator>(unsigned int, std::string::const_iterator, std::string::const_iterator) const;

template std::vector<std::pair<unsigned int, Vector>> DFA<char>::read_infra<std::string::const_iterator>(unsigned int, std::string::const_iterator, std::string::const_iterator) const;

template std::vector<std::pair<unsigned int, Vector>> DFA<char>::read_infra<std::string::iterator>(unsigned int, std::string::iterator, std::string::iterator) const;
