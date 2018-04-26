#include "DFA.h++"

using state_t = DFA<char>::state_t;

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
    return it_successor->second;
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

mpz_class euclid(mpz_class a, mpz_class b) {
    if (a <= 0 || b <= 0) {
        throw std::domain_error("gcd: negative argument");
    }
    while (1) {
        if (a == 1) {return a;}
        if (a == b) {return a;}
        if (b < a) { a.swap(b); }
        b -= a;
    }
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
    
    // Compute n# (OEIS A034386) "Primorial 2. definition":
    mpz_class lcm = mpz_class(1);
    mpz_class counter = mpz_class(1);
    for (size_t i = 2; i <= mNumQ; ++i) {
        counter += mpz_class(1);
        mpz_class gcd = euclid(lcm, counter);
        if (gcd == mpz_class(1)) {
            lcm *= counter;
        }
    }

    Matrix thePower = getIdentityMatrix();

    mpz_class c    = lcm;
    mpz_class crev = mpz_class(0);

    size_t trailing = 0;

    while (c > mpz_class(0)) {
        if (c % mpz_class(2) == 0) {
            c >>= 1;
            crev *= 2;
        } else {
            c -= 1;
            if (crev == 0) {
                ++trailing;
            } else {
            }
            crev += 1;
        }
    }

    while (crev >= mpz_class(1)) {
        if (crev % mpz_class(2) == mpz_class(0)) {
            crev /= mpz_class(2);
            thePower *= thePower;
        } else {
            crev -= mpz_class(1);
            thePower *= getNumericMatrix();
        }
    }
    
    for (size_t i = 0 ; i < trailing ; i++) {
        thePower *= thePower;
    }
    
    Vector biLoopingStates(mNumQ);

    for (size_t i = 0 ; i < mNumQ ; i++) {
        if (thePower(i,i) > mpz_class(1)) {
            biLoopingStates[i] = 1;
        } else {
            biLoopingStates[i] = 0;
        }
    }

    std::vector<Matrix> powersums;
    precomputeMatrixPowerSumsUpto(
      mNumQ, getIdentityMatrix(),
      getNumericMatrix(), powersums);
    
    Vector v1 = powersums[mNumQ] * mNumericVectorQi;
    for (size_t i = 0 ; i < mNumQ ; ++i) {
        if (v1[i] > 0 && biLoopingStates[i] > 0) {
            v1[i] = 1;
        } else {
            v1[i] = 0;
        }
    }

    mpz_class result = getNumericVectorQf() * v1 ;

    return result == 0;
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

void precomputeMatrixPowerSumsUpto(
    size_t max, const Matrix& I,
    const Matrix& M,
    std::vector<Matrix>& powerSums) {

    if (!powerSums.size()) {
        powerSums.push_back(I);
    }

    for (size_t n = powerSums.size(); n <= max; ++n) {
        Matrix& P = powerSums.back();
        powerSums.emplace_back(P * M + I);
    }
}

template SMatrix DFA<char>::getSparseIdentityMatrix() const;
template Matrix DFA<char>::getIdentityMatrix() const;
template const Matrix& DFA<char>::getNumericMatrix() const;
template void DFA<char>::updateNumericMatrix();
template void DFA<char>::updateSparseMatrix();
template bool DFA<char>::hasFiniteLanguage() const;
template bool DFA<char>::hasSparseLanguage() const;
template void DFA<char>::updateHasEpsilon();
template void DFA<char>::updateNumericVectorQf();
template void DFA<char>::updateNumericVectorQi();
template boost::optional<state_t> DFA<char>::succ(state_t q, char c) const;
