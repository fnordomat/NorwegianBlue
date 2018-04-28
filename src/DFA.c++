#include "DFA.h++"

#ifdef DEBUG_STDOUT
#include <iostream>
#endif

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

/**
 * @brief Numeric class that allows us to count "one, two, many"
 */
struct Ternary {
public:
    enum class T {
        Zero,
        One,
        Many,
        Undef
    };
    Ternary() : value(T::Zero) {}
    Ternary(T t) : value(t) {}
    Ternary(const mpz_class& i) {
        if (i == 0) value = T::Zero;
        else if (i == 1) value = T::One;
        else if (i >= 2) value = T::Many;
        else value = T::Undef;
    }
    Ternary(int i) {
        if (i == 0) value = T::Zero;
        else if (i == 1) value = T::One;
        else if (i >= 2) value = T::Many;
        else value = T::Undef;
    }
    bool operator>(const Ternary& other) const {
      return other.operator<(*this);
    }
    bool operator<(const Ternary& other) const {
        switch (value) {
        case T::Undef:
            return false;
        case T::Zero:
            switch (other.value) {
            case T::Zero:
            case T::Undef:
                return false;
            default:
                return true;
            }
        case T::One:
            switch (other.value) {
            case T::Many:
                return true;
            default:
                return false;
            }
        case T::Many:
            return false; // sic
        }
        return false;
    }
    Ternary operator*(const Ternary& other) const {
        switch (value) {
        case T::Undef:
            return T::Undef;
        case T::Zero:
            switch (other.value) {
            case T::Undef:
                return T::Undef;
            default:
                return T::Zero;
            }
        case T::One:
            return other.value;
        case T::Many:
            switch (other.value) {
            case T::One:
            case T::Many:
                return T::Many;
            case T::Zero:
                return T::Zero;
            case T::Undef:
                return T::Undef;
            }
        }
        return T::Undef;
    }
    const Ternary& operator+=(const Ternary& other) {
        value = operator+(other).value;
        return *this;
    }
    bool operator==(const Ternary& other) const {
        return value == other.value;
    }
    bool operator==(const Ternary::T& otherValue) const {
        return value == otherValue;
    }
    Ternary operator+(const Ternary& other) const {
        switch (value) {
        case T::Undef:
            return T::Undef;
        case T::Zero:
            return other.value;
        case T::One:
            switch (other.value) {
            case T::Zero:
                return T::One;
            case T::One:
            case T::Many:
                return T::Many;
            case T::Undef: return T::Undef;
            }
        case T::Many:
            switch (other.value) {
            case T::Undef: return T::Undef;
            default:    return T::Many;
            }
        }
        return T::Undef;
    }
    T value;
    friend std::ostream& operator<<(std::ostream& o, const Ternary& t) {
        switch (t.value) {
        case T::Undef: o << "T"; break;
        case T::Zero:  o << "0"; break;
        case T::One:   o << "1"; break;
        case T::Many:  o << "M"; break;
        }
        return o;
    }
};

namespace Eigen {
template<> struct NumTraits<Ternary>
    : GenericNumTraits<Ternary>
{

  enum {
    IsComplex = 0,
    IsInteger = 1,
    IsSigned = 0,
    RequireInitialization = 0,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1
  };

   static inline int digits10() { return 0; }
 
private:
    static inline Ternary epsilon();
    static inline Ternary dummy_precision();
    static inline Ternary lowest();
    static inline Ternary highest();
    static inline Ternary infinity();
    static inline Ternary quiet_NaN();
};
}

typedef Eigen::Matrix<Ternary,Eigen::Dynamic,Eigen::Dynamic> TMatrix;
typedef Eigen::Matrix<Ternary,Eigen::Dynamic,1> TVector;
typedef Eigen::Matrix<Ternary,1,Eigen::Dynamic> TRowVector;

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
