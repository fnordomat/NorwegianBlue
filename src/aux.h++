#pragma once

#include "declarations.h++"

/**
 * @brief Computes the gcd of two integers.
 */
mpz_class euclid(mpz_class a, mpz_class b);

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
    Ternary(const mpz_class& i);
    Ternary(int i);
    bool operator>(const Ternary& other) const;
    bool operator<(const Ternary& other) const;
    Ternary operator*(const Ternary& other) const;
    const Ternary& operator+=(const Ternary& other);
    bool operator==(const Ternary& other) const;
    bool operator==(const Ternary::T& otherValue) const;
    Ternary operator+(const Ternary& other) const;
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

