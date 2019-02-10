#include "aux.h++"

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

Ternary::Ternary(const mpz_class& i) {
    if (i == 0) value = T::Zero;
    else if (i == 1) value = T::One;
    else if (i >= 2) value = T::Many;
    else value = T::Undef;
}

Ternary::Ternary(int i) {
    if (i == 0) value = T::Zero;
    else if (i == 1) value = T::One;
    else if (i >= 2) value = T::Many;
    else value = T::Undef;
}

bool Ternary::operator>(const Ternary& other) const {
    return other.operator<(*this);
}

bool Ternary::operator<(const Ternary& other) const {
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

Ternary Ternary::operator*(const Ternary& other) const {
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

const Ternary& Ternary::operator+=(const Ternary& other) {
    value = operator+(other).value;
    return *this;
}

bool Ternary::operator==(const Ternary& other) const {
    return value == other.value;
}

bool Ternary::operator==(const Ternary::T& otherValue) const {
    return value == otherValue;
}

Ternary Ternary::operator+(const Ternary& other) const {
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
	break;
    case T::Many:
        switch (other.value) {
        case T::Undef: return T::Undef;
        default:    return T::Many;
        }
    }
    return T::Undef;
}

