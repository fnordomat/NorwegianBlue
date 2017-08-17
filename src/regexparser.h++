#pragma once

#include "DFA.h++"

#include <boost/variant.hpp>
#include <unordered_map>
#include <unordered_set>

namespace RegEx {
struct nil {
    friend std::ostream& operator<<(std::ostream& o, const nil);
    bool operator==(const nil&) const {
        return true;
    }
};
struct epsilon {
    friend std::ostream& operator<<(std::ostream& o, const epsilon);
    bool operator==(const epsilon&) const {
        return true;
    }
};
struct choice {};
struct concat {};
struct kleene {};
struct Literal {
    explicit Literal() {};
    explicit Literal(char c):mC(c) {};
    char mC;
    friend std::ostream& operator<<(std::ostream& o, const RegEx::Literal l);
    bool operator==(const Literal& other) const {
        return mC == other.mC;
    }
};
template<typename tag> struct assocOp;
template<typename tag> struct unaryOp;

typedef boost::variant<
nil, epsilon, Literal,
     boost::recursive_wrapper<assocOp<choice> >,
     boost::recursive_wrapper<assocOp<concat> >,
     boost::recursive_wrapper<unaryOp<kleene> > > regex;

template<typename tag>
std::ostream& operator<<(std::ostream& o, const assocOp<tag>);
template<typename tag>
std::ostream& operator<<(std::ostream& o, const unaryOp<tag>);

template<typename tag> struct assocOp {
    assocOp() : mContents {} {}
    template <typename InputIterator>
    assocOp(const InputIterator inputBegin,
            const InputIterator inputEnd)
        : mContents(inputBegin, inputEnd)
    {
    }
// this would be wrong:  assocOp(const regex& x) : mContents{x} {}
    std::vector<regex> mContents;
    friend std::ostream& operator<< <>(std::ostream& o, const assocOp<tag> l);
    bool operator==(const assocOp<tag>& other) const {
        return mContents == other.mContents;
    }
    // required for debug output:
    bool empty() const {
        return mContents.empty();
    }
    typedef regex value_type;
    typedef std::vector<regex>::const_iterator const_iterator;
    const_iterator insert(const_iterator it, const value_type& val) {
        return mContents.insert(it, val);
    }
    const_iterator begin() const {
        return mContents.cbegin();
    }
    const_iterator end()   const {
        return mContents.cend();
    }
};
template<typename tag> struct unaryOp {
    unaryOp() {}; // why?
    explicit unaryOp(const regex& x) : mX(x) {};
    regex mX;
    friend std::ostream& operator<< <>(std::ostream& o, const unaryOp<tag> l);
    bool operator==(const unaryOp<tag>& other) const {
        return mX == other.mX;
    }
};

/**
 * @return true iff \epsilon \in Language(r)
 */
bool containsEpsilon(const regex& r);

/**
 * return something close to {(a, \partial_a(r)), ...}
 * except epsilon-steps are excluded, so
 * XY -> c(X)Y only
 */
std::unordered_map<char, std::unordered_set<regex> >
computeSuccessors(const regex& r);

struct RegexComparator {
    bool operator()(const regex& a, const regex& b) const;
};

/**
 * Apply a number of obvious simplifications to r
 */
regex simplify(const regex& r);
} // namespace

/**
 * @return a valid shared_ptr to a RegEx, or a nullptr-shared_ptr in case of failure
 */
boost::optional<RegEx::regex> parseRegEx(const std::string& input);

/**
 * @return a valid shared_ptr to a DFA, or a nullptr-shared_ptr in case of failure
 */
std::shared_ptr<DFA<char> > makeDFAfromRegEx(const std::string& input);

