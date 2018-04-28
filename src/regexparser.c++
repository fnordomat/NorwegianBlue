#define BOOST_RESULT_OF_USE_DECLTYPE
#define BOOST_SPIRIT_USE_PHOENIX_V3

#include "declarations.h++"
#include "regexparser.h++"

#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/define_struct.hpp>

#include <boost/fusion/adapted/struct/define_struct_inline.hpp>
#include <boost/fusion/include/define_struct_inline.hpp>

#include <boost/format.hpp>

#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>

namespace RegEx {
std::ostream& operator<<(std::ostream& o, RegEx::epsilon) {
    o << "()*";
    return o;
}
std::ostream& operator<<(std::ostream& o, RegEx::nil) {
    o << "()";
    return o;
}
std::ostream& operator<<(std::ostream& o, RegEx::Literal l) {
    o << l.mC;
    return o;
}
template<>
std::ostream& operator<<(std::ostream& o, RegEx::assocOp<concat> l) {
    if (l.mContents.size() == 0) {
        o << "Concat()";
        return o;
    }
    for (const auto& c: l.mContents) {
        o << c;
    }
    return o;
}
template<>
std::ostream& operator<<(std::ostream& o, RegEx::assocOp<choice> l) {
    o << "(";
    for (auto it = l.mContents.cbegin();
            it != l.mContents.cend();
            ++it) {
        o << *it;
        if (it != l.mContents.cend() - 1) {
            o << "|";
        }
    }
    o << ")";
    return o;
}
template<>
std::ostream& operator<<(std::ostream& o, RegEx::unaryOp<kleene> l) {
    o << "(" << l.mX << ")*";
    return o;
}

template struct assocOp<choice>;
template struct assocOp<concat>;
template struct unaryOp<kleene>;
} //namespace

namespace std {
template<>
struct hash<RegEx::regex> {

    typedef RegEx::regex argument_type;
    typedef std::size_t value_type;

    value_type operator()(const argument_type& r) const {
        value_type hashvalue = r.type().hash_code();
        if (r.type() == typeid(RegEx::epsilon)
                || r.type() == typeid(RegEx::nil)) {
            return hashvalue;
        }
        if (r.type() == typeid(RegEx::Literal)) {
            return hashvalue ^ hash<char>()(boost::get<RegEx::Literal>(r).mC);
        }
        if (r.type() == typeid(RegEx::assocOp<RegEx::choice>)) {            
            const auto& vec = boost::get<RegEx::assocOp<RegEx::choice>>(r).mContents;
            for (const auto& c : vec) {
                hashvalue ^= hash()(c);
            }
            return hashvalue;
        }
        if (r.type() == typeid(RegEx::assocOp<RegEx::concat>)) {
            const auto& vec = boost::get<RegEx::assocOp<RegEx::concat>>(r).mContents;
            for (const auto& c : vec) {
                hashvalue ^= hash()(c);
            }
            return hashvalue;
        }
        if (r.type() == typeid(RegEx::unaryOp<RegEx::kleene>)) {
            return hashvalue ^ hash()(boost::get<RegEx::unaryOp<RegEx::kleene>>(r).mX);
        }
        return hashvalue;
    }
};

/**
 * Must be a congruence w.r.t. operator== equality
 */
template<>
struct hash<std::unordered_set<RegEx::regex>> {
    typedef std::unordered_set<RegEx::regex> argument_type;
    typedef std::size_t value_type;

    value_type operator()(const argument_type& r) const {
        value_type hashvalue = r.size(); //
        return hashvalue;
    }
};
} // namespace std

namespace RegEx {

regex simplify(const regex& r) {
    if (r.type() == typeid(nil)) return r;
    else if (r.type() == typeid(epsilon)) return r;
    else if (r.type() == typeid(Literal)) return r;
    else if (r.type() == typeid(unaryOp<kleene>)) {
        const RegEx::regex& s = simplify(boost::get<unaryOp<kleene>>(r).mX);
        if (s.type() == typeid(unaryOp<kleene>)) return s;
        if (s.type() == typeid(epsilon)) return s;
        if (s.type() == typeid(nil)) return epsilon();
        else return unaryOp<kleene>(s);
    }
    else if (r.type() == typeid(assocOp<concat>)) {
        const auto& contents = boost::get<assocOp<concat>>(r).mContents;
        if (contents.size() == 0)
            return epsilon();
        if (contents.size() == 1)
            return simplify(contents.at(0));

        std::vector<RegEx::regex> vec = {};

        for (const auto& s: contents) {
            RegEx::regex ss = simplify(s);
            if (ss.type() == typeid(assocOp<concat>)) {
                for (const regex& t : boost::get<assocOp<concat>>(ss).mContents) {
                    // shouldn't be necessary:
                    RegEx::regex tt = simplify(t);
                    if (tt.type() == typeid(nil)) {
                        return nil();
                    }
                    if (tt.type() == typeid(epsilon)) {
                        continue;
                    }
                    vec.emplace_back(tt);
                }
            } else {
                if (ss.type() == typeid(nil)) {
                    return nil();
                }
                if (ss.type() == typeid(epsilon)) {
                    continue;
                }
                vec.emplace_back(ss);
            }
        }

        if (vec.size() == 0) {
            return epsilon();
        }
        if (vec.size() == 1) {
            return vec.at(0);
        }

        assocOp<concat> result;
        result.mContents = vec;
        return result;
    }
    else if (r.type() == typeid(assocOp<choice>)) {

        const auto& contents = boost::get<assocOp<choice>>(r).mContents;
        if (contents.size() == 0)
            return nil();
        if (contents.size() == 1)
            return simplify(contents.at(0));

        std::set<RegEx::regex, RegEx::RegexComparator> set = {};

        for (const auto& s: contents) {
            RegEx::regex ss = simplify(s);
            if (ss.type() == typeid(assocOp<choice>)) {
                for (const regex& t : boost::get<assocOp<choice>>(ss).mContents) {
                    RegEx::regex tt = simplify(t);
                    if (tt.type() == typeid(nil)) {
                        continue;
                    }
                    set.emplace(tt);
                }
            } else {
                if (ss.type() == typeid(nil)) {
                    continue;
                }
                set.emplace(ss);
            }
        }

        if (set.size() == 0) {
            return nil();
        }
        if (set.size() == 1) {
            return *set.begin();
        }

        assocOp<choice> result;
        std::copy(set.cbegin(), set.cend(),
                  std::back_inserter(result.mContents));
        return result;
    }
    throw std::exception();
}

/**
 * Return epsilonClosure(r) - {r}
 */
std::unordered_set<regex> epsilonReachable(const regex& r) {
    if (r.type() == typeid(nil)) {
        return {};
    }
    else if (r.type() == typeid(epsilon)) {
        return {epsilon()};
    }
    else if (r.type() == typeid(Literal)) {
        return {};
    }
    else if (r.type() == typeid(unaryOp<kleene>))
    {
        std::unordered_set<regex> result = {epsilon()};
        auto tmpResult = epsilonReachable(boost::get<unaryOp<kleene>>(r).mX);
        for (const auto& s: tmpResult) {
            assocOp<concat> x;
            x.mContents.push_back(s);
            x.mContents.push_back(r);
            result.emplace(simplify(x));
        }
        return result;
    }
    else if (r.type() == typeid(assocOp<choice>)) {

        const auto& contents = boost::get<assocOp<choice>>(r).mContents;

        if (contents.size() == 1) { // cannot occur with simplified input
            return epsilonReachable(contents.at(0));
        }

        std::unordered_set<regex> result;
        for (const auto& sub: contents) {
            auto subresult = epsilonReachable(sub);
            result.insert(subresult.begin(), subresult.end());
        }
        return result;
    }
    else if (r.type() == typeid(assocOp<concat>)) {

        const auto& contents = boost::get<assocOp<concat>>(r).mContents;

        if (contents.size() == 1) { // cannot occur with simplified input
            return epsilonReachable(contents.at(0));
        }

        std::unordered_set<regex> result = {};
        for (size_t i = 0; i < contents.size(); ++i) {
            if (containsEpsilon(contents.at(i))) {
                result.insert(assocOp<concat>(contents.begin()+i+1, contents.end()));
            } else {
                for (const auto& s: epsilonReachable(contents.at(i))) {
                    assocOp<concat> tmp;
                    tmp.mContents.push_back(s);
                    std::copy(contents.begin()+i+1, contents.end(),
                              std::back_inserter(tmp.mContents));
                    result.emplace(simplify(tmp));
                }
                break;
            }
        }
        return result;
    }
    throw std::exception();
}

std::unordered_map<char, std::unordered_set<regex>>
computeSuccessors(const regex& r) {

    if (r.type() == typeid(nil)) return {};
    else if (r.type() == typeid(epsilon)) return {};
    else if (r.type() == typeid(Literal))
        return {std::make_pair(boost::get<Literal>(r).mC,
                               std::unordered_set<regex>{epsilon()})
               };
    else if (r.type() == typeid(assocOp<choice>)) {
        std::unordered_map<char, std::unordered_set<regex>> result = {};
        for (const auto& sub: boost::get<assocOp<choice>>(r).mContents) {
            const auto& subSuccessors = computeSuccessors(sub);
            for (const auto& kv : subSuccessors) {
                result[kv.first].insert(kv.second.begin(),
                                        kv.second.end());
            }
        }

        return result;
    }
    else if (r.type() == typeid(assocOp<concat>)) {
        const assocOp<concat>& rex = boost::get<assocOp<concat>>(r);
        if (rex.mContents.size() == 0) {
            return {}; // an empty product is equivalent to epsilon
        } else {
            std::unordered_map<char, std::unordered_set<regex>> result = {};
            for (const auto& kv : computeSuccessors(rex.mContents.at(0))) {
                assocOp<concat> x;
                x.mContents.push_back(
                    assocOp<choice>(kv.second.begin(),
                                    kv.second.end()));
                std::copy(rex.mContents.begin()+1,
                          rex.mContents.end(),
                          std::back_inserter(x.mContents));
                result[kv.first].insert(x);

            }
            return result;
        }
    }
    else if (r.type() == typeid(unaryOp<kleene>)) {
        const unaryOp<kleene>& rex = boost::get<unaryOp<kleene>>(r);
        std::unordered_map<char, std::unordered_set<regex>> result = {};
        for (const auto& kv: computeSuccessors(rex.mX)) {
            for (const auto& s : kv.second) {
                assocOp<concat> x;
                x.mContents.push_back(s);
                x.mContents.push_back(rex);
                result[kv.first].insert(x);
            }
        }
        return result;
    }
    throw std::exception();
}

bool containsEpsilon(const regex& r) {
    if (r.type() == typeid(nil)) return false;
    else if (r.type() == typeid(epsilon)) return true;
    else if (r.type() == typeid(Literal)) return false;
    else if (r.type() == typeid(unaryOp<kleene>)) return true;
    else if (r.type() == typeid(assocOp<concat>)) {
        for (const auto& sub: boost::get<assocOp<concat>>(r).mContents) {
            if (!containsEpsilon(sub)) return false;
        }
        return true;
    }
    else if (r.type() == typeid(assocOp<choice>)) {
        for (const auto& sub: boost::get<assocOp<choice>>(r).mContents) {
            if (containsEpsilon(sub)) return true;
        }
        return false;
    }
    throw std::exception();
}

bool RegexComparator::operator()(const regex& a, const regex& b) const {
    if (a.which() < b.which()) return true;
    if (a.which() > b.which()) return false;
    if (a.type() == typeid(nil)) return true;
    if (a.type() == typeid(epsilon)) return true;
    if (a.type() == typeid(Literal)) {
        return boost::get<Literal>(a).mC <
               boost::get<Literal>(b).mC;
    }
    if (a.type() == typeid(unaryOp<kleene>)) {
        return
            RegexComparator()(boost::get<unaryOp<kleene>>(a).mX,
                              boost::get<unaryOp<kleene>>(b).mX);
    }
    if (a.type() == typeid(assocOp<concat>)) {
        const auto& vecA = boost::get<assocOp<concat>>(a).mContents;
        const auto& vecB = boost::get<assocOp<concat>>(b).mContents;
        if (vecA.size() < vecB.size()) {
            return true;
        }
        if (vecB.size() < vecA.size()) {
            return false;
        }
        auto itA = vecA.cbegin(), itB = vecB.cbegin();
        for (; itA != vecA.cend(); ++itA, ++itB) {
            if (RegexComparator()(*itA, *itB)) return true;
            if (RegexComparator()(*itB, *itA)) return false;
        }
        return false;
    }
    if (a.type() == typeid(assocOp<choice>)) {
        const auto& vecA = boost::get<assocOp<choice>>(a).mContents;
        const auto& vecB = boost::get<assocOp<choice>>(b).mContents;
        if (vecA.size() < vecB.size()) {
            return true;
        }
        if (vecB.size() < vecA.size()) {
            return false;
        }
        auto itA = vecA.cbegin(), itB = vecB.cbegin();
        for (; itA != vecA.cend(); ++itA, ++itB) {
            if (RegexComparator()(*itA, *itB)) return true;
            if (RegexComparator()(*itB, *itA)) return false;
        }
        return false;
    }
    return false; // make compiler happy
}
} // namespace


namespace boost {
namespace spirit {
namespace traits
{
template<> struct is_container<RegEx::assocOp<RegEx::choice>> : boost::mpl::true_ {};

template<> struct container_value<RegEx::assocOp<RegEx::choice>>
{
    typedef RegEx::regex type;
};

template<>
struct push_back_container<RegEx::assocOp<RegEx::choice>, RegEx::regex>
{
    static bool call(RegEx::assocOp<RegEx::choice>& c, const RegEx::regex& val)
    {
        c.mContents.push_back(val);
        return true;
    }
};
}
}
}

boost::optional<RegEx::regex> parseRegEx(const std::string& input) {

    if (input == "") {
        return boost::make_optional(RegEx::regex(RegEx::epsilon()));
    }

    using namespace boost::spirit::qi;

    using iter_t = std::string::const_iterator;

    rule<iter_t, RegEx::nil()> emptyparenp;
    rule<iter_t, RegEx::assocOp<RegEx::choice>()> choicep;
    rule<iter_t, RegEx::assocOp<RegEx::concat>()> concatp;
    rule<iter_t, RegEx::unaryOp<RegEx::kleene>()> kleenep;
    rule<iter_t, RegEx::Literal()> literalp;
    rule<iter_t, RegEx::Literal()> escliteralp;
    rule<iter_t, RegEx::Literal()> escxnumberp;
    rule<iter_t, RegEx::regex()> delimregexp;
    rule<iter_t, RegEx::regex()> nakedregexp;
    // Auxiliary category for subexpressions of choice (delimited or binding stronger than "|")
    rule<iter_t, RegEx::regex()> xchoicep;

    // Careful, avoid infinite left recursion!
    emptyparenp = eps >> omit [ lit("()") ];
    escliteralp = lit('\\') >> boost::spirit::ascii::char_;
    escxnumberp = lit('\\') >> lit('x') >> int_parser<unsigned char, 16, 2, 2>();
    literalp = escxnumberp | escliteralp | ~char_("\\\\()|*");
    kleenep = (emptyparenp | literalp | lit('(') >> nakedregexp >> lit(')')) >> +lit('*');
    xchoicep = concatp | delimregexp | eps;
    concatp = +delimregexp;
    choicep = xchoicep % lit('|');
    delimregexp = kleenep | emptyparenp | literalp | lit('(') >> nakedregexp >> lit(')');
    nakedregexp = choicep | concatp | delimregexp;

#ifdef BOOST_SPIRIT_DEBUG
    BOOST_SPIRIT_DEBUG_NODE( emptyparenp );
    BOOST_SPIRIT_DEBUG_NODE( escliteralp );
    BOOST_SPIRIT_DEBUG_NODE( literalp );
    BOOST_SPIRIT_DEBUG_NODE( kleenep );
    BOOST_SPIRIT_DEBUG_NODE( delimregexp );
    BOOST_SPIRIT_DEBUG_NODE( xchoicep );
    BOOST_SPIRIT_DEBUG_NODE( concatp );
    BOOST_SPIRIT_DEBUG_NODE( choicep );
    BOOST_SPIRIT_DEBUG_NODE( nakedregexp );
#endif

    RegEx::regex r;

    iter_t it = input.cbegin();
    const iter_t end = input.cend();
    bool status = parse( it, end, nakedregexp | delimregexp, r );

    size_t remaining = end - it;

#ifdef DEBUG_STDOUT
    std::cout << "remaining: " << remaining << "/" << input.size() << ", status: " << status << std::endl;
    std::cout << "trailing: ";
    std::cout << std::string(it, end) << std::endl;
#endif

    if (!status) {
        return boost::none;
    }
    // incomplete parse:
    if (remaining > 0) {
        return boost::none;
    }

    return boost::make_optional(r);
}

std::shared_ptr<DFA<char>> makeDFAfromRegEx(const std::string& input) {
    boost::optional<RegEx::regex> maybeRex = parseRegEx(input);
    if (!maybeRex) {
        return std::shared_ptr<DFA<char>>(nullptr);
    }

    // If our DFA is going to have more than INT_MAX states, we're in trouble anyway
    typedef uint32_t state_t;
    typedef std::unordered_set<RegEx::regex> actual_powerstate_t;
    typedef state_t powerstate_idx_t;

    // Regexen and their Epsilon closures -> these are the DFA's states.
    std::vector<actual_powerstate_t> remainders;
    // Initially, we are in one of these start NFA states:
    remainders.push_back({simplify(maybeRex.get())});
    auto reachable = epsilonReachable(simplify(maybeRex.get()));
    remainders.back().insert(reachable.begin(), reachable.end());

    // Mapping of unions of epsilon closures (sets of regexes, NFA states) to numbers in the new DFA.
    std::unordered_map<actual_powerstate_t, powerstate_idx_t> dfaStates;

#ifdef DOT_DEBUG
    std::cout << "[DOT] digraph {\n";
#endif

    // Initialize reverse mapping
    dfaStates[remainders.back()] = 0;

    // Transition mapping of regexes (ignoring epsilon)
    using transitionMap_t = std::unordered_map<RegEx::regex,
          std::unordered_map<char, std::unordered_set<RegEx::regex>> >;
    transitionMap_t transitionMap;

    std::set<powerstate_idx_t> dfaExplore = {0};
    signed long max_explored = -1; // initially, state 0 is unexplored.

    std::unordered_map<powerstate_idx_t, std::unordered_map<char, powerstate_idx_t>> deltaMap;
    std::vector<std::tuple<state_t, char, state_t>> deltaList;
    std::vector<state_t> finalList;

    for (std::set<powerstate_idx_t>::iterator it = dfaExplore.begin()
            ; !dfaExplore.empty(); it = dfaExplore.erase(it),
            max_explored++) {
        powerstate_idx_t idx_q = *it;

        std::unordered_map<char, actual_powerstate_t> temporaryMap;

        auto Q = remainders[idx_q];

#ifdef DOT_DEBUG
        std::stringstream nodecontents;

        for (const auto& x : Q) {
            nodecontents << x << "\\n";
        }
#endif

        if (std::find_if(Q.cbegin(), Q.cend(),
        [](const RegEx::regex& rex) -> bool {
        return containsEpsilon(rex);
        }) != Q.cend()) {
#ifdef DOT_DEBUG
            std::cout << boost::format("[DOT]   node%d [label=\"%s\", color=blue];\n") % idx_q % nodecontents.str();
#endif
            finalList.push_back(idx_q);
        }
        else {
#ifdef DOT_DEBUG
            std::cout << boost::format("[DOT]   node%d [label=\"%s\"];\n") % idx_q % nodecontents.str();
#endif
        }

        if ((signed long)idx_q < max_explored) {
            continue;
        }

        for (const RegEx::regex& rex : remainders[idx_q]) {

            transitionMap_t::iterator mapIt = transitionMap.find(rex);

            if (mapIt == transitionMap.end()) {

                mapIt = transitionMap.insert(std::make_pair(rex,
                                             computeSuccessors(rex))) . first;

                for (const auto& kv: mapIt->second) {
                    const auto symbol = kv.first;
                    temporaryMap[symbol].insert(kv.second.cbegin(),
                                                kv.second.cend());
                }
            } else {

                for (const auto& kv: computeSuccessors(rex)) {
                    const auto symbol = kv.first;
                    // Cache the result for next time rex is encountered:
                    mapIt->second[symbol].insert(kv.second.cbegin(),
                                                 kv.second.cend());
                    temporaryMap[symbol].insert(kv.second.cbegin(),
                                                kv.second.cend());
                }
            }
        }

        for (const auto& kv: temporaryMap) {
            const char symbol = kv.first;

            const auto& successors = kv.second;
            actual_powerstate_t p;
            for (const auto& successor: successors) {
                p.insert(simplify(successor));
                auto reachable = epsilonReachable(simplify(successor));
                p.insert(reachable.begin(), reachable.end());
            }

            const auto itsucc = dfaStates.find(p);

            if (itsucc == dfaStates.cend()) {
                auto idx = remainders.size();

                remainders.emplace_back(p);
                dfaStates[p] = idx;
                deltaMap[idx_q][symbol] = idx;
                dfaExplore.insert(idx);

#ifdef DOT_DEBUG
                std::cout << boost::format("[DOT]   node%d -> node%d [label=\"%s\"];\n") % idx_q % idx % symbol;
#endif

                deltaList.push_back(std::make_tuple(idx_q, symbol, idx));
            } else {

                deltaMap[idx_q][symbol] = itsucc->second;
#ifdef DOT_DEBUG
                std::cout << boost::format("[DOT]   node%d -> node%d [label=\"%s\"];\n") % idx_q % itsucc->second % symbol;
#endif

                deltaList.push_back(std::make_tuple(idx_q, symbol, itsucc->second));
            }
        }
    } // exploration

#ifdef DOT_DEBUG
    std::cout << "[DOT] }\n";
#endif

    return std::make_shared<DFA<char>>(
        dfaStates.size(), 0,
        deltaList.begin(), deltaList.end(),
        finalList.begin(), finalList.end());
}

