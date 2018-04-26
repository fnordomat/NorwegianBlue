#include "norwegian.h++"

#include "gtest/gtest.h"

#include <iostream>
#include <boost/optional/optional_io.hpp>

#include <memory>
#include <unordered_set>

std::shared_ptr<DFA<char>> dfa0(new DFA<char>(2, 0, {std::make_tuple(0,'a',1), std::make_tuple(0,'c',0), std::make_tuple(1,'b',0)}, {0}));

std::shared_ptr<DFA<char>> dfa1(new DFA<char>(3, 0, {std::make_tuple(0,'a',1), std::make_tuple(1,'c',2), std::make_tuple(2,'b',0)}, {0}));

std::shared_ptr<DFA<char>> dfa2a = makeDFAfromRegEx("(a|b)*c");
std::shared_ptr<DFA<char>> dfa2b = makeDFAfromRegEx("c(a|b)*");

std::vector<std::string> regexen =
{   ""
    , "a|b"
    , "a|"
    , "|a"
    , "||a"
    , "|ab"
    , "ab|"
    , "(|)*"
    , "\\(e*"
    , "\\(*"
    , "\\)*"
    , "(|)"
    , "(a|(ab*c)*)*|"
    , "()"
    , "()*"
    , "a|b|()*"
    , "()*|(a|b)|()*"
    , "(c|d)|(a|b)"
    , "()*|(a|b)"
    , "(a|b)|()*"
    , "(a|b|()*)c*"
    , "a"
    , "a*"
    , "a**"
    , "()*"
    , "(a|b)cdef(g|(hi|j))()*|()*"
    , "(\\)|\\()\\(\\*\\|\\)(\\\\|(\\)\\)|\\())()*|()*"
    , "(a|b)cd|e(a|(bc|d))()*|()*"
    , "(a|b|()*)c*"
    , "ab"
    , "abc"
    , "a|b"
    , "a|ba"
    , "a|bc" 
    , "(a|b)"
    , "(a|b)c"
    , "aa*"
    , "a|aa*"
    , "a*|aa*"
    , "a**|aa*"
    , "aa*|aa*"
    , "(a|(ab*c)*)*|d"
    , "(a|(ab*c)*|b(a|c))*|d"
};

std::vector<std::string> regexenWhoseLanguageContainsEpsilon =
{   ""
    , "a**"
    , "()*"
    , "(()*)"
    , "a|b|()*"
    , "(a|b)|()*"
    , "(a|b|()*)c*"
    , "a*"
    , "aa*|a*"
    , "(a|(ab*c)*)*|d"
    , "(a|(ab*c)*|b(a|c))*|d"
};

std::vector<std::string> regexenWhoseLanguageDoesNotContainEpsilon =
{   "a"
    , "()"
    , "ab"
    , "abc"
    , "a|b"
    , "a|ba"
    , "a|bc"
    , "(a|b)"
    , "(a|b)c"
    , "aa*"
    , "(a|(ab*c)*)*d|da*"
    , "(a|(ab*c)*|b(a|c))*d|d(b+c)a*"
};

std::vector<std::string> regexenWhoseLanguageIsSparse =
{   "(a|b)cd|e(a|(bc|d))()*|()*"
    , "()*"
    , "a|b"
    , "(a|b|()*)c*"
    , "a"
    , "a*"
    , "ab"
    , "abc"
    , "a*bc"
    , "ab*c"
    , "abc*"
    , "aaaa*"
    , "a|aaaa*"
    , "a*|b*"
    , "a*b*c"
};

std::vector<std::string> regexenWhoseLanguageIsNotSparse =
{   
      "(bb|c)*"
    , "(bb*|c)*"
    , "(a|(ab*c)*)*|d"
    , "(a|(ab*c)*|b(a|c))*|d"
};

std::vector<std::string> not_regexen =
{   "*"
    , "*a"
    , "a|*"
    , "|*"
    , "(a|b"
    , "a|b)c"
    , "(a|(ab*c)*|b(a|c))|*d"
};

std::vector<std::pair<std::string, integer>> pairs0 =
{   {"",0},
    {"c",1},
    {"ab",2},
    {"cc",3},
    {"abc",4},
    {"cab",5},
    {"ccc",6},
    {"abab",7},
    {"abcc",8},
    {"cabc",9},
    {"ccab",10},
    {"cccc",11},
    {"ababc",12},
    {"abcab",13},
    {"abccc",14},
    {"cabab",15},
    {"cabcc",16},
    {"ccabc",17},
    {"cccab",18},
    {"ccccc",19},
    {"ababab",20},
    {"ababcc",21},
    {"abcabc",22},
    {"abccab",23},
    {"abcccc",24},
    {"cababc",25},
    {"cabcab",26},
    {"cabccc",27},
    {"ccabab",28},
    {"ccabcc",29},
    {"cccabc",30},
    {"ccccab",31},
    {"cccccc",32},
    {"abababc",33},
    {"ababcab",34},
};

std::vector<std::pair<std::string, integer>> pairs1 =
{   {"",0},
    {"acb",1},
    {"acbacb",2},
    {"acbacbacb",3},
    {"acbacbacbacb",4},
    {"acbacbacbacbacb",5}
};

// (a+b)c
std::vector<std::pair<std::string, integer>> pairs2a =
{   {"c",0},
    {"ac",1},
    {"bc",2},
    {"aac",3},
    {"bbc",6}
};

std::vector<std::pair<std::string, integer>> pairs2b =
{   {"c",0},
    {"ca",1},
    {"cb",2},
    {"caa",3},
    {"caaa",7}
};

TEST(dfa_test_case, norwegian_test)
{
    DFA<char> dfa(2, 0, {std::make_tuple(0,'a',1), std::make_tuple(1,'b',0)}, {0});
    uint32_t input = 0;
    boost::optional<uint32_t> output = dfa.read(input, "ab");
    EXPECT_EQ(0, output.value_or(-1));
    output = dfa.read(input, "a");
    EXPECT_EQ(1, output.value_or(-1));
    output = dfa.read(input, "b");
    EXPECT_EQ(boost::none, output);

    boost::optional<uint32_t> single_output = dfa.read(0, "ababa");
    EXPECT_EQ(single_output.value_or(-1), 1);
}

TEST(dfa_language_finiteness_test_case, norwegian_test)
{
    std::vector<std::shared_ptr<DFA<char>> >
    finite_language_dfas =
    {   makeDFAfromRegEx("(a|b)c")
        , makeDFAfromRegEx("()*finite")
        , makeDFAfromRegEx("(a|c)b")
    };

    std::vector<std::shared_ptr<DFA<char>> >
    infinite_language_dfas =
    { dfa0, dfa1, dfa2a, dfa2b };

    for (const auto& example : finite_language_dfas) {
        EXPECT_EQ(true, example->hasFiniteLanguage());
    }

    for (const auto& example : infinite_language_dfas) {
        EXPECT_EQ(false, example->hasFiniteLanguage());
    }
}

TEST(dfa_test_case2, norwegian_test)
{
    DFA<char> dfa(3, 0, {std::make_tuple(0,'a',1), std::make_tuple(1,'b',2), std::make_tuple(2,'c',0)}, {0});
    uint32_t input = 0;
    boost::optional<uint32_t> output = dfa.read(input, "");
    EXPECT_EQ(0, output.value_or(-1));
    output = dfa.read(input, "abc");
    EXPECT_EQ(0, output.value_or(-1));
}

TEST(dfa_advanced_test_case, norwegian_test)
{
    DFA<char> dfa(2, 0, {std::make_tuple(0,'a',1), std::make_tuple(0,'c',0), std::make_tuple(1,'b',0)}, {0});
    std::string input = "cabab";
    auto trace = dfa.read_infra(0, input.begin(), input.end());
    EXPECT_EQ(5, trace.size());

    input = "dabab";
    EXPECT_LE(dfa.read_infra(0, input.begin(), input.end()).size(), 1);
}

TEST(encode_test_case2, norwegian_test)
{
    using namespace Norwegian;

    std::shared_ptr<IStringCodec> codec = std::make_shared<DFAStringCodec>(dfa1);

    std::string out;

    for (const auto& pair : pairs1) {
        out = codec->encode(pair.second);
        EXPECT_EQ(out, pair.first);
    }

    codec = std::make_shared<DFAStringCodec>(dfa2a);
    for (const auto& pair : pairs2a) {
        out = codec->encode(pair.second);
        EXPECT_EQ(out, pair.first);
    }

    codec = std::make_shared<DFAStringCodec>(dfa2b);
    for (const auto& pair : pairs2b) {
        out = codec->encode(pair.second);
        EXPECT_EQ(out, pair.first);
    }
}

TEST(encode_test_case, norwegian_test)
{
    using namespace Norwegian;

    std::shared_ptr<IStringCodec> codec = std::make_shared<DFAStringCodec>(dfa0);

    std::string out;

    for (const auto& pair : pairs0) {
        out = codec->encode(pair.second);
        EXPECT_EQ(out, pair.first);
    }
}

TEST(decode_test_case, norwegian_test)
{
    using namespace Norwegian;

    std::shared_ptr<IStringCodec> codec = std::make_shared<DFAStringCodec>(dfa0);

    boost::optional<mpz_class> z1;

    for (const auto& pair : pairs0) {
        z1 = codec->decode(pair.first);
        EXPECT_EQ(z1.value_or(-1), pair.second);
    }
}

TEST(regex_parse_negative_test_case, norwegian_test)
{
    for (const auto& bad_example: not_regexen) {
        boost::optional<RegEx::regex> maybeRex = parseRegEx(bad_example);
        EXPECT_EQ(boost::none, maybeRex);
    }
}

TEST(regex_containing_epsilon_test_case, norwegian_test)
{
    for (const auto& example: regexenWhoseLanguageContainsEpsilon) {
        boost::optional<RegEx::regex> maybeRex = parseRegEx(example);
        EXPECT_NE(boost::none, maybeRex);
        if (maybeRex.is_initialized()) {
            try {
                EXPECT_EQ(true, containsEpsilon(*maybeRex));
            } catch ( boost::bad_get& e ) {
                std::cerr << e.what() << " at " << example << std::endl;;
            }
        } else {
            std::cout << "Failure(parse) with " << example << std::endl;
        }
    }
}

TEST(regex_not_containing_epsilon_test_case, norwegian_test)
{
    for (const auto& example: regexenWhoseLanguageDoesNotContainEpsilon) {
        boost::optional<RegEx::regex> maybeRex = parseRegEx(example);
        EXPECT_NE(boost::none, maybeRex);
        EXPECT_EQ(false, containsEpsilon(maybeRex.get()));
    }
}

TEST(dfa_from_regex_aux_test_case, norwegian_test)
{
    using namespace RegEx;

    std::string example1 = {"(ab*c()*)*|()"};
    regex rex1 = parseRegEx(example1).get();
    Literal la('a'), lb('b'), lc('c');
    unaryOp<kleene> kle1(lb);
    assocOp<concat> con1;
    unaryOp<kleene> kle0;
    kle0.mX = epsilon();
    assocOp<concat> con0;
    con0.mContents.push_back(kle0);
    con1.mContents.push_back(con0);
    con1.mContents.push_back(la);
    con1.mContents.push_back(kle1);
    con1.mContents.push_back(lc);
    unaryOp<kleene> kle2(con1);
    regex rex2 = kle2;
    EXPECT_EQ(simplify(rex1), simplify(rex2));
}

TEST(dfa_from_regex_test_case, norwegian_test)
{
    std::shared_ptr<DFA<char>> dfa;

    std::string example2a = {"()*()*(()*)(ab*c)*"};
    dfa = makeDFAfromRegEx(example2a);
    EXPECT_NE(dfa.get(), nullptr);
    if (dfa.get() != nullptr) {
        // Expect unsuccessful parses 
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "c"));
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "cba"));
// well. this one ends in a non-final state.
//		EXPECT_NE(boost::make_optional(0), dfa->read(dfa->getQi(), "ab"));
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "abab"));
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "babab"));
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "abbcbac"));
        // Expected successful parses 
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), ""));
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "abbcabc"));
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "acabbbbcabc"));
    }

    std::string example = {"a|b"};
    dfa = makeDFAfromRegEx(example);
    EXPECT_NE(dfa.get(), nullptr);
    if (dfa.get() != nullptr) {
        // Expected unsuccessful parses
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "c"));
        // Expected successful parse
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "a"));
    }

    std::string example2 = {"((ab*c)*|b(a|c)*)*"};
    dfa = makeDFAfromRegEx(example2);
    EXPECT_NE(dfa.get(), nullptr);
    if (dfa.get() != nullptr) {
        // Expected incomplete parses
        auto q = dfa->read(dfa->getQi(), "ab");
        EXPECT_NE(q, boost::none);
        EXPECT_EQ(dfa->getQf().cend(), dfa->getQf().find(q.get()));
        // Expected wholly unsuccessful parses
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "c"));
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "cba"));
        EXPECT_EQ(boost::none, dfa->read(dfa->getQi(), "abab"));
        // Expected successful parses
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), ""));
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "babab"));
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "abbcbac"));
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "abbcabc"));
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "bacca"));
        EXPECT_NE(boost::none, dfa->read(dfa->getQi(), "acabbbbcabc"));
    }
}


TEST(dfa_from_regexen_test_case, norwegian_test)
{
    boost::optional<RegEx::regex> maybeRex;
    for (const auto& example: regexen) {
        try{
            maybeRex = parseRegEx(example);
            EXPECT_NE(boost::none, maybeRex);
            if (maybeRex.is_initialized()) {
                std::shared_ptr<DFA<char>> dfa = makeDFAfromRegEx(example);
                EXPECT_EQ(true, dfa.use_count() > 0);
            }
        } catch (const boost::bad_get& e) {
            std::cerr << e.what() << '\n';
            throw;
        }
    }
}

TEST(regex_sparse_test_case, norwegian_test)
{
    for (const auto& example: regexenWhoseLanguageIsSparse) {
        auto dfa = makeDFAfromRegEx(example);
        std::cout << "Sparse? " << example << "\n";
        EXPECT_NE(dfa, nullptr);
        EXPECT_EQ(true, dfa->hasSparseLanguage());
    }
}

TEST(regex_not_sparse_test_case, norwegian_test)
{
    for (const auto& example: regexenWhoseLanguageIsNotSparse) {
        auto dfa = makeDFAfromRegEx(example);
        std::cout << "Sparse? " << example << "\n";
        EXPECT_NE(dfa, nullptr);
        EXPECT_EQ(false, dfa->hasSparseLanguage());
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
