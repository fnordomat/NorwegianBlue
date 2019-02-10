#include "Learner.h++"

#include <ctime>

#include "Markov.h++"
#include <random>
#include <vector>

namespace Norwegian {

template<class Char>
std::shared_ptr<DFA<Char>> DFALearner<Char>::makeDFA() {
    Trie<Char> t;
    
    for (const auto& example : mExamples) {
        t.insert(example.c_str(), example.size());
    }
    
#ifdef DEBUG_STDOUT
    std::cout << "Input trie has " << t.getNumStates() << " states, " << t.getNumFinalStates() << " final\n";
#endif
    
    auto delta = t.getTransitionTuples();
    auto qf = t.getFinalStates();
    
    auto trieDFA = std::make_shared<DFA<Char>>(t.getNumStates(), 0, 
        delta.cbegin(), delta.cend(), qf.cbegin(), qf.cend()
    );
    
    // Greedily merge states to minimize MDL (at each iteration, merge 2 randomly chosen states and merge all states that must be merged to obtain a DFA again, i.e. if {q1,q2} are merged and q1 |a> q3 and q2 |a> q4 then {q3,q4} must also me merged)
    
    auto dfa = std::make_shared<DFA<Char>>();
    *dfa = *trieDFA;
    auto result = dfa->computeMDL(mExamples);
    double bestMDL = std::get<0>(result);
#ifdef DEBUG_STDOUT
    std::cout << "MDL " << bestMDL << ".\n";
#endif
    
    bool done = false;
    
    while (!done) {
goOn:
        done = true;
        
        std::vector<std::pair<uint32_t, uint32_t>> tryThese;
        
        uint32_t numQ = dfa->numStates();
        for (uint32_t i=0; i<numQ; ++i) {
            for (uint32_t j=i+1; j<numQ; ++j) {
                tryThese.push_back({i,j});
            }
        }
        
        if (tryThese.size() == 0) { done=true; break; }
        
        double fraction = 0.2;
        
        std::random_device gen{"default"};
        std::uniform_int_distribution<> distri(0, tryThese.size());
        for (uint32_t i=0; i<tryThese.size(); ++i) 
        {
            uint32_t diceThrow = static_cast<uint32_t>(distri(gen));
            uint32_t iprime = diceThrow;
            uint32_t x = tryThese[iprime].first;
            tryThese[iprime].first = tryThese[i].first;
            tryThese[i].first = x;
            x = tryThese[iprime].second;
            tryThese[iprime].second = tryThese[i].second;
            tryThese[i].second = x;
            
        };
        
        int k = 0;
        
        for (auto ij: tryThese) {
            if (tryThese.size() > 10 && (k > std::floor(fraction * tryThese.size()))) { break; }
            k++;
            uint32_t i = ij.first;
            uint32_t j = ij.second;
            std::cout << "Trying " << i << "," << j << std::endl;
                
            auto newdfa = std::make_shared<DFA<Char>>();
            *newdfa = *dfa;
            
            try {
                newdfa->mergeStates({i,j});
            }
            catch (const std::out_of_range &e) {
                // Would be a bug
                std::cerr << "Unfortunately, an " << e.what() << " exception occurred. Input was:" << std::endl;
                std::cout << i << "," << j << std::endl;
                std::cout << dfa->serialize() << std::endl;
            }
            
            auto result = newdfa->computeMDL(mExamples, kappa);
            
            double modelBits = std::get<1>(result);
            double dataBits = std::get<2>(result);
            
            double mdl = std::get<0>(result);
            if (mdl < bestMDL) { // greedy
                bestMDL = mdl;
                dfa = newdfa;
#ifdef DEBUG_STDOUT
             std::cout << boost::format("MDL = %s%f%s (%f + %f * %f)") % ANSI_BOLD % (modelBits + kappa * dataBits) % ANSI_RESET % modelBits % kappa % dataBits << std::endl;
#endif
                goto goOn;
            } else {
#ifdef DEBUG_STDOUT
            std::cout << boost::format("MDL = %f (%f + %f * %f)") % (modelBits + kappa * dataBits) % modelBits % kappa % dataBits << std::endl;
#endif
            }
        }
     
        done = true;
    }
        
    dfa->updatePrecomputedProperties();
    return dfa;
}

template class DFALearner<char>;

} // namespace Norwegian
