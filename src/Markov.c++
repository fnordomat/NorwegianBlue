#include "Markov.h++"

std::string sampleRandomWord(DFA<char>& dfa, std::random_device* pRnd, bool samplePrefixes){
    
    if (dfa.isDead(dfa.getQi()) && !samplePrefixes) {
        throw std::logic_error("Cannot sample from dead input state");
    }
    
    DFA<char>::state_t q = dfa.getQi();
    std::string output;
    for (;;) {
        auto transitions = dfa.getOutEdges(q);
        
        decltype(transitions) filteredTransitions;
        for (const auto tr : transitions) {
            if (samplePrefixes || !dfa.isDead(std::get<1>(tr))) {
                filteredTransitions.insert(tr);
            }
        }
        
        int maxDice = filteredTransitions.size() + ((samplePrefixes || dfa.isFinal(q)) ? 0 : -1);
        
        std::uniform_int_distribution<> distri(0, maxDice);
        size_t diceThrow = static_cast<size_t>(distri(*pRnd));
        
        if (diceThrow == filteredTransitions.size()) {
            return output;
        }
        
        auto it = filteredTransitions.begin();
        std::advance(it, diceThrow);
        auto tr = *it;
        output += std::get<0>(tr);
        q = std::get<1>(tr);
    }
}

