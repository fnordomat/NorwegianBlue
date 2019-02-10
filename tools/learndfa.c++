#include "DFA.h++"
#include "Learner.h++"

#include <iostream>
#include <boost/program_options.hpp>

//
#include "Markov.h++"
#include <random>

int main(int argc, char** argv) {

    namespace getopt = boost::program_options;
    getopt::variables_map optionVars;
    double kappa = 1.0;
    
    try
    {
        getopt::options_description desc("Program Usage", 1024, 512);

        std::string regex;

        desc.add_options()
        ("help,h", "produce help message")
        ("kappa,k", boost::program_options::value<double>(&kappa)->default_value(1.0), "data weight (default 1.0)");

        getopt::store(getopt::parse_command_line(argc, argv, desc), optionVars);
        getopt::notify(optionVars);

        if (optionVars.count("help"))
        {
            std::cout << desc << "\n";
            return -1;
        }
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return -1;
    }
    catch(...)
    {
        std::cerr << "Unknown error!" << "\n";
        return -1;
    }

    std::shared_ptr<Norwegian::DFALearner<char>> learner
      = std::make_shared<Norwegian::DFALearner<char>>();
    learner->setDataWeight(kappa);

    std::cout << "LearnDFA input:\n";
      
    std::string input;
    while (std::getline(std::cin, input)) {
        learner->positiveExample(input);
	//        std::cout << learner->getStatus() << std::endl;
    }

    std::shared_ptr<DFA<char>> dfa = learner->makeDFA();

    if (dfa->hasFiniteLanguage()) {
        std::cerr << "Warning, your language is finite." << std::endl;
    }

    std::cout << dfa->serialize();
    
    std::cout << "\n";
    std::cout << "------------------" << std::endl;
    std::cout << "examples:" << std::endl;

    std::random_device gen{"default"};
    for (int i=0; i<25; ++i) {
        std::string testWord = sampleRandomWord(*dfa, &gen, false);
        std::cout << testWord << std::endl;
    }
    
    return 0;
}
