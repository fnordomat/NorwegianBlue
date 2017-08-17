#include "norwegian.h++"

#include <iostream>
#include <boost/program_options.hpp>

int main(int argc, char** argv) {

	namespace getopt = boost::program_options;
	getopt::variables_map vm;

    try
    {
        getopt::options_description desc("Program Usage", 1024, 512);

		std::string regex;

        desc.add_options()
          ("help,h",   "produce help message")
          ("regex,r",  getopt::value<std::string>(&regex), "set the regex");

        getopt::store(getopt::parse_command_line(argc, argv, desc), vm);
        getopt::notify(vm);
		
        if (vm.count("help"))
        {
            std::cout << desc << "\n";
            return false;
        }
		
        // There must be an easy way to handle the relationship between the
        // option "help" and "host"-"port"-"config"
        if (vm.count("regex"))
        {
            std::cout << "regex: " << vm["regex"].as<std::string>() << "\n";
        }
        else
        {
            std::cout << "regex is required!" << "\n";
            return false;
        }
	}
	
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    catch(...)
    {
        std::cerr << "Unknown error!" << "\n";
        return false;
    }

	std::shared_ptr<DFA<char> > dfa; 

	dfa = makeDFAfromRegEx(vm["regex"].as<std::string>());
	if (!dfa) {
		std::cerr << "Regex does not parse, quitting." << std::endl;
		return -2;
	}
	if (dfa->hasFiniteLanguage()) {
		std::cerr << "Warning, your language is finite." << std::endl;
	}
	// TODO: emit a warning when language is sparse

	std::shared_ptr<Norwegian::IStringCodec> codec
		= std::make_shared<Norwegian::DFAStringCodec>(dfa);

	integer input;
	while (std::cin >> input) {
		std::cout << codec->encode(input) << std::endl;
	}

	return 0;
}
