#include "norwegian.h++"

#include <chrono>
#include <boost/format.hpp>
#include <iostream>

int main() {
	std::vector<integer> numbers;
	integer n = 2;
	for (int i = 0; i < 256; ++i) {
		numbers.push_back(n);
		n *= 10;
	}
	std::vector<std::string> encoded;

	using namespace Norwegian;

	std::shared_ptr<DFA<char> > dfa1 = makeDFAfromRegEx("(0|abcdefghijklmnopqrstuvwxyz)*");
	std::shared_ptr<IStringCodec> codec = std::make_shared<DFAStringCodec>(dfa1);

	int i = 0;
	for (auto it = numbers.begin(); it != numbers.end(); ++it, ++i) {
//	for (auto it = numbers.rbegin(); it != numbers.rend(); ++it) {
		const auto n = *it;

		auto startenc = std::chrono::steady_clock::now();
		encoded.emplace_back(codec->encode(n));
		auto stopenc = std::chrono::steady_clock::now();	

		auto startdec = std::chrono::steady_clock::now();
		codec->decode(encoded.back());
		auto stopdec = std::chrono::steady_clock::now();	

		auto diffenc = stopenc - startenc;
		auto diffdec = stopdec - startdec;
		std::cout << boost::format("%d | %d | %d") % i % std::chrono::duration<double, std::milli>(diffenc).count() % std::chrono::duration<double, std::milli>(diffdec).count() << std::endl << std::flush;
	}
	
	return 0;
}
