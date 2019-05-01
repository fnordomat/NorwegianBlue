#pragma once

#include <random>
#include "DFA.h++"

std::string sampleRandomWord(DFA<char>& dfa, std::random_device* pRnd, bool samplePrefixes = false);
