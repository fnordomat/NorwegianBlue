# NorwegianBlue

Beautiful plumage to disguise your encrypted messages.

## Description

NorwegianBlue computes a 1:1 mapping between the natural numbers and the words of a given regular language L. Given a number, the encoding algorithm outputs a string of L. The decoding algorithm unambiguously retrieves the number from the string.

### Why the name?

Houmansadr, Brubaker and Shmatikov assert that the parrot, i.e. the protocol misidentification approach to censorship circumvention, is [dead][1]. We don't necessarily think so. Wherever clear text is transmitted, steganography can potentially be applied.

### Caveat

* It is **not** encryption and provides **no** security by itself.
* It is based on rank-unrank but distinct from [libfte][2] & co.

### Known issues / Project status

* This is an alpha release. Do not rely on it for anything. Please report bugs.
* Bad performance when the automaton has too many states (~ several dozen).

### References / Inspiration

* Paper: https://www.cs.cornell.edu/~shmat/shmat_oak13parrot.pdf
* kpdyer's libfte: https://github.com/kpdyer/libfte
* kpdyer's fteproxy: https://github.com/kpdyer/fteproxy

## How to build

Install dependencies:

      $ sudo apt install libboost-program-options-dev libgmp-dev libeigen3-dev

Compile:

      $ touch config.rpath
      $ autoreconf -iv
      $ ./configure CPPFLAGS=-DNDEBUG CXXFLAGS="-O3" --prefix=/usr/local
      $ make

[1]: https://www.cs.cornell.edu/~shmat/shmat_oak13parrot.pdf
[2]: https://github.com/kpdyer/libfte

