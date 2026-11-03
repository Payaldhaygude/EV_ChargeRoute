# Linux / macOS / MSYS2 convenience Makefile (CMake works too - see README)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
SRC      := $(wildcard src/*.cpp)
SFML     := -lsfml-graphics -lsfml-window -lsfml-system

all: ev_chargeroute

# full build: 2D simulation window + CLI
ev_chargeroute: $(SRC) $(wildcard include/*.h)
	$(CXX) $(CXXFLAGS) -Iinclude $(SRC) -o $@ $(SFML)

# algorithms only (no SFML needed)
cli: $(SRC) $(wildcard include/*.h)
	$(CXX) $(CXXFLAGS) -DEV_NO_GUI -Iinclude $(SRC) -o ev_chargeroute_cli

# run the 5 test scenarios from the roadmap
test: cli
	./ev_chargeroute_cli --test

clean:
	rm -f ev_chargeroute ev_chargeroute_cli

.PHONY: all cli test clean
