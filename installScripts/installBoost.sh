#!/usr/bin/env bash
set -e

## Boost 1.61:
    wget https://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.gz &&
	tar -xvzf boost_1_61_0.tar.gz &&
	rm boost_1_61_0.tar.gz &&
	cd boost_1_61_0 && 
	./bootstrap.sh --prefix=/usr/local &&
	sudo ./b2 install



