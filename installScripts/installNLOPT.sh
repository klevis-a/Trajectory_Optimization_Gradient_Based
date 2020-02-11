#!/usr/bin/env bash
set -e

wget http://ab-initio.mit.edu/nlopt/nlopt-2.4.2.tar.gz &&
tar -xvzf nlopt-2.4.2.tar.gz &&
rm nlopt-2.4.2.tar.gz &&
cd nlopt-2.4.2 && ./configure &&
make -j8 && sudo make install
