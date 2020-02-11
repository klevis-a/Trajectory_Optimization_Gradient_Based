#!/usr/bin/env bash
set -e

wget https://github.com/esa/pagmo_plugins_nonfree/archive/v0.9.tar.gz &&
tar -xvzf v0.9.tar.gz &&
rm v0.9.tar.gz &&
mkdir pagmo_plugins_nonfree-0.9/build &&
cd pagmo_plugins_nonfree-0.9/build && cmake .. &&
make -j8 && sudo make install
