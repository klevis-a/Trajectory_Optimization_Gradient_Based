#!/usr/bin/env bash
set -e

wget https://github.com/esa/pagmo2/archive/v2.8.tar.gz &&
tar -xvzf v2.8.tar.gz &&
rm v2.8.tar.gz &&
mkdir pagmo2-2.8/build &&
cd pagmo2-2.8/build && 
cmake ../ -DPAGMO_WITH_NLOPT=ON -DPAGMO_WITH_EIGEN3=ON &&
make && 
sudo make install

