#!/bin/sh
set -e
set -x
export AMRISIM=`pwd`
export PATH=$PATH:`pwd`/stage/gtk-win/bin
./AMRISim.exe $*
