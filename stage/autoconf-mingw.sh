#!/bin/sh
set -e
set -x
aclocal -I /usr/local/share/aclocal 
autoconf -I /usr/local/share/aclocal 
set +x
echo OK, now run ./configure
