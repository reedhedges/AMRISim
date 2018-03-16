
set -e

export NAME=ArTerabotArm
export LIB_INSTALL_DIR=/usr/local/Aria/lib

if test -f ./install-common.sh; then CMD=./install-common.sh; 
elif test -f dist/install-common.sh; then CMD=dist/install-common.sh; 
elif test -f $ARIA/dist/install-common.sh; then CMD=$ARIA/dist/install-common.sh;
elif test -f ../dist/install-common.sh; then CMD=../dist/install-common.sh;
elif test -f ../../dist/install-common.sh; then CMD=../../dist/install-common.sh; 
else echo $NAME install.sh: no install-common.sh script could be found; exit 5;
fi

echo Executing $CMD...
$CMD
