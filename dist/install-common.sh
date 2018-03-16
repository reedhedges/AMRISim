

set -e

#set -x 

# Typical install script that does the usual installation steps for most ARIA
# components and libraries. Some may need to also do some additional steps. This
# script makes some assumptions about files to be distributed and their
# locations, though you can supply a few parameters via environment variables:
# Supply name of package in NAME environment variable. NAME is required.
# Override installation locations with INSTALL_DIR and/or DESTDIR environment variable.
# The final installation path for most files will be ${DESTDIR}${INSTALL_DIR}.
# If you supply DESTDIR, make sure it has a trailing slash. Normally DESTDIR is
# empty, and files are installed into the filesystem root.
# To put libraries in a different directory than ${DESTDIR}${INSTALL_DIR}/lib,
# set the LIB_INSTALL_DIR environment variable, in which case libraries are
# installed into ${DESTDIR}${LIB_INSTALL_DIR} instead.
# If LIBS is not set, then one library is assumed, lib/lib$NAME.so. Set LIBS
# to a list of one or more library files with relative path during installation
# to override.
# If SRC_INSTALL is set, installation of libraries is skipped.
# Things are installed group-writable so as to be hacked upon.
#
# This script can run on Linux in sh or bash, and on Windows in Busybox.
# (though currently defaults are only set for Linux)

test -n "$NAME"
if test -z "$INSTALL_DIR"; then INSTALL_DIR=/usr/local/Aria/$NAME; fi
if test -z "$INSTALL"; then INSTALL="install --preserve-timestamps --verbose"; fi

# Defaults for lib install dir.  LIB_INSTALL_DIR is set when doing a temporary "install"
# to put files together for TGZ package.
: ${ARIA_INSTALL_DIR:=$DESTDIR/usr/local/Aria}
if test -z "$LIB_INSTALL_DIR"; then echo LIB_INSTALL_DIR was empty.; LIB_INSTALL_DIR=${ARIA_INSTALL_DIR}/lib; fi

#echo install-common LIB_INSTALL_DIR is $LIB_INSTALL_DIR

# Install all files found in directories given as function arguments.
# Intermediate and temporary files are skipped. Directories are created.
# Symlinks are not followed. Executables that aren't static libreries are
# stripped.
# Note: older versions of find accepted "-perm +1" to mean any executable bit was set (for either u, g, or o), but this was deprecated at some point, so we now use "-perm /u=x" below.
installdirs()
{
  find $* \
        \( -name \*.o -or -name core -or -name CVS -or -name .\* -or -name \*~ -or -name tmp -or -name proprietary* -or -name \*.bak -or -name \*.class \) -prune  \
        -or -type d   -exec ${INSTALL} -d -m 777 ${DESTDIR}${INSTALL_DIR}/\{\} \; \
        -or -type l   -exec cp --no-dereference \{\} ${DESTDIR}${INSTALL_DIR}/\{\} \; \
        -or -name \*.a -exec ${INSTALL} -D -m 666 \{\}  ${DESTDIR}${INSTALL_DIR}/\{\} \; \
        -or -perm /u=x  -exec ${INSTALL} -D --strip -m 777 \{\}  ${DESTDIR}${INSTALL_DIR}/\{\} \; \
        -or           -exec ${INSTALL} -D -m 666 \{\} ${DESTDIR}${INSTALL_DIR}/\{\} \; \
  || echo find returned $?
}

echo      	--------------------------------------
echo		    Installing $NAME in ${DESTDIR}${INSTALL_DIR}...
echo      	--------------------------------------
echo INSTALL=$INSTALL
echo DESTDIR=$DESTDIR
echo INSTALL_DIR=$INSTALL_DIR
echo ARIA_INSTALL_DIR=$ARIA_INSTALL_DIR
echo LIB_INSTALL_DIR=$LIB_INSTALL_DIR
echo DIST_INSTALL=$DIST_INSTALL
echo SRC_INSTALL=$SRC_INSTALL


$INSTALL -m 775 -d ${DESTDIR}${INSTALL_DIR}

installdirs include src tests docs examples

${INSTALL} -D -m 664 LICENSE.txt README.txt Makefile version.txt Changes.txt ${DESTDIR}${INSTALL_DIR}/

if test -f INSTALL.txt
then
  $INSTALL -D -m 664 INSTALL.txt ${DESTDIR}${INSTALL_DIR}/
fi
WINDOWS_EXTRA_FILES=`find . -maxdepth 1 -name \*.sln -or -name \*.vcxproj`
${INSTALL} -D -m 666  doxygen.conf $WINDOWS_EXTRA_FILES ${DESTDIR}${INSTALL_DIR}/

if test -f icon.png; then $INSTALL -m 644 icon.png $DESTDIR$INSTALL_DIR/icon.png; fi
if test -f icon.ico; then $INSTALL -m 644 icon.ico $DESTDIR$INSTALL_DIR/icon.ico; fi

if test -z "$SRC_INSTALL"
then

  if test -z "$LIBS"
  then 
    LIB=lib${NAME}.so
    if test -f lib/$LIB
    then
      # Installing from a package
      export LIBS=lib/$LIB
    elif test -f ../lib/$LIB
    then
      # Installing from source tree in order to create package. 
      export LIBS=../lib/$LIB
      ## Override LIB_INSTALL_DIR.
      #export LIB_INSTALL_DIR=${INSTALL_DIR}/lib
      export DIST_INSTALL=1
    fi
  fi

  ${INSTALL} -d -m 777 ${DESTDIR}${LIB_INSTALL_DIR}/

  echo LIBS=$LIBS

  for l in $LIBS
  do
    ${INSTALL} -D --strip -m 666 $l ${DESTDIR}${LIB_INSTALL_DIR}/
  done

fi

echo       ------------------------------------------------------------------------------------ 
echo       $NAME has been installed in ${DESTDIR}${INSTALL_DIR}. 
echo 
echo       Header files have been installed in ${DESTDIR}${INSTALL_DIR}/include.
echo       Example programs have been installed in ${DESTDIR}${INSTALL_DIR}/examples.
echo       Documentation has been installed in ${DESTDIR}${INSTALL_DIR}/docs.
echo
if test -z "$SRC_INSTALL"
then
  echo       Libraries have been installed in ${DESTDIR}$LIB_INSTALL_DIR.
  if test "$DIST_INSTALL" != "1"
  then
    echo       To be able to use the libraries, you must now add $DESTDIR$LIB_INSTALL_DIR 
    echo       to your LD_LIBRARY_PATH environment variable, or to the /etc/ld.so.conf system file. 
    echo       then run \'ldconfig\';\
  fi
fi
echo
echo       The license for $NAME can be read at ${DESTDIR}${INSTALL_DIR}/LICENSE.txt
echo     	 ------------------------------------------------------------------------------------ 
