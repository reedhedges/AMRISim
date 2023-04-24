
# Makefile for AMRISim.
# 
# This Makefile should work on Linux, MinGW on Windows, Mac OSX, and perhaps other
# Unix-like systems as well.
#
# Run "make help" for a reminder of available targets. Run "make info" for a summary
# of some variable values.
#
# Some variables you can override with environment variables:
#   DESTDIR      Set an alternative filesystem root prefix (normally only used for making packages)
#   INSTALL_DIR  Where to install AMRISim, default /usr/local/AMRISim
#   installed_bindir Overrides desired 'bin' directory (normally $(DESTDIR)/$(INSTALL_DIR))
#   CXX	         C++ compiler (default is make's default, usually 'c++')
#   CC           C compiler (default is make's default, usually 'cc')
#   EXTRA_CXXFLAGS       Compilation flags.  This Makefile will also append various options (see below)
#   EXTRA_CFLAGS       Compilation flags.  This Makefile will also append various options (see below)
#   EXTRA_LFLAGS       Linker flags (e.g. library directories). This Makefile will also append various options (see below)
#   STAGEDIR     Where to find the Stage source code (default is stage/)
#   STAGELIBDIR  Where to find the Stage library for linking (default is $(STAGEDIR)/src/)
#   ARIA         Build against ARIA or AriaCoda source repository or isolated
#                 installation at this location.  (Headers in $(ARIA)/include, libraries in
#                 $(ARIA)/lib, etc.) 
#   ROS1RELEASE   Name of ROS1 release to use if ROS1 enabled. Default is noetic. 
#
# Some variables that set build options:
#   AMRISIM_DEBUG     If defined, then build an unoptimized debug version instead of release version.
#   AMRISIM_RELEASE   If defined, disable DEBUG (default).
#   AMRISIM_PROFILE_GPROF   If defined, then gprof profiling will be enabled with -pg.
#   AMRISIM_PROFILE_TRACY   If defined, then Tracy profiling will be enabled by compiling in the Tracy client stub from $(TRACY)
#   AMRISIM_INCLUDE_ROS1 If set to yes, then include ROS1 interface. Default is yes on Linux, no on Windows
#   AMRISIM_INCLUDE_PIONEER if set to yes, then include Pioneer interface.  Default is yes.
#
# Used for variations on the tar.gz bindist package:
#   TAR_DIRECTORY       If defined, put files to be tarred in a new directory with this name, rather than AMRISim-$(VERSION). 
#   BINDIST_SUFFIX      If defined, append this string to the tar.gz package name.
#
# Other:
#   TRACE        If set, trace this Makefile's commands for debugging
#
# The version number is set in dist/version.num. 
#
# You can also set DEBIAN_PKG_REV_APPEND to append something to a Debian package version field
# (e.g. forpackaging fix)
#
# This Makefile tries to detect the host OS (MinGW, Mac OSX (darwin), fallback on assuming Linux)
# using the $(host) Make variable.  If $(host) is MINGW32 or MINGW64, it is reset to MINGW, and
# both platforms are treated the same.  (However only 32-bit MinGW install is really supported.) 
#
# The "pandoc" tool is required to convert README.md into other formats for packaging.
#
# GNU autotools are required to build libstage.
#
# libnetpbm is needed to build convertBitmapToArMap (not built by default)



ifdef TRACE
REAL_SHELL = $(SHELL)
SHELL=$(warning at rule   $@.   Modified prerequisites are: [$?]    All prerequisites are: [$^])$(REAL_SHELL)
endif

ifdef AMRISIM_RELEASE
AMRISIM_DEBUG=""
endif

default: all

host = $(shell uname | cut -d _ -f 1)

DATESTAMP=$(shell date +'%Y%m%d')
GITCOMMIT_ABBREV=$(shell git log --pretty=reference -n 1  | cut -f 1 -d ' ')
GITCOMMIT_LONG=$(shell git log --pretty=oneline -n 1 | cut -f 1 -d ' ')

ifndef VERSION
ifeq (dist/version.num,$(wildcard dist/version.num))
VERSION = $(shell cat dist/version.num)
VERSION_NUM_FILE=dist/version.num
else
VERSION = dev-$(GITCOMMIT_ABBREV)
VERSION_NUM_FILE=
endif
endif

DATESTR = $(shell date +'%B %e, %Y')

DEBIAN_PKG_REV_APPEND = 
# e.g. DEBIAN_PKG_REV_APPEND = "-3"
RPM_PKG_REV = 0

ifndef ROS1RELEASE
ROS1RELEASE = noetic
endif

ifeq ($(host),Linux)
arch = $(shell dpkg-architecture -qDEB_BUILD_ARCH || echo "")
debver = $(shell cat /etc/debian_version | tr '/\\ \t' '-' | cut -d '.' -f 1)
ifneq ($(debver), 6)
$(info debver=$(debver))
SYSTEM_SUFFIX = +debian$(debver)
endif
endif

ifeq ($(arch),)
arch=i386
endif

ifneq ($(arch), i386)
$(info arch=$(arch))
SYSTEM_SUFFIX+=+$(arch)
endif

ICONSET = $(shell find icons.iconset)

# TODO automatically use debchange to change debian changelog (see aria/arnl
# makefiles)


COMMON_DISTRIBUTED_FILES=Makefile LICENSE.txt Changes.txt README.md README.txt README.html screenshot.png PioneerRobotModels.world.inc AMROffice.map columbia.map icon.png AMRISim.desktop version.txt INSTALL.txt

BINARY_DISTRIBUTED_FILES=$(COMMON_DISTRIBUTED_FILES) AMRISim$(binary_suffix) gdbhelper 

SOURCE_DISTRIBUTED_FILES=$(COMMON_DISTRIBUTED_FILES) main.cc README.src.txt \
  *.cpp *.h *.hh *.cc stage/src/*.c stage/src/*.h stage/src/stagecpp.cc \
  stage/src/worldfile.hh stage/src/worldfile.cc stage/src/config.h.in \
  stage/tests/*.c stage/configure.in stage/*/Makefile.in stage/*/Makefile.am \
  stage/*/*/Makefile.in stage/*/*/Makefile.am stage/Makefile.in \
  stage/Makefile.am stage/AUTHORS stage/COPYING stage/ChangeLog stage/README \
  stage/README-Windows.txt stage/aclocal.m4 stage/doxygen.conf.in stage/INSTALL \
  stage/NEWS stage/autoconf-mingw.sh stage/stage.pc.in stage/replace/*.c \
  stage/replace/*.h convertBitmapToArMap.cc util.h \
  stage/gtk-win/* stage/gtk-win/*/* stage/gtk-win/*/*/* stage/gtk-win/*/*/*/* \
  $(ICONSET) AMRISim.app/Info.plist
SOURCE_DISTRIBUTED_FILES_EXEC=stage/configure stage/config.guess stage/config.sub\
  stage/install-sh stage/missing  gdbhelper 

SOURCE_DISTRIBUTED_FILES_MAYBE=stage/compile stage/depcomp stage/ltmain.sh

CFLAGS += -std=c++17 -DAMRISIM
CFLAGS += -Wall -Wextra -Wpedantic -Wshadow -Wsign-conversion -Wconversion \
-Wmisleading-indentation \
-Wnull-dereference -Woverloaded-virtual \
-Wcast-align 
# -Wduplicated-cond -Wduplicated-barnches -Wlogical-op # not on clang, todo only add if CXX is gcc
# -Wuseless-cast("useless" casts happen alot with GTK)
# -Wnon-virtual-dtol -Wlifetime (clang only)

CXXFLAGS = $(CFLAGS)
CFLAGS += $(EXTRA_CFLAGS)
CXXFLAGS += $(EXTRA_CXXFLAGS)



ifdef AMRISIM_DEBUG

$(info Debug build)
CFLAGS += -g -Og
STAGE_CONFIGURE_ARGS = --disable-optimize --enable-debug 
datestamp = $(shell date +%Y%m%d)
-include lastDevReleaseVer
ifdef lastDevReleaseVer
lastDevReleaseVerDatestamp = $(shell echo $(lastDevReleaseVer) | cut -d '.' -f 1)
lastDevReleaseVerRev = $(shell echo $(lastDevReleaseVer) | cut -d '.' -f 2)
ifeq ($(lastDevReleaseVerDatestamp),$(datestamp))
 ifeq ($(lastDevReleaseVerDatestamp),$(lastDevReleaseVerRev))
  # has no revision
  DEV_RELEASE_VER = $(datestamp).1
 else
  # increment revision
  DEV_RELEASE_VER = $(datestamp).$(shell let foo=$(lastDevReleaseVerRev)+1; echo $$foo)
 endif
else
  # new date
  DEV_RELEASE_VER = $(datestamp)
endif #ifeq lastDevRelease, datestamp
else
  DEV_RELEASE_VER = $(datestamp)
endif #ifdef lastDevReleaseVer
# In debug mode, replace version with special dev tag with timestamp:
VERSION:=$(VERSION)-dev$(DEV_RELEASE_VER)

dist-all:
	$(warning Re-running make dist-all with AMRISIM_RELEASE=1)
	$(MAKE) dist-all AMRISIM_RELEASE=1

else

CFLAGS += -O2
STAGE_CONFIGURE_ARGS = --enable-debug --enable-optimize=2 

LFLAGS += $(EXTRA_LFLAGS) $(RELEASE_EXTRA_LFLAGS)

dist-all: all

endif	 #ifdef AMRISIM_RELEASE


ifdef AMRISIM_PROFILE_GPROF
CXX += -pg -g -O0
CC += -pg -g -O0
STAGE_CONFIGURE_ARGS += --enable-profile --enable-debug --disable-optimize
endif	 #AMRISIM_PROFILE


ifndef TAR_DIRECTORY
TAR_DIRECTORY = AMRISim-$(VERSION)
endif

#ifndef ARIA
#ARIA = ../AriaCoda
#endif

ifdef ARIA
ARIA_LFLAG=-L$(ARIA)/lib
ARIA_IFLAG=-I$(ARIA)
else
ARIA_LFLAG=
ARIA_IFLAG=
endif

#### Different options for Windows or Linux:
$(info host is $(host))

ifeq ($(host),MINGW32)
host=MINGW
endif

ifeq ($(host),MINGW64)
host=MINGW
endif

ifeq ($(host),MINGW)

  $(info Determined host platform to be MinGW on Windows.)

  ifdef AMRISIM_DEBUG
    # Omitting -mwindows makes it show stdout to a DOS command window:
    CFLAGS += -mms-bitfields -DMINGW -D__MINGW__ -DARIA_STATIC
  else
    CFLAGS += -mwindows -mms-bitfields -DMINGW -D__MINGW__ -DARIA_STATIC
  endif #AMRISIM_DEBUG
  binary_suffix = .exe

  platformsuffix = _WIN
  #SYSTEM_LINK = -lpthreadGC2 -lwinmm -lws2_32 -lstdc++ 
  SYSTEM_LINK = -lpthreadGC-3 -lwinmm -lws2_32 -lpthread #-lstdc++ 

  LIBNETPBM = libnetpbm/lib/libnetpbm.a

$(LIBNETPBM):
	$(MAKE) -C libnetpbm

  ARIA_LINK=$(ARIA_LFLAG) -Wl,-Bstatic -lAria -Wl,-Bdynamic -lm
  STAGE_AUTOCONF_ARGS = -I gtk-win/share/aclocal

  PKG_CONFIG_PATH = $(PKG_CONFIG_PATH):stage/gtk-win/lib/pkgconfig
  PKG_CONFIG = stage/gtk-win/bin/pkg-config.exe

  $(info On MinGW, will expect GTK libraries and other resources in stage/gtk-win/:)
  $(info      PKG_CONFIG=$(PKG_CONFIG))
  $(info      PKG_CONFIG_PATH=$(PKG_CONFIG_PATH))
  $(info      PKG_CONFIG=$(PKG_CONFIG))

ifdef AMRISIM_INCLUDE_PIONEER
  $(info AMRISIM_INCLUDE_PIONEER was set in environment to $(AMRISIM_INCLUDE_PIONEER). Unset to use default for this platform instead, and rebuild make dependencies using "make dep all".)
else
  $(info Pioneer interface WILL be included by default on Windows. Set AMRISIM_INCLUDE_PIONEER=no to omit and build new make dependencies using "make dep all")
  AMRISIM_INCLUDE_PIONEER=yes
endif

ifdef AMRISIM_INCLUDE_ROS1
  $(info AMRISIM_INCLUDE_ROS1 was set in environment to $(AMRISIM_INCLUDE_ROS1).  Unset to use default for this platform instead, and build new make dependencies "make dep all")
else
  $(info ROS1 interface WILL NOT be included by default on Windows. Set AMRISIM_INCLUDE_ROS1=yes to include and build new make dependencies using "make dep all")
  AMRISIM_INCLUDE_ROS1=no
endif

else #else assume Linux or Unix-like (e.g MacOSX):

  CFLAGS += -fPIC
  SYSTEM_LINK = -ldl -lm
  LIBNETPBM = -lnetpbm
ifdef ARIA
  ARIA_LINK = $(ARIA)/lib/libAria.a
else
  ARIA_LINK = -Wl,-Bstatic -lAria -Wl,-Bdynamic -lm
endif

  platformsuffix = _LIN

  ifeq ($(host),Darwin)
    $(info Building on Mac OSX (Darwin))
    CFLAGS += -DMACOSX
    ifdef GTK_DIR
      $(info GTK_DIR environment variable set to $(GTK_DIR) will look there for GTK)
      PATH = $(PATH):$(GTK_DIR)/bin
      PKG_CONFIG_PATH = $(PKG_CONFIG_PATH):$(GTK_DIR)/lib/pkgconfig:$(GTK_DIR)/share/pkgconfig
      STAGE_AUTOCONF_ARGS = -I $(GTK_DIR)/share/aclocal
    else
      $(warning Warning: GTK_DIR environment variable not set. Build may fail unless GTK has been installed on the system in default locations.)
    endif
    EXTRA_TARGETS=AMRISimAppBundle
    EXTRA_TARGETS=AMRISim.app 
    AUTOCONF=$(GTK_DIR)/bin/autoconf
    AUTORECONF=$(GTK_DIR)/bin/autoreconf
    ACLOCAL=$(GTK_DIR)/bin/aclocal
    AUTOHEADER=$(GTK_DIR)/bin/autoheader
  else
    SYSTEM_LINK+=-lrt
    STAGE_AUTOCONF_ARGS = 
    #ARIA_LINK = -L$(ARIA)/lib -Wl,-Bstatic -lAria -Wl,-Bdynamic  
    RELEASE_EXTRA_LFLAGS=-Wl,--gc-sections
  endif

  PKG_CONFIG=pkg-config

ifdef AMRISIM_INCLUDE_PIONEER
  $(info AMRISIM_INCLUDE_PIONEER was set in environment to $(AMRISIM_INCLUDE_PIONEER). Unset to use default for this platform instead. Rebuild with new make dependencies using "make dep all".)
else
  $(info Pioneer interface WILL be included by default on Linux. Set AMRISIM_INCLUDE_PIONEER=no to omit. Rebuild with new make dependencies using "make dep all".)
  AMRISIM_INCLUDE_PIONEER=yes
endif

ifdef AMRISIM_INCLUDE_ROS1
  $(info AMRISIM_INCLUDE_ROS1 was set in environment to $(AMRISIM_INCLUDE_ROS1).  Unset to use default for this platform instead. Rebuild with new make dependencies using "make dep all".)
else
  $(info ROS1 interface WILL be included by default on Linux. ROS1 must be installed in /opt/$(ROS1RELEASE). Set AMRISIM_INCLUDE_ROS1=no to omit. Rebuild with new make dependencies using "make dep all".)
  AMRISIM_INCLUDE_ROS1=yes
endif

endif #host is MINGW32 or not

ifdef ARIA
# libAria.a file used as make rule dependency. use ARIA_LINK variable for link
# option in link commands instead..
LIBARIA=$(ARIA)/lib/libAria.a
else
LIBARIA=
endif

ifdef ARIA
ARIA_CFLAGS=-I$(ARIA)/include  -I$(ARIA)/include/Aria
else
ARIA_CFLAGS=
endif



SOURCES = main.cc \
  RobotFactory.cc \
  StageInterface.cc \
  StageRobotFactory.cc \
  CrashHandler.cc \
  Config.cc \
  MapLoader.cc \
  ClientPacketReceiver.cpp \
  Socket.cc \
  ListeningSocket.cc 

HEADERS = RobotFactory.hh \
  StageInterface.hh \
  StageRobotFactory.hh \
  Config.hh \
  MapLoader.hh \
  ClientPacketReceiver.h \
  Socket.hh \
  ListeningSocket.hh \
  RobotFactory.hh \
  util.h \
  NetworkDiscovery.hh

ifeq ($(AMRISIM_INCLUDE_PIONEER),yes)
SOURCES += EmulatePioneer.cc
HEADERS += EmulatePioneer.hh
endif

ifeq ($(AMRISIM_INCLUDE_ROS1),yes)
SOURCES += ROS1Node.cc
HEADERS += ROS1Node.hh
endif


ifdef AMRISIM_PROFILE_TRACY
ifndef TRACY
TRACY := ../tracy
endif
SOURCES += $(TRACY)/TracyClient.cpp
HEADERS += $(TRACY)/Tracy.hpp
CFLAGS += -DTRACY_ENABLE -I$(TRACY) -DTracyZoneScoped=ZoneScoped #-D"TracyZoneScopedN(n)=ZoneScopedN(n)"
else
CFLAGS += -DTracyZoneScoped=
endif

_stage_all_src=$(shell ls stage/src/*.c stage/src/*.h stage/src/*.cc stage/src/*.hh)
_stage_unused_src=$(shell ls stage/src/zoo_* stage/src/p_* stage/src/ptest.c stage/src/stest.c)
STAGE_SRC=$(filter-out $(_stage_unused_src),$(_stage_all_src))
  

OBJS = $(patsubst %.cc,%.o,$(patsubst %.cpp,%.o,$(SOURCES)))


CFLAGS += -I/usr/local/include
LFLAGS += -L/usr/local/lib

ifndef STAGEDIR
STAGEDIR = stage
endif

ifndef STAGELIBDIR
STAGELIBDIR = $(STAGEDIR)/src
endif

# Root directory that install is relative too. Normally relative 
# to nothing (i.e. in /)
ifndef DESTDIR
DESTDIR = 
endif

STAGELIBS = $(STAGELIBDIR)/libstage.a $(STAGEDIR)/replace/libreplace.a


# Run pkg-config to get GTK flags: 
#GTK_LIBS=`pkg-config --libs gtk+-2.0`
#GTK_CFLAGS=`pkg-config --cflags gtk+-2.0`
GTK_LIBS = $(shell PKG_CONFIG_PATH="$(PKG_CONFIG_PATH)" $(PKG_CONFIG) --libs gtk+-2.0)
GTK_CFLAGS = $(shell PKG_CONFIG_PATH="$(PKG_CONFIG_PATH)" $(PKG_CONFIG) --cflags gtk+-2.0)
$(info ok)

# For dynamic linkage (always works fine):
GTK_LINK = $(GTK_LIBS)

# A try at static linkage, has mysterious problems with gobject initialization at runtime:
#GTK_CFLAGS=`pkg-config --cflags --static gtk+-2.0`
#GTK_LINK = \
#	-Wl,-Bstatic \
#		`pkg-config --libs --static gtk+-2.0 | sed 's/-ldl//' | sed 's/-lm//'` \
#		`pkg-config --libs --static xft | sed 's/-ldl//'` \
#		`pkg-config --libs --static xcursor | sed 's/-ldl//' | sed 's/-lX11//'` \
#		`pkg-config --libs --static xrender | sed 's/-ldl//' | sed 's/-lX11//'` \
#		-lXinerama -lXrandr -lfreetype -lexpat -ltiff -ljpeg -lpng -lz -lm \
#	-Wl,-Bdynamic \
#		-lXi -lX11 -ldl


#ROS1_CFLAGS= 
#ROS1_LINK= 
ifeq ($(AMRISIM_INCLUDE_ROS1),yes)



$(info Using ROS1 "$(ROS1RELEASE)" release. Set ROS1RELEASE environment variable to change. Expecting it to be installed in /opt/oos/$(ROS1RELEASE or on the system).)
ros_modules_used = roscpp std_msgs sensor_msgs geometry_msgs tf nav_msgs std_srvs
ROS1_CFLAGS = -DAMRISIM_ROS1 $(shell PKG_CONFIG_PATH="$(PKG_CONFIG_PATH):/opt/ros/$(ROS1RELEASE)/lib/pkgconfig" $(PKG_CONFIG) --cflags $(ros_modules_used))
ROS1_LINK = $(shell PKG_CONFIG_PATH="$(PKG_CONFIG_PATH):/opt/ros/$(ROS1RELEASE)/lib/pkgconfig" $(PKG_CONFIG) --libs $(ros_modules_used))

# For more info about using ROS1 from Make or CMake without using catkin etc see https://github.com/gerkey/ros1_external_use

endif

ifeq ($(AMRISIM_INCLUDE_PIONEER),yes)
MSIM_CFLAGS = -DAMRISIM_PIONEER
endif

MSIM_CFLAGS += -DAMRISIM_VERSION=\"$(VERSION)\" -DAMRISIM_BUILDDATE="\"$(DATESTR)\"" \
  -I. $(CFLAGS) -I$(STAGEDIR) -I$(STAGEDIR)/replace  -I$(STAGEDIR)/src \
	$(GTK_CFLAGS) $(ARIA_CFLAGS) $(ROS1_CFLAGS)

MSIM_LFLAGS  =  $(LFLAGS) 


# For installation:
ifndef INSTALL_DIR
INSTALL_DIR = /usr/local/AMRISim
endif

bindir = $(DESTDIR)/$(INSTALL_DIR)
docdir = $(DESTDIR)/$(INSTALL_DIR)
confdir = $(DESTDIR)/$(INSTALL_DIR)
ourlibdir = $(DESTDIR)/$(INSTALL_DIR)
sysbindir = $(DESTDIR)/usr/local/bin

# Bypases DESTDIR:
ifndef installed_bindir
installed_bindir = $(bindir)
endif



all: AMRISim$(binary_suffix) $(EXTRA_TARGETS) columbia.map


altdebug:  AMRISim_debug$(binary_suffix)

%.o: %.cc
	$(CXX) -c $(MSIM_CFLAGS) -o $@ $<

# Make sure main.o gets rebuilt if dist/version.num changes, unless its missing (e.g. in source distribution package)
ifneq ($(VERSION_NUM_FILE),)
main.o: main.cc $(VERSION_NUM_FILE)
	$(CXX) -c $(MSIM_CFLAGS) -o $@ $<
endif

%.o: %.cpp
	$(CXX) -c $(MSIM_CFLAGS) -o $@ $<

%.o: %.c
	$(CC) -c $(MSIM_CFLAGS) -o $@ $<

# Manually rebuild dep
dep: clean cleandep 
	$(MAKE) Makefile.dep

include Makefile.dep

Makefile.dep:  $(STAGEDIR)/src/config.h
	$(info Building Makefile.dep)
	$(CXX) $(MSIM_CFLAGS) -MM $(SOURCES) >Makefile.dep


cleandep:
	-rm Makefile.dep

# We have source code, depend on it being built
# could depend on $(STAGELIBS) instead of just libstage.a to get both libs, but that can cause make to try
# to build stage twice in parallel if using parallel jobserver, which causes
# corrupted output files.
AMRISim$(binary_suffix): $(STAGEDIR)/src/stage.h $(STAGEDIR)/src/config.h $(STAGELIBDIR)/libstage.a $(OBJS) $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -o $@ $(OBJS) $(STAGELIBS) $(GTK_LINK) $(ARIA_LINK) $(ROS1_LINK) $(SYSTEM_LINK)

mobilesimd: $(STAGE_DIR)/src/stage.h $(STAGEDIR)/src/config.h $(STAGELIBDIR)/libstage_nogui.a $(OBJS) $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) -DAMRISIM_NOGUI $(MSIM_LFLAGS) -o mobilesimd $(OBJS) $(STAGELIBS) $(ARIA_LINK) $(ROS1_LINK) $(SYSTEM_LINK)

AMRISim_debug$(binary_suffix): AMRISim
	cp AMRISim$(binary_suffix) AMRISim_debug$(binary_suffix)

AMRISimAppBundle: AMRISim.app AMRISim.app/Contents/MacOS/AMRISim AMRISim.app/Contents/PkgInfo AMRISim.app/Info.plist AMRISim.app/Resources/AMRISim.icns

AMRISim.app:
	-mkdir $@

AMRISim.app/Contents/MacOS/AMRISim: AMRISim
	-mkdir -p AMRISim.app/Contents/MacOS
	cp $< $@

AMRISim.app/Contents/PkgInfo:
	echo -n 'APPL????' > $@

AMRISim.app/Resources/AMRISim.icns: $(ICONSET)
	-mkdir -p AMRISim.app/Resources
	iconutil -c icns --output $@ icons.iconset

# TODO should include PioneerRobotModels.world.inc, README etc. here and get AMRISim to 
# use that instead of /usr/local/AMRISim/...

# Add a resource fork to AMRISim plain binary. Only used on Mac OSX.
rezAMRISim: AMRISim
	rez -o $^

%.map: $(ARIA)/maps/%.map
	cp $< $@

%.rtf: %.md
	pandoc -s --toc --template pandoc_template -f markdown -t rtf -o $@ $<

%.txt: %.md
	pandoc -s --toc -f markdown -t plain -o $@ $<

%.pdf: %.md
	pandoc -s --toc -f markdown -o $@ $<

%.html: %.md
	pandoc -s --toc -f markdown -o $@ $<

$(ARIA)/lib/libAria.a: FORCE
	$(MAKE) -C $(ARIA) lib/libAria.a

#test_mainloop: test_mainloop.cc $(ARIA_OBJS)
#	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_OBJS) $(SYSTEM_LINK) && strip $@
#
#test_sleep_time: test_sleep_time.cc $(ARIA_OBJS)
#	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_OBJS) $(SYSTEM_LINK) && strip $@

test_mainloop: test_mainloop.cc $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_LINK) $(SYSTEM_LINK) && strip $@

test_sleep_time: test_sleep_time.cc $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_LINK) $(SYSTEM_LINK) && strip $@


ifndef AUTOCONF
AUTOCONF = autoconf
endif

ifndef AUTORECONF
AUTORECONF = autoreconf
endif

ifndef ACLOCAL
ACLOCAL = aclocal
endif

ifndef AUTOHEADER
AUTOHEADER = autoheader
endif

$(STAGEDIR)/config.status $(STAGEDIR)/Makefile $(STAGEDIR)/src/config.h: $(STAGEDIR)/configure
	test -d $(STAGEDIR) || { echo "STAGEDIR \"$(STAGEDIR)\" does not exist. Set STAGEDIR in the environment to correct path or omit to use stage subdirectory provided with AMRISim source code."; false; }
	+cd $(STAGEDIR); \
    if test "$(host)" = "MINGW"; then export PATH="$$PATH:gtk-win/bin";  \
    elif test "$(host)" = "Darwin"; then export PATH="$$PATH:$(GTK_DIR)/bin"; \
    fi; \
    export PKG_CONFIG_PATH="$(PKG_CONFIG_PATH):/usr/local/lib/pkgconfig"; \
    echo PKG_CONFIG_PATH="$$PKG_CONFIG_PATH" ;\
    CC="$(CC)" CXX="$(CXX)" CFLAGS="$(EXTRA_CFLAGS)" CXXFLAGS="$(EXTRA_CXXFLAGS)" ./configure --disable-shared --enable-static \
      --disable-fiducial-model --disable-blobfinder-model --disable-gripper-model \
      --disable-save-world --disable-grid-labels --disable-reload-world \
	  	$(STAGE_CONFIGURE_ARGS)
	touch $@


$(STAGEDIR)/configure:  $(STAGEDIR)/configure.in
	+cd $(STAGEDIR); \
	  if test -d /usr/local/share/aclocal; then args="$(STAGE_AUTOCONF_ARGS) -I /usr/local/share/aclocal"; else args="$(STAGE_AUTOCONF_ARGS)"; fi; \
	  echo running $(AUTORECONF) -i $$args and $(AUTOCONF) $$args ; \
	  if test "$(host)" = "Darwin"; then export PATH=$$PATH:$(GTK_DIR)/bin; fi; \
	  $(AUTORECONF) -i $$args && $(ACLOCAL) $$args && $(AUTOCONF) $$args

stageconf: $(STAGEDIR)/config.status

#	@echo "ERROR: configure script missing from stage! enter stage directory and run 'autoreconf -i' (use --force to force creation or re-creation of files), or unpack the tar archive again."
#	false


convertBitmapToArMap: convertBitmapToArMap.cc $(LIBARIA)
	$(CXX) $(CFLAGS) $(ARIA_CFLAGS) -o $@ $< $(ARIA_LINK) $(LIBNETPBM) $(SYSTEM_LINK) -lpthread


help:
	@echo 'Usage:'
	@echo '       make help                     This message'
	@echo '       make all or make AMRISim      Build AMRISim program'
	@echo '       make debug                    Build AMRISim program in debug mode'
	@echo '       make clean'
	@echo '       make install'
	@echo '       make distclean                Clean temporary files, but leave binaries to be distributed'
	@echo '       make dist-release'
	@echo '       make dist-dev'
	@echo '       make srcdist-release'
	@echo '       make srcdist-dev'
	@echo '       make bindist-release'
	@echo '       make bindist-dev'
	@echo '       make rpm'
	@echo '       make dev-rpm'
	@echo '       make deb'
	@echo '       make dev-deb'
	@echo 'Set environment variable ARIA to specify location of AriaCoda or ARIA library directory. (Will assume ARIA header files in include subdirectory and library in lib subdirectory.)'
	@echo 'Set environment variable AMRISIM_DEBUG to build a debug version (with optimization disabled, more debugger information, no compiler warnings, and with logging to terminal on Windows)'
	@echo 'Set environment variable AMRISIM_PROFILE_GPROF to build with profiling information for use with gprof'
	@echo 'Set environment variable AMRISIM_PROFILE_TRACY to build with profiling support with the Tracy profiling/analysis tool. Set TRACY to tracy source directory.'
	@echo 'Set enviroment variable AMRISIM_INCLUDE_PIONEER to yes to force inclusion of Pioneer interface or no to force omission (regardless of platform defaults). Rebuild with new make dependencies using "make dep all".'
	@echo 'Set enviroment variable AMRISIM_INCLUDE_ROS1 to yes to force inclusion of ROS1 interface or no to force omission (regardless of platform defaults). Rebuild with new make dependencies using "make dep all".'
	@echo 'Set CC, CXX, EXTRA_CFLAGS, EXTRA_CXXFLAGS, EXTRA_LFLAGS variables to customize compilation of both AMRISim and stage internal library.'

info:
	@echo SOURCES=$(SOURCES)
	@echo HEADERS=$(HEADERS)
	@echo STAGE_SRC=$(STAGE_SRC)
	@echo
	@echo OBJS=$(OBJS)
	@echo
	@echo MSIM_CFLAGS=$(MSIM_CFLAGS)
	@echo
	@echo MSIM_LFLAGS=$(MSIM_LFLAGS)
	@echo
	@echo GTK_DIR=$(GTK_DIR)
	@echo GTK_LINK=$(GTK_LINK)
	@echo GTK_LIBS=$(GTK_LIBS)
	@echo GTK_CFLAGS=$(GTK_CFLAGS)
	@echo
	@echo ROS1_CFLAGS=$(ROS1_CFLAGS)
	@echo ROS1_LINK=$(ROS1_LINK)
	@echo
	@echo SYSTEM_LINK=$(SYSTEM_LINK)
	@echo
	@echo host=$(host)
	@echo arch=$(arch)
	@echo SYSTEM_SUFFIX=$(SYSTEM_SUFFIX)
	@echo BINDIST_SUFFIX=$(BINDIST_SUFFIX)
	@echo
	@echo PKG_CONFIG_PATH=$(PKG_CONFIG_PATH)
	@echo PATH=$(PATH)
	@echo 
	@echo AMRISIM_INCLUDE_PIONEER=$(AMRISIM_INCLUDE_PIONEER)
	@echo AMRISIM_INCLUDE_ROS1=$(AMRISIM_INCLUDE_ROS1)

$(STAGELIBDIR)/libstage.a $(STAGEDIR)/replace/libreplace.a: $(STAGEDIR)/config.status $(STAGEDIR)/src/*.c $(STAGEDIR)/src/*.h Makefile $(STAGEDIR)/Makefile
	test -d $(STAGEDIR) || { echo "STAGEDIR \"$(STAGEDIR)\" does not exist. Set STAGEDIR in the environment, or check if there is something wrong with your original source archive or VCS checkout..."; false; }
	$(MAKE) -C $(STAGEDIR) -j2

clean: cleanStage cleanAMRISim

cleanStage:
	+$(MAKE) -C $(STAGEDIR) -j1 clean

cleanAMRISim:
	-rm AMRISim$(binary_suffix) *.o

cleanAll: clean

distclean: 
	-$(MAKE) -C $(STAGEDIR) -k -j1 clean distclean 
	-rm *.o
	-rm tags


sudo-install:
	sudo $(MAKE) install

ifdef AMRISIM_RELEASE
dist-install: $(BINARY_DISTRIBUTED_FILES) 
	$(MAKE) install
else
dist-install:
	$(MAKE) dist-install AMRISIM_RELEASE=1
endif

install:
	install -d $(bindir) $(docdir) $(confdir) $(DESTDIR)/usr/share/applications $(sysbindir) 
	install -s -m 755 AMRISim$(binary_suffix) $(bindir)/AMRISim$(binary_suffix)
	install -m 644 icon.png $(confdir)/icon.png
	install -m 644 columbia.map $(confdir)/columbia.map
	install -m 644 AMROffice.map $(confdir)/AMROffice.map
	install -m 644 PioneerRobotModels.world.inc $(confdir)/PioneerRobotModels.world.inc
	install -m 644 AMRISim.desktop $(DESTDIR)/usr/share/applications/AMRISim.desktop
	install -m 644 README.html $(docdir)/README.html
	install -m 644 README.txt $(docdir)/README.txt
	install -m 644 README.md $(docdir)/README.md
	install -m 644 screenshot.png $(docdir)/screenshot.png
	install -m 644 Changes.txt $(docdir)/Changes.txt
	install -m 644 INSTALL.txt $(docdir)/INSTALL.txt
	install -m 644 LICENSE.txt $(docdir)/LICENSE.txt
	install -m 644 version.txt $(docdir)/version.txt
	install -m 755 gdbhelper $(bindir)/gdbhelper
	ln -s -f $(installed_bindir)/AMRISim$(binary_suffix) $(sysbindir)/AMRISim$(binary_suffix)

uninstall:
	-rm $(bindir)/AMRISim$(binary_suffix) $(confdir)/icon.png \
		$(docdir)/README.html $(docdir)/Changes.txt $(docdir)/LICENSE.txt \
    $(docdir)/README.md $(docdir)/README.txt \
		$(docdir)/INSTALL.txt \
		/usr/share/applications/AMRISim.desktop \
    $(sysbindir)/AMRISim


debian:
	fakeroot debian/rules binary AMRISIM_RELEASE=1

debian-dev: 
	fakeroot debian/rules binary
	echo lastDevReleaseVer=$(DEV_RELEASE_VER) > lastDevReleaseVer


#srcdist: srcdist-release

srcdist-release:
	$(MAKE) srcdistbase AMRISIM_RELEASE=1

srcdist-dev: srcdistbase
	echo lastDevReleaseVer=$(DEV_RELEASE_VER) > lastDevReleaseVer

srcdistbase: $(SOURCE_DISTRIBUTED_FILES)  $(SOURCE_DISTRIBUTED_FILES_EXEC)
	$(INSTALL) -m 775 -d $(DESTDIR)$(INSTALL_DIR)
	for d in $(SOURCE_DISTRIBUTED_FILES); do install -m 644 -D -p $$d $(DESTDIR)$(INSTALL_DIR)/$$d; done
	for d in $(SOURCE_DISTRIBUTED_FILES_MAYBE); do if test -f $$d; then install -m 644 -D -p $$d $(DESTDIR)$(INSTALL_DIR)/$$d; fi; done
	for d in $(SOURCE_DISTRIBUTED_FILES_EXEC); do install -m 755 -D -p $$d $(DESTDIR)$(INSTALL_DIR)/$$d; done

#	if test -n "$(ZIPFILE)"; then zip -9 -r AMRISim-src-$(VERSION).zip AMRISim-src-$(VERSION);\
#  else tar cf AMRISim-src-$(VERSION).tar AMRISim-src-$(VERSION) && gzip -9 AMRISim-src-$(VERSION).tar && mv AMRISim-src-$(VERSION).tar.gz AMRISim-src-$(VERSION).tgz; fi


tgz: bindist-release

#bindist: bindist-release

bindist-release:
	$(MAKE) bindistbase AMRISIM_RELEASE=1

bindist-dev: bindistbase
	echo lastDevReleaseVer=$(VERSION) > lastDevReleaseVer


PKGFILE_TAR=AMRISim-$(VERSION)$(BINDIST_SUFFIX)$(SYSTEM_SUFFIX).tar
PKGFILE_TGZ=AMRISim-$(VERSION)$(BINDIST_SUFFIX)$(SYSTEM_SUFFIX).tgz
PKGFILE=$(PKGFILE_TGZ)
DISTINFO_FILE=AMRISim-$(VERSION)$(BINDIST_SUFFIX)$(SYSTEM_SUFFIX)__info.txt

bindistbase: $(BINARY_DISTRIBUTED_FILES)
	-mkdir -p tmp/$(TAR_DIRECTORY)
	for f in $(BINARY_DISTRIBUTED_FILES); do cp $$f tmp/$(TAR_DIRECTORY)/$$f; done
	if test -n "$(EXCLUDE_FILES)"; then for f in $(EXCLUDE_FILES); do rm tmp/$(TAR_DIRECTORY)/$$f; done; fi
	strip tmp/$(TAR_DIRECTORY)/AMRISim
	cd tmp; tar cf ../$(PKGFILE_TAR) $(TAR_DIRECTORY)
	gzip -9 $(PKGFILE_TAR) && mv $(PKGFILE_TAR).gz $(PKGFILE_TGZ) 
	ln -sf $(PKGFILE_TGZ) AMRISim-latest.tgz
	rm -r tmp
	$(MAKE) $(DISTINFO_FILE)

bindist-info: $(DISTINFO_FILE)

optimize: opt

opt: FORCE
	$(MAKE) AMRISIM_RELEASE=1

debug: FORCE
	$(MAKE) AMRISIM_DEBUG=1

%-opt:
	$(MAKE) $* AMRISIM_RELEASE=1

ctags: tags

tags: $(SOURCES) $(HEADERS) $(STAGE_SRC)
	ctags $(SOURCES) $(HEADERS) $(STAGE_SRC)


tidy: compile_flags.txt
	clang-tidy  -header-filter=".*\.hh"   $(SOURCES) $(HEADERS) -- -x c++ -std=c++17 $(MSIM_CFLAGS)

clang-tidy: tidy

compile_flags.txt: FORCE
	-rm $@
	for f in $(MSIM_CFLAGS); do echo "$$f" >> $@; done

cppcheck: FORCE
	cppcheck --enable=all --language=c++ --std=c++17 --relative-paths=./ -I . -j 4 -DAMRISIM -DARIA_STATIC -DAMRISIM_VERSION="$(VERSION)" -DAMRISIM_BUILDDATE="$(DATESTR)" $(SOURCES) $(HEADERS)
	
#| sed 's/^.*\.(cc|cpp|hh|h):.*:.*:/.\/&'

cppclean: FORCE
	cppclean --include-path=. --include-path=stage/src --include-path=$(ARIA)/include --include-path=$(ARIA)/include/Aria $(SOURCES) $(HEADERS)
  

.PHONY: all clean distclean dep cleandep debug opt optimize install uninstall deb debian dist srcdist bindist test-dist undo-dist opt optimize  stageconf bindistbase srcdistbase debian-dev srcdist-dev bindist-dev srcdist-release bindist-release ctags AMRISimAppBundle tidy

FORCE:
