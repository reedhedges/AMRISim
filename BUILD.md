

AMRISim Build Instructions and Internal Developer Information
==========================

This document provides an overview of AMRISim for people interested in
compiling AMRISim from source, modifying AMRISim, or just understanding 
how it works.

AMRISim is software for simulating MobileRobots mobile robots
and their environments, for debugging and experimentation.

AMRISim is based on the Stage library, created by the 
Player/Stage project <http://playerstage.sourceforge.net>,
but modify to add some needed features.  

All of the actual simulation is done by the Stage library,
as well as the graphical display of the robot, sensors and map.
AMRISim merely packages up the library into an easy to use
application, and interprets Pioneer robot-compatible commands
and data for Stage.

Player, Stage and all AMRISim components are free software,
distributed under the terms of the GNU General Public License:
see the file LICENSE.txt for details.

Modifying AMRISim or Stage
----------------------------

Both Stage and AMRISim are distributed to you under the terms of the
GNU General Public Lincese (GPL).  See the file named LICENSE.txt for the
text of this copyright license, which in particular, requires you to
release source code of any modified versions make under the
GPL as well, if you also distribute the modified program.  See
LICENSE.txt for the details.

If you do make any modifications and would like to share them, please
do so via a pull request or issue report on the github page
<http://github.com/reedhedges/AMRISim>, or contact me.

How to Compile Stage and AMRISim
----------------------------------

Both Pioneer and ROS1 support are included by default.  To omit ROS1 or Pioneer
support, see instructions below.

### Linux

To build AMRISim on Linux, you will (optionally) need ROS1 noetic, 
Aria or AriaCoda, GTK 2.x, including 
development packages, and the full GNU development tools: 
G++, make, libtool, automake, and autoconf.  
If you have multiple versions of automake or autoconf, or have
them installed somewhere unusual, you can choose which commands
to use by setting the AUTOMAKE, AUTOCONF, AUTORECONF and ACLOCAL
environment variables before running make (e.g set them to 
run the newer version).  A modified version of Stage required by 
AMRISim is included in the 'stage' subdirectory; you may enter 
that directory and manually configure and build libstage if 
required, or you may just run 'make' here in the main AMRISim 
directory to build both libstage.a and AMRISim.  Then run 
'make install' to install AMRISim into /usr/local.  

On Debian and Ubuntu Linux, you can use apt or apt-get to install the required
packages as follows:
    apt install libgtk2.0-dev automake autoconf libtool make g++

Follow the instructions from ROS1 to install ROS1 noetic.  ROS1 noetic
is expected to be installed in /opt/ros/noetic.

The ROS1 environment must be setup before building:
  
    . /opt/ros/noetic/setup.bash

If you do not wish to include ROS1 support in AMRISim, it can
be disabled during build (see below).

Follow the instructions below to build AMRISim.


### Windows (MinGW)

On Windows, you must use MinGW  (http://www.mingw.org)
to build AMRISim. Download the MinGW-Get setup tool and run it to begin
installing MinGW. It will download and run the MinGW Installation Manager.
Select mingw-developer-toolkit, mingw32-base, mingw32-gcc-g++, msys-base,
and  in the Libraries section, mingw32-pthreads-w32-dev and
mingw32-libpthreadgc-dev.  ("Mark for Installation"). With these selections,
autotools, required compilation tools (pkg-config, binutils etc.) and various
shell utilities should be also selected automatically. You can select any
additional shell tools you may wish to have as well.  Then choose "Apply Changes"
from the "Installation" menu to install or upgrade any extra packages selected.  After
installing, run the MinGW Shell from the Start menu (or navigate to the
`C:\MinGW\msys\1.0` directory and run `msys.bat`).  Enter the AMRISim source directory
(You can use the `cd /c/` command to switch to the Windows `C:\` drive root), and follow the steps below to build AMRISim.

Note: ARIA/AriaCoda is normally built using Visual C++ on Windows, but it must be rebuilt
using MinGW for use with AMRISim.  In addition to being built with a different
compiler, ARIA uses pthreads and a few other POSIX features when built with
MinGW rather than Windows implementations; it will still use ws2_32
from Windows for network sockets, and winmm for sound playback, joystick
and other features however.  When you run "make" to build AMRISim,
it will enter the AriaCoda directory (see below) and rebuild ARIA using MinGW,
resulting in `libAria.a` in ARIA's `lib` directory. If the `ARIA` environment variable
is set, then that directory is used to build and find the ARIA library. It defaults
to `../AriaCoda`, so set the `ARIA` environment
variable to a location containing the ARIA source code if you are using a different ARIA source directory.

Stage and AMRISim use GTK 2 for the GUI.  GTK libraries for MinGW are
included in stage/gtk-win and are used automatically when built in MinGW.

The AMRISim Makefile uses the `pkg-config` tool to query required compiler and
linker flags for GTK, so the pkg-config tool must be installed and in
your path.  (Or you can manually specify the location of the pkg-config
tool by setting a PKG_CONFIG variable.)

AMRISim looks for required resources such as the default model
definitions in a default installation directory. To instead use
the source directory, set the AMRISIM environment variable prior
to running AMRISim. You can set to the directory from which
you launch AMRISim like this:

    export AMRISIM=.

Or use the "pwd" command to set it to the current directory at time
you set the variable:

    export AMRISIM=`pwd`

The GTK DLLs must be in the system path, or in the same directory as 
AMRISim, in order to run AMRISim on Linux.  (If not, you will see
an error such as "The program can't start because libgdk-win32-2.0-0.dll
is missing from your computer...".   So to run AMRISim from
the MSYS shell after building, add stage/gtk-win/bin to your PATH:

    export PATH=$PATH:stage/gtk-win/bin

Then you can run AMRISim:

    ./AMRISim.exe

(When installed on the system from an installer package, they
are placed in the same directory, which is why this is not neccesary
except after building from source code.)

More details on how the Windows/MinGW build of the Stage library with
GTK works is described in `stage/README-Windows.txt`, including how to
generate build files using autotools (additional options are required,
the AMRISim Makefile uses these if on MinGW).


### Mac OSX

It is possible to build AMRISim for OSX 10.04 (Tiger) or later using the 
OSX port of GTK.

You will need to install XCode, and then install the command-line development
tools (Run XCode, then open "Preferences..." from the "XCode" application menu.
Select the "Downloads" tab. Select "Command Line Tools" and click its "Install"
button. This allows you to use Terminal shells to build using "make",
"c++" and other commands directly.

Follow the instructions at https://wiki.gnome.org/GTK+/OSX/Building to
download and build GTK 2 and other required libraries.

You will also need to downoald the ARIA source code.  If the ARIA
environment variable is set, then that directory is used to build and
find the ARIA library. It defaults to /usr/local/Aria which is probably
only valid if installed, so set the ARIA environment variable to a
location containing the ARIA source code if in a different location.

Follow the instructions below to build AMRISim, but add the tools installed
for GTK to a GTK_DIR variable and your PATH first. For example, if you downloaded
and built GTK in a "gtk" directory in your home directory:

     export GTK_DIR=~/gtk/inst
     export PKG_CONFIG_PATH=$GTK_DIR/lib/pkgconfig
     export PATH=$PATH:$GTK_DIR/bin

If you placed GTK in another directory instead, replace "~/gtk" with that
location.

AMRISim looks for required resources such as the default model
definitions in a default installation directory. To instead use
the source directory, set the AMRISIM environment variable prior
to running AMRISim. You can set to the directory from which
you launch AMRISim like this:

    export AMRISIM=.

Or use the "pwd" command to set it to the current directory at time
you set the variable:

    export AMRISIM=`pwd`

Then you can run the bare AMRISim executable:

    ./AMRISim

An application bundle is also built, AMRISim.app, which you can
launch by double clicking in the Finder or with the open command:

    open AMRISim.app


Building AMRISim
------------------

Unless an ARIA environment variable was set, AMRISim looks for
ARIACoda in an "../AriaCoda" directory (in the parent directory, i.e.
both the AriaCoda source code and AMRISim source code should be
in the same parent directory).  

If you want to use a version of ARIA or AriaCoda in a different location,
set the ARIA environment variable to its installation or source location:

Examples:

    export ARIA=/usr/local/Aria

    export ARIA=$HOME/stuff/AriaCoda

    export ARIA=~/ARIA-src-2.9.2

    export ARIA="/c/Program Files/MobileRobots/Aria"

AMRISim uses `pkg-config` to determine appropriate build flags for ROS1.
To allow `pkg-config` to determine these, the ROS1 build/run environment
must be imported  before building AMRISim:

    . /opt/ros/noetic/setup.bash

When building AMRISim, you can set environment variables that affect
compilation options:
 
    CXX                     Set the C++ compiler; if unset, "c++" is used.

    CC                      Set the C compiler; if unset, "cc" is used.

    EXTRA_CFLAGS            Additional compilation flags for C code. E.g. EXTRA_CFLAGS="-march=core2 -mtune=intel"

    EXTRA_CXXFLAGS          Additional compilation flags for C++ code (not used for C files.)  E.g. EXTRA_CXXFLAGS=-std=c++20
    
    LFLAGS                  Additional linker flags.
  
    AMRISIM_DEBUG           Build AMRISim with optimizations disabled and
                            debugging enabled if set.
  
    AMRISIM_RELEASE         Build with optimizations enabled and debugging
                            disabled. This is the default state.

    AMRISIM_RELEASE_NATIVE  Same as AMRISIM_RELEASE but also adds -march=native -mtune=native compilation 
                            flags to optimize for the build system. Use this if just running AMRISim on the same 
                            local machine that you are building it on.
  
    AMRISIM_PROFILE_GPROF   Build with profiling enabled with -pg; analyze later
                            with gprof.

    AMRISIM_PROFILE_TRACY   Build with profiling enabled for the "Tracy" tool.

    AMRISIM_SANITIZERS      Build with address and undefined behavior sanitizers enabled. (ARIA and stage are also built with the same flags.)

    AMRISIM_STACKPROTECT    Build with stack protection compilation flags. (ARIA and Stage are also built with the same flags.)
  
    prefix                  Installation directory base (default is /usr/local)
     
    DESTDIR                 Alternate root for installation (default is none,
                            therefore the root filesystem is used).
  
    STAGEDIR                Alternate Stage directory (default is stage/)
  
    STAGELIBDIR             Alternate Stage library directory (default is
                            stage/src/.libs)
  
    AUTORECONF              autoreconf command
  
    AUTOCONF                autoconf command
  
    ACLOCAL                 aclocal command
  
    AUTOMAKE                automake command
  
    ROS1RELEASE             ROS1 release distribution name (default is noetic)
  
    AMRISIM_INCLUDE_PIONEER Include (yes) or omit (no) Pioneer support (see details below)
  
    AMRISIM_INCLUDE_ROS1    Include (yes) or omit (no) ROS1 support (see details below)

For example, to build with debugging options enabled but ROS1 support disabled:

    make AMRISIM_DEBUG=1 AMRISIM_INCLUDE_ROS1=no


Note: AMRISim requires certain resources at runtime (such the robot models definitions
file) to work correctly, so you must either install AMRISim with
'make install', or set a AMRISIM environment variable to the AMRISim
source directory (or another directory containing AMRISim's resources):

For example:
  export AMRISIM=~/AMRISim

If you do not set this, AMRISim will attempt to load the robot models
definitions from /usr/local/AMRISim/PioneerRobotModels.world.inc, which
will fail if AMRISim is not installed or the AMRISIM environment variable
was not set.

For example, if you want to run AMRISim after building it in a source
directory ~/AMRISim:

    export AMRISIM=~/AMRISim
    ./AMRISim

### How to omit ROS1 or Pioneer support

To omit ROS1 or Pioneer support, set the `AMRISIM_INCLUDE_PIONEER` or
`AMRISIM_INCLUDE_ROS1` variables to "no" when building. For example:

    make AMRISIM_INCLUDE_ROS1=no

or

    make AMRISIM_INCLUDE_PIONEER=no

NOTE: If you want to rebuild AMRISim with different options for
`AMRISIM_INCLUDE_PIONEER` or `AMRISIM_INCLUDE_ROS1` you must also
rebuild the `make` dependencies (`Makefile.dep`) to include or omit 
source files and headers containing ROS1 and Pioneer support. Do 
so with the `dep` rule and the desired variable value:

    make dep AMRISIM_INCLUDE_ROS1=no
    make AMRISIM_INCLUDE_ROS1=no

Or together:

    make dep all AMRISIM_INCLUDE_ROS1=no

You can also set environment variables as well:

    export AMRISIM_INCLUDE_ROS1=no
    make dep
    make all


