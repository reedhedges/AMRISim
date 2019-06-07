
=========================================
AMRISim Source Code Information:
 * Building AMRISim 
 * How AMRISim works
=========================================

Reed Hedges <reed@mobilerobots.com>

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



----------------------------
Modifying AMRISim or Stage
----------------------------

Both Stage and AMRISim are distributed to you under the terms of the 
GNU General Public Lincese (GPL).  See the file named LICENSE.txt for the
text of this copyright license, which in particular, requires you to
release source code of any modified versions make under the
GPL as well, if you also distribute the modified program.  See 
LICENSE.txt for the details.

If you do make any modifications and would like to share them, please
do so via the appropriate channel:
 * For modifications to Stage (libstage), you can share the change with all
   Stage developers and users (not just AMRISim developers and users) via 
   the patch tracker and mailing list at <http://playerstage.sf.net>. 
 * For modifications to just AMRISim (emulatePioneer; AMRISim main.cc or tools)
   post them to the public aria-users mailing list or email them to MobileRobots
   technical support. See <http://robots.mobilerobots.com>

If you have any questions about how AMRISim works or about modifying it,
please discuss on the aria-users mailing list.



----------------------------------
How to Compile Stage and AMRISim
----------------------------------


Linux
-----

To build AMRISim on Linux, you will need ROS melodic, 
Aria or AriaCoda, GTK 2.x, including 
development packages, and the full GNU development tools: 
G++ (3.2 or later), make, libtool, automake, and autoconf.  
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

Follow the instructions from ROS to install ROS melodic.  ROS melodic
is expected to be installed in /opt/ros/melodic.

The ROS environment must be setup before building:
  
  . /opt/ros/melodic/setup.bash

Follow the instructions below to build AMRISim.

Windows (MinGW)
---------------

On Windows, you must use MinGW, MSYS and the MSYS DTK (http://www.mingw.org) 
to build AMRISim. Download the MinGW-Get setup tool and run it to begin 
installing MinGW. It will download and run the MinGW Installation Manager.
Select mingw-developer-toolkit, mingw32-base, mingw32-gcc-g++, msys-base,
and  in the Libraries section, mingw32-pthreads-w32 (dev) and 
mingw32-libpthreadgc (dev).  ("Mark for Installation"). With these selections, 
autotools, required compilation tools (pkg-config, binutils etc.) and various 
shell utilities should be also selected automatically. You can select any 
additional shell tools you may wish to have as well.  Choose Apply Changes 
from the Packages menu to install or upgrade any extra packages selected.  After 
installing, run the MinGW Shell from the Start menu (or run msys.bat 
from C:\MinGW\msys\1.0).  Enter the AMRISim directory (unpacked source 
package), and follow the steps below to build AMRISim.

Note: ARIA is normally built using Visual C++ on Windows, but it must be rebuilt
using MinGW for use with AMRISim.  In addition to being built with a different
compiler, ARIA uses pthreads and a few other POSIX features when built with
MinGW rather than Windows implementations; it will still use ws2_32 
from Windows for network sockets, and winmm for sound playback, joystick
and other features however.  When you run "make" to build AMRISim,
it will enter the Aria directory (see below) and rebuild ARIA using MinGW,
resulting in libAria.a in ARIA's lib directory. If the ARIA environment variable
is set, then that directory is used to build and find the ARIA library. It defaults
to /usr/local/Aria which is probably only valid on Linux, so set the ARIA environment
variable to a location containing the ARIA source code when building in MinGW.

Stage and AMRISim use GTK 2 for the GUI.  GTK libraries for MinGW are
included in stage/gtk-win and are used automatically when built in MinGW.

The AMRISim Makefile uses the pkg-config tool to query required compiler and
linker flags for GTK, so the pkg-config tool must be installed and in 
your path.  (Or you can manually specify the location of the pkg-config
tool by setting a PKG_CONFIG variable.)

If you copy or unpack the ARIA source code to ../Aria (next to AMRISim source), then 
after installing and setting up GTK, you can build AMRISim  like this:

  export ARIA=../Aria
  make

If the ARIA source code is in a different directory name or location, change the
name above, using MinGW paths.

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
GTK works is described in stage/README-Windows.txt, including how to 
generate build files using autotools (additional options are required,
the AMRISim Makefile uses these if on MinGW). 


Mac OSX
-------

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
in the same parent directory).  If you want to use the version of
ARIA already installed on your system, set the ARIA environment variable to 
its installation location:

For Linux:
  export ARIA=/usr/local/Aria

For Windows with MinGW:
  export ARIA="/c/Program Files/MobileRobots/Aria"

Or, if you have unpacked and built the ARIA source code in ~/ARIA-src-2.9.2:
  export ARIA=~/ARIA-src-2.9.2

AMRISim uses `pkg-config` to determine appropriate build flags for ROS.
To allow `pkg-config` to determine these, the ROS build/run environment 
must be imported  before building AMRISim:

  . /opt/ros/melodic/setup.bash

When building AMRISim, you can set environment variables that affect
compilation options:
 
  CXX                     Set the C++ compiler; if unset, "c++" is used.

  CC                      Set the C compiler; if unset, "cc" is used.

  CFLAGS                  Additional compilation flags for C.
  
  CXXFLAGS                Additional compilation flags for C++.

  LFLAGS                  Additional linker flags.

  AMRISIM_DEBUG         Build AMRISim with optimizations disabled and
                          debugging enabled if set.

  AMRISIM_RELEASE       Build with optimizations enabled and debugging
                          disabled. This is the default state.

  AMRISIM_PROFILE       Build with profiling enabled with -pg; analyze later
                          with gprof.

  prefix                  Installation directory base (default is /usr/local)
   
  DESTDIR                 Alternate root for installation (default is none,
                          therefore the root filesystem is used).

  STAGEDIR                Alternate Stage directory (default is stage/)

  STAGELIBDIR             Alternate Stage library directory (default is
                          stage/src/.libs)

  AUTORECONF		          autoreconf command

  AUTOCONF                autoconf command

  ACLOCAL                 aclocal command

  AUTOMAKE                automake command

  ROSRELEASE              ROS release distribution name (default is melodic)

For example, to build with debugging options enabled:

  make AMRISIM_DEBUG=1


You can also edit Makefile if neccesary.

Note: AMRISim requires certain resources at runtime (such the robot models definitions 
file) to work correctly, so you must either install AMRISim with 
'make install', or set a AMRISIM environment variable to the AMRISim
source directory (or another directory containing AMRISim's resources):

For example:
  export AMRISIM=~/AMRISim-0.7.3

If you do not set this, AMRISim will attempt to load the robot models
definitions from /usr/local/AMRISim/PioneerRobotModels.world.inc, which
will fail if AMRISim is not installed or the AMRISIM environment variable
was not set.

For example, if you want to run AMRISim after building it in a source 
directory ~/AMRISim:

  export AMRISIM=~/AMRISim
  ./AMRISim

