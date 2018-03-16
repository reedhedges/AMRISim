
=========================================
MobileSim Source Code Information:
 * Building MobileSim 
 * How MobileSim works
=========================================

Reed Hedges <reed@mobilerobots.com>

This document provides an overview of MobileSim for people interested in
compiling MobileSim from source, modifying MobileSim, or just understanding 
how it works.

MobileSim is software for simulating MobileRobots mobile robots
and their environments, for debugging and experimentation.

MobileSim is based on the Stage library, created by the 
Player/Stage project <http://playerstage.sourceforge.net>,
but modify to add some needed features.  

All of the actual simulation is done by the Stage library,
as well as the graphical display of the robot, sensors and map.
MobileSim merely packages up the library into an easy to use
application, and interprets Pioneer robot-compatible commands
and data for Stage.

Player, Stage and all MobileSim components are free software,
distributed under the terms of the GNU General Public License:
see the file LICENSE.txt for details.



----------------------------
Modifying MobileSim or Stage
----------------------------

Both Stage and MobileSim are distributed to you under the terms of the 
GNU General Public Lincese (GPL).  See the file named LICENSE.txt for the
text of this copyright license, which in particular, requires you to
release source code of any modified versions make under the
GPL as well, if you also distribute the modified program.  See 
LICENSE.txt for the details.

If you do make any modifications and would like to share them, please
do so via the appropriate channel:
 * For modifications to Stage (libstage), you can share the change with all
   Stage developers and users (not just MobileSim developers and users) via 
   the patch tracker and mailing list at <http://playerstage.sf.net>. 
 * For modifications to just MobileSim (emulatePioneer; MobileSim main.cc or tools)
   post them to the public aria-users mailing list or email them to MobileRobots
   technical support. See <http://robots.mobilerobots.com>

If you have any questions about how MobileSim works or about modifying it,
please discuss on the aria-users mailing list.



----------------------------------
How to Compile Stage and MobileSim
----------------------------------


Linux
-----

To build MobileSim on Linux, you will need Aria 2.8 or later, GTK 2.x, including 
development packages, and the full GNU development tools: 
G++ (3.2 or later), make, libtool, automake, and autoconf.  
If you have multiple versions of automake or autoconf, or have
them installed somewhere unusual, you can choose which commands
to use by setting the AUTOMAKE, AUTOCONF, AUTORECONF and ACLOCAL
environment variables before running make (e.g set them to 
run the newer version).  A modified version of Stage required by 
MobileSim is included in the 'stage' subdirectory; you may enter 
that directory and manually configure and build libstage if 
required, or you may just run 'make' here in the main MobileSim 
directory to build both libstage.a and MobileSim.  Then run 
'make install' to install MobileSim into /usr/local.  

On Debian and Ubuntu Linux, you can use apt or apt-get to install the required
packages as follows:
  apt install libgtk2.0-dev automake autoconf libtool make g++

Follow the instructions below to build MobileSim.

Windows (MinGW)
---------------

On Windows, you must use MinGW, MSYS and the MSYS DTK (http://www.mingw.org) 
to build MobileSim. Download the MinGW-Get setup tool and run it to begin 
installing MinGW. It will download and run the MinGW Installation Manager.
Select mingw-developer-toolkit, mingw32-base, mingw32-gcc-g++, msys-base,
and  in the Libraries section, mingw32-pthreads-w32 (dev) and 
mingw32-libpthreadgc (dev).  ("Mark for Installation"). With these selections, 
autotools, required compilation tools (pkg-config, binutils etc.) and various 
shell utilities should be also selected automatically. You can select any 
additional shell tools you may wish to have as well.  Choose Apply Changes 
from the Packages menu to install or upgrade any extra packages selected.  After 
installing, run the MinGW Shell from the Start menu (or run msys.bat 
from C:\MinGW\msys\1.0).  Enter the MobileSim directory (unpacked source 
package), and follow the steps below to build MobileSim.

Note: ARIA is normally built using Visual C++ on Windows, but it must be rebuilt
using MinGW for use with MobileSim.  In addition to being built with a different
compiler, ARIA uses pthreads and a few other POSIX features when built with
MinGW rather than Windows implementations; it will still use ws2_32 
from Windows for network sockets, and winmm for sound playback, joystick
and other features however.  When you run "make" to build MobileSim,
it will enter the Aria directory (see below) and rebuild ARIA using MinGW,
resulting in libAria.a in ARIA's lib directory. If the ARIA environment variable
is set, then that directory is used to build and find the ARIA library. It defaults
to /usr/local/Aria which is probably only valid on Linux, so set the ARIA environment
variable to a location containing the ARIA source code when building in MinGW.

Stage and MobileSim use GTK 2 for the GUI.  GTK libraries for MinGW are
included in stage/gtk-win and are used automatically when built in MinGW.

The MobileSim Makefile uses the pkg-config tool to query required compiler and
linker flags for GTK, so the pkg-config tool must be installed and in 
your path.  (Or you can manually specify the location of the pkg-config
tool by setting a PKG_CONFIG variable.)

If you copy or unpack the ARIA source code to ../Aria (next to MobileSim source), then 
after installing and setting up GTK, you can build MobileSim  like this:

  export ARIA=../Aria
  make

If the ARIA source code is in a different directory name or location, change the
name above, using MinGW paths.

MobileSim looks for required resources such as the default model 
definitions in a default installation directory. To instead use
the source directory, set the MOBILESIM environment variable prior
to running MobileSim. You can set to the directory from which
you launch MobileSim like this:

  export MOBILESIM=.

Or use the "pwd" command to set it to the current directory at time
you set the variable:

  export MOBILESIM=`pwd`

The GTK DLLs must be in the system path, or in the same directory as 
MobileSim, in order to run MobileSim on Linux.  (If not, you will see
an error such as "The program can't start because libgdk-win32-2.0-0.dll
is missing from your computer...".   So to run MobileSim from
the MSYS shell after building, add stage/gtk-win/bin to your PATH:

  export PATH=$PATH:stage/gtk-win/bin

Then you can run MobileSim:

  ./MobileSim.exe

(When installed on the system from an installer package, they
are placed in the same directory, which is why this is not neccesary
except after building from source code.)

More details on how the Windows/MinGW build of the Stage library with
GTK works is described in stage/README-Windows.txt, including how to 
generate build files using autotools (additional options are required,
the MobileSim Makefile uses these if on MinGW). 


Mac OSX
-------

It is possible to build MobileSim for OSX 10.04 (Tiger) or later using the 
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

Follow the instructions below to build MobileSim, but add the tools installed
for GTK to a GTK_DIR variable and your PATH first. For example, if you downloaded 
and built GTK in a "gtk" directory in your home directory:

   export GTK_DIR=~/gtk/inst
   export PKG_CONFIG_PATH=$GTK_DIR/lib/pkgconfig
   export PATH=$PATH:$GTK_DIR/bin

If you placed GTK in another directory instead, replace "~/gtk" with that 
location.

MobileSim looks for required resources such as the default model 
definitions in a default installation directory. To instead use
the source directory, set the MOBILESIM environment variable prior
to running MobileSim. You can set to the directory from which
you launch MobileSim like this:

  export MOBILESIM=.

Or use the "pwd" command to set it to the current directory at time
you set the variable:

  export MOBILESIM=`pwd`

Then you can run the bare MobileSim executable:

  ./MobileSim

An application bundle is also built, MobileSim.app, which you can
launch by double clicking in the Finder or with the open command:

  open MobileSim.app


Building MobileSim
------------------

Unless an ARIA environment variable was set, MobileSim looks for 
ARIA in an "../Aria" directory (in the parent directory, i.e.
both the Aria source code and MobileSim source code should be
in the same parent directory).  If you want to use the version of
ARIA already installed on your system, set the ARIA environment variable to 
its installation location:

For Linux:
  export ARIA=/usr/local/Aria

For Windows with MinGW:
  export ARIA="/c/Program Files/MobileRobots/Aria"

Or, if you have unpacked and built the ARIA source code in ~/ARIA-src-2.9.2:
  export ARIA=~/ARIA-src-2.9.2

When building MobileSim, you can set environment variables that affect
compilation options:
 
  CXX                     Set the C++ compiler; if unset, "c++" is used.

  CC                      Set the C compiler; if unset, "cc" is used.

  CFLAGS                  Additional compilation flags for C.
  
  CXXFLAGS                Additional compilation flags for C++.

  LFLAGS                  Additional linker flags.

  MOBILESIM_DEBUG         Build MobileSim with optimizations disabled and
                          debugging enabled if set.

  MOBILESIM_RELEASE       Build with optimizations enabled and debugging
                          disabled. This is the default state.

  MOBILESIM_PROFILE       Build with profiling enabled with -pg; analyze later
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


For example, to build with debugging options enabled:

  make MOBILESIM_DEBUG=1


You can also edit Makefile if neccesary.

Note: MobileSim requires certain resources at runtime (such as the robot models definitions 
file) to work correctly, so you must either install MobileSim with 
'make install', or set a MOBILESIM environment variable to the MobileSim
source directory (or another directory containing MobileSim's resources):

For example:
  export MOBILESIM=~/MobileSim-0.7.3

If you do not set this, MobileSim will attempt to load the robot models
definitions from /usr/local/MobileSim/PioneerRobotModels.world.inc, which
will fail if MobileSim is not installed or the MOBILESIM environment variable
was not set.

For example, if you want to run MobileSim after building it in a source 
directory ~/MobileSim:

  export MOBILESIM=~/MobileSim
  ./MobileSim

-------------------
How MobileSim Works
-------------------

MobileSim glues together three main components: Stage simulation; Pioneer emulation; 
and various utilities from ARIA to load a map file and to send and
receive packets to and from clients.  The source file "main.cc" contains 
the main() function and brings these three components together, and provides 
a command-line interface and also an initial dialog box for loading a map 
file and selecting a robot model to create. The files RobotFactory.cc,
RobotFactory.hh, StageRobotFactory.cc and StageRobotFactory.hh implement
the "robot factory" feature (on-demand model creation).

Stage
-----

Stage is the core of the simulator. It provides the main GUI, and simulation of 
all the robots, devices, their environment, and collisions between them. 
Stage is not a MobileRobots project, but was created by a variety of individuals
and is related to the Player project.   Stage consists of a library, libstage.a,
which MobileSim uses to set up the simulated world, the GUI, and requested
robots and their devices. 

We have modified Stage to add some missing features and to allow it 
to be built in MSYS/MinGW.  This modified version of stage is included
in the source code package in the 'stage' subdirectory, and must be built
before building MobileSim.  (Patches are available on the Stage project
patch tracker at http://sourceforge.net/p/playerstage/patches/)

The source code to Stage is in the 'stage/src' subdirectory.  Simulation
of robots and devices ("models") are in files beginning with the "model_"
prefix. E.g. a movable robot base is a "position" model and is implemented
in "model_position.c".  Sonar is a kind of "ranger" model.  The laser LRF
is a "laser" model.  Stage begins by creating a "world" (MobileSim creates
a temporary world file for Stage to start with).  MobileSim then creates
models for the robots and devices, represented using stg_model_t structs
which store properties and other state.  These are initialized with an init
function for the appropriate type (e.g. position_init for position models,
ranger_init for ranger models, etc.)  which creates the model's property
objects, registers callbacks for future events, etc.  It then calls the
load function (e.g. position_load for position models, etc.) which reads
configuration information from the world.  Then, after the simulation has
been started, MobileSim's main loop calls Stage's main update function, which
calls the update functions for all model in the simulation (e.g. position_update
for position models) every 100ms (by default).  This update function can examine 
"command" properties and make appropriate changes, and otherwise maintain the 
model's state in the simulation.  Each model-specific function ends by calling 
a common update function that updates the model's position in the simulated world 
(based on its current velocity properties) and updates the GUI.

In setting up the world configuration for Stage, MobileSim relies on definitions of various 
Pioneer robot models, which are kept in a file called PioneerRobotModels.world.inc.  
When installed, this file is alongside the MobileSim program (e.g. 
/usr/local/MobileSim directory on Linux).  

More about Stage (including library API documentation) is on the web at 
<http://playerstage.sf.net>.


Pioneer emulation
-----------------

The Pioneer emulator component encapsulates the Pioneer robot specific aspects
of the MobileSim code. The central class in this part is called EmulatePioneer.
EmulatePioneer accepts command packets and sends information packets through a 
TCP port, the same way that a real Pioneer robot communicates over a serial port.  
These commands are de-marshalled and EmulatePioneer makes appropriate calls into 
a RobotInterface object.  Likewise, information is retrieved from the 
RobotInterface to be sent in information packets. RobotInterface is the 
interface through which the Pioneer emulation component communicates with the 
simulation component.   In MobileSim, a subclass of RobotInterface called 
StageInterface is used to connect the Pioneer server emulation with the Stage 
simulation.   The StageInterface class also contains a function to load the map 
file and set up the Stage world accordingly.

For each robot requested, MobileSim's main() function creates a 
StageInterface and an  EmulatePioneer object. The StageInterface object 
locates appropriate "model" pointers in the Stage simulation for the robot 
(a "position" model), sonar (a "ranger" model) and possibly a laser model.
EmulatePioneer uses a StageInterface object to communicate with Stage 
through it's public functional API or by getting and setting 
"properties" (a generic data container) of these models.



GDB Crash Handler
-----------------

On Linux and MacOSX, when a fatal signal is received indicating the
program is crashing (SIGSEGV, SIGABRT, SIGILL, SIGILL, SIGFPE, SIGBUS),
MobileSim will try to run the gdb debugger and log stack traces for
each thread.  

To allow gdb to do this on some systems such as Ubuntu, 
you must change PTRACE level in Linux. You can do this temporarly 
by modifying /proc/sys/kernel/yama/ptrace_scope:
  sudo sh -c 'echo 0 >/proc/sys/kernel/yama/ptrace_scope'
or permanently by editing /etc/sysctl.d/10-ptrace.conf.

You can disable the crash handler with the --no-crash-debug
command-line options.  

If in noninteractive mode, MobileSim will also try to restart after
logging crash information.  This can be disabled wiht --no-crash-restart.
In the normal interactive mode, automatic restart is disabled by default.

