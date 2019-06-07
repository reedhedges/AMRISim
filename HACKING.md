
How AMRISim Works
-------------------

AMRISim glues together Stage simulation and multiple client
interfaces. Current (2019) client interfaces are Pioneer protocol
(using ARIA) and ROS.
Various utilities from ARIA are also used to load a map file.
The source file "main.cc" contains 
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
which AMRISim uses to set up the simulated world, the GUI, and requested
robots and their devices. 

We have modified Stage to add some missing features and to allow it 
to be built in MSYS/MinGW.  This modified version of stage is included
in the source code package in the 'stage' subdirectory, and must be built
before building AMRISim.  (Patches are available on the Stage project
patch tracker at http://sourceforge.net/p/playerstage/patches/)

The source code to Stage is in the 'stage/src' subdirectory.  Simulation
of robots and devices ("models") are in files beginning with the "model_"
prefix. E.g. a movable robot base is a "position" model and is implemented
in "model_position.c".  Sonar is a kind of "ranger" model.  The laser LRF
is a "laser" model.  Stage begins by creating a "world" (AMRISim creates
a temporary world file for Stage to start with).  AMRISim then creates
models for the robots and devices, represented using stg_model_t structs
which store properties and other state.  These are initialized with an init
function for the appropriate type (e.g. position_init for position models,
ranger_init for ranger models, etc.)  which creates the model's property
objects, registers callbacks for future events, etc.  It then calls the
load function (e.g. position_load for position models, etc.) which reads
configuration information from the world.  Then, after the simulation has
been started, AMRISim's main loop calls Stage's main update function, which
calls the update functions for all model in the simulation (e.g. position_update
for position models) every 100ms (by default).  This update function can examine 
"command" properties and make appropriate changes, and otherwise maintain the 
model's state in the simulation.  Each model-specific function ends by calling 
a common update function that updates the model's position in the simulated world 
(based on its current velocity properties) and updates the GUI.

In setting up the world configuration for Stage, AMRISim relies on definitions of various 
Pioneer robot models, which are kept in a file called PioneerRobotModels.world.inc.  
When installed, this file is alongside the AMRISim program (e.g. 
/usr/local/AMRISim directory on Linux).  

More about Stage (including library API documentation) is on the web at 
<http://playerstage.sf.net>.


Pioneer emulation
-----------------

The EmulatePioneer client tinterface encapsulates the Pioneer robot specific aspects
of the AMRISim code. The central class in this part is called EmulatePioneer.
EmulatePioneer accepts command packets and sends information packets through a 
TCP port, the same way that a real Pioneer robot communicates over a serial port.  
These commands are de-marshalled and EmulatePioneer makes appropriate calls into 
a RobotInterface object.  Likewise, information is retrieved from the 
RobotInterface to be sent in information packets. RobotInterface is the 
interface through which the Pioneer emulation component communicates with the 
simulation component.   In AMRISim, a subclass of RobotInterface called 
StageInterface is used to connect the Pioneer server emulation with the Stage 
simulation.   The StageInterface class also contains a function to load the map 
file and set up the Stage world accordingly.

For each robot requested, AMRISim's main() function creates a 
StageInterface and an  EmulatePioneer object. The StageInterface object 
locates appropriate "model" pointers in the Stage simulation for the robot 
(a "position" model), sonar (a "ranger" model) and possibly a laser model.
EmulatePioneer uses a StageInterface object to communicate with Stage 
through it's public functional API or by getting and setting 
"properties" (a generic data container) of these models.


ROS Interface
-------------

The new ROS interface is implemnted in ROSNode.  Each simulaion loop,
data from each robot is published to topics, and command and service
requests are handled.   Like EmulatePioneer, ROSNode interacts
with the Stage simulation through a RobotInterface (the same StageInterface
object created at startup in main()).



GDB Crash Handler
-----------------

On Linux and MacOSX, when a fatal signal is received indicating the
program is crashing (SIGSEGV, SIGABRT, SIGILL, SIGILL, SIGFPE, SIGBUS),
AMRISim will try to run the gdb debugger and log stack traces for
each thread.  

To allow gdb to do this on some systems such as Ubuntu, 
you must change PTRACE level in Linux. You can do this temporarly 
by modifying /proc/sys/kernel/yama/ptrace_scope:
  sudo sh -c 'echo 0 >/proc/sys/kernel/yama/ptrace_scope'
or permanently by editing /etc/sysctl.d/10-ptrace.conf.

You can disable the crash handler with the --no-crash-debug
command-line options.  

If in noninteractive mode, AMRISim will also try to restart after
logging crash information.  This can be disabled wiht --no-crash-restart.
In the normal interactive mode, automatic restart is disabled by default.

