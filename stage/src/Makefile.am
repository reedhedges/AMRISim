
# Automake script to build Stage's Makefile
#
# RTV 20021026
# $Id$

# install the header file for the Stage library
include_HEADERS = stage.h 

#AM_CFLAGS = -Wall -I. -I$(top_builddir)/replace @PLAYER_CFLAGS@ @GLIB_CFLAGS@ @GTK_CFLAGS@
#AM_CPPFLAGS = -Wall -I. -I$(top_builddir)/replace @PLAYER_CFLAGS@ @GLIB_LIBS@ @GTK_CFLAGS@
AM_CFLAGS = -Wall -I. -I$(top_builddir)/replace @PLAYER_CFLAGS@ @GTK_CFLAGS@ @GLIB_CFLAGS@
AM_CPPFLAGS = -Wall -I. -I$(top_builddir)/replace @PLAYER_CFLAGS@ @GTK_CFLAGS@ @GLIB_CFLAGS@

# build the stage library and the stage plugin

#if HAVE_PLAYER
#lib_LTLIBRARIES = libstage.la libstageplugin.la
#else
#lib_LTLIBRARIES = libstage.a 
#endif

lib_LIBRARIES = libstage.a 

#if BUILD_TESTS
# build stest, a stand-alone test binary using the stage library, 
# and ptest, a player client for testing the player plugin
#if HAVE_PLAYER
#bin_PROGRAMS = stest ptest
#else
#bin_PROGRAMS = stest
#endif
#endif


if BUILD_WITH_BLOBFINDER
blobfinder_c=model_blobfinder.c
else
blobfinder_c=
endif

if BUILD_WITH_GRIPPER 
gripper_c=model_gripper.c
else
gripper_c=
endif

if BUILD_WITH_FIDUCIAL
fiducial_c=model_fiducial.c
else
fiducial_c=
endif

# The stage library contains everything you need to build a robot
# simulation, including graphics and a world description file
# parser. It does NOT contain a Player interface, so it can not be
# used as a player plugin. See libstageplugin (below) for good Player
# satisfaction.
#
libstage_a_SOURCES = \
	config.h \
	gui.c \
	gui.h \
	gui_menus.c \
	matrix.c \
	model.c \
	model_laser.c \
	model_position.c \
	model_ranger.c \
	model_messages.c \
	raytrace.c \
	stage.c \
	stage.h \
	stage_internal.h \
	stagecpp.cc \
	typetable.c \
	world.c \
	worldfile.cc \
	worldfile.hh \
	rtk.c \
    rtk.h \
    rtk_canvas.c \
    rtk_fig.c \
    rtk_region.c \
    rtkprivate.h \
	blinker.c blinker.h
	$(blobfinder_c) $(gripper_c) $(fiducial_c)


# TODO
#	model_energy.c

# libstage_a_LDFLAGS = -version-info 2:0:0 
# for dynamic libs only libstage_a_LIBADD = @GTK_LIBS@

# build a Player plugin from libstage plus Player driver hooks.
#if HAVE_PLAYER
#libstageplugin_la_SOURCES = \
#	p_blobfinder.cc \
#	p_driver.cc \
#	p_driver.h \
#	p_energy.cc \
#	p_fiducial.cc \
#	p_gripper.cc \
#	p_laser.cc \
#	p_map.cc \
#	p_position.cc \
#	p_simulation.cc \
#	p_sonar.cc \
#	stg_time.cc \
#	stg_time.h \
#	zoo_driver.cc \
#	zoo_driver.h 
#
#libstageplugin_la_LDFLAGS = -export-dynamic -version-info 1:0:0 
#libstageplugin_la_LIBADD = -L.libs -lstage
#endif

#if BUILD_TESTS
# build a stand-alone test program using libstage
#stest_SOURCES = stest.c
#stest_LDADD = libstage.la

# build a stand-alone test program using libstage
#ptest_SOURCES = ptest.c
#ptest_CFLAGS = @PLAYERC_CFLAGS@ -std=c99
#ptest_LDADD = @PLAYERC_LIBS@
#endif
