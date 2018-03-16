

EXTRA_DIST = createmodel_worldfile.world

if BUILD_TESTS 


AM_CFLAGS =   -Wall -I$(top_srcdir)/src -I$(top_srcdir)/replace @GTK_CFLAGS@  -std=c99 
AM_CPPFLAGS = -Wall -I$(top_srcdir)/src -I$(top_srcdir)/replace @GTK_CFLAGS@

TESTSLDADD=-lstage -lreplace  -lstdc++ #libstdc++ is needed if libstage is a static library
# -lpthread and -lrt (or similar for diff. platforms) ought to have been put in LIBS by
# configure.


bin_PROGRAMS = modeldestroy resizeworld busycursor messagesmodel createmodel numberutils timetest height log blinker gdatalist_perf math fastwander
#threads

if HAVE_PLAYER
  # bin_PROGRAMS += ptest                  #currently broken
endif




fastwander_SOURCES = fastwander.c
fastwander_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
fastwander_LDADD = $(TESTSLDADD)
fastwander_DEPENDENCIES = ../src/libstage.la

# TODO - fix this
#ptest_DEPENDENCIES = 
#ptest_SOURCES = ptest.c
#ptest_CFLAGS = $(AM_CFLAGS) @PLAYER_CFLAGS@ 
#ptest_LDADD = @PLAYERC_LIBS@


modeldestroy_SOURCES = modeldestroy.c
modeldestroy_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
modeldestroy_LDADD = $(TESTSLDADD)
#modeldestroy_DEPENDENCIES = ../src/libstage.la

resizeworld_SOURCES = resizeworld.c
resizeworld_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
resizeworld_LDADD = $(TESTSLDADD)
#resizeworld_DEPENDENCIES = ../src/libstage.la

busycursor_SOURCES = busycursor.c
busycursor_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
busycursor_LDADD = $(TESTSLDADD)
#busycursor_DEPENDENCIES = ../src/libstage.la


messagesmodel_SOURCES = messagesmodel.c
messagesmodel_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
messagesmodel_LDADD = $(TESTSLDADD)
#messagesmodel_DEPENDENCIES = ../src/libstage.la

timetest_SOURCES = timetest.c
timetest_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
timetest_LDADD = $(TESTSLDADD)
#timetest_DEPENDENCIES = ../src/libstage.la

createmodel_SOURCES = createmodel.c
createmodel_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
createmodel_LDADD = $(TESTSLDADD)
#createmodel_DEPENDENCIES = ../src/libstage.la

numberutils_SOURCES = numberutils.c
numberutils_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
numberutils_LDADD = $(TESTSLDADD)
#numberutils_DEPENDENCIES = ../src/libstage.la

#threads_SOURCES = threads.c
#threads_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
#threads_LDADD = $(TESTSLDADD)
#threads_DEPENDENCIES = ../src/libstage.la

height_SOURCES = height.c
height_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
height_LDADD = $(TESTSLDADD)
#height_DEPENDENCIES = ../src/libstage.la

log_SOURCES = log.c
log_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
log_LDADD = $(TESTSLDADD)
#log_DEPENDENCIES = ../src/libstage.la

blinker_SOURCES = blinker.c
blinker_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
blinker_LDADD = $(TESTSLDADD)
#blinker_DEPENDENCIES = ../src/libstage.la

gdatalist_perf_SOURCES = gdatalist_perf.c
gdatalist_perf_LDFLAGS = -L$(top_srcdir)/replace -L$(top_srcdir)/src 
gdatalist_perf_LDADD = $(TESTSLDADD)


endif # if BUILD_TESTS

