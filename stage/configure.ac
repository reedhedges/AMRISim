dnl -----------------------------------------------------------------------
dnl Stage Autoconf script
dnl -----------------------------------------------------------------------

dnl Author: Richard Vaughan, 2005.
dnl Modified: 
dnl  * Reed Hedges 2005, 2006 to support MinGW, set alternate version number,
dnl    remove -std=c99 compile option (source code has been modified), make
dnl    some features optional, etc. Added debug, optimize, profile options.
dnl  * Reed Hedges MobileRobots added various fixes and changes
dnl  * Reed Hedges Adept MobileRobots 2011 removed use of libtool. We only build static libraries anyway.
dnl  * Reed Hedges 2023 modernized. add --enable-lto.



AC_INIT([stage], [2.0.0a-amr13])
AC_CONFIG_SRCDIR([src/stage.c])
AM_INIT_AUTOMAKE

AC_CONFIG_MACRO_DIR([m4])

dnl determine system type.  this is used in a couple of places to infer the 
dnl types of arguments to some networking library functions.  if/when we 
dnl upgrade to newer Autotools, this can probably go away.
dnl AC_CANONICAL_TARGET

dnl Need AC_PROG_MAKE_SET for recursive building
AC_PROG_MAKE_SET

dnl Generate a configuration header called <config.h> and put all the C
dnl preprocessor defines in there

dnl AM_CONFIG_HEADER(src/config.h)
AC_CONFIG_HEADERS(src/config.h)

dnl Checks for programs.
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_RANLIB

dnl Include /usr/local
CPPFLAGS="$CPPFLAGS -I/usr/local/include"
LDFLAGS="$LDFLAGS -L/usr/local/lib"
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

dnl MinGW-specific stuff
case $host in
  *-*-mingw32)
    echo Target host is MinGW. Adding Windows-specific compile flags...
    CPPFLAGS="$CPPFLAGS -I/mingw/include -D__MINGW__"
    CFLAGS="$CFLAGS -mms-bitfields -D__MINGW__"
    #CXXFLAGS="$CXXFLAGS -mms-bitfields -D__MINGW__"
    LDFLAGS="$LDFLAGS -L/mingw/lib"
    PKG_CONFIG_PATH=gtk-win/lib/pkg-config:$PKG_CONFIG_PATH
    PATH=$PATH:gtk-win/bin
  ;;
esac



dnl Checks for header files of required libraries
AC_CHECK_HEADERS(stdint.h strings.h sys/time.h sys/timeb.h stdlib.h time.h)


dnl check for platform specific functions and libraries
AC_CHECK_FUNCS(_sleep _ftime usleep gettimeofday srand48 drand48 rindex)

AC_CHECK_FUNC(Sleep,
	AC_DEFINE(HAVE_WIN32_SLEEP, 1, [Define to '1' if you have the MS Windows Sleep() function]),
	AC_CHECK_LIB(kernel32, Sleep,
		AC_DEFINE(HAVE_WIN32_SLEEP, 1, [Define to '1' if you have the MS Windows Sleep() function]),
	)
)


AC_SEARCH_LIBS(sincos, [m],
  AC_DEFINE(HAVE_SINCOS, 1, [Define to '1' if you have the GNU sincos function in libm]),
)

AC_SEARCH_LIBS(clock_gettime, [rt], 
  AC_DEFINE(HAVE_CLOCK_GETTIME, 1, [Define to '1' if you have the POSIX clock_gettime function in librt]),
)




AC_MSG_CHECKING([For timeGetTime in -lwinmm])
oldLIBS=$LIBS
LIBS="-lwinmm $LIBS"
AC_LINK_IFELSE([AC_LANG_PROGRAM([[#include <windef.h>
  #include <mmsystem.h>]], [[timeGetTime();]])],[AC_MSG_RESULT([yes])
	],[AC_MSG_RESULT([no])
	 LIBS=$oldLIBS
        
])

dnl build mplementations of these in libreplace.a if these are 
dnl missing
dnl AC_REPLACE_FUNCS(poll dirname scandir basename)
AC_REPLACE_FUNCS(dirname scandir basename usleep gettimeofday rindex )

dnl check for platform specific typedefs
AC_TYPE_PID_T

dnl Do we have pkg-config?
AC_CHECK_PROG(have_pkg_config,pkg-config,yes,no)
if test "x$have_pkg_config" = "xno"; then
  AC_MSG_ERROR([pkg-config is required to build Stage.  Install pkg-config, then-reconfigure.])
fi

dnl add prefix to the pkg-config path in case things were installed there
dnl and the  user didn't add it manually
export PKG_CONFIG_PATH=$prefix/lib/pkgconfig:/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

dnl We need glib
PKG_CHECK_MODULES(GLIB, glib-2.0, have_glib=yes, have_glib=no)
if test "x$have_glib" = "xno"; then
  AC_MSG_ERROR([glib is required to build Stage.  Install glib, then re-configure.])
fi
dnl pkg-config is too clever by half: if glib.pc isn't found, it tries to run 
dnl glib-config, but interprets the resulting failure of exec as a success (!).
dnl So we'll double-check here by seeing if GLIB_LIBS is empty (actually, a single 
dnl space character).
if test "$GLIB_LIBS" = " "; then
  AC_MSG_ERROR([glib is required to build Stage.  Install glib, then re-configure.])
fi

PKG_CHECK_MODULES(GTK, gtk+-2.0 >= 2.0.2,
dnl PKG_CHECK_MODULES(GTK, libgnomecanvas-2.0 >= 2.0,
AC_DEFINE(INCLUDE_GTK,1,[building GTK components]),
AC_MSG_ERROR([no suitable GTK was found. ])
)


dnl we need Player 1.6 or higher
have_player=yes
PKG_CHECK_MODULES( PLAYER, player >= 1.6.5,
AC_DEFINE(INCLUDE_PLAYER,1,[building Player plugin]),
have_player=no
AC_MSG_WARN([Player was not found -- Player plugin will not be built ])
)
AM_CONDITIONAL(HAVE_PLAYER,[test $have_player = yes])

dnl we need libplayerc
PKG_CHECK_MODULES( PLAYERC, libplayerc >= 1.6.5,
AC_DEFINE(INCLUDE_LIBPLAYERC,1,[building test Player client]),
AC_MSG_WARN([libplayerc was not found. ])
)

dnl where's the X11-style color database?
CDB=""

AC_CHECK_FILE([/usr/X11R6/lib/X11/rgb.txt],
  CDB="/usr/X11R6/lib/X11/rgb.txt",
  AC_CHECK_FILE([/usr/openwin/lib/X11/rgb.txt],
    CDB="/usr/openwin/lib/X11/rgb.txt",
    AC_MSG_WARN([No X11 color database was found -- internal defaults will be used at runtime instead])
  ))

AC_ARG_WITH(color, [AS_HELP_STRING([--with-color=file],[Location of X11-style color file to use at runtime to look up color names])],
  CDB=$withval
  AC_MSG_WARN("Overriding color database file with path given with --with-color: $CDB")
)

if test -n "$CDB"
then
  AC_DEFINE(HAVE_COLOR_DATABASE, 1, [we have an X11 color database])
  AC_DEFINE_UNQUOTED(COLOR_DATABASE,["$CDB"],[the X11-style color database])
fi

build_tests=yes
AC_ARG_ENABLE([tests], 
  [AS_HELP_STRING([--disable-tests],[Do not build test programs stest and ptest])],
  build_tests=$enableval,
)
AM_CONDITIONAL(BUILD_TESTS, test $build_tests = yes)


dnl   Build options:
opt_debug=no
opt_optimize=yes
opt_profile=no

AC_ARG_ENABLE([debug],
  [AS_HELP_STRING([--enable-debug],[Enable debugging with -g option.])],
  opt_debug=$enableval,
)

AC_ARG_ENABLE([optimize],
  [AS_HELP_STRING([--enable-optimize],[Enable optimization with -O3 plus other options.])]
)

AC_ARG_ENABLE([profile],
  [AS_HELP_STRING([--enable-profile],[Enable profiling with -pg option.])]
)

AC_ARG_ENABLE([lto],
  [AS_HELP_STRING([--enable-lto],[Enable link-time optimization if --enable-optimize was also given.])]
)

AC_ARG_ENABLE([address-sanitizer], [AS_HELP_STRING([Enable memory/address runtime sanitizers])])

AC_ARG_ENABLE([ub-sanitizer], [AS_HELP_STRING([Enable undefined behavior runtime sanitizers])])

AC_ARG_ENABLE([stack-protection], [AS_HELP_STRING([Enable stack protection compile options])])


dnl Check if we can use -flto=thin (i.e. clang).  Otherwise we try -flto.
dnl AX_CHECK_COMPILE_FLAG is in m4/ax_check_compile_flag.m4.  It sholud have been included in aclocal.m4 by running autoreconf or aclocal.
have_flto_thin=no
have_flto=no
have_fuse_linker_plugin=no
AX_CHECK_COMPILE_FLAG([-flto], [have_flto=yes], [have_flto=no])
AX_CHECK_COMPILE_FLAG([-flto=thin], [have_flto_thin=yes], [have_flto_thin=no])
AX_CHECK_LINK_FLAG([-fuse-linker-plugin], [have_fuse_linker_plugin=yes], [have_fuse_linker_plugin=no])

if test $opt_debug = yes
then
  CXXFLAGS="$CXXFLAGS -g"
  CFLAGS="$CFLAGS -g"
else
  CXXFLAGS=$(echo $CXXFLAGS | sed 's/-g//g')
  CFLAGS=$(echo $CFLAGS | sed 's/-g//g')
fi

OPT_EXTRA_CFLAGS="-ffunction-sections -fdata-sections"
if test "x$enable_optimize" = "xyes"
then
  AC_MSG_NOTICE([Optimization enabled])
  CXXFLAGS="$CXXFLAGS -O3 $OPT_EXTRA_CFLAGS"
  CFLAGS="$CFLAGS -O3 $OPT_EXTRA_CFLAGS"
  if test x"$enable_lto" = "xyes"
  then
    AC_MSG_NOTICE([Link-time optimization enabled])
    if test x"$have_flto_thin" = "xyes"
    then
      AC_MSG_NOTICE([...using -flto=thin for clang])
      CXXFLAGS="$CXXFLAGS -flto=thin"
      CFLAGS="$CFLAGS -flto=thin"
    elif test x"$have_flto" = "xyes"
    then
      AC_MSG_NOTICE([...using -flto for gcc])
      CXXFLAGS="$CXXFLAGS -flto"
      CFLAGS="$CFLAGS -flto"
    else
      AC_MSG_ERROR([--enable-optimize and --enable-lto specified, but compiler does not support -flto or -flto=thin])
    fi
  fi
elif test "x$opt_optimize" = "xno"
then
  AC_MSG_NOTICE([Optimization not enabled])
  CXXFLAGS=$(echo $CXXFLAGS | sed 's/-O.//g')
  CFLAGS=$(echo $CFLAGS | sed 's/-O.//g')
fi

if test "x$enable_profile" = "xyes"
then
  CXX="$CXX -pg -g -fprofile-arcs"
  CC="$CC -pg -g -fprofile-arcs"
fi


if test x"$enable_address_sanitizer" = "xyes"
then
# TODO use different options if compiler is clang
AC_MSG_NOTICE([Address sanitizer enabled])
CFLAGS="$CFLAGS -fsanitize=address -fsanitize-recover=address -fsanitize-address-use-after-scope"
LDFLAGS="$LDFLAGS -lasan"
fi

if test x"$enable_ub_sanitizer" = "xyes"
then
# TODO use different options if compiler is clang
AC_MSG_NOTICE([UB sanitizer enabled])
CFLAGS="$CFLAGS -fsanitize=undefined -fsanitize-recover=undefined -fsanitize=float-cast-overflow -fsanitize=float-divide-by-zero"
LDFLAGS="$LDFLAGS -lubsan"
fi

if test x"$enable_stack_protection" = "xyes"
then
# TODO use different options if compiler is clang
AC_MSG_NOTICE([Stack protection enabled])
CXXFLAGS="$CXXFLAGS -fstack-protector-strong -mshstk -fcf-protection=full"
fi


dnl   Optional models:
dnl   TODO: define a macro for doing these things

opt_gripper_model=yes
AC_ARG_ENABLE([gripper-model], 
  [AS_HELP_STRING([--disable-gripper-model],[Don't include gripper model])],
  opt_gripper_model=$enableval,
)
AM_CONDITIONAL(BUILD_WITH_GRIPPER, test $opt_gripper_model = yes)
if test $opt_gripper_model = yes
then
  AC_DEFINE(ENABLE_GRIPPER_MODEL, 1, [Enable built in gripper model])
else
  AC_MSG_WARN(Gripper model disabled.)
fi

opt_blobfinder_model=yes
AC_ARG_ENABLE([blobfinder-model], 
  [AS_HELP_STRING([--disable-blobfinder-model],[Don't include blobfinder model])],
  opt_blobfinder_model=$enableval,
)
AM_CONDITIONAL(BUILD_WITH_BLOBFINDER, test $opt_blobfinder_model = yes)
if test $opt_blobfinder_model = yes
then
  AC_DEFINE(ENABLE_BLOBFINDER_MODEL, 1, [Enable built in blobfinder model])
else
  AC_MSG_WARN(Blobfinder model disabled.)
fi

opt_fiducial_model=yes
AC_ARG_ENABLE([fiducial-model], 
  [AS_HELP_STRING([--disable-fiducial-model],[Do not include fiducial model])],
  opt_fiducial_model=$enableval,
)
AM_CONDITIONAL(BUILD_WITH_FIDUCIAL, test $opt_fiducial_model = yes)
if test $opt_fiducial_model = yes
then
  AC_DEFINE(ENABLE_FIDUCIAL_MODEL, 1, [Enable built in fiducial model])
else
  AC_MSG_WARN(Fiducial model disabled.)
fi


opt_save_world=yes
AC_ARG_ENABLE([save-world], 
  [AS_HELP_STRING([--disable-save-world],[Do not include menu item to save current world state])],
  opt_save_world=$enableval,
)
if test $opt_save_world = yes
then
  AC_DEFINE(ENABLE_SAVE_WORLD, 1, [Enable ability to save world state])
else
  AC_MSG_WARN(Save world command disabled.)
fi

opt_reload_world=yes
AC_ARG_ENABLE([reload-world], 
  [AS_HELP_STRING([--disable-reload-world],[Don't include menu item to reload current world state])],
  opt_reload_world=$enableval,
)
if test $opt_reload_world = yes
then
  AC_DEFINE(ENABLE_RELOAD_WORLD, 1, [Enable ability to reload world state])
else
  AC_MSG_WARN(Reload world command disabled.)
fi

opt_grid_control=yes
AC_ARG_ENABLE([grid-control], 
  [AS_HELP_STRING([--disable-grid-control],[Don't include menu item to toggle grid display])],
  opt_grid_control=$enableval,
)
if test $opt_grid_control = yes
then
  AC_DEFINE(ENABLE_GRID_CONTROL, 1, [Enable grid view menu item])
else
  AC_MSG_WARN(Show grid command disabled)
fi

opt_grid_labels=yes
AC_ARG_ENABLE([grid-labels],
  [AS_HELP_STRING([--disable-grid-labels],[Don't draw gridline index labels])],
  opt_grid_labels=$enableval,
)
if test $opt_grid_labels = yes
then
  AC_DEFINE(ENABLE_GRID_LABELS, 1, [Enable gridline index labels])
else
  AC_MSG_WARN(Gridline index labels disabled)
fi

opt_laser_noise=yes
AC_ARG_ENABLE([laser-noise], 
  [AS_HELP_STRING([--disable-laser-noise],[Don't include laser reading angle noise or range noise])],
  opt_laser_noise=$enableval,
)
if test $opt_laser_noise = yes
then
  AC_DEFINE(ENABLE_LASER_NOISE, 1, [Enable laser noise simulation])
else
  AC_MSG_WARN(Laser noise simulation code disabled)
fi

opt_ranger_noise=yes
AC_ARG_ENABLE([ranger-noise], 
  [AS_HELP_STRING([--disable-ranger-noise],[Don't include ranger noise])],
  opt_ranger_noise=$enableval,
)
if test $opt_ranger_noise = yes
then
  AC_DEFINE(ENABLE_RANGER_NOISE, 1, [Enable ranger noise simulation])
else
  AC_MSG_WARN(Ranger device noise simulation code disabled)
fi

opt_locking=no
AC_ARG_ENABLE([locking],
  [AS_HELP_STRING([--enable-locking],[Activate model and world mutex locks if used by client program])],
  opt_locking=$enableval,
)
if test $opt_locking = yes
then
  AC_DEFINE(STG_ENABLE_MODEL_LOCK, 1, [Enable model locks])
  AC_DEFINE(STG_ENABLE_WORLD_LOCK, 1, [Enable world lock])
  AC_CHECK_HEADERS(pthread.h semaphore.h)
  AC_SEARCH_LIBS(pthread_mutex_init, [pthread pthreadGC2])
  AC_SEARCH_LIBS(pthread_mutex_trylock, [pthread pthreadGC2])
fi

AC_CONFIG_FILES([Makefile
          src/Makefile
          worlds/Makefile
          worlds/bitmaps/Makefile
          replace/Makefile
          tests/Makefile
          stage.pc
          doxygen.conf])
AC_OUTPUT

dnl should use AC_MSG_NOTICE(), but that's apparently not defined in
dnl autoconf 2.13.
AC_MSG_RESULT([])
AC_MSG_RESULT([*************************************************************])
AC_MSG_RESULT([])
AC_MSG_RESULT([Stage will be built for a $host system, with the following tools and options:])
AC_MSG_RESULT([  C   compiler: $CC $CFLAGS])
AC_MSG_RESULT([  C++ compiler: $CXX $CXXFLAGS])
AC_MSG_RESULT([])

AC_MSG_RESULT([Stage will be installed in:])
AC_MSG_RESULT([    $prefix/])
AC_MSG_RESULT([])
AC_MSG_RESULT([To see the configuration options, do:])
AC_MSG_RESULT([    ./configure --help])
AC_MSG_RESULT([])
AC_MSG_RESULT([When you're satisfied with the configuration, type 'make install'. ])
AC_MSG_RESULT([])
