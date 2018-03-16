
------------------------------------
Building Stage on Windows with MinGW
------------------------------------
Reed Hedges <reed@mobilerobots.com>
March 2011

Stage can be built in Windows using MinGW and other ported software.

MinGW is a minimal GNU development environment for Windows. It includes
a Unix system environment (shell and tools), some ported libraries, and 
most importantly, a port of GCC.  The MinGW/MSYS homepage is at 
<http://www.mingw.org>.  

In addition to MinGW and MSYS, you will need ports of GTK and pthreads.

Note, I have not tested Player or the Stage Player driver plugin, only 
the base library libstage.a.

From MinGW <http://mingw.org> I installed MinGW, the developer toolkit and 
MSYS using the installer tool (mingw-get-inst-20110316.exe linked from top 
of "Downloads" page). Select the MinGW C++ compiler, MSYS base, and developer 
kit.

From <http://www.gtk.org/download/win32.php> I downloaded the 
"bundle" for GTK  and installed it.

If you just want to download pieces, you need glib, gtk+, pango, atk, 
pkg-config, libiconv, gettext, fontconfig, libpng, libjpeg, libtiff, 
freetype and zlib.

I got a port of pthreads from either the MinGW downloads page, or from 
<http://sources.redhat.com/pthreads-win32>.
(or maybe it's now at <http://sourceware.org/pthreads-win32/>?)

I got libtool from the MinGW downloads pages
because the MSYS development kit did not include libltdl.

I installed the GTK stuff and pthreads by unpacking the packages into
C:\mingw\msys\1.0\local. (Which ends up being both /local and also /usr/local in
MSYS-- in MSYS /usr is the same as /).  Or you could edit /etc/fstab to
map /usr/local to wherever you unpack these.  I added
C:\msys\1.0\lib;C:\msys\1.0\local\lib to PATH in the System control
panel (click Advanced...) so Windows could find the DLLs at program runtime.
(When we package our MobileSim simulator based on stage, then we just copy
the required DLLs and other runtime files into the same directory as MobileSim.)
    
I generated the Stage configure script like this:

 aclocal -I /usr/local/share/aclocal && autoconf -I /usr/local/share/aclocal

This is needed for aclocal to find pkg-config's autoconf macro 
definitions (since I put pkg-config in /usr/local).  One problem is
that when config.status or something goes to run aclocal again, it won't
use that option.  You could avoid it by unpacking everything into
c:\mingw\lib but then they might be deleted if you uninstalled or
upgraded mingw, or if mingw included another version of one of those
libraries.

I guess I was missing "ltmain.sh" but it appeared after I manually ran
libtoolize.

Set the PKG_CONFIG_PATH before running configure:
  export PKG_CONFIG_PATH=/local/lib/pkgconfig

I made assorted changes to Stage and the configure script, mostly
fixing some header files  and adding some new "replacement" functions.  
Patches to Stage are posted to the sourceforge patch manager. 

You may need to use the 'dos2unix' program to remove carriage returns
from configure.in and other files, if your unzip or CVS program adds
them, though maybe MinGW is okay with them-- they're annoying to look at
though so I did.

There is a GTK theme that looks like Windows at
<http://gtk-wimp.sourceforge.net/> by the way.


Good luck

Reed

--

Copyright 2006, 2007, 2008, 2009 MobileRobots Inc. This text file ("software") is distributed 
under the terms of the GNU General Public License. It is free software; you
can redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

See the file COPYING for the full text of the license.


