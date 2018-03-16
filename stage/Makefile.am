SUBDIRS = replace src 
#ACLOCAL_AMFLAGS = -I /usr/share/aclocal --install
#tests
#worlds

# create the pkg-config entries
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stage.pc

EXTRA_DIST = README AUTHORS COPYING ChangeLog INSTALL \
	NEWS README README-Windows.txt stage.pc.in

clean-local:
	rm -f config.cache

doc: docs/index.html

docs/index.html: doxygen.conf src/*.h src/*.c
	doxygen doxygen.conf

#LIBTOOL_DEPS = @LIBTOOL_DEPS@

#libtool: $(LIBTOOL_DEPS)
#	$(SHELL) ./config.status --recheck
