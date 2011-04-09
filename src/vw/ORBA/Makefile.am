# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__

lib_LTLIBRARIES =

if MAKE_MODULE_ORBA

include_HEADERS =              \
  ObservationSet.hpp           \
  ORBADecisionVariableSet.hpp  \
  ORBAErrorEstimator.hpp       \
  ORBAGradientSet.hpp          \
  ORBARefiner.hpp              \
  OrbitalCameraReading.hpp

libvwORBA_la_SOURCES =   \
  ORBAErrorEstimator.cpp \
  ORBARefiner.cpp

libvwORBA_la_LIBADD = @MODULE_ORBA_LIBS@
libvwORBA_la_LDFLAGS = @VW_LDFLAGS@ $(LIBTOOL_LIB_FLAGS)

lib_LTLIBRARIES += libvwORBA.la

# Adding some custom binaries
orba_utility_SOURCES = main.cpp
orba_utility_LDADD   = libvwORBA.la @MODULE_ORBA_LIBS@

bin_PROGRAMS = orba_utility

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @VW_CPPFLAGS@
AM_LDFLAGS = @VW_LDFLAGS@ -lboost_system

SUBDIRS = . tests

includedir = $(prefix)/include/vw/ORBA

include $(top_srcdir)/config/rules.mak

