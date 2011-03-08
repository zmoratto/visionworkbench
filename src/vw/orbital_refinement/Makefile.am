# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__

lib_LTLIBRARIES =

if MAKE_MODULE_ORBITALREFINE

include_HEADERS =                   \
  DataRefiner.hpp                   \
  GravityAccelerationFunctor.hpp    \
  OrbitalReader.hpp                 \
  OrbitalReading.hpp                \
  OrbitalWriter.hpp                 \
  TrajectoryCalculator.hpp          \
  TrajectoryDecisionVariableSet.hpp \
  TrajectoryErrorEstimator.hpp      \
  TrajectoryGradientSet.hpp

libvwOrbitalRefine_la_SOURCES =   \
  DataRefiner.cpp                 \
  OrbitalReader.cpp               \
  OrbitalWriter.cpp               \
  TrajectoryCalculator.cpp        \
  TrajectoryErrorEstimator.cpp    \
  WeightCalculator.cpp            

libvwOrbitalRefine_la_LIBADD = @MODULE_ORBITALREFINE_LIBS@
libvwOrbitalRefine_la_LDFLAGS = @VW_LDFLAGS@ $(LIBTOOL_LIB_FLAGS)

lib_LTLIBRARIES += libvwOrbitalRefine.la

# Adding some custom binaries
or_utility_SOURCES = main.cpp
or_utility_LDADD   = libvwOrbitalRefine.la @MODULE_ORBITALREFINE_LIBS@

bin_PROGRAMS = or_utility

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @VW_CPPFLAGS@
AM_LDFLAGS = @VW_LDFLAGS@ -lboost_system

SUBDIRS = . tests

includedir = $(prefix)/include/vw/orbital_refinement

include $(top_srcdir)/config/rules.mak

