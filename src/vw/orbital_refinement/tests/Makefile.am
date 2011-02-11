# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

if MAKE_MODULE_ORBITALREFINE

TestTrajectoryCalculator_SOURCES      = TestTrajectoryCalculator.cpp

TESTS = TestTrajectoryCalculator

#include $(top_srcdir)/config/instantiate.am

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @VW_CPPFLAGS@
AM_LDFLAGS  = @VW_LDFLAGS@ @PKG_MATH_LIBS@ @MODULE_ORBITALREFINE_LIBS@

check_PROGRAMS = $(TESTS)

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am
