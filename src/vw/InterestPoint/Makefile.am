# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

if MAKE_MODULE_INTERESTPOINT

protocol_headers =
protocol_sources =

if HAVE_PKG_PROTOBUF
protocol_headers += InterestData.pb.h
protocol_sources += InterestData.pb.cc
BUILT_SOURCES          = $(protocol_sources)
nodist_include_HEADERS = $(protocol_headers)
endif

include_HEADERS = Detector.h Descriptor.h Matcher.h Extrema.h         \
                  Localize.h InterestOperator.h WeightedHistogram.h   \
                  ImageOctave.h InterestData.h ImageOctaveHistory.h   \
                  InterestTraits.h MatrixIO.h VectorIO.h LearnPCA.h   \
                  IntegralImage.h IntegralInterestOperator.h          \
                  IntegralDetector.h BoxFilter.h IntegralDescriptor.h

libvwInterestPoint_la_SOURCES = InterestData.cc Descriptor.cc   \
                  IntegralDetector.cc IntegralInterestOperator.cc
libvwInterestPoint_la_LIBADD = @MODULE_INTERESTPOINT_LIBS@

lib_LTLIBRARIES = libvwInterestPoint.la

if HAVE_PKG_PROTOBUF
include_HEADERS += InterestIO.h
libvwInterestPoint_la_SOURCES += $(protocol_sources);
endif

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @VW_CPPFLAGS@
AM_LDFLAGS = @VW_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = . tests

includedir = $(prefix)/include/vw/InterestPoint

include $(top_srcdir)/config/rules.mak
if HAVE_PKG_PROTOBUF
include $(top_srcdir)/thirdparty/protobuf.mak
endif
