## -*- Makefile -*-
##
## User: hfung
## Time: Jan 28, 2011 10:17:35 PM
## Makefile created by Oracle Solaris Studio.
##
## This file is generated automatically.
##


#### Compiler and tool definitions shared by all build targets #####
CCC = g++
CXX = g++
BASICOPTS = -g
CCFLAGS = $(BASICOPTS)
CXXFLAGS = $(BASICOPTS)
CCADMIN = 


# Define the target directories.
TARGETDIR_a.out=GNU-x86_64-MacOSX


all: $(TARGETDIR_a.out)/a.out

## Target: a.out
#CPPFLAGS_a.out = \
#	-IDataRefiner.cpp \
	-Imain.cpp \
	-IOrbitalReader.cpp \
	-IOrbitalWriter.cpp
OBJS_a.out =  \
	$(TARGETDIR_a.out)/DataRefiner.o \
	$(TARGETDIR_a.out)/main.o \
	$(TARGETDIR_a.out)/OrbitalReader.o \
	$(TARGETDIR_a.out)/OrbitalWriter.o
SYSLIBS_a.out = -lm 
USERLIBS_a.out = $(SYSLIBS_a.out) 
DEPLIBS_a.out =  
LDLIBS_a.out = $(USERLIBS_a.out)


# Link or archive
$(TARGETDIR_a.out)/a.out: $(TARGETDIR_a.out) $(OBJS_a.out) $(DEPLIBS_a.out)
	$(LINK.cc) $(CCFLAGS_a.out) $(CPPFLAGS_a.out) -o $@ $(OBJS_a.out) $(LDLIBS_a.out)


# Compile source files into .o files
$(TARGETDIR_a.out)/DataRefiner.o: $(TARGETDIR_a.out) DataRefiner.cpp
	$(COMPILE.cc) $(CCFLAGS_a.out) $(CPPFLAGS_a.out) -o $@ DataRefiner.cpp

$(TARGETDIR_a.out)/main.o: $(TARGETDIR_a.out) main.cpp
	$(COMPILE.cc) $(CCFLAGS_a.out) $(CPPFLAGS_a.out) -o $@ main.cpp

$(TARGETDIR_a.out)/OrbitalReader.o: $(TARGETDIR_a.out) OrbitalReader.cpp
	$(COMPILE.cc) $(CCFLAGS_a.out) $(CPPFLAGS_a.out) -o $@ OrbitalReader.cpp

$(TARGETDIR_a.out)/OrbitalWriter.o: $(TARGETDIR_a.out) OrbitalWriter.cpp
	$(COMPILE.cc) $(CCFLAGS_a.out) $(CPPFLAGS_a.out) -o $@ OrbitalWriter.cpp



#### Clean target deletes all generated files ####
clean:
	rm -f \
		$(TARGETDIR_a.out)/a.out \
		$(TARGETDIR_a.out)/DataRefiner.o \
		$(TARGETDIR_a.out)/main.o \
		$(TARGETDIR_a.out)/OrbitalReader.o \
		$(TARGETDIR_a.out)/OrbitalWriter.o
	$(CCADMIN)
	rm -f -r $(TARGETDIR_a.out)


# Create the target directory (if needed)
$(TARGETDIR_a.out):
	mkdir -p $(TARGETDIR_a.out)


# Enable dependency checking
.KEEP_STATE:
.KEEP_STATE_FILE:.make.state.GNU-x86_64-MacOSX

