## -*- Makefile -*-
##
## User: hfung
## Time: Jan 25, 2011 1:24:36 AM
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
TARGETDIR_main.cpp=GNU-x86_64-MacOSX


all: $(TARGETDIR_main.cpp)/main.cpp

## Target: main.cpp
OBJS_main.cpp =  \
	$(TARGETDIR_main.cpp)/main.o \
	$(TARGETDIR_main.cpp)/OrbitalReader.o
USERLIBS_main.cpp = $(SYSLIBS_main.cpp) 
DEPLIBS_main.cpp =  
LDLIBS_main.cpp = $(USERLIBS_main.cpp)


# Link or archive
$(TARGETDIR_main.cpp)/main.cpp: $(TARGETDIR_main.cpp) $(OBJS_main.cpp) $(DEPLIBS_main.cpp)
	$(LINK.cc) $(CCFLAGS_main.cpp) $(CPPFLAGS_main.cpp) -o $@ $(OBJS_main.cpp) $(LDLIBS_main.cpp)


# Compile source files into .o files
$(TARGETDIR_main.cpp)/main.o: $(TARGETDIR_main.cpp) main.cpp
	$(COMPILE.cc) $(CCFLAGS_main.cpp) $(CPPFLAGS_main.cpp) -o $@ main.cpp

$(TARGETDIR_main.cpp)/OrbitalReader.o: $(TARGETDIR_main.cpp) OrbitalReader.cpp
	$(COMPILE.cc) $(CCFLAGS_main.cpp) $(CPPFLAGS_main.cpp) -o $@ OrbitalReader.cpp



#### Clean target deletes all generated files ####
clean:
	rm -f \
		$(TARGETDIR_main.cpp)/main.cpp \
		$(TARGETDIR_main.cpp)/main.o \
		$(TARGETDIR_main.cpp)/OrbitalReader.o
	$(CCADMIN)
	rm -f -r $(TARGETDIR_main.cpp)


# Create the target directory (if needed)
$(TARGETDIR_main.cpp):
	mkdir -p $(TARGETDIR_main.cpp)


# Enable dependency checking
.KEEP_STATE:
.KEEP_STATE_FILE:.make.state.GNU-x86_64-MacOSX

