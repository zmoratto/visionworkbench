

#### Compiler and tool definitions shared by all build targets #####
VW_ROOT=../..

CCC = g++
CXX = g++
BASICOPTS = -g -I$(VW_ROOT)
CCFLAGS = $(BASICOPTS)
CXXFLAGS = $(BASICOPTS)
CCADMIN =
PREFIX = @



# Define the target directories.
TARGETDIR=build
EXE_NAME=orbital_refinement

SRCS = \
  main.cpp \
  OrbitalReader.cpp \
  OrbitalWriter.cpp \
  DataRefiner.cpp \
  TrajectoryCalculator.cpp


all: $(TARGETDIR)/$(EXE_NAME)

OBJS = $(SRCS:%.cpp= $(TARGETDIR)/%.o)
DEPS = $(SRCS:%.cpp= $(TARGETDIR)/%.d)

SYSLIBS = -lm 
USERLIBS = $(SYSLIBS) $(VW_ROOT)/vw/Math/libvwMath.la
DEPLIBS =  
LDLIBS = $(USERLIBS)

$(DEPS): $(TARGETDIR)/%.d : %.cpp $(TARGETDIR)
	@echo "Building dependencies for $<"
	$(PREFIX)echo -n $@ $(TARGETDIR)/ > $@
	$(PREFIX)$(COMPILE.cc) $(CCFLAGS) $(CPPFLAGS) -MM $< >> $@

$(OBJS): $(TARGETDIR)/%.o : %.cpp
	@echo "Compiling $<"
	$(PREFIX)$(COMPILE.cc) $(CCFLAGS) $(CPPFLAGS) -o $@ $<

-include $(DEPS)

# Link or archive
$(TARGETDIR)/$(EXE_NAME): $(TARGETDIR) $(OBJS) $(DEPLIBS)
	@echo "Linking $@"
	$(PREFIX)$(LINK.cc) $(CCFLAGS) $(CPPFLAGS) -o $@ $(OBJS) $(LDLIBS)


#### Clean target deletes all generated files ####
clean:
	$(CCADMIN)
	rm -f -r $(TARGETDIR)


# Create the target directory (if needed)
$(TARGETDIR):
	mkdir -p $(TARGETDIR)


# Enable dependency checking
depend: $(DEPS)
