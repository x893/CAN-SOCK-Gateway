TARGETNAME := can-sock-gw

TOOLCHAIN_ROOT :=
SYSROOT        :=
CROSS_COMPILE  :=

macros-to-preprocessor-directives = $(foreach __macro, $(strip $1), \
	$(if $(findstring =,$(__macro)), \
		--pd "$(firstword $(subst =, ,$(__macro))) SETA $(subst ",\",$(lastword $(subst =, ,$(__macro))))", \
		--pd "$(__macro) SETA 1")) \

CONFIG ?= DEBUG

CONFIGURATION_FLAGS_FILE := debug.mak
TOOLCHAIN_ROOT ?= $(toolchain_root)

include $(CONFIGURATION_FLAGS_FILE)

CONFIGURATION_LINKER_SCRIPT := $(LINKER_SCRIPT)

include $(ADDITIONAL_MAKE_FILES)

ifneq ($(CONFIGURATION_LINKER_SCRIPT),)
LINKER_SCRIPT := $(CONFIGURATION_LINKER_SCRIPT)
endif

ifneq ($(LINKER_SCRIPT),)

ifeq ($(TOOLCHAIN_SUBTYPE),iar)
LDFLAGS += --config $(LINKER_SCRIPT)
else
LDFLAGS += -T$(LINKER_SCRIPT)
endif

endif

ifeq ($(AS),)
AS := as
endif

ifeq ($(AS),as)
AS := $(CC)
ASFLAGS := $(CFLAGS) $(ASFLAGS)
endif

CC_DEPENDENCY_FILE_SPECIFIER := -MD -MF
ASM_DEPENDENCY_FILE_SPECIFIER := -MD -MF

ifeq ($(BINARYDIR),)
error:
	$(error Invalid configuration, please check your inputs)
endif

SOURCEFILES := can-sock-gw.c log.c can_service.c uds.c
EXTERNAL_LIBS := 
EXTERNAL_LIBS_COPIED := $(foreach lib, $(EXTERNAL_LIBS),$(BINARYDIR)/$(notdir $(lib)))

ifneq ($(SYSROOT),)
COMMONFLAGS +=  --sysroot $(SYSROOT)
endif

CFLAGS   += $(COMMONFLAGS)
CXXFLAGS += $(COMMONFLAGS)
ASFLAGS  += $(COMMONFLAGS)
LDFLAGS  += $(COMMONFLAGS)

CFLAGS   += $(addprefix -I,$(INCLUDE_DIRS))
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRS))

CFLAGS   += $(addprefix -D,$(PREPROCESSOR_MACROS))
CXXFLAGS += $(addprefix -D,$(PREPROCESSOR_MACROS))

ASFLAGS  += $(addprefix -D,$(PREPROCESSOR_MACROS))

CXXFLAGS += $(addprefix -framework ,$(MACOS_FRAMEWORKS))
CFLAGS   += $(addprefix -framework ,$(MACOS_FRAMEWORKS))
LDFLAGS  += $(addprefix -framework ,$(MACOS_FRAMEWORKS))

LDFLAGS  += $(addprefix -L,$(LIBRARY_DIRS))

ifeq ($(GENERATE_MAP_FILE),1)
LDFLAGS += -Wl,-Map=$(BINARYDIR)/$(basename $(TARGETNAME)).map
endif

LIBRARY_LDFLAGS = $(addprefix -l,$(LIBRARY_NAMES))

ifeq ($(IS_LINUX_PROJECT),1)
	RPATH_PREFIX := -Wl,--rpath='$$ORIGIN/../
	LIBRARY_LDFLAGS += $(EXTERNAL_LIBS)
	LIBRARY_LDFLAGS += -Wl,--rpath='$$ORIGIN'
	LIBRARY_LDFLAGS += $(addsuffix ',$(addprefix $(RPATH_PREFIX),$(dir $(EXTERNAL_LIBS))))
else
	LIBRARY_LDFLAGS += $(EXTERNAL_LIBS)
endif

all_make_files := $(firstword $(MAKEFILE_LIST)) $(CONFIGURATION_FLAGS_FILE) $(ADDITIONAL_MAKE_FILES)

ifeq ($(STARTUPFILES),)
	all_source_files := $(SOURCEFILES)
else
	all_source_files := $(STARTUPFILES) $(filter-out $(STARTUPFILES),$(SOURCEFILES))
endif

source_obj1 := $(all_source_files:.cpp=.o)
source_obj2 := $(source_obj1:.c=.o)
source_obj3 := $(source_obj2:.s=.o)
source_obj4 := $(source_obj3:.S=.o)
source_obj5 := $(source_obj4:.cc=.o)
source_objs := $(source_obj5:.cxx=.o)

all_objs := $(addprefix $(BINARYDIR)/, $(notdir $(source_objs)))

PRIMARY_OUTPUTS := $(BINARYDIR)/$(TARGETNAME)

all: $(PRIMARY_OUTPUTS)

EXTRA_DEPENDENCIES :=

ifneq ($(LINKER_SCRIPT),)
EXTRA_DEPENDENCIES += $(LINKER_SCRIPT)
endif

$(BINARYDIR)/$(TARGETNAME): $(all_objs) $(EXTERNAL_LIBS) $(EXTRA_DEPENDENCIES)
	@echo "--- Link $@"
	@$(LD) -o $@ $(LDFLAGS) $(START_GROUP) $(all_objs) $(LIBRARY_LDFLAGS) $(END_GROUP)

-include $(all_objs:.o=.dep)

clean:
	rm -rf $(BINARYDIR)

$(BINARYDIR):
	mkdir $(BINARYDIR)

$(BINARYDIR)/%.o : %.cpp $(all_make_files) |$(BINARYDIR)
	@echo "--- Compile: $<"
	@$(CXX) $(CXXFLAGS) -c $< -o $@ $(CC_DEPENDENCY_FILE_SPECIFIER) $(@:.o=.dep)

$(BINARYDIR)/%.o : %.c $(all_make_files) |$(BINARYDIR)
	@echo "--- Compile: $<"
	@$(CC) $(CFLAGS) -c $< -o $@ $(CC_DEPENDENCY_FILE_SPECIFIER) $(@:.o=.dep)

$(BINARYDIR)/%.o : %.S $(all_make_files) |$(BINARYDIR)
	@echo "--- Compile: $<"
	@$(AS) $(ASFLAGS) -c $< -o $@ $(ASM_DEPENDENCY_FILE_SPECIFIER) $(@:.o=.dep)

$(BINARYDIR)/%.o : %.s $(all_make_files) |$(BINARYDIR)
	@echo "--- Compile: $<"
	@$(AS) $(ASFLAGS) -c $< -o $@ $(ASM_DEPENDENCY_FILE_SPECIFIER) $(@:.o=.dep)

$(BINARYDIR)/%.o : %.cc $(all_make_files) |$(BINARYDIR)
	@echo "--- Compile: $<"
	@$(CC) $(CFLAGS) $(CXXFLAGS) -c $< -o $@ $(CC_DEPENDENCY_FILE_SPECIFIER) $(@:.o=.dep)

$(BINARYDIR)/%.o : %.cxx $(all_make_files) |$(BINARYDIR)
	@echo "--- Compile: $<"
	@$(CC) $(CFLAGS) $(CXXFLAGS) -c $< -o $@ $(CC_DEPENDENCY_FILE_SPECIFIER)F $(@:.o=.dep)
