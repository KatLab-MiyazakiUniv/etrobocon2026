\USE_RASPIKE_ART=1

mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

MODULE_DIRS := $(mkfile_path)modules/

SRCS := $(foreach dir,$(MODULE_DIRS),$(shell find $(dir) -name '*.cpp'))
OBJS := $(notdir $(SRCS:.cpp=.o))
APPL_CXXOBJS += $(OBJS)

SRCLANG := c++

APPL_LIBS += -lm

APPL_DIRS += $(mkfile_path)modules

INCLUDES += -I$(mkfile_path)modules