USE_RASPIKE_ART=1

mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

MODULE_DIRS := $(mkfile_path)modules/
COMMON_DIRS := $(mkfile_path)common/

SRCS := $(foreach dir,$(MODULE_DIRS),$(shell find $(dir) -name '*.cpp'))
SRCS += $(shell find ${mkfile_path}../../common/library/libcpp-spike/src -name '*.cpp')
OBJS := $(notdir $(SRCS:.cpp=.o))
APPL_CXXOBJS += $(OBJS)

SRCLANG := c++

APPL_LIBS += -lm

APPL_DIRS += \
  $(mkfile_path)modules \
  $(mkfile_path)modules/calculators \
  $(mkfile_path)modules/API \
  $(mkfile_path)modules/motions \
  $(mkfile_path)modules/motions/continuation_conditions \
  $(mkfile_path)common \
  $(mkfile_path)../../common/library/libcpp-spike/src

INCLUDES += \
  -I$(mkfile_path)modules \
  -I$(mkfile_path)modules/calculators \
  -I$(mkfile_path)modules/API \
  -I$(mkfile_path)modules/motions \
  -I$(mkfile_path)modules/motions/continuation_conditions \
  -I$(mkfile_path)common \
  -I$(mkfile_path)../../common/library/libcpp-spike/include
