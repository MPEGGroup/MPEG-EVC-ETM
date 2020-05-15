DIR_TOP = ../..
DIR_INC = $(DIR_TOP)/inc
DIR_SRC = $(DIR_TOP)/src
DIR_APP = $(DIR_TOP)/app
DIR_LIB = $(DIR_TOP)/lib
DIR_BIN = $(DIR_TOP)/bin
DIR_DEP := .d

$(shell mkdir -p $(DIR_DEP) >/dev/null)

DEPFLAGS = -MT $@ -MMD -MP -MF $(DIR_DEP)/$*.Td
POSTCOMPILE = @mv -f $(DIR_DEP)/$*.Td $(DIR_DEP)/$*.d && touch $@

CC = gcc
CXX = g++ 
AR = ar
LD = ld
SED = sed
MV = mv

MKDIR = mkdir -p
RM = rm -rf


CFLAGS  = -g -O3 -fomit-frame-pointer -Wall -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Werror -Wno-strict-overflow -Wno-pointer-sign -Wno-stringop-overflow -Wno-unknown-pragmas -std=c99
CFLAGS += -msse4.2
CFLAGS += -I$(DIR_INC) -DIFVC_TRACE=0 -DIFVC_ASSERT=1

CXXFLAGS  = -g -O3 -fomit-frame-pointer -Wall -Werror -std=c++11
CXXFLAGS += -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Wno-strict-overflow -Wno-sign-compare -Wno-unused-value -Wno-unknown-pragmas
CXXFLAGS += -msse4.2
CXXFLAGS += -I$(DIR_INC)

LDFLAGS = -L$(DIR_LIB) -levc -lm -lstdc++
ARFLAGS = r

CXXSRCS = $(DIR_SRC)/EncAdaptiveLoopFilter.cpp \
          $(DIR_SRC)/evce_ibc_hashmap.cpp \
	  
CSRCS = $(DIR_SRC)/evc_alf.c \
		$(DIR_SRC)/evc_dra.c \
		$(DIR_SRC)/evc.c \
		$(DIR_SRC)/evc_img.c \
		$(DIR_SRC)/evc_ipred.c \
		$(DIR_SRC)/evc_mc.c \
		$(DIR_SRC)/evc_recon.c \
		$(DIR_SRC)/evc_tbl.c \
		$(DIR_SRC)/evc_util.c \
		$(DIR_SRC)/evc_itdq.c \
		$(DIR_SRC)/evc_df.c \
		$(DIR_SRC)/evc_picman.c \
		$(DIR_SRC)/evcd.c \
		$(DIR_SRC)/evcd_eco.c \
		$(DIR_SRC)/evcd_util.c \
		$(DIR_SRC)/evcd_bsr.c \
		$(DIR_SRC)/evce.c \
		$(DIR_SRC)/evce_eco.c \
		$(DIR_SRC)/evce_mode.c \
		$(DIR_SRC)/evce_pinter.c \
		$(DIR_SRC)/evce_pintra.c \
		$(DIR_SRC)/evce_tq.c \
		$(DIR_SRC)/evce_util.c \
		$(DIR_SRC)/evce_tbl.c \
		$(DIR_SRC)/evce_bsw.c \
		$(DIR_SRC)/evce_sad.c \
		$(DIR_SRC)/evce_pibc.c \
		

CSRCS_APP =	$(DIR_APP)/evca_encoder.c \
			$(DIR_APP)/evca_decoder.c \
			$(DIR_APP)/evca_bitstream_merge.c

OBJS = $(subst .c,.o,$(subst $(DIR_SRC)/,,$(CSRCS)))
HEADS = $(subst .c,.h,$(subst $(DIR_SRC)/,,$(CSRCS)))
CXXOBJS = $(subst .cpp,.o,$(subst $(DIR_SRC)/,,$(CXXSRCS)))
CXXHEADS = $(subst .cpp,.h,$(subst $(DIR_SRC)/,,$(CXXSRCS)))

all: RELEASE

RELEASE: Makefile $(CSRCS) $(CXXHEADS) $(CXXOBJS) $(OBJS)
	$(MKDIR) $(DIR_LIB)
	$(AR) $(ARFLAGS) $(DIR_LIB)/libevc.a *.o
	$(MKDIR) $(DIR_BIN)
	$(CC) -o $(DIR_BIN)/evca_encoder $(CFLAGS) $(DIR_APP)/evca_encoder.c $(LDFLAGS)
	$(CC) -o $(DIR_BIN)/evca_decoder $(CFLAGS) $(DIR_APP)/evca_decoder.c $(LDFLAGS)
	$(CC) -o $(DIR_BIN)/evca_bitstream_merge $(CFLAGS) $(DIR_APP)/evca_bitstream_merge.c $(LDFLAGS)

.PHONY: clean
clean:
	$(RM) $(OBJS) $(CXXOBJS) $(DIR_LIB) $(DIR_BIN) *.o *.s *.i *.ii *.d

.PHONY: $(CXXHEADS) $(HEADS)

%.o: $(DIR_SRC)/%.c $(DIR_DEP)/%.d
	$(CC) $(DEPFLAGS) -c -o $(subst $(DIR_SRC)/,,$@) $(CFLAGS) $<
	$(POSTCOMPILE)

%.o: $(DIR_SRC)/%.cpp $(DIR_DEP)/%.d
	$(CXX) $(DEPFLAGS) -c -o $(subst $(DIR_SRC)/,,$@) $(CXXFLAGS) $<
	$(POSTCOMPILE)

$(DIR_DEP)/%.d: ;
.PRECIOUS: $(DIR_DEP)/%.d

include $(wildcard $(patsubst %,$(DIR_DEP)/%.d,$(basename $(CSRCS) $(CXXSRCS))))
