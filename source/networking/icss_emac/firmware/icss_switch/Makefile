# standalone makefile for pru-icss-fw-emac
CODEGEN_INCLUDE = $(CL_PRU_INSTALL_PATH)/include
CODEGEN_BIN_INCLUDE = $(CL_PRU_INSTALL_PATH)/lib
CCS_PRU_INCLUDE = $(SDK_INSTALL_PATH)/ccsv6/ccs_base/pru/include
CC = $(CL_PRU_INSTALL_PATH)/bin/clpru
AR = $(CL_PRU_INSTALL_PATH)/bin/arpru
LNK = $(CL_PRU_INSTALL_PATH)/bin/clpru
SIZE = $(CL_PRU_INSTALL_PATH)/bin/ofdpru
STRIP = $(CL_PRU_INSTALL_PATH)/bin/strippru -p
HEX = $(CL_PRU_INSTALL_PATH)/bin/hexpru

HEX_LNKCMD_DIR = ../cmd

SRCDIR += ./src ../icss_dualemac/src
INCDIR += ./src ../icss_dualemac/src ./
# Common source files across all platforms and cores
SRCS += resource_table.c
SRCS_ASM += micro_scheduler.asm emac_MII_Rcv.asm emac_MII_Xmt.asm emac_statistics.asm switch_collision_task.asm emac_ptp.asm

LNKFLAGS_LOCAL_COMMON = --entry_point=micro_scheduler

LNKCMD_FILE = ./src/pru.cmd

# Internal CFLAGS - normally doesn't change
CFLAGS_INTERNAL = -v3 -g --endian=$(ENDIAN)
CFLAGS_INTERNAL += --diag_wrap=off --diag_warning=225 --display_error_number --hardware_mac=on --preproc_with_compile -eo.$(OBJEXT) 

ENDIAN = little
LNKFLAGS_INTERNAL_BUILD_PROFILE = -v3 -g --endian=$(ENDIAN) --diag_wrap=off --diag_warning=225 --display_error_number --hardware_mac=on -z --stack_size=0 --heap_size=0 --reread_libs --warn_sections

VPATH = $(SRCDIR)
OBJS = $(patsubst %.c, %.o, $(SRCS))
OBJS_ASM = $(patsubst %.asm, %.o, $(SRCS_ASM))

FW_NAME = icss_stp_switch
FW_ELF_EXT = .elf
FW_BIN_EXT = .bin

PRU_VERSION_LIST = REV1 REV2
PRU_CORE_LIST = PRU0 PRU1
PRU_HOST_LIST = ARM C66

BASE_ELF_DIR = ./elf
BASE_BIN_DIR = ./bin/$(PRU_HOST)
ELF_DIR = $(BASE_ELF_DIR)/$(PRU_VERSION)
BIN_DIR = $(BASE_BIN_DIR)/$(PRU_VERSION)
HEX_LNKCMD_FILE = $(HEX_LNKCMD_DIR)/$(PRU_CORE)_$(PRU_VERSION)_to_$(PRU_HOST).cmd
PRU_FIRMWARE_ELF = $(ELF_DIR)/$(FW_NAME)_$(PRU_CORE)$(FW_ELF_EXT)
PRU_FIRMWARE_BIN = $(BIN_DIR)/$(FW_NAME)_$(PRU_CORE)$(FW_BIN_EXT)

OBJEXT = opru
OBJDIR = ./obj/$(PRU_VERSION)/$(PRU_CORE)
OBJ_PATHS = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(SRCS))
OBJ_PATHS_ASM = $(patsubst %.asm, $(OBJDIR)/%.$(OBJEXT), $(SRCS_ASM))

CFLAGS_DIROPTS = -fr=$(OBJDIR) -fs=$(OBJDIR)
CFLAGS_PRU = -v3 -DICSS_$(PRU_VERSION) -D$(PRU_CORE) -DICSS_SWITCH_BUILD -DPRU -DTWO_PORT_CFG -DICSS_STP_SWITCH -DPTP

CFLAGS = $(CFLAGS_PRU) $(CFLAGS_INTERNAL)
INCLUDE_DIRS = $(INCDIR) $(CODEGEN_INCLUDE)
INCLUDES = $(addprefix -I,$(INCLUDE_DIRS))
LNKFLAGS =  $(LNKFLAGS_INTERNAL_BUILD_PROFILE) $(LNKFLAGS_LOCAL_COMMON)

PRU_ALL_TARGET = _pru_versions
PRU_FIRMWARE = $(PRU_FIRMWARE_ELF)
ifneq ($(PRU_HOST_LIST),)
    PRU_ALL_TARGET = _pru_hosts
    PRU_FIRMWARE += $(PRU_FIRMWARE_BIN)
endif


# This sort of recursion does not work with parallel make
.NOTPARALLEL:

.PHONY: all _pru_revs _pru_cores $(PRU_VERSION_LIST) $(PRU_CORE_LIST) $(PRU_HOST_LIST)
all: version $(PRU_ALL_TARGET)

_pru_hosts: $(PRU_HOST_LIST)
_pru_versions: $(PRU_VERSION_LIST)
_pru_cores: $(PRU_CORE_LIST)
_pru_firmware: $(PRU_FIRMWARE)

version:
	@./makeversion.sh

$(PRU_HOST_LIST) :
	@$(MAKE) _pru_versions PRU_HOST=$@

$(PRU_VERSION_LIST) :
	@$(MAKE) _pru_cores PRU_VERSION=$@

$(PRU_CORE_LIST) :
	@$(MAKE) _pru_firmware PRU_CORE=$@


$(OBJDIR) $(dir $(PRU_FIRMWARE_ELF) $(PRU_FIRMWARE_BIN)) :
	mkdir -p $@

$(OBJ_PATHS) : $(OBJDIR)/%.$(OBJEXT): %.c | $(OBJDIR) $(DEPDIR)
	@echo "Compiling $< ... (PWD = $(PWD)"
	@$(CC) $(CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) $<

$(OBJ_PATHS_ASM) : $(OBJDIR)/%.$(OBJEXT): %.asm | $(OBJDIR) $(DEPDIR)
	@echo "Assembling $< ..."
	@$(CC) $(CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) $<

$(PRU_FIRMWARE_ELF) : $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(LNKCMD_FILE) | $(dir $(PRU_FIRMWARE_ELF))
	@echo "Linking $^ into $@ ..."
	@$(LNK) $(LNKFLAGS) $^ -o $@ -m $@.map --xml_link_info="$@_linkInfo.xml" $(LNK_INCLUDES)
	@$(STRIP) $@

$(PRU_FIRMWARE_BIN) : $(PRU_FIRMWARE_ELF) $(HEX_LNKCMD_FILE) | $(dir $(PRU_FIRMWARE_BIN))
	$(HEX) $^ -o $@

clean:
	@find . -name "*\.$(OBJEXT)" -exec rm -v {} \;
	@find . -type d -empty -delete
	@rm -rf $(BIN_DIR) $(ELF_DIR)
	@rm -f ./src/.version
