
export MCU_PLUS_SDK_PATH=$(abspath ../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak

DOXYGEN ?= doxygen
export DEVICE ?= am64x
export API_GUIDE_OUT_DIR=$(MCU_PLUS_SDK_PATH)/docs/api_guide_$(DEVICE)

all:
	$(MAKE) -C $(MCU_PLUS_SDK_PATH)/test -f makefile.$(DEVICE) doxy_samples_$(DOC_COMBO)
	$(MKDIR) $(API_GUIDE_OUT_DIR)
	$(DOXYGEN) doxygen.cfg 2> doxy_warnings_$(DEVICE).txt
	cd theme && $(COPY) *.png $(API_GUIDE_OUT_DIR)
	cd videos && $(COPY) *.mp4 $(API_GUIDE_OUT_DIR)


clean:
	$(MAKE) -C $(MCU_PLUS_SDK_PATH)/test -f makefile.$(DEVICE) doxy_samples_$(DOC_COMBO)_clean
	$(RMDIR) $(API_GUIDE_OUT_DIR)

help:
	@echo "# Supported build targets:"
	@echo "# ========================"
	@echo "# $(MAKE) -s all DEVICE=<device>"
	@echo "# $(MAKE) -s clean DEVICE=<device>"
	@echo "# $(MAKE) -s help"
	@echo "#"

.PHONY: all clean help
