export MCU_PLUS_SDK_PATH?=$(abspath ../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/appimage_x509_cert_gen.py


# User needs to point to the path where the unsigned ATF (bl31), OPTEE (bl32.bin)
# and A53 SPL (u-boot-spl.bin) binaries are build.
# User needs to build these binaries as mentioned in the Linux SDK documentation
# The prebuilt images available with Linux SDK is the signed ATF, OPTEE and A53 SPL,
# but we requires unsigned images to create the linux.appimage and sign the final image
# for HS-FS/HS devices
PSDK_LINUX_IMAGE_PATH=$(HOME)/prebuilt_spl

#Path for prebuit images in Processor SDK linux
PSDK_LINUX_PREBUILT_IMAGES=$(PSDK_LINUX_IMAGE_PATH)

#Input linux binaries
ATF_BIN_NAME=bl31.bin
OPTEE_BIN_NAME=bl32.bin
SPL_BIN_NAME=u-boot-spl.bin

#Linux image load address
ATF_LOAD_ADDR=0x0701a0000
OPTEE_LOAD_ADDR=0x9e800000
SPL_LOAD_ADDR=0x80080000

#Output appimage name
LINUX_BOOTIMAGE_NAME=linux.appimage