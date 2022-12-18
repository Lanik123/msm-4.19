# dtb-$(CONFIG_ARCH_SDM439) += 
# dtb-$(CONFIG_ARCH_MSM8917) +=
# dtb-$(CONFIG_ARCH_SDM429) += 

ifeq ($(CONFIG_MACH_XIAOMI_SDM439),y)
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
subdir-y += xiaomi
endif # CONFIG_BUILD_ARM64_DT_OVERLAY
endif # CONFIG_MACH_XIAOMI_SDM439

always		:= $(dtb-y)
subdir-y	+= $(dts-dirs)
clean-files    := *.dtb *.dtbo
