CFLAGS_vinc-neon.o = -ffreestanding -mfpu=neon -mfloat-abi=softfp -O3

obj-y += math/

vinc-objs += vinc-driver.o vinc-ctrls.o vinc-hw.o vinc-neon.o math/math.o

obj-$(CONFIG_VIDEO_VINC) += vinc.o
