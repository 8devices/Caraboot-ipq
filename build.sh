#!/bin/bash

if [ -z $CROSS_COMPILE ]; then
	CROSS_COMPILE="../buildroot/output/host/usr/bin/arm-linux-"
fi
CROSS_COMPILE=$(realpath $CROSS_COMPILE)

make ipq40xx_cdp_config
STAGING_DIR="" CROSS_COMPILE=$CROSS_COMPILE make "$@"

cp u-boot u-boot-jalapeno.elf
"$CROSS_COMPILE"strip u-boot-jalapeno.elf

size=$(stat -c%s u-boot-jalapeno.elf)
if [ $size -gt 524288 ]; then
	echo "WARNING: image size too big, do not flash it!"
fi

# Avoid accidentally flashing u-boot.bin
cp u-boot-jalapeno.elf u-boot.bin

