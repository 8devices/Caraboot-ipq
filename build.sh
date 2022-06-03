#!/bin/bash

help(){
    echo "Usage: $0 board-name [ make-args ... ]"
    echo "valid board names: cherry"
    exit
}

maxsize=0

if [[ $# -lt 1 ]]
then
    help
else
    case $1 in
        cherry)
            config="cherry"
            maxsize=655360
            ;;
        help|--help|-h|*)
            help
            ;;
    esac
fi

echo "Building bootloader using config: '$config'"

shift

if [ -z $CROSS_COMPILE ]; then
	CROSS_COMPILE="../buildroot/output/host/usr/bin/arm-linux-"
fi
CROSS_COMPILE=$(realpath $CROSS_COMPILE)

make ipq5018_"$config"_config
STAGING_DIR="" CROSS_COMPILE=$CROSS_COMPILE make "$@"

tools/mbn/elf2mbn.py -i u-boot -o u-boot-$config.mbn

size=$(stat -c%s u-boot-$config.mbn)
if [ $size -gt $maxsize ]; then
	echo "WARNING: image size too big, do not flash it!"
fi

# Avoid accidentally flashing u-boot.bin
cp u-boot-$config.mbn u-boot.bin



