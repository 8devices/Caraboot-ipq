#!/bin/bash

help(){
    echo "Usage: $0 board-name [ make-args ... ]"
    echo "valid board names: jalapeno habanero"
    exit
}

maxsize=0

if [[ $# -lt 1 ]]
then
    help
else
    case $1 in
        jalapeno)
            config=$1
            maxsize=524288
            ;;
        habanero)
            config=$1
            maxsize=524288
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

make ipq40xx_"$config"_config
STAGING_DIR="" CROSS_COMPILE=$CROSS_COMPILE make "$@"

cp u-boot u-boot-$config.elf
"$CROSS_COMPILE"strip u-boot-$config.elf

size=$(stat -c%s u-boot-$config.elf)
if [ $size -gt $maxsize ]; then
	echo "WARNING: image size too big, do not flash it!"
fi

# Avoid accidentally flashing u-boot.bin
cp u-boot-$config.elf u-boot.bin



