Caraboot-ipq
========

U-Boot for Cherry (IPQ50xx) based boards


Build
-------

----
1) Build toolchain (http://buildroot.org)

```
git clone git://git.buildroot.net/buildroot
cd buildroot
git checkout 2018.08.x
make menuconfig
```
Select options:
* Target options -> Target Architecture -> ARM (little endian)
* Target options -> Target Architecture Variant -> cortex-A7
* Target options -> Target ABI -> EABI
* Toolchain -> GCC compiler Version -> gcc 5.x

Save and exit.

Build:
```
make toolchain
```

```
sudo apt-get install device-tree-compiler
```
----
2) Build Caraboot (U-Boot)

```
cd ..
git clone https://github.com/8devices/Caraboot-ipq.git
cd Caraboot-ipq/
git checkout 8dev/cherry
./build.sh BOARD_NAME  #valid BOARD_NAMEs: cherry
```

If your toolchain is located elsewhere, you can build like this:
```
CROSS_COMPILE=/your_toolchain_dir/buildroot/output/host/usr/bin/arm-linux- ./build.sh BOARD_NAME
```


The bootloader binary will be saved to ```u-boot-cherry.mbn```. Use this file to upgrade bootloader on your board.

Upgrade
-------

```diff
- ATTENTION! Bootloader upgrade is a risky operation and should not be performed unless
- absolutely necessary!
- If anything goes wrong during this process (power shortage, mistype, bad compile etc.)
- the board will be BRICKED (broken).
- Keep in mind that warranty does not cover module if it is damaged due to user's fault.
```

Make sure that you can reach your TFTP server over ethernet conncetion. Set `ipaddr` and `serverip` environment variables according to you network setup.
Use these commands to upgrade from u-boot shell:

Cherry:
```
tftpboot 44000000 u-boot-cherry.mbn
sf probe; sf erase 0x110000 0xa0000; sf write 44000000 0x110000 ${filesize}
```
