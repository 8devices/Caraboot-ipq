Caraboot-ipq
========

U-Boot for Jalapeno (IPQ40xx) based boards


Build
-------

----
1) Build toolchain (http://buildroot.org)

```
git clone git://git.buildroot.net/buildroot
cd buildroot
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
----
2) Build Caraboot (U-Boot)

```
cd ..
git clone https://github.com/8devices/Caraboot-ipq.git
cd Caraboot-ipq/
git checkout 8dev/jalapeno
./build.sh BOARD_NAME  #valid BOARD_NAMEs: jalapeno, habanero
```

If your toolchain is located elsewhere, you can build like this:
```
CROSS_COMPILE=/your_toolchain_dir/buildroot/output/host/usr/bin/arm-linux- ./build.sh BOARD_NAME
```


The bootloader binary will be saved to ```u-boot-jalapeno.elf``` or ```u-boot-habanero.elf```. Use this file to upgrade bootloader on your board.

Upgrade
-------

Make sure that you can reach your TFTP server over ethernet conncetion. Set `ipaddr` and `serverip` environment variables according to you network setup.
Use these commands to upgrade from u-boot shell:

Jalapeno:
```
tftpboot 84000000 u-boot-jalapeno.elf
sf probe
sf erase 0xf0000 +0x80000
sf write 0x84000000 0xf0000 ${filesize}
```
Habanero:
```
tftpboot 84000000 u-boot-habanero.elf
sf probe
sf erase 0xf0000 +0x80000
sf write 0x84000000 0xf0000 ${filesize}
```
