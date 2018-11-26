/*
 * USB boot functions for IPQ40xx
 *
 * Copyright (C) 2014-2019 Mantas Pucka <mantas@8devices.com>
 *
 * SPDX-License-Identifier:GPL-2.0
 */

#include <common.h>
#include <command.h>
#include <part.h>
#include <usb.h>
#include <fat.h>
#include <asm/io.h>
#include <asm/arch-ipq40xx/iomap.h>
#include <asm/arch-qcom-common/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

#define XMK_STR(x) #x
#define MK_STR(x) XMK_STR(x)


int read_reset_button()
{
	uint32_t val;

	gpio_tlmm_config(CONFIG_RESET_BUTTON_GPIO, 0, GPIO_OUT_HIGH, GPIO_PULL_UP,
			 GPIO_2MA, GPIO_OE_DISABLE, GPIO_VM_ENABLE, GPIO_OD_DISABLE, GPIO_PULL_RES2);
	udelay(5);
	val = readl(GPIO_IN_OUT_ADDR(CONFIG_RESET_BUTTON_GPIO));

	return !(val & 0x1);
}

int usbboot_boot(char * boot_dev_part, char * boot_file_name)
{
	int fat_read_ret;

	char* fatload_argv[] = {"fatload", "usb", boot_dev_part,
				CFG_USB_RECOVERY_LOAD_ADDR,
				boot_file_name,
				CFG_USB_RECOVERY_MAX_BOOT_SIZE};
	char* bootm_argv[] = {"bootm", CFG_USB_RECOVERY_LOAD_ADDR};

	fat_read_ret = do_fat_fsload(NULL, 0, 6, fatload_argv);
	if (fat_read_ret == 0){
		printf ("Booting image %s from usb device %s \n",
			boot_file_name, boot_dev_part);
		do_bootm(NULL, 0 , 2, bootm_argv);
	}
	return -1; //Boot failed
}

int usbboot_recovery(char * boot_dev_part, char * boot_file_name)
{
	int fat_read_ret;
	char flash_cmd [128];
	char image_size[16];
	long long size = 0;

	char* fatload_argv[] = {"fatload", "usb", boot_dev_part,
				CFG_USB_RECOVERY_LOAD_ADDR,
				boot_file_name,
				CFG_USB_RECOVERY_MAX_FILE_SIZE};
	fat_read_ret = do_fat_fsload(NULL, 0, 6, fatload_argv);

	if (fat_read_ret == 0){
		printf ("Flashing image %s (%llu bytes) from usb device %s \n",
			boot_file_name , size, boot_dev_part);
		sprintf(image_size,"%x", (unsigned int)size);

		// Example command
		//nand erase.part ubi; nand write 0x80060000 ubi 3FA000"
		sprintf(flash_cmd,"nand device %s; nand erase.part %s; nand write %s %s %s",
			  CFG_USB_RECOVERY_NAND_DEVICE,
			  CFG_USB_RECOVERY_FW_PART_NAME,
			  CFG_USB_RECOVERY_LOAD_ADDR,
			  CFG_USB_RECOVERY_FW_PART_NAME,
			  "${filesize}");
		printf ("\nFlashing image\n%s\n", flash_cmd);
		setenv("recovery_flash_cmd", flash_cmd);
		int  argc_flash=2;
		char* argv_flash[]={"run", "recovery_flash_cmd"};
		
		int flash_ret = do_run(NULL, 0, argc_flash, argv_flash);
		printf ("\nFlashing sucsessful. Remove USB storage with recovery image.");
		do_reset(NULL, 0, 0, NULL);
	}
	return -1; //Boot failed
}

int usbboot_scan()
{
	int i;
	char part_id[8];
	block_dev_desc_t *stor_dev = NULL;
	disk_partition_t disk_part_info;
	disk_partition_t *info = &disk_part_info;
	char* boot_file_name; //Can include directories eg. "boot/vmlinux.uimage"
	char* recovery_file_name;
	char* boot_dev_part; 
	/* boot_dev_part format "<usb device>:<partition>", 
	 * eg. "0:1" - frst usb storage device's first partition
	 * */

	boot_file_name = getenv ("bootfile");
	if (boot_file_name == NULL)
		boot_file_name = CFG_USB_BOOT_FILENAME;
	
	recovery_file_name = getenv ("recoveryfile");
	if (recovery_file_name == NULL)
		recovery_file_name = CFG_USB_RECOVERY_FILENAME;

	boot_dev_part = getenv ("bootdev");
	
	usb_stop();

	if (usb_init() < 0)
		return -1;
	/* try to recognize storage devices immediately */
	usb_stor_scan(0);

	int devno = 0;

	//if given device name just boot it
	if (boot_dev_part != NULL){
		usbboot_recovery(boot_dev_part, recovery_file_name);
		usbboot_boot(boot_dev_part, boot_file_name);
		return -2; //if returns, means boot failed
	}

	// scan all devices and partitions for boot image
	for (devno = 0; ; ++devno) {
		stor_dev = usb_stor_get_dev(devno);
		if (stor_dev == NULL)
			break;

		//try boot from full device (no partitions)
		sprintf(part_id,"%d:%d", devno, 0);
		usbboot_recovery(part_id, recovery_file_name);
		usbboot_boot(part_id, boot_file_name);

		//scan partitions
		if (stor_dev->part_type != PART_TYPE_DOS)
			continue;
		for (i=1; i<=CFG_USB_BOOT_MAX_PARTITIONS_SCAN; i++){
			printf("SCAN partition %d\n",i);

			if (get_partition_info_dos(stor_dev, i, info) == 0){
				printf("Partition start %lu size %lu name %.32s\n", info->start, info->size, info->name);
				sprintf(part_id,"%d:%d", devno, i);
				usbboot_recovery(part_id, recovery_file_name);
				usbboot_boot(part_id, boot_file_name);
			}
		}
	}
	printf ("Boot or recovery image (%s or %s) not found in usb storage\n", boot_file_name, recovery_file_name);
	return -1;
}

int do_usb_boot (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char* boot_mode;
	int s_force=0, s_gpio=0, s_never = 0;

	boot_mode = getenv ("usbboot");
	if (boot_mode != NULL){
		s_force = !(strcmp(boot_mode, "force"));
		s_gpio = !(strcmp(boot_mode, "gpio"));
		s_never = !(strcmp(boot_mode, "never"));
	}

	if ( (s_force && s_never == 0) || (boot_mode == NULL))
		s_gpio = 1;
	
	if (s_never){
		printf("USB boot is disabled in environment\n");
		return 0;
	}
	if (s_force){
		printf("USB boot is forced in environment\n");
		usbboot_scan();
	}

	if ( s_gpio ){
		debug("USB GPIO: %d\n", read_reset_button());
		if (read_reset_button()==1)
			usbboot_scan();
		}

	return 0;
}

U_BOOT_CMD(
	usb_boot_file,	5,	1,	do_usb_boot,
	"usb_boot_file - Automatic boot/recovery from file in USB drive\n",
	" "
);

