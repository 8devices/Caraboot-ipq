/*
 * Copyright (c) 2015-2017 The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <ubi_uboot.h>
#include <linux/stddef.h>
#include <linux/compat.h>
#include <asm-generic/errno.h>
#include <asm/io.h>
#include <asm/arch-qca-common/scm.h>
#include <common.h>

/**
 * alloc_scm_command() - Allocate an SCM command
 * @cmd_size: size of the command buffer
 * @resp_size: size of the response buffer
 *
 * Allocate an SCM command, including enough room for the command
 * and response headers as well as the command and response buffers.
 *
 * Returns a valid &scm_command on success or %NULL if the allocation fails.
 */
static struct scm_command *alloc_scm_command(size_t cmd_size, size_t resp_size)
{
	struct scm_command *cmd;
	size_t len;

	len = sizeof(*cmd) + sizeof(struct scm_response) + cmd_size;
	if (cmd_size > len)
		return NULL;

	len += resp_size;
	if (resp_size > len)
		return NULL;

	cmd = kzalloc(PAGE_ALIGN(len), GFP_KERNEL);
	if (cmd) {
		cmd->len = len;
		cmd->buf_offset = offsetof(struct scm_command, buf);
		cmd->resp_hdr_offset = cmd->buf_offset + cmd_size;
	}
	return cmd;
}

/**
 * free_scm_command() - Free an SCM command
 * @cmd: command to free
 *
 * Free an SCM command.
 */
static inline void free_scm_command(struct scm_command *cmd)
{
	kfree(cmd);
}

/**
 * scm_command_to_response() - Get a pointer to a scm_response
 * @cmd: command
 *
 * Returns a pointer to a response for a command.
 */
static inline struct scm_response *scm_command_to_response(
		const struct scm_command *cmd)
{
	return (void *)cmd + cmd->resp_hdr_offset;
}

/**
 * scm_get_command_buffer() - Get a pointer to a command buffer
 * @cmd: command
 *
 * Returns a pointer to the command buffer of a command.
 */
static inline void *scm_get_command_buffer(const struct scm_command *cmd)
{
	return (void *)cmd->buf;
}

/**
 * scm_get_response_buffer() - Get a pointer to a response buffer
 * @rsp: response
 *
 * Returns a pointer to a response buffer of a response.
 */
static inline void *scm_get_response_buffer(const struct scm_response *rsp)
{
	return (void *)rsp + rsp->buf_offset;
}

static int scm_remap_error(int err)
{
	switch (err) {
	case SCM_ERROR:
		return -EIO;
	case SCM_EINVAL_ADDR:
	case SCM_EINVAL_ARG:
		return -EINVAL;
	case SCM_EOPNOTSUPP:
		return -EOPNOTSUPP;
	case SCM_ENOMEM:
		return -ENOMEM;
	case SCM_EBUSY:
		return -EBUSY;
	}
	return -EINVAL;
}

static u32 smc(u32 cmd_addr)
{
	int context_id;
	register u32 r0 asm("r0") = 1;
	register u32 r1 asm("r1") = (u32)&context_id;
	register u32 r2 asm("r2") = cmd_addr;
	do {
		asm volatile(
			__asmeq("%0", "r0")
			__asmeq("%1", "r0")
			__asmeq("%2", "r1")
			__asmeq("%3", "r2")
			".arch_extension sec\n"
			"smc	#0	@ switch to secure world\n"
			: "=r" (r0)
			: "r" (r0), "r" (r1), "r" (r2)
			: "r3");
	} while (r0 == SCM_INTERRUPTED);

	return r0;
}

static int __scm_call(const struct scm_command *cmd)
{
	int ret;
	u32 cmd_addr = virt_to_phys((void *)cmd);

	/*
	 * Flush the entire cache here so callers don't have to remember
	 * to flush the cache when passing physical addresses to the secure
	 * side in the buffer.
	 */
	flush_dcache_all();
	ret = smc(cmd_addr);
	if (ret < 0)
		ret = scm_remap_error(ret);

	return ret;
}

static u32 cacheline_size;

static void scm_inv_range(unsigned long start, unsigned long end)
{
	start = round_down(start, cacheline_size);
	end = round_up(end, cacheline_size);
	while (start < end) {
		asm ("mcr p15, 0, %0, c7, c6, 1" : : "r" (start)
		     : "memory");
		start += cacheline_size;
	}
}

/**
 * scm_call() - Send an SCM command
 * @svc_id: service identifier
 * @cmd_id: command identifier
 * @cmd_buf: command buffer
 * @cmd_len: length of the command buffer
 * @resp_buf: response buffer
 * @resp_len: length of the response buffer
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 */
int scm_call(u32 svc_id, u32 cmd_id, const void *cmd_buf, size_t cmd_len,
		void *resp_buf, size_t resp_len)
{
	int ret;
	struct scm_command *cmd;
	struct scm_response *rsp;
	unsigned long start, end;

	cmd = alloc_scm_command(cmd_len, resp_len);
	if (!cmd)
		return -ENOMEM;

	cmd->id = (svc_id << 10) | cmd_id;
	if (cmd_buf)
		memcpy(scm_get_command_buffer(cmd), cmd_buf, cmd_len);

	ret = __scm_call(cmd);

	if (ret)
		goto out;

	rsp = scm_command_to_response(cmd);
	start = (unsigned long)rsp;

	do {
		scm_inv_range(start, start + sizeof(*rsp));
	} while (!rsp->is_complete);

	end = (unsigned long)scm_get_response_buffer(rsp) + resp_len;
	scm_inv_range(start, end);

	if (resp_buf)
		memcpy(resp_buf, scm_get_response_buffer(rsp), resp_len);
out:
	free_scm_command(cmd);
	return ret;
}

int scm_init(void)
{
	u32 ctr;

	asm volatile("mrc p15, 0, %0, c0, c0, 1" : "=r" (ctr));
	cacheline_size =  4 << ((ctr >> 16) & 0xf);
	return 0;
}

#ifdef CONFIG_SCM_TZ64

int __qca_scm_call_armv8_32(u32 x0, u32 x1, u32 x2, u32 x3, u32 x4, u32 x5,
				u32 *ret1, u32 *ret2, u32 *ret3)
{
	register u32 r0 __asm__("r0") = x0;
	register u32 r1 __asm__("r1") = x1;
	register u32 r2 __asm__("r2") = x2;
	register u32 r3 __asm__("r3") = x3;
	register u32 r4 __asm__("r4") = x4;
	register u32 r5 __asm__("r5") = x5;

	asm volatile(
		__asmeq("%0", "r0")
		__asmeq("%1", "r1")
		__asmeq("%2", "r2")
		__asmeq("%3", "r3")
		__asmeq("%4", "r0")
		__asmeq("%5", "r1")
		__asmeq("%6", "r2")
		__asmeq("%7", "r3")
		__asmeq("%8", "r4")
		__asmeq("%9", "r5")
		".arch_extension sec\n"
		"smc    #0\n"
		: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
		: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4),
		"r" (r5));

	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	if (ret3)
		*ret3 = r3;

	return r0;
}


/**
 * scm_call_64() - Invoke a syscall in the secure world
 *  <at> svc_id: service identifier
 *  <at> cmd_id: command identifier
 *  <at> fn_id: The function ID for this syscall
 *  <at> desc: Descriptor structure containing arguments and return values
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 *
 */
static int scm_call_64(u32 svc_id, u32 cmd_id, struct qca_scm_desc *desc)
{
	int arglen = desc->arginfo & 0xf;
	int ret;
	u32 fn_id = QCA_SCM_SIP_FNID(svc_id, cmd_id);


	if (arglen > QCA_MAX_ARG_LEN) {
		printf("Error Extra args not supported\n");
		hang();
	}

	desc->ret[0] = desc->ret[1] = desc->ret[2] = 0;

	ret = __qca_scm_call_armv8_32(fn_id, desc->arginfo,
			desc->args[0], desc->args[1],
			desc->args[2], desc->x5,
			&desc->ret[0], &desc->ret[1],
			&desc->ret[2]);

	return ret;
}

void __attribute__ ((noreturn)) jump_kernel64(void *kernel_entry,
				void *fdt_addr)
{
	struct qca_scm_desc desc = {0};
	int ret = 0;
	kernel_params param = {0};

	param.reg_x0 = (u32)fdt_addr;
	param.kernel_start = (u32)kernel_entry;

	desc.arginfo = QCA_SCM_ARGS(2, SCM_READ_OP);
	desc.args[0] = (u32) &param;
	desc.args[1] = sizeof(param);

	printf("Jumping to AARCH64 kernel via monitor\n");
	ret = scm_call_64(SCM_ARCH64_SWITCH_ID, SCM_EL1SWITCH_CMD_ID,
		&desc);

	printf("Can't boot kernel: %d\n", ret);
	hang();
}

#endif

int qca_scm_call(u32 svc_id, u32 cmd_id, void *buf, size_t len)
{
	struct qca_scm_desc desc = {0};
	int ret = 0;

	desc.arginfo = QCA_SCM_ARGS(2, SCM_READ_OP);
	desc.args[0] = (u32)buf;
	desc.args[1] = len;
#ifdef CONFIG_SCM_TZ64
	ret = scm_call_64(svc_id, cmd_id, &desc);
#else
	ret = scm_call(svc_id, cmd_id, NULL, 0, buf, len);
#endif
	return ret;
}

int qca_scm_call_read(u32 svc_id, u32 cmd_id, u32 *addr, u32 *rsp)
{
	struct qca_scm_desc desc = {0};
	int ret = 0;

	desc.arginfo = QCA_SCM_ARGS(1, SCM_READ_OP);
	desc.args[0] = (u32)addr;
#ifdef CONFIG_SCM_TZ64
	ret = scm_call_64(svc_id, cmd_id, &desc);
	if (!ret)
		*rsp = desc.ret[0];
#endif
	return ret;
}

int qca_scm_call_write(u32 svc_id, u32 cmd_id, u32 *addr, u32 val)
{
	struct qca_scm_desc desc = {0};
	int ret = 0;

	desc.arginfo = QCA_SCM_ARGS(2, SCM_READ_OP);
	desc.args[0] = (u32)addr;
	desc.args[1] = val;
#ifdef CONFIG_SCM_TZ64
	ret = scm_call_64(svc_id, cmd_id, &desc);
#endif
	return ret;
}

int qca_scm_sdi_v8(void)
{
	struct qca_scm_desc desc = {0};
	int ret;

	desc.args[0] = 1ul;    /* Disable wdog debug */
	desc.args[1] = 0ul;    /* SDI Enable */
	desc.arginfo = QCA_SCM_ARGS(2, SCM_VAL, SCM_VAL);
	ret = scm_call_64(SCM_SVC_BOOT,
			     SCM_CMD_TZ_CONFIG_HW_FOR_RAM_DUMP_ID, &desc);

	if (ret)
		return ret;

	return le32_to_cpu(desc.ret[0]);
}
