#include <common.h>
#include <net.h>
#include <asm-generic/errno.h>
#include <asm/io.h>
#include <malloc.h>
#include <phy.h>
#include "ipq_phy.h"
#include "rtl826xb_patch.h"
#include "conf_rtl8264b.c"

#define PHY_PATCH_WAIT_TIMEOUT_MS     10000

#define REG_FIELD_GET(data, offset, mask)	((data & (mask)) >> (offset))
#define REG_FIELD_SET(data, val, offset, mask)	((data & ~(mask)) | ((val << (offset)) & (mask)))

#define SSDK_ERROR printf
#ifdef DEBUG
#define SSDK_DEBUG printf
#else
#define SSDK_DEBUG(...)
#endif

#define PHY_INVALID_DATA 0xffff
#define PHY_RTN_ON_READ_ERROR(phy_data) \
    do { if (phy_data == PHY_INVALID_DATA) return(SW_READ_ERROR); } while(0);

#define PHY_RTN_ON_ERROR(rv) \
    do { if (rv != SW_OK) return(rv); } while(0);

extern int ipq_mdio_write(int mii_id,
		int regnum, u16 value);
extern int ipq_mdio_read(int mii_id,
		int regnum, ushort *data);

u16 rtl8221_phy_mmd_read(u32 dev_id, u32 phy_id, u32 mmd_addr, u32 mmd_reg)
{
	u32 reg = (1<<30) | ((mmd_addr & 0x1f) << 16) | (mmd_reg & 0xffff);
	return ipq_mdio_read(phy_id, reg, NULL);
}

u16 rtl8221_phy_mmd_write(u32 dev_id, u32 phy_id, u32 mmd_addr, u32 mmd_reg, u16 value)
{
	u32 reg = (1<<30) | ((mmd_addr & 0x1f) << 16) | (mmd_reg & 0xffff);
	return ipq_mdio_write(phy_id, reg, value);
}

int
rtl826xb_patch_mask_get(u8 msb, u8 lsb, u32 *mask)
{
	u8 i = 0;

	if ((msb > 15) || (lsb > 15) || (msb < lsb))
	{
		return SW_BAD_VALUE;
	}

	*mask = 0;
	for (i = lsb; i <= msb; i++) {
		*mask |= (1 << i);
	}

	return SW_OK;
}

int
rtl826xb_patch_wait(u32 dev_id, u32 phy_addr, u32 mmd_addr, u32 mmd_reg, u32 data, u32 mask)
{
	u16 phy_data = 0;
	int rv = SW_OK;
	int timeout = PHY_PATCH_WAIT_TIMEOUT_MS;

	while (timeout > 0) {
		timeout--;
		phy_data = rtl8221_phy_mmd_read(dev_id, phy_addr, mmd_addr, mmd_reg);
		//PHY_RTN_ON_READ_ERROR(phy_data);

		if ((phy_data & mask) == data)
			break;

		mdelay(1);
	}

	if(timeout <= 0)
	{
		SSDK_ERROR("dev:%u phy_addr:%u 826XB patch wait[%u,0x%X,0x%X,0x%X]:0x%X timeout:%u\n",
			   dev_id, phy_addr, mmd_addr, mmd_reg, data, mask, phy_data, timeout);
		return SW_TIMEOUT;
	}

	return rv;
}

int
rtl826xb_patch_wait_not_equal(u32 dev_id, u32 phy_addr, u32 mmd_addr, u32 mmd_reg, u32 data, u32 mask)
{
	u16 phy_data = 0;
	int rv = SW_OK;
	int timeout = PHY_PATCH_WAIT_TIMEOUT_MS;

	while (timeout > 0) {
		timeout--;
		phy_data = rtl8221_phy_mmd_read(dev_id, phy_addr, mmd_addr, mmd_reg);
		//PHY_RTN_ON_READ_ERROR(phy_data);

		if ((phy_data & mask) != data)
			break;

		mdelay(1);
	}

	if (timeout <= 0) {
		SSDK_ERROR("dev:%u phy_addr:%u 826xb patch wait[%u,0x%X,0x%X,0x%X]:0x%X timeout:%u\n",
			   dev_id, phy_addr, mmd_addr, mmd_reg, data, mask, phy_data, timeout);
		return SW_TIMEOUT;
	}

	return rv;
}

int
rtl826xb_patch_top_get(u32 dev_id, u32 phy_addr, u32 top_page, u32 top_reg, u32 *pdata)
{
	int rv = SW_OK;
	u16 phy_data = 0;
	u32 top_addr = (top_page * 8) + (top_reg - 16);

	phy_data = rtl8221_phy_mmd_read(dev_id, phy_addr, PHY_MMD_VEND1, top_addr);
	PHY_RTN_ON_READ_ERROR(phy_data);

	*pdata = phy_data;
	return rv;
}

int
rtl826xb_patch_top_set(u32 dev_id, u32 phy_addr, u32 top_page, u32 top_reg, u32 data)
{
	int rv = SW_OK;
	uint32_t top_addr = (top_page * 8) + (top_reg - 16);

	rv = rtl8221_phy_mmd_write(dev_id, phy_addr, PHY_MMD_VEND1, top_addr, data);
	return rv;
}

int
rtl826xb_patch_sds_get(u32 dev_id, u32 phy_id, u32 sds_page, u32 sds_reg, u32 *pdata)
{
	int rv = SW_OK;
	u32 data = 0;
	u32 sds_addr = 0x8000 + (sds_reg << 6) + sds_page;

	rv = rtl826xb_patch_top_set(dev_id, phy_id, 40, 19, sds_addr);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_top_get(dev_id, phy_id, 40, 18, &data);
	PHY_RTN_ON_ERROR(rv);

	*pdata = data;
	return rtl826xb_patch_wait(dev_id, phy_id, PHY_MMD_VEND1, 0x143, 0, BIT(15));
}

int
rtl826xb_patch_sds_set(u32 dev_id, u32 phy_id, u32 sds_page, u32 sds_reg, u32 data)
{
	int rv = SW_OK;
	u32 sds_addr = 0x8800 + (sds_reg << 6) + sds_page;

	rv = rtl826xb_patch_top_set(dev_id, phy_id, 40, 17, data);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_top_set(dev_id, phy_id, 40, 19, sds_addr);
	PHY_RTN_ON_ERROR(rv);
	return rtl826xb_patch_wait(dev_id, phy_id, PHY_MMD_VEND1, 0x143, 0, BIT(15));
}

int
rtl826xb_patch_process_op(u32 dev_id, u32 phy_id, rtk_phy_hwpatch_t *op)
{
	int rv = SW_OK;
	u32 mask = 0, data = 0;
	u16 phy_data = 0;

	rv = rtl826xb_patch_mask_get(op->msb, op->lsb, &mask);
	PHY_RTN_ON_ERROR(rv);

	switch (op->patch_op)
	{
		case RTK_HWPATCH_OP_PHY:
			if ((op->msb != 15) || (op->lsb != 0))
			{
				phy_data = rtl8221_phy_mmd_read(dev_id, phy_id, PHY_MMD_VEND2, op->addr);
				PHY_RTN_ON_READ_ERROR(phy_data);
			}
			data = REG_FIELD_SET(phy_data, op->data, op->lsb, mask);
			rv = rtl8221_phy_mmd_write(dev_id, phy_id, PHY_MMD_VEND2, op->addr, data);
			PHY_RTN_ON_ERROR(rv);

			break;

		case RTK_HWPATCH_OP_TOP:
			if ((op->msb != 15) || (op->lsb != 0))
			{
				rv = rtl826xb_patch_top_get(dev_id, phy_id, op->pagemmd, op->addr, &data);
				PHY_RTN_ON_ERROR(rv);
			}
			data = REG_FIELD_SET(data, op->data, op->lsb, mask);
			rv = rtl826xb_patch_top_set(dev_id, phy_id, op->pagemmd, op->addr, data);
			PHY_RTN_ON_ERROR(rv);

			break;

		case RTK_HWPATCH_OP_SDS:
			if ((op->msb != 15) || (op->lsb != 0))
			{
				rv = rtl826xb_patch_sds_get(dev_id, phy_id, op->pagemmd, op->addr, &data);
				PHY_RTN_ON_ERROR(rv);
			}
			data = REG_FIELD_SET(data, op->data, op->lsb, mask);

			rv = rtl826xb_patch_sds_set(dev_id, phy_id, op->pagemmd, op->addr, data);
			PHY_RTN_ON_ERROR(rv);

			break;

		case RTK_HWPATCH_OP_UNKNOWN:
		default:
			return SW_BAD_VALUE;
	}

	return SW_OK;
}

int
rtl826xb_patch_op(u32 dev_id, u32 phy_id, u8 patch_op, u8 portmask, u16 pagemmd,
		      u16 addr, u8 msb, u8 lsb, u16 data)
{
	rtk_phy_hwpatch_t op;

	op.patch_op = patch_op;
	op.portmask = portmask;
	op.pagemmd  = pagemmd;
	op.addr     = addr;
	op.msb      = msb;
	op.lsb      = lsb;
	op.data     = data;

	return rtl826xb_patch_process_op(dev_id, phy_id, &op);
}

int
rtl826xb_patch_process(u32 dev_id, u32 phy_id, rtk_phy_hwpatch_t *patch, int size, u32 *cnt)
{
	int rv = SW_OK;
	int i = 0;
	int n;

	if (size <= 0)
	{
		*cnt = 0;
		return SW_OK;
	}
	n = size/sizeof(rtk_phy_hwpatch_t);

	for (i = 0; i < n; i++)
	{
		rv = rtl826xb_patch_process_op(dev_id, phy_id, &patch[i]);
		if (rv) {
			SSDK_ERROR("dev:%u phy_addr:%u patch failed! i=%u rv=%d\n", dev_id, phy_id, i, rv);
		}
		PHY_RTN_ON_ERROR(rv);
	}
	*cnt = i;
	return SW_OK;
}

int
rtl826xb_patch(u32 dev_id, u32 phy_id)
{
	u32 cnt = 0;
	int rv = SW_OK;
	u16 phy_data = 0;

	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_TOP, 0xff, 90, 18, 15, 0, RTL8264B_MAIN_VER);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_TOP, 0xff, 90, 19, 15, 0, RTL8264B_SW_VER);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_TOP, 0xff, 90, 21, 15, 0, RTL8264B_TOP_VER);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_TOP, 0xff, 90, 21, 15, 0, RTL8264B_AFEFW_VER);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xb820, 15, 0, 0x0010);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_wait(dev_id, phy_id, PHY_MMD_VEND2, 0xB800, BIT(6), BIT(6));
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_process(dev_id, phy_id, rtl8264B_patch_fwpr_conf, sizeof(rtl8264B_patch_fwpr_conf), &cnt);
	if (rv) {
		SSDK_ERROR("dev:%u phy_addr:%u 826XB fwpr patch failed. rv:0x%X\n", dev_id, phy_id, rv);
	}
	PHY_RTN_ON_ERROR(rv);

	SSDK_DEBUG("dev:%u phy_addr:%u 826XB fwpr patch done. rv:0x%X cnt:%d\n", dev_id, phy_id, rv, cnt);

	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xb820, 15, 0, 0x0000);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_wait(dev_id, phy_id, PHY_MMD_VEND2, 0xB800, 0, BIT(6));
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xA4A0, 10, 10, 0x1);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_wait(dev_id, phy_id, PHY_MMD_VEND2, 0xa600, 0x1, 0xFF);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_process(dev_id, phy_id, rtl8264B_patch_fwlm_conf, sizeof(rtl8264B_patch_fwlm_conf), &cnt);
	if (rv)  {
		SSDK_ERROR("dev:%u phy_addr:%u 826XB fwlm patch failed. rv:0x%X\n", dev_id, phy_id, rv);
	}
	PHY_RTN_ON_ERROR(rv);
	SSDK_DEBUG("dev:%u phy_addr:%u 826XB fwlm patch done. rv:0x%X cnt:%d\n", dev_id, phy_id, rv, cnt);

	//xg_patch_en_flag
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 9, 9, 0x1);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 8, 8, 0x0);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 7, 7, 0x1);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 6, 6, 0x1);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 5, 5, 0x1);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 4, 4, 0x1);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 6, 6, 0x0);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 9, 9, 0x0);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 7, 7, 0x0);
	PHY_RTN_ON_ERROR(rv);

	phy_data = rtl8221_phy_mmd_read(dev_id, phy_id, PHY_MMD_VEND2, 0xbc62);
	PHY_RTN_ON_READ_ERROR(phy_data);
	phy_data = REG_FIELD_GET(phy_data, 8, 0x1F00);
	for (cnt = 0; cnt <= phy_data; cnt++)
	{
		rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbc62, 12, 8, cnt);
		PHY_RTN_ON_ERROR(rv);
	}

	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 6, 6, 0x1);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 9, 9, 0x1);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbf86, 7, 7, 0x1);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xbc04, 9, 2, 0xff);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xA4A0, 15, 0, 0x0180);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_wait_not_equal(dev_id, phy_id, PHY_MMD_VEND2, 0xa600, 0x1, 0xFF);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xA436, 15, 0, 0x801E);
	PHY_RTN_ON_ERROR(rv);
	rv = rtl826xb_patch_op(dev_id, phy_id, RTK_HWPATCH_OP_PHY, 0xff, 0x00, 0xA438, 15, 0, RTL8264B_FW_VER);
	PHY_RTN_ON_ERROR(rv);

	rv = rtl826xb_patch_process(dev_id, phy_id, rtl8264B_patch_afe_conf, sizeof(rtl8264B_patch_afe_conf), &cnt);
	if (rv)    {
		SSDK_ERROR("dev:%u phy_addr:%u 826XB afe patch failed. rv:0x%X\n", dev_id, phy_id, rv);
	}
	PHY_RTN_ON_ERROR(rv);

	SSDK_DEBUG("dev:%u phy_addr:%u 826XB afe patch done. rv:0x%X cnt:%d\n", dev_id, phy_id, rv, cnt);

	rv = rtl826xb_patch_process(dev_id, phy_id, rtl8264B_patch_top_conf, sizeof(rtl8264B_patch_top_conf), &cnt);
	if (rv) {
		SSDK_ERROR("dev:%u phy_addr:%u 826XB top patch failed. rv:0x%X\n", dev_id, phy_id, rv);
	}
	PHY_RTN_ON_ERROR(rv);

	SSDK_DEBUG("dev:%u phy_addr:%u 826XB top patch done. rv:0x%X cnt:%d\n", dev_id, phy_id, rv, cnt);

	rv = rtl826xb_patch_process(dev_id, phy_id, rtl8264B_patch_sds_conf, sizeof(rtl8264B_patch_sds_conf), &cnt);
	if (rv) {
		SSDK_ERROR("dev:%u phy_addr:%u 826XB sds patch failed. rv:0x%X\n", dev_id, phy_id, rv);
	}
	PHY_RTN_ON_ERROR(rv);

	SSDK_DEBUG("dev:%u phy_addr:%u 826XB sds patch done. rv:0x%X cnt:%d\n", dev_id, phy_id, rv, cnt);

	return rv;
}

