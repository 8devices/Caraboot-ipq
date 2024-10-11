#ifndef _RTL826XB_PATCH_H_
#define _RTL826XB_PATCH_H_

#define RTK_HWPATCH_OP_UNKNOWN 0
#define RTK_HWPATCH_OP_SDS     1
#define RTK_HWPATCH_OP_PHY     2
#define RTK_HWPATCH_OP_PHYW    3
#define RTK_HWPATCH_OP_ALGO    4
#define RTK_HWPATCH_OP_TOP     5
#define RTK_HWPATCH_OP_MMD     6
#define RTK_HWPATCH_OP_DATARAM 7

#define PHY_MMD_PMAPMD              1
#define PHY_MMD_PCS                 3
#define PHY_MMD_AN                  7
#define PHY_MMD_VEND1               30   /* Vendor specific 1 */
#define PHY_MMD_VEND2               31   /* Vendor specific 2 */

typedef struct rtk_phy_hwpatch_s
{
	uint8_t patch_op;
	uint8_t portmask;
	uint16_t pagemmd;
	uint16_t addr;
	uint8_t msb;
	uint8_t lsb;
	uint16_t data;
} rtk_phy_hwpatch_t;

int
rtl826xb_patch(u32 dev_id, u32 phy_id);

typedef enum {
	SW_OK              = 0,       /* Operation succeeded                 */
	SW_FAIL            = -1,      /* Operation failed                    */
	SW_BAD_VALUE       = -2,      /* Illegal value                       */
	SW_OUT_OF_RANGE    = -3,      /* Value is out of range               */
	SW_BAD_PARAM       = -4,      /* Illegal parameter(s)                */
	SW_BAD_PTR         = -5,      /* Illegal pointer value               */
	SW_BAD_LEN         = -6,      /* Wrong length                        */
	SW_BAD_STATE       = -7,      /* Wrong state of state machine        */
	SW_READ_ERROR      = -8,      /* Read operation failed               */
	SW_WRITE_ERROR     = -9,      /* Write operation failed              */
	SW_CREATE_ERROR    = -10,     /* Fail in creating an entry           */
	SW_DELETE_ERROR    = -11,     /* Fail in deleteing an entry          */
	SW_NOT_FOUND       = -12,     /* Entry not found                     */
	SW_NO_CHANGE       = -13,     /* The parameter(s) is the same        */
	SW_NO_MORE         = -14,     /* No more entry found                 */
	SW_NO_SUCH         = -15,     /* No such entry                       */
	SW_ALREADY_EXIST   = -16,     /* Tried to create existing entry      */
	SW_FULL            = -17,     /* Table is full                       */
	SW_EMPTY           = -18,     /* Table is empty                      */
	SW_NOT_SUPPORTED   = -19,     /* This request is not support         */
	SW_NOT_IMPLEMENTED = -20,     /* This request is not implemented     */
	SW_NOT_INITIALIZED = -21,     /* The item is not initialized         */
	SW_BUSY            = -22,     /* Operation is still running          */
	SW_TIMEOUT         = -23,     /* Operation Time Out                  */
	SW_DISABLE         = -24,     /* Operation is disabled               */
	SW_NO_RESOURCE     = -25,     /* Resource not available (memory ...) */
	SW_INIT_ERROR      = -26,     /* Error occured while INIT process    */
	SW_NOT_READY       = -27,     /* The other side is not ready yet     */
	SW_OUT_OF_MEM      = -28,     /* Cpu memory allocation failed.       */
	SW_ABORTED         = -29      /* Operation has been aborted.         */
} sw_error_t;

#endif /* _RTL826XB_PATCH_H_ */
