#ifndef IPQ806X_NAND_H
#define IPQ806X_NAND_H

int ipq_nand_scan(struct mtd_info *mtd);
int ipq_nand_init(struct mtd_info *mtd);
int ipq_nand_post_scan_init(struct mtd_info *mtd);

#endif
