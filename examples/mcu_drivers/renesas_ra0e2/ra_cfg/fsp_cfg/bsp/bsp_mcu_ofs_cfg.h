/* generated configuration header file - do not edit */
#ifndef BSP_MCU_OFS_CFG_H_
#define BSP_MCU_OFS_CFG_H_
#ifndef BSP_CFG_OPTION_SETTING_OFS0
#define OFS_IWDT (0xFFFFA001 | 0 << 1 | 3 << 2 | 5 << 4 | 3 << 8 | 3 << 10 | 1 << 12 | 1 << 14)
#define BSP_CFG_OPTION_SETTING_OFS0  (OFS_IWDT)
#endif
#ifndef BSP_CFG_OPTION_SETTING_OFS1
#define BSP_CFG_OPTION_SETTING_OFS1_NO_HOCOFRQ  (0xE0000FC3 | (1 <<2) | (6 << 3)  \
                     | (1 << 15) | (0x3F << 22) | (0x01 << 16)  \
                     | (1 << 28))

#define BSP_CFG_OPTION_SETTING_OFS1  ((uint32_t) BSP_CFG_OPTION_SETTING_OFS1_NO_HOCOFRQ | ((uint32_t) BSP_CFG_HOCO_FREQUENCY << BSP_FEATURE_BSP_OFS1_HOCOFRQ_OFFSET))
#endif
#endif /* BSP_MCU_OFS_CFG_H_ */
