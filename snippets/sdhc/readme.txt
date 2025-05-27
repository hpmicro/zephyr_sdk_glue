This driver used to run sdxc driver on hpmicro boards, default runs sd-card;
if you want to run emmc:
- board hpm6800evk
(1)please add CONFIG_HPMICRO_BOARD_SUPPORT_EMMC=y on sdk_glue/snippets/sdhc/sdhc.conf
(2)change aliases sdhc0 = &emmc on sdk_glue/snippets/sdhc/boards/hpm6800evk.overlay
(3)change &sd status and sdmmc status both as disabled, same time change &emmc status and sdmmc status both as okay
- board hpm6750evk2
(1)please add CONFIG_HPMICRO_BOARD_SUPPORT_EMMC=y on sdk_glue/snippets/sdhc/sdhc.conf