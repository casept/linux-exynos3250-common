#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/system.h>
#include <mach/regs-pmu.h>
#include <mach/gpio-exynos.h>
#include <linux/gpio.h>
#include "common.h"
#include <mach/sec_debug.h>

/* TODO: Need to move to header file */
#define GPIO_POWER_BUTTON	EXYNOS3_GPX2(7)

static void sec_power_off(void)
{
	int poweroff_try = 0;
#if !defined (CONFIG_KEYBOARD_S2MPW01)
	int powerkey_gpio = GPIO_POWER_BUTTON;
#endif
	local_irq_disable();
	sec_debug_set_upload_magic(0x0, NULL);

	while (1) {
		/* Check reboot charging */
		if ((is_cable_attached || poweroff_try >= 5)) {
			pr_emerg
			    ("%s: charger connected(%d) or power"
			     "off failed(%d), reboot!\n",
			     __func__, is_cable_attached, poweroff_try);
			/* To enter LP charging */
			writel(0x0, EXYNOS_INFORM2);

			flush_cache_all();
			outer_flush_all();
			exynos5_restart(0, 0);

			pr_emerg("%s: waiting for reboot\n", __func__);
			while (1)
				;
		}
/* New PMIC(S2MPW01) do not support HW power key,
	so it is impossible to use gpio to check power key state */
#if defined (CONFIG_KEYBOARD_S2MPW01)
		if (1) { /*TODO:: Need to replace to proper function */
#else
		/* wait for power button release */
		if (gpio_get_value(powerkey_gpio)) {
#endif
			pr_emerg("%s: set PS_HOLD low\n", __func__);

			/* power off code
			 * PS_HOLD Out/High -->
			 * Low PS_HOLD_CONTROL, R/W, 0x1002_330C
			 */
			writel(readl(EXYNOS_PS_HOLD_CONTROL) & 0xFFFFFEFF,
			       EXYNOS_PS_HOLD_CONTROL);

			pr_emerg
			    ("%s: Should not reach here! (poweroff_try:%d)\n",
			     __func__, poweroff_try);
		} else {
		/* if power button is not released, wait and check TA again */
			pr_info("%s: PowerButton is not released.\n", __func__);
		}
		++poweroff_try;

		mdelay(1000);
	}
}

#define REBOOT_MODE_PREFIX	0x12345670
#define REBOOT_MODE_NONE	0
#define REBOOT_MODE_DOWNLOAD	1
#define REBOOT_MODE_UPLOAD	2
#define REBOOT_MODE_CHARGING	3
#define REBOOT_MODE_RECOVERY	4
#define REBOOT_MODE_FOTA	5
#define REBOOT_MODE_FOTA_BL	6	/* update bootloader */
#define REBOOT_MODE_SECURE	7	/* image secure check fail */
#define REBOOT_MODE_SILENT	0xa	/* silent reboot */
#define REBOOT_MODE_W_DOWNLOAD	0xb	/* wireless-download */
#define REBOOT_MODE_W_UPLOAD	0xc	/* wireless-upload */
#define REBOOT_MODE_W_DOWNLOAD_UPDATE	0xd	/* wireless-download update binary */
#define REBOOT_MODE_W_DOWNLOAD_RW_UPDATE	0xe	/* wireless-download rw update binary */
#define REBOOT_MODE_TUP			0x50555400  /* Tizen Update Partition */
#define REBOOT_MODE_TUP_UPDATE	0x50555401  /* TUP for update */

#define REBOOT_SET_PREFIX	0xabc00000
#define REBOOT_SET_SALES_CODE  0x000a0000
#define REBOOT_SET_DEBUG	0x000d0000
#define REBOOT_SET_SWSEL	0x000e0000
#define REBOOT_SET_SUD		0x000f0000

#define REBOOT_SALES_CODE_BASE_PHYS     (0x40000000)
#define REBOOT_SALES_CODE_MAGIC_ADDR    (phys_to_virt(REBOOT_SALES_CODE_BASE_PHYS))
#define REBOOT_SALES_CODE_ADDR          (phys_to_virt(REBOOT_SALES_CODE_BASE_PHYS+4))

static void sec_reboot(char str, const char *cmd)
{
	local_irq_disable();
	sec_debug_set_upload_magic(0x0, NULL);

	pr_emerg("%s (%d, %s)\n", __func__, str, cmd ? cmd : "(null)");

	writel(0x12345678, EXYNOS_INFORM2);	/* Don't enter lpm mode */

	if (!cmd) {
		writel(REBOOT_MODE_PREFIX | REBOOT_MODE_NONE, EXYNOS_INFORM3);
	} else {
		unsigned long value;
		if (!strcmp(cmd, "fota"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_FOTA,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "fota_bl"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_FOTA_BL,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "recovery"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_RECOVERY,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "wdownload"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_W_DOWNLOAD,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "wdownload_update"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_W_DOWNLOAD_UPDATE,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "wdownload_rw_update"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_W_DOWNLOAD_RW_UPDATE,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "download"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_DOWNLOAD,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "upload"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_UPLOAD,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "wupload"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_W_UPLOAD,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "tup"))
			writel(REBOOT_MODE_TUP_UPDATE, EXYNOS_INFORM3);
		else if (!strcmp(cmd, "secure"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_SECURE,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "silent"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_SILENT,
			       EXYNOS_INFORM3);
		else if (!strncmp(cmd, "customcsc", 9) && (cmd + 9) != NULL) {
			memset(REBOOT_SALES_CODE_MAGIC_ADDR, 0, 8);
			writel(REBOOT_SET_PREFIX | REBOOT_SET_SALES_CODE, REBOOT_SALES_CODE_MAGIC_ADDR);
			snprintf(REBOOT_SALES_CODE_ADDR, 4, "%s", cmd + 9);
			writel(REBOOT_SET_PREFIX | REBOOT_SET_SALES_CODE, EXYNOS_INFORM3);
		}
		else if (!strncmp(cmd, "debug", 5)
			 && !kstrtoul(cmd + 5, 0, &value))
			writel(REBOOT_SET_PREFIX | REBOOT_SET_DEBUG | value,
			       EXYNOS_INFORM3);
		else if (!strncmp(cmd, "swsel", 5)
			 && !kstrtoul(cmd + 5, 0, &value))
			writel(REBOOT_SET_PREFIX | REBOOT_SET_SWSEL | value,
			       EXYNOS_INFORM3);
		else if (!strncmp(cmd, "sud", 3)
			 && !kstrtoul(cmd + 3, 0, &value))
			writel(REBOOT_SET_PREFIX | REBOOT_SET_SUD | value,
			       EXYNOS_INFORM3);
		else if (!strncmp(cmd, "emergency", 9))
			writel(0, EXYNOS_INFORM3);
		else
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_NONE,
			       EXYNOS_INFORM3);
	}

	flush_cache_all();
	outer_flush_all();

	exynos5_restart(0, 0);

	pr_emerg("%s: waiting for reboot\n", __func__);
	while (1)
		;
}

static int __init sec_reboot_init(void)
{
	pm_power_off = sec_power_off;
	arm_pm_restart = sec_reboot;
	return 0;
}

subsys_initcall(sec_reboot_init);
