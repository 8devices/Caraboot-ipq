#include <common.h>
#include <watchdog.h>
#include <asm/io.h>

#define WDT_TIMEOUT 	30	/* in seconds */
#define WDT_RATE 	32000

#define WDT_RST		0x0
#define WDT_EN		0x8
#define WDT_BARK_TIME	0x10
#define WDT_BITE_TIME	0x14
#define WDT_BASE	0xB017000

struct qcom_wdt {
	unsigned long		rate;
	void __iomem		*base;
	void __iomem		*wdt_reset;
	void __iomem		*wdt_enable;
	void __iomem		*wdt_bark_time;
	void __iomem		*wdt_bite_time;
};

void qcom_watchdog_init(void)
{
	struct qcom_wdt wdt_data;
	struct qcom_wdt *wdt;

	wdt = &wdt_data;
	wdt->base = (void __iomem*) WDT_BASE;
	wdt->wdt_reset = wdt->base + WDT_RST;
	wdt->wdt_enable = wdt->base + WDT_EN;
	wdt->wdt_bark_time = wdt->base + WDT_BARK_TIME;
	wdt->wdt_bite_time = wdt->base + WDT_BITE_TIME;
	wdt->rate = WDT_RATE;

	writel(0, wdt->wdt_enable);
	writel(1, wdt->wdt_reset);
	writel(((WDT_TIMEOUT - 1) * wdt->rate), wdt->wdt_bark_time);
	writel(WDT_TIMEOUT * wdt->rate, wdt->wdt_bite_time);
	writel(1, wdt->wdt_enable);
}
