/* Copyright (c) 2009-2014, Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
enum msm_otg_phy_reg_mode {
	USB_PHY_REG_OFF,
	USB_PHY_REG_ON,
	USB_PHY_REG_LPM_ON,
	USB_PHY_REG_LPM_OFF,
};

static DECLARE_COMPLETION(pmic_vbus_init);
static struct msm_otg *the_msm_otg;
static bool debug_aca_enabled;
static bool debug_bus_voting_enabled;
static bool mhl_det_in_progress;

static struct regulator *hsusb_3p3;
static struct regulator *hsusb_1p8;
static struct regulator *hsusb_vdd;
static struct regulator *vbus_otg;
static struct regulator *mhl_usb_hs_switch;
static struct power_supply *psy;

static bool aca_id_turned_on;
static bool legacy_power_supply; // Mike, always false.
static inline bool aca_enabled(void)
{
#ifdef CONFIG_USB_MSM_ACA
	return true;
#else
	return debug_aca_enabled;
#endif
}

static int vdd_val[VDD_TYPE_MAX][VDD_VAL_MAX] = {
		{  /* VDD_CX CORNER Voting */
			[VDD_NONE]	= RPM_VREG_CORNER_NONE,
			[VDD_MIN]	= RPM_VREG_CORNER_NOMINAL,
			[VDD_MAX]	= RPM_VREG_CORNER_HIGH,
		},
		{ /* VDD_CX Voltage Voting */
			[VDD_NONE]	= USB_PHY_VDD_DIG_VOL_NONE,
			[VDD_MIN]	= USB_PHY_VDD_DIG_VOL_MIN,
			[VDD_MAX]	= USB_PHY_VDD_DIG_VOL_MAX,
		},
};

static int msm_hsusb_ldo_init(struct msm_otg *motg, int init)
{
	int rc = 0;

	if (init) {
		hsusb_3p3 = devm_regulator_get(motg->phy.dev, "HSUSB_3p3");
		if (IS_ERR(hsusb_3p3)) {
			dev_err(motg->phy.dev, "unable to get hsusb 3p3\n");
			return PTR_ERR(hsusb_3p3);
		}

		rc = regulator_set_voltage(hsusb_3p3, USB_PHY_3P3_VOL_MIN,
				USB_PHY_3P3_VOL_MAX);
		if (rc) {
			dev_err(motg->phy.dev, "unable to set voltage level for"
					"hsusb 3p3\n");
			return rc;
		}
		hsusb_1p8 = devm_regulator_get(motg->phy.dev, "HSUSB_1p8");
		if (IS_ERR(hsusb_1p8)) {
			dev_err(motg->phy.dev, "unable to get hsusb 1p8\n");
			rc = PTR_ERR(hsusb_1p8);
			goto put_3p3_lpm;
		}
		rc = regulator_set_voltage(hsusb_1p8, USB_PHY_1P8_VOL_MIN,
				USB_PHY_1P8_VOL_MAX);
		if (rc) {
			dev_err(motg->phy.dev, "unable to set voltage level "
					"for hsusb 1p8\n");
			goto put_1p8;
		}

		return 0;
	}

put_1p8:
	regulator_set_voltage(hsusb_1p8, 0, USB_PHY_1P8_VOL_MAX);
put_3p3_lpm:
	regulator_set_voltage(hsusb_3p3, 0, USB_PHY_3P3_VOL_MAX);
	return rc;
}

static int msm_hsusb_config_vddcx(int high)
{
	struct msm_otg *motg = the_msm_otg;
	enum usb_vdd_type vdd_type = motg->vdd_type;
	int max_vol = vdd_val[vdd_type][VDD_MAX];
	int min_vol;
	int ret;

	min_vol = vdd_val[vdd_type][!!high];
	ret = regulator_set_voltage(hsusb_vdd, min_vol, max_vol);

	pr_debug("%s: min_vol:%d max_vol:%d\n", __func__, min_vol, max_vol);

	return ret;
}

static int msm_hsusb_ldo_enable(struct msm_otg *motg,
	enum msm_otg_phy_reg_mode mode)
{
	int ret = 0;

	if (IS_ERR(hsusb_1p8)) {
		pr_err("%s: HSUSB_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(hsusb_3p3)) {
		pr_err("%s: HSUSB_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	switch (mode) {
	case USB_PHY_REG_ON:
		ret = regulator_set_optimum_mode(hsusb_1p8,
				USB_PHY_1P8_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator "
				"HSUSB_1p8\n", __func__);
			return ret;
		}

		ret = regulator_enable(hsusb_1p8);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to enable the hsusb 1p8\n",
				__func__);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_3p3,
				USB_PHY_3P3_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator "
				"HSUSB_3p3\n", __func__);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			regulator_disable(hsusb_1p8);
			return ret;
		}

		ret = regulator_enable(hsusb_3p3);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to enable the hsusb 3p3\n",
				__func__);
			regulator_set_optimum_mode(hsusb_3p3, 0);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			regulator_disable(hsusb_1p8);
			return ret;
		}

		break;

	case USB_PHY_REG_OFF:
		ret = regulator_disable(hsusb_1p8);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to disable the hsusb 1p8\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_1p8, 0);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator "
				"HSUSB_1p8\n", __func__);

		ret = regulator_disable(hsusb_3p3);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to disable the hsusb 3p3\n",
				 __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(hsusb_3p3, 0);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator "
				"HSUSB_3p3\n", __func__);

		break;

	case USB_PHY_REG_LPM_ON:
		ret = regulator_set_optimum_mode(hsusb_1p8,
				USB_PHY_1P8_LPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set LPM of the regulator: HSUSB_1p8\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_3p3,
				USB_PHY_3P3_LPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set LPM of the regulator: HSUSB_3p3\n",
				__func__);
			regulator_set_optimum_mode(hsusb_1p8, USB_PHY_REG_ON);
			return ret;
		}

		break;

	case USB_PHY_REG_LPM_OFF:
		ret = regulator_set_optimum_mode(hsusb_1p8,
				USB_PHY_1P8_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator: HSUSB_1p8\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_3p3,
				USB_PHY_3P3_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator: HSUSB_3p3\n",
				__func__);
			regulator_set_optimum_mode(hsusb_1p8, USB_PHY_REG_ON);
			return ret;
		}

		break;

	default:
		pr_err("%s: Unsupported mode (%d).", __func__, mode);
		return -ENOTSUPP;
	}

	pr_debug("%s: USB reg mode (%d) (OFF/HPM/LPM)\n", __func__, mode);
	return ret < 0 ? ret : 0;
}

static void msm_hsusb_mhl_switch_enable(struct msm_otg *motg, bool on)
{
	struct msm_otg_platform_data *pdata = motg->pdata;

	if (!pdata->mhl_enable)
		return;

	if (!mhl_usb_hs_switch) {
		pr_err("%s: mhl_usb_hs_switch is NULL.\n", __func__);
		return;
	}

	if (on) {
		if (regulator_enable(mhl_usb_hs_switch))
			pr_err("unable to enable mhl_usb_hs_switch\n");
	} else {
		regulator_disable(mhl_usb_hs_switch);
	}
}

static int ulpi_read(struct usb_phy *phy, u32 reg)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);
	int cnt = 0;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		dev_err(phy->dev, "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		dev_err(phy->dev, "PORTSC: %08x USBCMD: %08x\n",
			readl_relaxed(USB_PORTSC), readl_relaxed(USB_USBCMD));
		return -ETIMEDOUT;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct usb_phy *phy, u32 val, u32 reg)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);
	int cnt = 0;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		dev_err(phy->dev, "ulpi_write: timeout\n");
		dev_err(phy->dev, "PORTSC: %08x USBCMD: %08x\n",
			readl_relaxed(USB_PORTSC), readl_relaxed(USB_USBCMD));
		return -ETIMEDOUT;
	}
	return 0;
}

static struct usb_phy_io_ops msm_otg_io_ops = {
	.read = ulpi_read,
	.write = ulpi_write,
};

static void ulpi_init(struct msm_otg *motg)
{
	struct msm_otg_platform_data *pdata = motg->pdata;
	int aseq[10];
	int *seq = NULL;

	if (override_phy_init) {
		pr_debug("%s(): HUSB PHY Init:%s\n", __func__,
				override_phy_init);
		get_options(override_phy_init, ARRAY_SIZE(aseq), aseq);
		seq = &aseq[1];
	} else {
		seq = pdata->phy_init_seq;
	}

	if (!seq)
		return;

	while (seq[0] >= 0) {
		if (override_phy_init)
			pr_debug("ulpi: write 0x%02x to 0x%02x\n",
					seq[0], seq[1]);

		dev_vdbg(motg->phy.dev, "ulpi: write 0x%02x to 0x%02x\n",
				seq[0], seq[1]);
		ulpi_write(&motg->phy, seq[0], seq[1]);
		seq += 2;
	}
}

static int msm_otg_link_clk_reset(struct msm_otg *motg, bool assert)
{
	int ret;

	if (assert) {
		if (!IS_ERR(motg->clk)) {
			ret = clk_reset(motg->clk, CLK_RESET_ASSERT);
		} else {
			/* Using asynchronous block reset to the hardware */
			dev_dbg(motg->phy.dev, "block_reset ASSERT\n");
			clk_disable_unprepare(motg->pclk);
			clk_disable_unprepare(motg->core_clk);
			ret = clk_reset(motg->core_clk, CLK_RESET_ASSERT);
		}
		if (ret)
			dev_err(motg->phy.dev, "usb hs_clk assert failed\n");
	} else {
		if (!IS_ERR(motg->clk)) {
			ret = clk_reset(motg->clk, CLK_RESET_DEASSERT);
		} else {
			dev_dbg(motg->phy.dev, "block_reset DEASSERT\n");
			ret = clk_reset(motg->core_clk, CLK_RESET_DEASSERT);
			ndelay(200);
			clk_prepare_enable(motg->core_clk);
			clk_prepare_enable(motg->pclk);
		}
		if (ret)
			dev_err(motg->phy.dev, "usb hs_clk deassert failed\n");
	}
	return ret;
}

static int msm_otg_phy_reset(struct msm_otg *motg)
{
	u32 val;
	int ret;
	struct msm_otg_platform_data *pdata = motg->pdata;

	/*
	 * AHB2AHB Bypass mode shouldn't be enable before doing
	 * async clock reset. If it is enable, disable the same.
	 */
	val = readl_relaxed(USB_AHBMODE);
	if (val & AHB2AHB_BYPASS) {
		pr_err("%s(): AHB2AHB_BYPASS SET: AHBMODE:%x\n",
				__func__, val);
		val &= ~AHB2AHB_BYPASS_BIT_MASK;
		writel_relaxed(val | AHB2AHB_BYPASS_CLEAR, USB_AHBMODE);
		pr_err("%s(): AHBMODE: %x\n", __func__,
				readl_relaxed(USB_AHBMODE));
	}

	ret = msm_otg_link_clk_reset(motg, 1);
	if (ret)
		return ret;

	/* wait for 1ms delay as suggested in HPG. */
	usleep_range(1000, 1200);

	ret = msm_otg_link_clk_reset(motg, 0);
	if (ret)
		return ret;

	if (pdata && pdata->enable_sec_phy)
		writel_relaxed(readl_relaxed(USB_PHY_CTRL2) | (1<<16),
							USB_PHY_CTRL2);
	val = readl(USB_PORTSC) & ~PORTSC_PTS_MASK;
	writel(val | PORTSC_PTS_ULPI, USB_PORTSC);

	dev_info(motg->phy.dev, "phy_reset: success\n");
	return 0;
}

#define LINK_RESET_TIMEOUT_USEC		(250 * 1000)
static int msm_otg_link_reset(struct msm_otg *motg)
{
	int cnt = 0;
	struct msm_otg_platform_data *pdata = motg->pdata;

	writel_relaxed(USBCMD_RESET, USB_USBCMD);
	while (cnt < LINK_RESET_TIMEOUT_USEC) {
		if (!(readl_relaxed(USB_USBCMD) & USBCMD_RESET))
			break;
		udelay(1);
		cnt++;
	}
	if (cnt >= LINK_RESET_TIMEOUT_USEC)
		return -ETIMEDOUT;

	/* select ULPI phy */
	writel_relaxed(0x80000000, USB_PORTSC);
	writel_relaxed(0x0, USB_AHBBURST);
	writel_relaxed(0x08, USB_AHBMODE);

	if (pdata && pdata->enable_sec_phy)
		writel_relaxed(readl_relaxed(USB_PHY_CTRL2) | (1<<16),
								USB_PHY_CTRL2);
	return 0;
}

static void usb_phy_reset(struct msm_otg *motg)
{
	u32 val;

	if (motg->pdata->phy_type != SNPS_28NM_INTEGRATED_PHY)
		return;

	/* Assert USB PHY_PON */
	val =  readl_relaxed(USB_PHY_CTRL);
	val &= ~PHY_POR_BIT_MASK;
	val |= PHY_POR_ASSERT;
	writel_relaxed(val, USB_PHY_CTRL);

	/* wait for minimum 10 microseconds as suggested in HPG. */
	usleep_range(10, 15);

	/* Deassert USB PHY_PON */
	val =  readl_relaxed(USB_PHY_CTRL);
	val &= ~PHY_POR_BIT_MASK;
	val |= PHY_POR_DEASSERT;
	writel_relaxed(val, USB_PHY_CTRL);

	/* Ensure that RESET operation is completed. */
	mb();
}

static int msm_otg_reset(struct usb_phy *phy)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);
	struct msm_otg_platform_data *pdata = motg->pdata;
	int ret;
	u32 val = 0;
	u32 ulpi_val = 0;

	msm_otg_phy_reset(motg);

	aca_id_turned_on = false;
	msm_otg_link_reset(motg);

	msleep(100);

	/* Reset USB PHY after performing USB Link RESET */
	usb_phy_reset(motg);

	/* Program USB PHY Override registers. */
	ulpi_init(motg);

	/*
	 * It is recommended in HPG to reset USB PHY after programming
	 * USB PHY Override registers.
	 */
	usb_phy_reset(motg);

	if (pdata->otg_control == OTG_PHY_CONTROL) {
		val = readl_relaxed(USB_OTGSC);
		if (pdata->mode == USB_OTG) {
			ulpi_val = ULPI_INT_IDGRD | ULPI_INT_SESS_VALID;
			val |= OTGSC_IDIE | OTGSC_BSVIE;
		} else if (pdata->mode == USB_PERIPHERAL) {
			ulpi_val = ULPI_INT_SESS_VALID;
			val |= OTGSC_BSVIE;
		}
		writel_relaxed(val, USB_OTGSC);
		ulpi_write(phy, ulpi_val, ULPI_USB_INT_EN_RISE);
		ulpi_write(phy, ulpi_val, ULPI_USB_INT_EN_FALL);
	} else if (pdata->otg_control == OTG_PMIC_CONTROL) {
		ulpi_write(phy, OTG_COMP_DISABLE,
			ULPI_SET(ULPI_PWR_CLK_MNG_REG));
		/* Enable PMIC pull-up */
		pm8xxx_usb_id_pullup(1);
	}

	// remove un-ised flag: ALLOW_VDD_MIN_WITH_RETENTION_DISABLED)

	return 0;
}

static const char *timer_string(int bit)
{
	switch (bit) {
	case A_WAIT_VRISE:		return "a_wait_vrise";
	case A_WAIT_VFALL:		return "a_wait_vfall";
	case B_SRP_FAIL:		return "b_srp_fail";
	case A_WAIT_BCON:		return "a_wait_bcon";
	case A_AIDL_BDIS:		return "a_aidl_bdis";
	case A_BIDL_ADIS:		return "a_bidl_adis";
	case B_ASE0_BRST:		return "b_ase0_brst";
	case A_TST_MAINT:		return "a_tst_maint";
	case B_TST_SRP:			return "b_tst_srp";
	case B_TST_CONFIG:		return "b_tst_config";
	default:			return "UNDEFINED";
	}
}

static enum hrtimer_restart msm_otg_timer_func(struct hrtimer *hrtimer)
{
	struct msm_otg *motg = container_of(hrtimer, struct msm_otg, timer);

	switch (motg->active_tmout) {
	case A_WAIT_VRISE:
		/* TODO: use vbus_vld interrupt */
		set_bit(A_VBUS_VLD, &motg->inputs);
		break;
	case A_TST_MAINT:
		/* OTG PET: End session after TA_TST_MAINT */
		set_bit(A_BUS_DROP, &motg->inputs);
		break;
	case B_TST_SRP:
		/*
		 * OTG PET: Initiate SRP after TB_TST_SRP of
		 * previous session end.
		 */
		set_bit(B_BUS_REQ, &motg->inputs);
		break;
	case B_TST_CONFIG:
		clear_bit(A_CONN, &motg->inputs);
		break;
	default:
		set_bit(motg->active_tmout, &motg->tmouts);
	}

	pr_debug("expired %s timer\n", timer_string(motg->active_tmout));
	queue_work(system_nrt_wq, &motg->sm_work);
	return HRTIMER_NORESTART;
}

static void msm_otg_del_timer(struct msm_otg *motg)
{
	int bit = motg->active_tmout;

	pr_debug("deleting %s timer. remaining %lld msec\n", timer_string(bit),
			div_s64(ktime_to_us(hrtimer_get_remaining(
					&motg->timer)), 1000));
	hrtimer_cancel(&motg->timer);
	clear_bit(bit, &motg->tmouts);
}

static void msm_otg_start_timer(struct msm_otg *motg, int time, int bit)
{
	clear_bit(bit, &motg->tmouts);
	motg->active_tmout = bit;
	pr_debug("starting %s timer\n", timer_string(bit));
	hrtimer_start(&motg->timer,
			ktime_set(time / 1000, (time % 1000) * 1000000),
			HRTIMER_MODE_REL);
}

static void msm_otg_init_timer(struct msm_otg *motg)
{
	hrtimer_init(&motg->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	motg->timer.function = msm_otg_timer_func;
}

static int msm_otg_start_hnp(struct usb_otg *otg)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);

	if (otg->phy->state != OTG_STATE_A_HOST) {
		pr_err("HNP can not be initiated in %s state\n",
				otg_state_string(otg->phy->state));
		return -EINVAL;
	}

	pr_debug("A-Host: HNP initiated\n");
	clear_bit(A_BUS_REQ, &motg->inputs);
	queue_work(system_nrt_wq, &motg->sm_work);
	return 0;
}

static int msm_otg_start_srp(struct usb_otg *otg)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	u32 val;
	int ret = 0;

	if (otg->phy->state != OTG_STATE_B_IDLE) {
		pr_err("SRP can not be initiated in %s state\n",
				otg_state_string(otg->phy->state));
		ret = -EINVAL;
		goto out;
	}

	if ((jiffies - motg->b_last_se0_sess) < msecs_to_jiffies(TB_SRP_INIT)) {
		pr_debug("initial conditions of SRP are not met. Try again"
				"after some time\n");
		ret = -EAGAIN;
		goto out;
	}

	pr_debug("B-Device SRP started\n");

	/*
	 * PHY won't pull D+ high unless it detects Vbus valid.
	 * Since by definition, SRP is only done when Vbus is not valid,
	 * software work-around needs to be used to spoof the PHY into
	 * thinking it is valid. This can be done using the VBUSVLDEXTSEL and
	 * VBUSVLDEXT register bits.
	 */
	ulpi_write(otg->phy, 0x03, 0x97);
	/*
	 * Harware auto assist data pulsing: Data pulse is given
	 * for 7msec; wait for vbus
	 */
	val = readl_relaxed(USB_OTGSC);
	writel_relaxed((val & ~OTGSC_INTSTS_MASK) | OTGSC_HADP, USB_OTGSC);

	/* VBUS plusing is obsoleted in OTG 2.0 supplement */
out:
	return ret;
}

static void msm_otg_host_hnp_enable(struct usb_otg *otg, bool enable)
{
	struct usb_hcd *hcd = bus_to_hcd(otg->host);
	struct usb_device *rhub = otg->host->root_hub;

	if (enable) {
		pm_runtime_disable(&rhub->dev);
		rhub->state = USB_STATE_NOTATTACHED;
		hcd->driver->bus_suspend(hcd);
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	} else {
		usb_remove_hcd(hcd);
		msm_otg_reset(otg->phy);
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
	}
}

static int msm_otg_set_suspend(struct usb_phy *phy, int suspend)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);

	/*
	 * UDC and HCD call usb_phy_set_suspend() to enter/exit LPM
	 * during bus suspend/resume.  Update the relevant state
	 * machine inputs and trigger LPM entry/exit.  Checking
	 * in_lpm flag would avoid unnecessary work scheduling.
	 */
	if (suspend) {
		switch (phy->state) {
		case OTG_STATE_A_WAIT_BCON:
			if (TA_WAIT_BCON > 0)
				break;
			/* fall through */
		case OTG_STATE_A_HOST:
			pr_debug("host bus suspend\n");
			clear_bit(A_BUS_REQ, &motg->inputs);
			if (!atomic_read(&motg->in_lpm))
				queue_work(system_nrt_wq, &motg->sm_work);
			break;
		case OTG_STATE_B_PERIPHERAL:
			pr_debug("peripheral bus suspend\n");
			if (!(motg->caps & ALLOW_LPM_ON_DEV_SUSPEND))
				break;
			set_bit(A_BUS_SUSPEND, &motg->inputs);
			if (!atomic_read(&motg->in_lpm))
				queue_delayed_work(system_nrt_wq,
					&motg->suspend_work,
					USB_SUSPEND_DELAY_TIME);
			break;

		default:
			break;
		}
	} else {
		switch (phy->state) {
		case OTG_STATE_A_WAIT_BCON:
			/* Remote wakeup or resume */
			set_bit(A_BUS_REQ, &motg->inputs);
			/* ensure hardware is not in low power mode */
			if (atomic_read(&motg->in_lpm))
				pm_runtime_resume(phy->dev);
			break;
		case OTG_STATE_A_SUSPEND:
			/* Remote wakeup or resume */
			set_bit(A_BUS_REQ, &motg->inputs);
			phy->state = OTG_STATE_A_HOST;

			/* ensure hardware is not in low power mode */
			if (atomic_read(&motg->in_lpm))
				pm_runtime_resume(phy->dev);
			break;
		case OTG_STATE_B_PERIPHERAL:
			pr_debug("peripheral bus resume\n");
			if (!(motg->caps & ALLOW_LPM_ON_DEV_SUSPEND))
				break;
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			if (atomic_read(&motg->in_lpm))
				queue_work(system_nrt_wq, &motg->sm_work);
			break;
		default:
			break;
		}
	}
	return 0;
}

static void msm_otg_bus_vote(struct msm_otg *motg, enum usb_bus_vote vote)
{
	int ret;
	struct msm_otg_platform_data *pdata = motg->pdata;

	/* Check if target allows min_vote to be same as no_vote */
	if (pdata->bus_scale_table &&
	    vote >= pdata->bus_scale_table->num_usecases)
		vote = USB_NO_PERF_VOTE;

	if (motg->bus_perf_client) {
		ret = msm_bus_scale_client_update_request(
			motg->bus_perf_client, vote);
		if (ret)
			dev_err(motg->phy.dev, "%s: Failed to vote (%d)\n"
				   "for bus bw %d\n", __func__, vote, ret);
	}
}

#define PHY_SUSPEND_TIMEOUT_USEC	(500 * 1000)
#define PHY_RESUME_TIMEOUT_USEC	(100 * 1000)

#ifdef CONFIG_PM_SLEEP
/* Caller:
	 msm_otg_runtime_suspend <-outside
	 msm_otg_pm_suspend <- outside.
Check.
1. ->in_lpm?
2. delay_lpm_hndshk_on_disconnect?
3. disable irq
4. bool dcp = ? is that connecting to DCP charger?
  
*/
static int msm_otg_suspend(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	struct usb_bus *bus = phy->otg->host;
	struct msm_otg_platform_data *pdata = motg->pdata;
	int cnt = 0;
	bool host_bus_suspend, device_bus_suspend, dcp, prop_charger;
	bool floated_charger;
	u32 phy_ctrl_val = 0, cmd_val;
	unsigned ret;
	u32 portsc, config2;
	u32 func_ctrl;

	if (atomic_read(&motg->in_lpm))
		return 0;

	motg->ui_enabled = 0;
	disable_irq(motg->irq);

	host_bus_suspend = phy->otg->host && !test_bit(MHL, &motg->inputs) && !test_bit(ID, &motg->inputs);

	device_bus_suspend = phy->otg->gadget 
		&& test_bit(ID, &motg->inputs) && test_bit(A_BUS_SUSPEND, &motg->inputs) 
		&& motg->caps & ALLOW_LPM_ON_DEV_SUSPEND;

	dcp = (motg->chg_type == USB_DCP_CHARGER);
	prop_charger = motg->chg_type == USB_PROPRIETARY_CHARGER;
	floated_charger = motg->chg_type == USB_FLOATED_CHARGER;

	/* Enable line state difference wakeup fix for only device and host
	 * bus suspend scenarios.  Otherwise PHY can not be suspended when
	 * a charger that pulls DP/DM high is connected.
	 */
	config2 = readl_relaxed(USB_GENCONFIG2);
	if (device_bus_suspend)
		config2 |= GENCFG2_LINESTATE_DIFF_WAKEUP_EN;
	else
		config2 &= ~GENCFG2_LINESTATE_DIFF_WAKEUP_EN;
	writel_relaxed(config2, USB_GENCONFIG2);

	/*
	 * Abort suspend when,
	 * 1. charging detection in progress due to cable plug-in
	 * 2. host mode activation in progress due to Micro-A cable insertion
	 */

	if ((test_bit(B_SESS_VLD, &motg->inputs) && !device_bus_suspend &&
			!dcp && !prop_charger && !floated_charger) ||
		test_bit(A_BUS_REQ, &motg->inputs)) {
		motg->ui_enabled = 1;
		enable_irq(motg->irq);
		return -EBUSY;
	}

	// Remove phy_type is CI_45NM_INTEGRATED_PHY

	// Remove unused flag:  ALLOW_VDD_MIN_WITH_RETENTION_DISABLED) 

	/* Set the PHCD bit, only if it is not set by the controller.
	 * PHY may take some time or even fail to enter into low power
	 * mode (LPM). Hence poll for 500 msec and reset the PHY and link
	 * in failure case.
	 */
	portsc = readl_relaxed(USB_PORTSC);
	if (!(portsc & PORTSC_PHCD)) {
		writel_relaxed(portsc | PORTSC_PHCD,
				USB_PORTSC);
		while (cnt < PHY_SUSPEND_TIMEOUT_USEC) {
			if (readl_relaxed(USB_PORTSC) & PORTSC_PHCD)
				break;
			udelay(1);
			cnt++;
		}
	}

	if (cnt >= PHY_SUSPEND_TIMEOUT_USEC) {
		dev_err(phy->dev, "Unable to suspend PHY\n");
		msm_otg_reset(phy);
		motg->ui_enabled = 1;
		enable_irq(motg->irq);
		return -ETIMEDOUT;
	}

	/*
	 * PHY has capability to generate interrupt asynchronously in low
	 * power mode (LPM). This interrupt is level triggered. So USB IRQ
	 * line must be disabled till async interrupt enable bit is cleared
	 * in USBCMD register. Assert STP (ULPI interface STOP signal) to
	 * block data communication from PHY.
	 *
	 * PHY retention mode is disallowed while entering to LPM with wall
	 * charger connected.  But PHY is put into suspend mode. Hence
	 * enable asynchronous interrupt to detect charger disconnection when
	 * PMIC notifications are unavailable.
	 */
	cmd_val = readl_relaxed(USB_USBCMD);
	if (host_bus_suspend || device_bus_suspend ||
		(motg->pdata->otg_control == OTG_PHY_CONTROL))
		cmd_val |= ASYNC_INTR_CTRL | ULPI_STP_CTRL;
	else
		cmd_val |= ULPI_STP_CTRL;
	writel_relaxed(cmd_val, USB_USBCMD);

	/*
	 * BC1.2 spec mandates PD to enable VDP_SRC when charging from DCP.
	 * PHY retention and collapse can not happen with VDP_SRC enabled.
	 */
	if (motg->caps & ALLOW_PHY_RETENTION && !device_bus_suspend && !dcp &&
		 (!host_bus_suspend || ((motg->caps & ALLOW_HOST_PHY_RETENTION)
		&& (pdata->dpdm_pulldown_added || !(portsc & PORTSC_CCS))))) {
		phy_ctrl_val = readl_relaxed(USB_PHY_CTRL);
		if (motg->pdata->otg_control == OTG_PHY_CONTROL) {
			/* Enable PHY HV interrupts to wake MPM/Link */
			if ((motg->pdata->mode == USB_OTG) ||
					(motg->pdata->mode == USB_HOST))
				phy_ctrl_val |= (PHY_IDHV_INTEN |
							PHY_OTGSESSVLDHV_INTEN);
			else
				phy_ctrl_val |= PHY_OTGSESSVLDHV_INTEN;
		}
		if (host_bus_suspend)
			phy_ctrl_val |= PHY_CLAMP_DPDMSE_EN;

		// Remove unsed flag:  ALLOW_VDD_MIN_WITH_RETENTION_DISABLED)) {
		writel_relaxed(phy_ctrl_val & ~PHY_RETEN, USB_PHY_CTRL);
		motg->lpm_flags |= PHY_RETENTIONED;
	}

	/* Ensure that above operation is completed before turning off clocks */
	mb();
	/* Consider clocks on workaround flag only in case of bus suspend */
	if (!(phy->state == OTG_STATE_B_PERIPHERAL &&
		test_bit(A_BUS_SUSPEND, &motg->inputs)) ||
	    !motg->pdata->core_clk_always_on_workaround) {
		clk_disable_unprepare(motg->pclk);
		clk_disable_unprepare(motg->core_clk);
		motg->lpm_flags |= CLOCKS_DOWN;
	}

	/* usb phy no more require TCXO clock, hence vote for TCXO disable */
	if (!host_bus_suspend || ((motg->caps & ALLOW_HOST_PHY_RETENTION) &&
		(pdata->dpdm_pulldown_added || !(portsc & PORTSC_CCS)))) {
		if (!IS_ERR(motg->xo_clk)) {
			clk_disable_unprepare(motg->xo_clk);
			motg->lpm_flags |= XO_SHUTDOWN;
		} else {
			ret = msm_xo_mode_vote(motg->xo_handle,
							MSM_XO_MODE_OFF);
			if (ret)
				dev_err(phy->dev, "%s fail to devote XO %d\n",
								 __func__, ret);
			else
				motg->lpm_flags |= XO_SHUTDOWN;
		}
	}

	if (motg->caps & ALLOW_PHY_POWER_COLLAPSE &&
			!host_bus_suspend && !dcp) {
		msm_hsusb_ldo_enable(motg, USB_PHY_REG_OFF);
		motg->lpm_flags |= PHY_PWR_COLLAPSED;
	} else if (motg->caps & ALLOW_PHY_REGULATORS_LPM &&
			!host_bus_suspend && !device_bus_suspend && !dcp) {
		msm_hsusb_ldo_enable(motg, USB_PHY_REG_LPM_ON);
		motg->lpm_flags |= PHY_REGULATORS_LPM;
	}

	// Remove us-used flag: ALLOW_VDD_MIN_WITH_RETENTION_DISABLED

	if (device_may_wakeup(phy->dev)) {
		if (motg->async_irq)
			enable_irq_wake(motg->async_irq);
		else
			enable_irq_wake(motg->irq);

		if (motg->pdata->pmic_id_irq)
			enable_irq_wake(motg->pdata->pmic_id_irq);
		if (pdata->otg_control == OTG_PHY_CONTROL &&
			pdata->mpm_otgsessvld_int)
			msm_mpm_set_pin_wake(pdata->mpm_otgsessvld_int, 1);
		if (host_bus_suspend && pdata->mpm_dpshv_int)
			msm_mpm_set_pin_wake(pdata->mpm_dpshv_int, 1);
		if (host_bus_suspend && pdata->mpm_dmshv_int)
			msm_mpm_set_pin_wake(pdata->mpm_dmshv_int, 1);
	}
	if (bus)
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &(bus_to_hcd(bus))->flags);

	msm_otg_bus_vote(motg, USB_NO_PERF_VOTE);

	motg->host_bus_suspend = host_bus_suspend;
	atomic_set(&motg->in_lpm, 1);
	/* Enable ASYNC IRQ (if present) during LPM */
	if (motg->async_irq)
		enable_irq(motg->async_irq);

	/* XO shutdown during idle , non wakeable irqs must be disabled */
	if (device_bus_suspend || host_bus_suspend || !motg->async_irq) {
		motg->ui_enabled = 1;
		enable_irq(motg->irq);
	}
	wake_unlock(&motg->wlock);

	dev_info(phy->dev, "USB in low power mode\n");

	return 0;
}

// Caller:
//	msm_otg_runtime_resume <- export to outside.
//	msm_otg_pm_resume  <- export to outside.
static int msm_otg_resume(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	struct usb_bus *bus = phy->otg->host;
	struct msm_otg_platform_data *pdata = motg->pdata;
	int cnt = 0;
	unsigned temp;
	u32 phy_ctrl_val = 0;
	unsigned ret;
	u32 func_ctrl;

	if (!atomic_read(&motg->in_lpm))
		return 0;

	if (motg->pdata->delay_lpm_hndshk_on_disconnect)
		msm_bam_notify_lpm_resume();

	if (motg->ui_enabled) {
		motg->ui_enabled = 0;
		disable_irq(motg->irq);
	}
	wake_lock(&motg->wlock);

	/* Some platforms require BUS vote to enable/disable clocks */
	msm_otg_bus_vote(motg, USB_MIN_PERF_VOTE);

	/* Vote for TCXO when waking up the phy */
	if (motg->lpm_flags & XO_SHUTDOWN) {
		if (!IS_ERR(motg->xo_clk)) {
			clk_prepare_enable(motg->xo_clk);
		} else {
			ret = msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_ON);
			if (ret)
				dev_err(phy->dev, "%s fail to vote for XO %d\n",
								__func__, ret);
		}
		motg->lpm_flags &= ~XO_SHUTDOWN;
	}

	if (motg->lpm_flags & CLOCKS_DOWN) {
		clk_prepare_enable(motg->core_clk);
		clk_prepare_enable(motg->pclk);
		motg->lpm_flags &= ~CLOCKS_DOWN;
	}

	if (motg->lpm_flags & PHY_PWR_COLLAPSED) {
		msm_hsusb_ldo_enable(motg, USB_PHY_REG_ON);
		motg->lpm_flags &= ~PHY_PWR_COLLAPSED;
	} else if (motg->lpm_flags & PHY_REGULATORS_LPM) {
		msm_hsusb_ldo_enable(motg, USB_PHY_REG_LPM_OFF);
		motg->lpm_flags &= ~PHY_REGULATORS_LPM;
	}

	if (motg->lpm_flags & PHY_RETENTIONED ) {
		// no need to check ALLOW_VDD_MIN_WITH_RETENTION_DISABLED.

		msm_hsusb_mhl_switch_enable(motg, 1);
		msm_hsusb_config_vddcx(1);
		phy_ctrl_val = readl_relaxed(USB_PHY_CTRL);
		phy_ctrl_val |= PHY_RETEN;
		if (motg->pdata->otg_control == OTG_PHY_CONTROL)
			/* Disable PHY HV interrupts */
			phy_ctrl_val &=
				~(PHY_IDHV_INTEN | PHY_OTGSESSVLDHV_INTEN);
		phy_ctrl_val &= ~(PHY_CLAMP_DPDMSE_EN);
		writel_relaxed(phy_ctrl_val, USB_PHY_CTRL);
		motg->lpm_flags &= ~PHY_RETENTIONED;
	}

	temp = readl(USB_USBCMD);
	temp &= ~ASYNC_INTR_CTRL;
	temp &= ~ULPI_STP_CTRL;
	writel(temp, USB_USBCMD);

	/*
	 * PHY comes out of low power mode (LPM) in case of wakeup
	 * from asynchronous interrupt.
	 */
	if (!(readl(USB_PORTSC) & PORTSC_PHCD))
		goto skip_phy_resume;

	writel(readl(USB_PORTSC) & ~PORTSC_PHCD, USB_PORTSC);
	while (cnt < PHY_RESUME_TIMEOUT_USEC) {
		if (!(readl(USB_PORTSC) & PORTSC_PHCD))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= PHY_RESUME_TIMEOUT_USEC) {
		/*
		 * This is a fatal error. Reset the link and
		 * PHY. USB state can not be restored. Re-insertion
		 * of USB cable is the only way to get USB working.
		 */
		dev_err(phy->dev, "Unable to resume USB."
				"Re-plugin the cable\n");
		msm_otg_reset(phy);
	}

skip_phy_resume:
	if (motg->caps & ALLOW_VDD_MIN_WITH_RETENTION_DISABLED) {
		/* put the controller in normal mode */
		func_ctrl = ulpi_read(phy, ULPI_FUNC_CTRL);
		func_ctrl &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
		func_ctrl |= ULPI_FUNC_CTRL_OPMODE_NORMAL;
		ulpi_write(phy, func_ctrl, ULPI_FUNC_CTRL);
	}

	if (device_may_wakeup(phy->dev)) {
		if (motg->async_irq)
			disable_irq_wake(motg->async_irq);
		else
			disable_irq_wake(motg->irq);

		if (motg->pdata->pmic_id_irq)
			disable_irq_wake(motg->pdata->pmic_id_irq);
		if (pdata->otg_control == OTG_PHY_CONTROL &&
			pdata->mpm_otgsessvld_int)
			msm_mpm_set_pin_wake(pdata->mpm_otgsessvld_int, 0);
		if (motg->host_bus_suspend && pdata->mpm_dpshv_int)
			msm_mpm_set_pin_wake(pdata->mpm_dpshv_int, 0);
		if (motg->host_bus_suspend && pdata->mpm_dmshv_int)
			msm_mpm_set_pin_wake(pdata->mpm_dmshv_int, 0);
	}
	if (bus)
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &(bus_to_hcd(bus))->flags);

	atomic_set(&motg->in_lpm, 0);

	if (motg->async_int) {
		/* Match the disable_irq call from ISR */
		enable_irq(motg->async_int);
		motg->async_int = 0;
	}
	motg->ui_enabled = 1;
	enable_irq(motg->irq);

	/* If ASYNC IRQ is present then keep it enabled only during LPM */
	if (motg->async_irq)
		disable_irq(motg->async_irq);

	dev_info(phy->dev, "USB exited from low power mode\n");

	return 0;
}
#endif

static void msm_otg_notify_host_mode(struct msm_otg *motg, bool host_mode)
{
	if (!psy) {
		pr_err("No USB power supply registered!\n");
		return;
	}

	if (legacy_power_supply) {
		/* legacy support */
		if (host_mode) {
			power_supply_set_scope(psy, POWER_SUPPLY_SCOPE_SYSTEM);
		} else {
			power_supply_set_scope(psy, POWER_SUPPLY_SCOPE_DEVICE);
			/*
			 * VBUS comparator is disabled by PMIC charging driver
			 * when SYSTEM scope is selected.  For ID_GND->ID_A
			 * transition, give 50 msec delay so that PMIC charger
			 * driver detect the VBUS and ready for accepting
			 * charging current value from USB.
			 */
			if (test_bit(ID_A, &motg->inputs))
				msleep(50);
		}
	} else {
		motg->host_mode = host_mode;
		power_supply_changed(psy);
	}
}

static int msm_otg_notify_chg_type(struct msm_otg *motg)
{
	static int charger_type;

	/*
	 * TODO
	 * Unify OTG driver charger types and power supply charger types
	 */
	if (charger_type == motg->chg_type)
		return 0;

	if (motg->chg_type == USB_SDP_CHARGER)
		charger_type = POWER_SUPPLY_TYPE_USB;
	else if (motg->chg_type == USB_CDP_CHARGER)
		charger_type = POWER_SUPPLY_TYPE_USB_CDP;
	else if (motg->chg_type == USB_DCP_CHARGER ||
			motg->chg_type == USB_PROPRIETARY_CHARGER ||
			motg->chg_type == USB_FLOATED_CHARGER)
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
	else if ((motg->chg_type == USB_ACA_DOCK_CHARGER ||
		motg->chg_type == USB_ACA_A_CHARGER ||
		motg->chg_type == USB_ACA_B_CHARGER ||
		motg->chg_type == USB_ACA_C_CHARGER))
		charger_type = POWER_SUPPLY_TYPE_USB_ACA;
	else
		charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (!psy) {
		pr_err("No USB power supply registered!\n");
		return -EINVAL;
	}

	pr_debug("setting usb power supply type %d\n", charger_type);
	power_supply_set_supply_type(psy, charger_type);
	return 0;
}

static int msm_otg_notify_power_supply(struct msm_otg *motg, unsigned mA)
{
	if (!psy) {
		dev_dbg(motg->phy.dev, "no usb power supply registered\n");
		goto psy_error;
	}

	if (motg->cur_power == 0 && mA > 2) {
		/* Enable charging */
		if (power_supply_set_online(psy, true))
			goto psy_error;
		if (power_supply_set_current_limit(psy, 1000*mA))
			goto psy_error;
	} else if (motg->cur_power > 0 && (mA == 0 || mA == 2)) {
		/* Disable charging */
		if (power_supply_set_online(psy, false))
			goto psy_error;
		/* Set max current limit */
		if (power_supply_set_current_limit(psy, 0))
			goto psy_error;
	} else {
		if (power_supply_set_online(psy, true))
			goto psy_error;
		/* Current has changed (100/2 --> 500) */
		if (power_supply_set_current_limit(psy, 1000*mA))
			goto psy_error;
	}

	power_supply_changed(psy);
	return 0;

psy_error:
	dev_dbg(motg->phy.dev, "power supply error when setting property\n");
	return -ENXIO;
}

static void msm_otg_notify_charger(struct msm_otg *motg, unsigned mA)
{
	struct usb_gadget *g = motg->phy.otg->gadget;

	if (g && g->is_a_peripheral)
		return;

	if ((motg->chg_type == USB_ACA_DOCK_CHARGER ||
		motg->chg_type == USB_ACA_A_CHARGER ||
		motg->chg_type == USB_ACA_B_CHARGER ||
		motg->chg_type == USB_ACA_C_CHARGER) &&
			mA > IDEV_ACA_CHG_LIMIT)
		mA = IDEV_ACA_CHG_LIMIT;

	if (msm_otg_notify_chg_type(motg))
		dev_err(motg->phy.dev,
			"Failed notifying %d charger type to PMIC\n",
							motg->chg_type);

	if (motg->cur_power == mA)
		return;

	dev_info(motg->phy.dev, "Avail curr from USB = %u\n", mA);

	/*
	 *  Use Power Supply API if supported, otherwise fallback
	 *  to legacy pm8921 API.
	 */
	if (msm_otg_notify_power_supply(motg, mA))
		pm8921_charger_vbus_draw(mA);

	motg->cur_power = mA;
}

static int msm_otg_set_power(struct usb_phy *phy, unsigned mA)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);

	/*
	 * Gadget driver uses set_power method to notify about the
	 * available current based on suspend/configured states.
	 *
	 * IDEV_CHG can be drawn irrespective of suspend/un-configured
	 * states when CDP/ACA is connected.
	 */
	if (motg->chg_type == USB_SDP_CHARGER)
		msm_otg_notify_charger(motg, mA);

	return 0;
}

static void msm_otg_start_host(struct usb_otg *otg, int on)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	struct msm_otg_platform_data *pdata = motg->pdata;
	struct usb_hcd *hcd;

	if (!otg->host)
		return;

	hcd = bus_to_hcd(otg->host);

	if (on) {
		dev_dbg(otg->phy->dev, "host on\n");

		if (pdata->otg_control == OTG_PHY_CONTROL)
			ulpi_write(otg->phy, OTG_COMP_DISABLE,
				ULPI_SET(ULPI_PWR_CLK_MNG_REG));

		/*
		 * Some boards have a switch cotrolled by gpio
		 * to enable/disable internal HUB. Enable internal
		 * HUB before kicking the host.
		 */
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_A_HOST);
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
	} else {
		dev_dbg(otg->phy->dev, "host off\n");

		usb_remove_hcd(hcd);
		/* HCD core reset all bits of PORTSC. select ULPI phy */
		writel_relaxed(0x80000000, USB_PORTSC);

		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_UNDEFINED);

		if (pdata->otg_control == OTG_PHY_CONTROL)
			ulpi_write(otg->phy, OTG_COMP_DISABLE,
				ULPI_CLR(ULPI_PWR_CLK_MNG_REG));
	}
}

static int msm_otg_usbdev_notify(struct notifier_block *self,
			unsigned long action, void *priv)
{
	struct msm_otg *motg = container_of(self, struct msm_otg, usbdev_nb);
	struct usb_otg *otg = motg->phy.otg;
	struct usb_device *udev = priv;

	if (action == USB_BUS_ADD || action == USB_BUS_REMOVE)
		goto out;

	if (udev->bus != otg->host)
		goto out;
	/*
	 * Interested in devices connected directly to the root hub.
	 * ACA dock can supply IDEV_CHG irrespective devices connected
	 * on the accessory port.
	 */
	if (!udev->parent || udev->parent->parent ||
			motg->chg_type == USB_ACA_DOCK_CHARGER)
		goto out;

	switch (action) {
	case USB_DEVICE_ADD:
		if (aca_enabled())
			usb_disable_autosuspend(udev);
		if (otg->phy->state == OTG_STATE_A_WAIT_BCON) {
			pr_debug("B_CONN set\n");
			set_bit(B_CONN, &motg->inputs);
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_HOST;
			/*
			 * OTG PET: A-device must end session within
			 * 10 sec after PET enumeration.
			 */
			if (udev->quirks & USB_QUIRK_OTG_PET)
				msm_otg_start_timer(motg, TA_TST_MAINT,
						A_TST_MAINT);
		}
		/* fall through */
	case USB_DEVICE_CONFIG:
		if (udev->actconfig)
			motg->mA_port = udev->actconfig->desc.bMaxPower * 2;
		else
			motg->mA_port = IUNIT;
		if (otg->phy->state == OTG_STATE_B_HOST)
			msm_otg_del_timer(motg);
		break;
	case USB_DEVICE_REMOVE:
		if ((otg->phy->state == OTG_STATE_A_HOST) ||
			(otg->phy->state == OTG_STATE_A_SUSPEND)) {
			pr_debug("B_CONN clear\n");
			clear_bit(B_CONN, &motg->inputs);
			/*
			 * OTG PET: A-device must end session after
			 * PET disconnection if it is enumerated
			 * with bcdDevice[0] = 1. USB core sets
			 * bus->otg_vbus_off for us. clear it here.
			 */
			if (udev->bus->otg_vbus_off) {
				udev->bus->otg_vbus_off = 0;
				set_bit(A_BUS_DROP, &motg->inputs);
			}
			queue_work(system_nrt_wq, &motg->sm_work);
		}
	default:
		break;
	}
	if (test_bit(ID_A, &motg->inputs))
		msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX -
				motg->mA_port);
out:
	return NOTIFY_OK;
}

static void msm_hsusb_vbus_power(struct msm_otg *motg, bool on)
{
	int ret;
	static bool vbus_is_on;

	if (vbus_is_on == on)
		return;

	if (motg->pdata->vbus_power) {
		ret = motg->pdata->vbus_power(on);
		if (!ret)
			vbus_is_on = on;
		return;
	}

	if (!vbus_otg) {
		pr_err("vbus_otg is NULL.");
		return;
	}

	/*
	 * if entering host mode tell the charger to not draw any current
	 * from usb before turning on the boost.
	 * if exiting host mode disable the boost before enabling to draw
	 * current from the source.
	 */
	if (on) {
		msm_otg_notify_host_mode(motg, on);
		ret = regulator_enable(vbus_otg);
		if (ret) {
			pr_err("unable to enable vbus_otg\n");
			return;
		}
		vbus_is_on = true;
	} else {
		ret = regulator_disable(vbus_otg);
		if (ret) {
			pr_err("unable to disable vbus_otg\n");
			return;
		}
		msm_otg_notify_host_mode(motg, on);
		vbus_is_on = false;
	}
}

static int msm_otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	struct usb_hcd *hcd;

	/*
	 * Fail host registration if this board can support
	 * only peripheral configuration.
	 */
	if (motg->pdata->mode == USB_PERIPHERAL) {
		dev_info(otg->phy->dev, "Host mode is not supported\n");
		return -ENODEV;
	}

	if (!motg->pdata->vbus_power && host) {
		vbus_otg = devm_regulator_get(motg->phy.dev, "vbus_otg");
		if (IS_ERR(vbus_otg)) {
			pr_err("Unable to get vbus_otg\n");
			return PTR_ERR(vbus_otg);
		}
	}

	if (!host) {
		if (otg->phy->state == OTG_STATE_A_HOST) {
			pm_runtime_get_sync(otg->phy->dev);
			usb_unregister_notify(&motg->usbdev_nb);
			msm_otg_start_host(otg, 0);
			msm_hsusb_vbus_power(motg, 0);
			otg->host = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			queue_work(system_nrt_wq, &motg->sm_work);
		} else {
			otg->host = NULL;
		}

		return 0;
	}

	hcd = bus_to_hcd(host);
	hcd->power_budget = motg->pdata->power_budget;

#ifdef CONFIG_USB_OTG
	host->otg_port = 1;
#endif
	motg->usbdev_nb.notifier_call = msm_otg_usbdev_notify;
	usb_register_notify(&motg->usbdev_nb);
	otg->host = host;
	dev_dbg(otg->phy->dev, "host driver registered w/ tranceiver\n");

	/*
	 * Kick the state machine work, if peripheral is not supported
	 * or peripheral is already registered with us.
	 */
	if (motg->pdata->mode == USB_HOST || otg->gadget) {
		pm_runtime_get_sync(otg->phy->dev);
		queue_work(system_nrt_wq, &motg->sm_work);
	}

	return 0;
}

static void msm_otg_start_peripheral(struct usb_otg *otg, int on)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	struct msm_otg_platform_data *pdata = motg->pdata;

	if (!otg->gadget)
		return;

	if (on) {
		dev_dbg(otg->phy->dev, "gadget on\n");
		/*
		 * Some boards have a switch cotrolled by gpio
		 * to enable/disable internal HUB. Disable internal
		 * HUB before kicking the gadget.
		 */
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_B_PERIPHERAL);

		/* Configure BUS performance parameters for MAX bandwidth */
		if (debug_bus_voting_enabled)
			msm_otg_bus_vote(motg, USB_MAX_PERF_VOTE);

		usb_gadget_vbus_connect(otg->gadget);
	} else {
		dev_dbg(otg->phy->dev, "gadget off\n");
		usb_gadget_vbus_disconnect(otg->gadget);
		/* Configure BUS performance parameters to default */
		msm_otg_bus_vote(motg, USB_MIN_PERF_VOTE);
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_UNDEFINED);
	}
}

static int msm_otg_set_peripheral(struct usb_otg *otg,
					struct usb_gadget *gadget)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);

	/*
	 * Fail peripheral registration if this board can support
	 * only host configuration.
	 */
	if (motg->pdata->mode == USB_HOST) {
		dev_info(otg->phy->dev, "Peripheral mode is not supported\n");
		return -ENODEV;
	}

	if (!gadget) {
		if (otg->phy->state == OTG_STATE_B_PERIPHERAL) {
			pm_runtime_get_sync(otg->phy->dev);
			msm_otg_start_peripheral(otg, 0);
			otg->gadget = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			queue_work(system_nrt_wq, &motg->sm_work);
		} else {
			otg->gadget = NULL;
		}

		return 0;
	}
	otg->gadget = gadget;
	dev_dbg(otg->phy->dev, "peripheral driver registered w/ tranceiver\n");

	/*
	 * Kick the state machine work, if host is not supported
	 * or host is already registered with us.
	 */
	if (motg->pdata->mode == USB_PERIPHERAL || otg->host) {
		pm_runtime_get_sync(otg->phy->dev);
		queue_work(system_nrt_wq, &motg->sm_work);
	}

	return 0;
}

static bool msm_otg_read_pmic_id_state(struct msm_otg *motg)
{
	unsigned long flags;
	int id;

	if (!motg->pdata->pmic_id_irq)
		return -ENODEV;

	local_irq_save(flags);
	id = irq_read_line(motg->pdata->pmic_id_irq);
	local_irq_restore(flags);

	/*
	 * If we can not read ID line state for some reason, treat
	 * it as float. This would prevent MHL discovery and kicking
	 * host mode unnecessarily.
	 */
	return !!id;
}

static int msm_otg_mhl_register_callback(struct msm_otg *motg,
						void (*callback)(int on))
{
	struct usb_phy *phy = &motg->phy;
	int ret;

	if (!motg->pdata->mhl_enable) {
		dev_dbg(phy->dev, "MHL feature not enabled\n");
		return -ENODEV;
	}

	if (motg->pdata->otg_control != OTG_PMIC_CONTROL ||
			!motg->pdata->pmic_id_irq) {
		dev_dbg(phy->dev, "MHL can not be supported without PMIC Id\n");
		return -ENODEV;
	}

	if (!motg->pdata->mhl_dev_name) {
		dev_dbg(phy->dev, "MHL device name does not exist.\n");
		return -ENODEV;
	}

	if (callback)
		ret = mhl_register_callback(motg->pdata->mhl_dev_name,
								callback);
	else
		ret = mhl_unregister_callback(motg->pdata->mhl_dev_name);

	if (ret)
		dev_dbg(phy->dev, "mhl_register_callback(%s) return error=%d\n",
						motg->pdata->mhl_dev_name, ret);
	else
		motg->mhl_enabled = true;

	return ret;
}

static void msm_otg_mhl_notify_online(int on)
{
	struct msm_otg *motg = the_msm_otg;
	struct usb_phy *phy = &motg->phy;
	bool queue = false;

	dev_dbg(phy->dev, "notify MHL %s%s\n", on ? "" : "dis", "connected");

	if (on) {
		set_bit(MHL, &motg->inputs);
	} else {
		clear_bit(MHL, &motg->inputs);
		queue = true;
	}

	if (queue && phy->state != OTG_STATE_UNDEFINED)
		schedule_work(&motg->sm_work);
}

static bool msm_otg_is_mhl(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	int is_mhl, ret;

	ret = mhl_device_discovery(motg->pdata->mhl_dev_name, &is_mhl);
	if (ret || is_mhl != MHL_DISCOVERY_RESULT_MHL) {
		/*
		 * MHL driver calls our callback saying that MHL connected
		 * if RID_GND is detected.  But at later part of discovery
		 * it may figure out MHL is not connected and returns
		 * false. Hence clear MHL input here.
		 */
		clear_bit(MHL, &motg->inputs);
		dev_dbg(phy->dev, "MHL device not found\n");
		return false;
	}

	set_bit(MHL, &motg->inputs);
	dev_dbg(phy->dev, "MHL device found\n");
	return true;
}

static bool msm_chg_mhl_detect(struct msm_otg *motg)
{
	bool ret, id;

	if (!motg->mhl_enabled)
		return false;

	id = msm_otg_read_pmic_id_state(motg);

	if (id)
		return false;

	mhl_det_in_progress = true;
	ret = msm_otg_is_mhl(motg);
	mhl_det_in_progress = false;

	return ret;
}

static void msm_otg_chg_check_timer_func(unsigned long data)
{
	struct msm_otg *motg = (struct msm_otg *) data;
	struct usb_otg *otg = motg->phy.otg;

	if (atomic_read(&motg->in_lpm) ||
		!test_bit(B_SESS_VLD, &motg->inputs) ||
		otg->phy->state != OTG_STATE_B_PERIPHERAL ||
		otg->gadget->speed != USB_SPEED_UNKNOWN) {
		dev_dbg(otg->phy->dev, "Nothing to do in chg_check_timer\n");
		return;
	}

	if ((readl_relaxed(USB_PORTSC) & PORTSC_LS) == PORTSC_LS) {
		dev_dbg(otg->phy->dev, "DCP is detected as SDP\n");
		set_bit(B_FALSE_SDP, &motg->inputs);
		queue_work(system_nrt_wq, &motg->sm_work);
	}
}

static bool msm_chg_aca_detect(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 int_sts;
	bool ret = false;
	// Mike, aca_enable() always failed.
	if (!aca_enabled())
		goto out;

	// Remove phy_type is CI_45NM_INTEGRATED_PHY
out:
	return ret;
}

static void msm_chg_enable_aca_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	// Mike, aca_enabled().
	if (!aca_enabled())
		return;
}

static void msm_chg_enable_aca_intr(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;

	if (!aca_enabled())
		return;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		/* Enable ACA Detection interrupt (on any RID change) */
		ulpi_write(phy, 0x01, 0x94);
		break;
	default:
		break;
	}
}

static void msm_chg_disable_aca_intr(struct msm_otg *motg)
{
	if (!aca_enabled())
		return;
}

static bool msm_chg_check_aca_intr(struct msm_otg *motg)
{
	if (!aca_enabled())
		return ret;
}
static void msm_otg_id_timer_func(unsigned long data)
{
	if (!aca_enabled())
		return;
}

static bool msm_chg_check_secondary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;
	bool ret = false;

	// Remove phy_type is CI_45NM_INTEGRATED_PHY
		chg_det = ulpi_read(phy, 0x87);
		ret = chg_det & 1;
	return ret;
}

static void msm_chg_enable_secondary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;

	// Remove phy_type is CI_45NM_INTEGRATED_PHY
		/*
		 * Configure DM as current source, DP as current sink
		 * and enable battery charging comparators.
		 */
		ulpi_write(phy, 0x8, 0x85);
		ulpi_write(phy, 0x2, 0x85);
		ulpi_write(phy, 0x1, 0x85);
}

static bool msm_chg_check_primary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;
	bool ret = false;

	// Remove phy_type is CI_45NM_INTEGRATED_PHY
		chg_det = ulpi_read(phy, 0x87);
		ret = chg_det & 1;
		/* Turn off VDP_SRC */
		ulpi_write(phy, 0x3, 0x86);
		msleep(20);

	return ret;
}

static void msm_chg_enable_primary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;

	// Remove phy_type is CI_45NM_INTEGRATED_PHY
	case SNPS_28NM_INTEGRATED_PHY:
		/*
		 * Configure DP as current source, DM as current sink
		 * and enable battery charging comparators.
		 */
		ulpi_write(phy, 0x2, 0x85);
		ulpi_write(phy, 0x1, 0x85);

}

static bool msm_chg_check_dcd(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 line_state;
	bool ret = false;
	// Remove phy_type is CI_45NM_INTEGRATED_PHY
	case SNPS_28NM_INTEGRATED_PHY:
		line_state = ulpi_read(phy, 0x87);
		ret = line_state & 2;
		break;
	return ret;
}

static void msm_chg_disable_dcd(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;
	// Remove phy_type is CI_45NM_INTEGRATED_PHY
	case SNPS_28NM_INTEGRATED_PHY:
		ulpi_write(phy, 0x10, 0x86);
		break;
}

static void msm_chg_enable_dcd(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;
	// Remove phy_type is CI_45NM_INTEGRATED_PHY
	case SNPS_28NM_INTEGRATED_PHY:
		/* Data contact detection enable */
		ulpi_write(phy, 0x10, 0x85);
		break;
}

static void msm_chg_block_on(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 func_ctrl, chg_det;

	/* put the controller in non-driving mode */
	func_ctrl = ulpi_read(phy, ULPI_FUNC_CTRL);
	func_ctrl &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
	func_ctrl |= ULPI_FUNC_CTRL_OPMODE_NONDRIVING;
	ulpi_write(phy, func_ctrl, ULPI_FUNC_CTRL);
	// Remove phy_type is CI_45NM_INTEGRATED_PHY
	case SNPS_28NM_INTEGRATED_PHY:
		/* disable DP and DM pull down resistors */
		ulpi_write(phy, 0x6, 0xC);
		/* Clear charger detecting control bits */
		ulpi_write(phy, 0x1F, 0x86);
		/* Clear alt interrupt latch and enable bits */
		ulpi_write(phy, 0x1F, 0x92);
		ulpi_write(phy, 0x1F, 0x95);
		udelay(100);
		break;
}

static void msm_chg_block_off(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 func_ctrl, chg_det;
	// Remove phy_type is CI_45NM_INTEGRATED_PHY
	case SNPS_28NM_INTEGRATED_PHY:
		/* Clear charger detecting control bits */
		ulpi_write(phy, 0x3F, 0x86);
		/* Clear alt interrupt latch and enable bits */
		ulpi_write(phy, 0x1F, 0x92);
		ulpi_write(phy, 0x1F, 0x95);
		/* re-enable DP and DM pull down resistors */
		ulpi_write(phy, 0x6, 0xB);
		break;

	/* put the controller in normal mode */
	func_ctrl = ulpi_read(phy, ULPI_FUNC_CTRL);
	func_ctrl &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
	func_ctrl |= ULPI_FUNC_CTRL_OPMODE_NORMAL;
	ulpi_write(phy, func_ctrl, ULPI_FUNC_CTRL);
}

static const char *chg_to_string(enum usb_chg_type chg_type)
{
	switch (chg_type) {
	case USB_SDP_CHARGER:		return "USB_SDP_CHARGER";
	case USB_DCP_CHARGER:		return "USB_DCP_CHARGER";
	case USB_CDP_CHARGER:		return "USB_CDP_CHARGER";
	case USB_ACA_A_CHARGER:		return "USB_ACA_A_CHARGER";
	case USB_ACA_B_CHARGER:		return "USB_ACA_B_CHARGER";
	case USB_ACA_C_CHARGER:		return "USB_ACA_C_CHARGER";
	case USB_ACA_DOCK_CHARGER:	return "USB_ACA_DOCK_CHARGER";
	case USB_PROPRIETARY_CHARGER:	return "USB_PROPRIETARY_CHARGER";
	case USB_FLOATED_CHARGER:	return "USB_FLOATED_CHARGER";
	default:			return "INVALID_CHARGER";
	}
}

#define MSM_CHG_DCD_TIMEOUT		(750 * HZ/1000) /* 750 msec */
#define MSM_CHG_DCD_POLL_TIME		(50 * HZ/1000) /* 50 msec */
#define MSM_CHG_PRIMARY_DET_TIME	(50 * HZ/1000) /* TVDPSRC_ON */
#define MSM_CHG_SECONDARY_DET_TIME	(50 * HZ/1000) /* TVDMSRC_ON */
static void msm_chg_detect_work(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg, chg_work.work);
	struct usb_phy *phy = &motg->phy;
	bool is_dcd = false, tmout, vout, is_aca;
	static bool dcd;
	u32 line_state, dm_vlgc;
	unsigned long delay;

	dev_dbg(phy->dev, "chg detection work\n");

	if (test_bit(MHL, &motg->inputs)) {
		dev_dbg(phy->dev, "detected MHL, escape chg detection work\n");
		return;
	}

	switch (motg->chg_state) {
	case USB_CHG_STATE_UNDEFINED:
		msm_chg_block_on(motg);
		msm_chg_enable_dcd(motg);
		msm_chg_enable_aca_det(motg);
		motg->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
		motg->dcd_time = 0;
		delay = MSM_CHG_DCD_POLL_TIME;
		break;
	case USB_CHG_STATE_WAIT_FOR_DCD:
		if (msm_chg_mhl_detect(motg)) {
			msm_chg_block_off(motg);
			motg->chg_state = USB_CHG_STATE_DETECTED;
			motg->chg_type = USB_INVALID_CHARGER;
			queue_work(system_nrt_wq, &motg->sm_work);
			return;
		}
		is_aca = msm_chg_aca_detect(motg);
		if (is_aca) {
			/*
			 * ID_A can be ACA dock too. continue
			 * primary detection after DCD.
			 */
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
			} else {
				delay = 0;
				break;
			}
		}
		is_dcd = msm_chg_check_dcd(motg);
		motg->dcd_time += MSM_CHG_DCD_POLL_TIME;
		tmout = motg->dcd_time >= MSM_CHG_DCD_TIMEOUT;
		if (is_dcd || tmout) {
			if (is_dcd)
				dcd = true;
			else
				dcd = false;
			msm_chg_disable_dcd(motg);
			msm_chg_enable_primary_det(motg);
			delay = MSM_CHG_PRIMARY_DET_TIME;
			motg->chg_state = USB_CHG_STATE_DCD_DONE;
		} else {
			delay = MSM_CHG_DCD_POLL_TIME;
		}
		break;
	case USB_CHG_STATE_DCD_DONE:
		vout = msm_chg_check_primary_det(motg);
		line_state = readl_relaxed(USB_PORTSC) & PORTSC_LS;
		dm_vlgc = line_state & PORTSC_LS_DM;
		if (vout && !dm_vlgc) { /* VDAT_REF < DM < VLGC */
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_type = USB_ACA_DOCK_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
				break;
			}
			if (line_state) { /* DP > VLGC */
				motg->chg_type = USB_PROPRIETARY_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
			} else {
				msm_chg_enable_secondary_det(motg);
				delay = MSM_CHG_SECONDARY_DET_TIME;
				motg->chg_state = USB_CHG_STATE_PRIMARY_DONE;
			}
		} else { /* DM < VDAT_REF || DM > VLGC */
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_type = USB_ACA_A_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
				break;
			}

			if (line_state) /* DP > VLGC or/and DM > VLGC */
				motg->chg_type = USB_PROPRIETARY_CHARGER;
			else if (!dcd && floated_charger_enable)
				motg->chg_type = USB_FLOATED_CHARGER;
			else
				motg->chg_type = USB_SDP_CHARGER;

			motg->chg_state = USB_CHG_STATE_DETECTED;
			delay = 0;
		}
		break;
	case USB_CHG_STATE_PRIMARY_DONE:
		vout = msm_chg_check_secondary_det(motg);
		if (vout)
			motg->chg_type = USB_DCP_CHARGER;
		else
			motg->chg_type = USB_CDP_CHARGER;
		motg->chg_state = USB_CHG_STATE_SECONDARY_DONE;
		/* fall through */
	case USB_CHG_STATE_SECONDARY_DONE:
		motg->chg_state = USB_CHG_STATE_DETECTED;
	case USB_CHG_STATE_DETECTED:
		/*
		 * Notify the charger type to power supply
		 * owner as soon as we determine the charger.
		 */
		msm_otg_notify_chg_type(motg);
		msm_chg_block_off(motg);
		msm_chg_enable_aca_det(motg);
		/*
		 * Spurious interrupt is seen after enabling ACA detection
		 * due to which charger detection fails in case of PET.
		 * Add delay of 100 microsec to avoid that.
		 */
		udelay(100);
		msm_chg_enable_aca_intr(motg);
		dev_dbg(phy->dev, "chg_type = %s\n",
			chg_to_string(motg->chg_type));
		queue_work(system_nrt_wq, &motg->sm_work);
		return;
	default:
		return;
	}

	queue_delayed_work(system_nrt_wq, &motg->chg_work, delay);
}

/*
 * We support OTG, Peripheral only and Host only configurations. In case
 * of OTG, mode switch (host-->peripheral/peripheral-->host) can happen
 * via Id pin status or user request (debugfs). Id/BSV interrupts are not
 * enabled when switch is controlled by user and default mode is supplied
 * by board file, which can be changed by userspace later.
 */
// Caller: msm_otg_sm_work, due to PYH state is UNDEFINED.
static void msm_otg_init_sm(struct msm_otg *motg)
{
	struct msm_otg_platform_data *pdata = motg->pdata;
	u32 otgsc = readl(USB_OTGSC);

	switch (pdata->mode) {
	case USB_OTG:
		} else if (pdata->otg_control == OTG_PMIC_CONTROL) {
			if (pdata->pmic_id_irq) { // mike, 288.
				if (msm_otg_read_pmic_id_state(motg))
					set_bit(ID, &motg->inputs);
				else
					clear_bit(ID, &motg->inputs);
			}
			/*
			 * VBUS initial state is reported after PMIC
			 * driver initialization. Wait for it.
			 */
			wait_for_completion(&pmic_vbus_init);
		}
		break;
	}
}

static void msm_otg_wait_for_ext_chg_done(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	unsigned long t;

	/*
	 * Defer next cable connect event till external charger
	 * detection is completed.
	 */

	if (motg->ext_chg_active) {

		pr_debug("before msm_otg ext chg wait\n");

		t = wait_for_completion_timeout(&motg->ext_chg_wait,
				msecs_to_jiffies(3000));
		if (!t)
			pr_err("msm_otg ext chg wait timeout\n");
		else
			pr_debug("msm_otg ext chg wait done\n");
	}

	if (motg->ext_chg_opened) {
		if (phy->flags & ENABLE_DP_MANUAL_PULLUP) {
			ulpi_write(phy, ULPI_MISC_A_VBUSVLDEXT |
					ULPI_MISC_A_VBUSVLDEXTSEL,
					ULPI_CLR(ULPI_MISC_A));
		}
		/* clear charging register bits */
		ulpi_write(phy, 0x3F, 0x86);
		/* re-enable DP and DM pull-down resistors*/
		ulpi_write(phy, 0x6, 0xB);
	}
}

static void msm_otg_sm_work(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg, sm_work);
	struct usb_otg *otg = motg->phy.otg;
	bool work = 0, srp_reqd, dcp;

	pm_runtime_resume(otg->phy->dev);
	if (motg->pm_done) {
		pm_runtime_get_sync(otg->phy->dev);
		motg->pm_done = 0;
	}
	pr_debug("%s work\n", otg_state_string(otg->phy->state));
	switch (otg->phy->state) {
	case OTG_STATE_UNDEFINED:
		msm_otg_reset(otg->phy);
		msm_otg_init_sm(motg);
		if (!psy && legacy_power_supply) {
			psy = power_supply_get_by_name("usb");

			if (!psy)
				pr_err("couldn't get usb power supply\n");
		}

		otg->phy->state = OTG_STATE_B_IDLE;
		if (!test_bit(B_SESS_VLD, &motg->inputs) &&
				test_bit(ID, &motg->inputs)) {
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
			break;
		}
		/* FALL THROUGH */
	case OTG_STATE_B_IDLE:
		if (test_bit(MHL, &motg->inputs)) {
			/* allow LPM */
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
		} else if ((!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs)) && otg->host) {
			pr_debug("!id || id_A\n");
			if (msm_chg_mhl_detect(motg)) {
				work = 1;
				break;
			}
			clear_bit(B_BUS_REQ, &motg->inputs);
			set_bit(A_BUS_REQ, &motg->inputs);
			otg->phy->state = OTG_STATE_A_IDLE;
			work = 1;
		} else if (test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("b_sess_vld\n");
			switch (motg->chg_state) {
			case USB_CHG_STATE_UNDEFINED:
				msm_chg_detect_work(&motg->chg_work.work);
				break;
			case USB_CHG_STATE_DETECTED:
				switch (motg->chg_type) {
				case USB_DCP_CHARGER:
					/* Enable VDP_SRC */
					ulpi_write(otg->phy, 0x2, 0x85);
					if (motg->ext_chg_opened) {
						init_completion(
							&motg->ext_chg_wait);
						motg->ext_chg_active = true;
					}
					/* fall through */
				case USB_PROPRIETARY_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_CHG_MAX);
					pm_runtime_put_sync(otg->phy->dev);
					break;
				case USB_FLOATED_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_CHG_MAX);
					pm_runtime_put_noidle(otg->phy->dev);
					pm_runtime_suspend(otg->phy->dev);
					break;
				case USB_ACA_B_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
					/*
					 * (ID_B --> ID_C) PHY_ALT interrupt can
					 * not be detected in LPM.
					 */
					break;
				case USB_CDP_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_CHG_MAX);
					msm_otg_start_peripheral(otg, 1);
					otg->phy->state =
						OTG_STATE_B_PERIPHERAL;
					break;
				case USB_ACA_C_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
					msm_otg_start_peripheral(otg, 1);
					otg->phy->state =
						OTG_STATE_B_PERIPHERAL;
					break;
				case USB_SDP_CHARGER:
					msm_otg_start_peripheral(otg, 1);
					otg->phy->state =
						OTG_STATE_B_PERIPHERAL;
					mod_timer(&motg->chg_check_timer,
							CHG_RECHECK_DELAY);
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
		} else if (test_bit(B_BUS_REQ, &motg->inputs)) {
			pr_debug("b_sess_end && b_bus_req\n");
			if (msm_otg_start_srp(otg) < 0) {
				clear_bit(B_BUS_REQ, &motg->inputs);
				work = 1;
				break;
			}
			otg->phy->state = OTG_STATE_B_SRP_INIT;
			msm_otg_start_timer(motg, TB_SRP_FAIL, B_SRP_FAIL);
			break;
		} else {
			pr_debug("chg_work cancel");
			del_timer_sync(&motg->chg_check_timer);
			clear_bit(B_FALSE_SDP, &motg->inputs);
			clear_bit(A_BUS_REQ, &motg->inputs);
			cancel_delayed_work_sync(&motg->chg_work);
			dcp = (motg->chg_type == USB_DCP_CHARGER);
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_notify_charger(motg, 0);
			if (dcp) {
				msm_otg_wait_for_ext_chg_done(motg);
				/* Turn off VDP_SRC */
				ulpi_write(otg->phy, 0x2, 0x86);
			}
			msm_otg_reset(otg->phy);
			/*
			 * There is a small window where ID interrupt
			 * is not monitored during ID detection circuit
			 * switch from ACA to PMIC.  Check ID state
			 * before entering into low power mode.
			 */
			if (!msm_otg_read_pmic_id_state(motg)) {
				pr_debug("process missed ID intr\n");
				clear_bit(ID, &motg->inputs);
				work = 1;
				break;
			}
			pm_runtime_put_noidle(otg->phy->dev);
			/*
			 * Only if autosuspend was enabled in probe, it will be
			 * used here. Otherwise, no delay will be used.
			 */
			pm_runtime_mark_last_busy(otg->phy->dev);
			pm_runtime_autosuspend(otg->phy->dev);
			motg->pm_done = 1;
		}
		break;
	case OTG_STATE_B_SRP_INIT:
		if (!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs) ||
				test_bit(ID_C, &motg->inputs) ||
				(test_bit(B_SESS_VLD, &motg->inputs) &&
				!test_bit(ID_B, &motg->inputs))) {
			pr_debug("!id || id_a/c || b_sess_vld+!id_b\n");
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_B_IDLE;
			/*
			 * clear VBUSVLDEXTSEL and VBUSVLDEXT register
			 * bits after SRP initiation.
			 */
			ulpi_write(otg->phy, 0x0, 0x98);
			work = 1;
		} else if (test_bit(B_SRP_FAIL, &motg->tmouts)) {
			pr_debug("b_srp_fail\n");
			pr_info("A-device did not respond to SRP\n");
			clear_bit(B_BUS_REQ, &motg->inputs);
			clear_bit(B_SRP_FAIL, &motg->tmouts);
			otg_send_event(OTG_EVENT_NO_RESP_FOR_SRP);
			ulpi_write(otg->phy, 0x0, 0x98);
			otg->phy->state = OTG_STATE_B_IDLE;
			motg->b_last_se0_sess = jiffies;
			work = 1;
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (test_bit(B_SESS_VLD, &motg->inputs) &&
				test_bit(B_FALSE_SDP, &motg->inputs)) {
			pr_debug("B_FALSE_SDP\n");
			msm_otg_start_peripheral(otg, 0);
			motg->chg_type = USB_DCP_CHARGER;
			clear_bit(B_FALSE_SDP, &motg->inputs);
			otg->phy->state = OTG_STATE_B_IDLE;
			work = 1;
		} else if (!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs) ||
				test_bit(ID_B, &motg->inputs) ||
				!test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("!id  || id_a/b || !b_sess_vld\n");
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_notify_charger(motg, 0);
			srp_reqd = otg->gadget->otg_srp_reqd;
			msm_otg_start_peripheral(otg, 0);
			if (test_bit(ID_B, &motg->inputs))
				clear_bit(ID_B, &motg->inputs);
			clear_bit(B_BUS_REQ, &motg->inputs);
			otg->phy->state = OTG_STATE_B_IDLE;
			motg->b_last_se0_sess = jiffies;
			if (srp_reqd)
				msm_otg_start_timer(motg,
					TB_TST_SRP, B_TST_SRP);
			else
				work = 1;
		} else if (test_bit(B_BUS_REQ, &motg->inputs) &&
				otg->gadget->b_hnp_enable &&
				test_bit(A_BUS_SUSPEND, &motg->inputs)) {
			pr_debug("b_bus_req && b_hnp_en && a_bus_suspend\n");
			msm_otg_start_timer(motg, TB_ASE0_BRST, B_ASE0_BRST);
			/* D+ pullup should not be disconnected within 4msec
			 * after A device suspends the bus. Otherwise PET will
			 * fail the compliance test.
			 */
			udelay(1000);
			msm_otg_start_peripheral(otg, 0);
			otg->phy->state = OTG_STATE_B_WAIT_ACON;
			/*
			 * start HCD even before A-device enable
			 * pull-up to meet HNP timings.
			 */
			otg->host->is_b_host = 1;
			msm_otg_start_host(otg, 1);
		} else if (test_bit(A_BUS_SUSPEND, &motg->inputs) &&
				   test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("a_bus_suspend && b_sess_vld\n");
			if (motg->caps & ALLOW_LPM_ON_DEV_SUSPEND) {
				pm_runtime_put_noidle(otg->phy->dev);
				pm_runtime_suspend(otg->phy->dev);
			}
		} else if (test_bit(ID_C, &motg->inputs)) {
			msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX);
		}
		break;
	case OTG_STATE_B_WAIT_ACON:
		if (!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs) ||
				test_bit(ID_B, &motg->inputs) ||
				!test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("!id || id_a/b || !b_sess_vld\n");
			msm_otg_del_timer(motg);
			/*
			 * A-device is physically disconnected during
			 * HNP. Remove HCD.
			 */
			msm_otg_start_host(otg, 0);
			otg->host->is_b_host = 0;

			clear_bit(B_BUS_REQ, &motg->inputs);
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			motg->b_last_se0_sess = jiffies;
			otg->phy->state = OTG_STATE_B_IDLE;
			msm_otg_reset(otg->phy);
			work = 1;
		} else if (test_bit(A_CONN, &motg->inputs)) {
			pr_debug("a_conn\n");
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			otg->phy->state = OTG_STATE_B_HOST;
			/*
			 * PET disconnects D+ pullup after reset is generated
			 * by B device in B_HOST role which is not detected by
			 * B device. As workaorund , start timer of 300msec
			 * and stop timer if A device is enumerated else clear
			 * A_CONN.
			 */
			msm_otg_start_timer(motg, TB_TST_CONFIG,
						B_TST_CONFIG);
		} else if (test_bit(B_ASE0_BRST, &motg->tmouts)) {
			pr_debug("b_ase0_brst_tmout\n");
			pr_info("B HNP fail:No response from A device\n");
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
			otg->host->is_b_host = 0;
			clear_bit(B_ASE0_BRST, &motg->tmouts);
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			clear_bit(B_BUS_REQ, &motg->inputs);
			otg_send_event(OTG_EVENT_HNP_FAILED);
			otg->phy->state = OTG_STATE_B_IDLE;
			work = 1;
		} else if (test_bit(ID_C, &motg->inputs)) {
			msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX);
		}
		break;
	case OTG_STATE_B_HOST:
		if (!test_bit(B_BUS_REQ, &motg->inputs) ||
				!test_bit(A_CONN, &motg->inputs) ||
				!test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("!b_bus_req || !a_conn || !b_sess_vld\n");
			clear_bit(A_CONN, &motg->inputs);
			clear_bit(B_BUS_REQ, &motg->inputs);
			msm_otg_start_host(otg, 0);
			otg->host->is_b_host = 0;
			otg->phy->state = OTG_STATE_B_IDLE;
			msm_otg_reset(otg->phy);
			work = 1;
		} else if (test_bit(ID_C, &motg->inputs)) {
			msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX);
		}
		break;
	case OTG_STATE_A_IDLE:
		otg->default_a = 1;
		if (test_bit(ID, &motg->inputs) &&
			!test_bit(ID_A, &motg->inputs)) {
			pr_debug("id && !id_a\n");
			otg->default_a = 0;
			clear_bit(A_BUS_DROP, &motg->inputs);
			otg->phy->state = OTG_STATE_B_IDLE;
			del_timer_sync(&motg->id_timer);
			msm_otg_link_reset(motg);
			msm_chg_enable_aca_intr(motg);
			msm_otg_notify_charger(motg, 0);
			work = 1;
		} else if (!test_bit(A_BUS_DROP, &motg->inputs) &&
				(test_bit(A_SRP_DET, &motg->inputs) ||
				 test_bit(A_BUS_REQ, &motg->inputs))) {
			pr_debug("!a_bus_drop && (a_srp_det || a_bus_req)\n");

			clear_bit(A_SRP_DET, &motg->inputs);
			/* Disable SRP detection */
			writel_relaxed((readl_relaxed(USB_OTGSC) &
					~OTGSC_INTSTS_MASK) &
					~OTGSC_DPIE, USB_OTGSC);

			otg->phy->state = OTG_STATE_A_WAIT_VRISE;
			/* VBUS should not be supplied before end of SRP pulse
			 * generated by PET, if not complaince test fail.
			 */
			usleep_range(10000, 12000);
			/* ACA: ID_A: Stop charging untill enumeration */
			if (test_bit(ID_A, &motg->inputs))
				msm_otg_notify_charger(motg, 0);
			else
				msm_hsusb_vbus_power(motg, 1);
			msm_otg_start_timer(motg, TA_WAIT_VRISE, A_WAIT_VRISE);
		} else {
			pr_debug("No session requested\n");
			clear_bit(A_BUS_DROP, &motg->inputs);
			if (test_bit(ID_A, &motg->inputs)) {
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
			} else if (!test_bit(ID, &motg->inputs)) {
				msm_otg_notify_charger(motg, 0);
				/*
				 * A-device is not providing power on VBUS.
				 * Enable SRP detection.
				 */
				writel_relaxed(0x13, USB_USBMODE);
				writel_relaxed((readl_relaxed(USB_OTGSC) &
						~OTGSC_INTSTS_MASK) |
						OTGSC_DPIE, USB_OTGSC);
				mb();
			}
		}
		break;
	case OTG_STATE_A_WAIT_VRISE:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_WAIT_VRISE, &motg->tmouts)) {
			pr_debug("id || a_bus_drop || a_wait_vrise_tmout\n");
			clear_bit(A_BUS_REQ, &motg->inputs);
			msm_otg_del_timer(motg);
			msm_hsusb_vbus_power(motg, 0);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("a_vbus_vld\n");
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);

			/* Clear BSV in host mode */
			clear_bit(B_SESS_VLD, &motg->inputs);
			msm_otg_start_host(otg, 1);
			msm_chg_enable_aca_det(motg);
			msm_chg_disable_aca_intr(motg);
			mod_timer(&motg->id_timer, ID_TIMER_FREQ);
			if (msm_chg_check_aca_intr(motg))
				work = 1;
		}
		break;
	case OTG_STATE_A_WAIT_BCON:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_WAIT_BCON, &motg->tmouts)) {
			pr_debug("(id && id_a/b/c) || a_bus_drop ||"
					"a_wait_bcon_tmout\n");
			if (test_bit(A_WAIT_BCON, &motg->tmouts)) {
				pr_info("Device No Response\n");
				otg_send_event(OTG_EVENT_DEV_CONN_TMOUT);
			}
			msm_otg_del_timer(motg);
			clear_bit(A_BUS_REQ, &motg->inputs);
			clear_bit(B_CONN, &motg->inputs);
			msm_otg_start_host(otg, 0);
			/*
			 * ACA: ID_A with NO accessory, just the A plug is
			 * attached to ACA: Use IDCHG_MAX for charging
			 */
			if (test_bit(ID_A, &motg->inputs))
				msm_otg_notify_charger(motg, IDEV_CHG_MIN);
			else
				msm_hsusb_vbus_power(motg, 0);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			clear_bit(B_CONN, &motg->inputs);
			msm_otg_del_timer(motg);
			msm_otg_start_host(otg, 0);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_reset(otg->phy);
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 0);
		} else if (!test_bit(A_BUS_REQ, &motg->inputs)) {
			/*
			 * If TA_WAIT_BCON is infinite, we don;t
			 * turn off VBUS. Enter low power mode.
			 */
			if (TA_WAIT_BCON < 0)
				pm_runtime_put_sync(otg->phy->dev);
		} else if (!test_bit(ID, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_HOST:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs)) {
			pr_debug("id_a/b/c || a_bus_drop\n");
			clear_bit(B_CONN, &motg->inputs);
			clear_bit(A_BUS_REQ, &motg->inputs);
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_host(otg, 0);
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			clear_bit(B_CONN, &motg->inputs);
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
		} else if (!test_bit(A_BUS_REQ, &motg->inputs)) {
			/*
			 * a_bus_req is de-asserted when root hub is
			 * suspended or HNP is in progress.
			 */
			pr_debug("!a_bus_req\n");
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_SUSPEND;
			if (otg->host->b_hnp_enable)
				msm_otg_start_timer(motg, TA_AIDL_BDIS,
						A_AIDL_BDIS);
			else
				pm_runtime_put_sync(otg->phy->dev);
		} else if (!test_bit(B_CONN, &motg->inputs)) {
			pr_debug("!b_conn\n");
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);
			if (msm_chg_check_aca_intr(motg))
				work = 1;
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_otg_del_timer(motg);
			msm_hsusb_vbus_power(motg, 0);
			if (motg->chg_type == USB_ACA_DOCK_CHARGER)
				msm_otg_notify_charger(motg,
						IDEV_ACA_CHG_MAX);
			else
				msm_otg_notify_charger(motg,
						IDEV_CHG_MIN - motg->mA_port);
		} else if (!test_bit(ID, &motg->inputs)) {
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_notify_charger(motg, 0);
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_SUSPEND:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_AIDL_BDIS, &motg->tmouts)) {
			pr_debug("id_a/b/c || a_bus_drop ||"
					"a_aidl_bdis_tmout\n");
			msm_otg_del_timer(motg);
			clear_bit(B_CONN, &motg->inputs);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			msm_otg_del_timer(motg);
			clear_bit(B_CONN, &motg->inputs);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
		} else if (!test_bit(B_CONN, &motg->inputs) &&
				otg->host->b_hnp_enable) {
			pr_debug("!b_conn && b_hnp_enable");
			otg->phy->state = OTG_STATE_A_PERIPHERAL;
			msm_otg_host_hnp_enable(otg, 1);
			otg->gadget->is_a_peripheral = 1;
			msm_otg_start_peripheral(otg, 1);
		} else if (!test_bit(B_CONN, &motg->inputs) &&
				!otg->host->b_hnp_enable) {
			pr_debug("!b_conn && !b_hnp_enable");
			/*
			 * bus request is dropped during suspend.
			 * acquire again for next device.
			 */
			set_bit(A_BUS_REQ, &motg->inputs);
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 0);
			msm_otg_notify_charger(motg,
					IDEV_CHG_MIN - motg->mA_port);
		} else if (!test_bit(ID, &motg->inputs)) {
			msm_otg_notify_charger(motg, 0);
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_PERIPHERAL:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs)) {
			pr_debug("id _f/b/c || a_bus_drop\n");
			/* Clear BIDL_ADIS timer */
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_peripheral(otg, 0);
			otg->gadget->is_a_peripheral = 0;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			/* Clear BIDL_ADIS timer */
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_start_peripheral(otg, 0);
			otg->gadget->is_a_peripheral = 0;
			msm_otg_start_host(otg, 0);
		} else if (test_bit(A_BIDL_ADIS, &motg->tmouts)) {
			pr_debug("a_bidl_adis_tmout\n");
			msm_otg_start_peripheral(otg, 0);
			otg->gadget->is_a_peripheral = 0;
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			set_bit(A_BUS_REQ, &motg->inputs);
			msm_otg_host_hnp_enable(otg, 0);
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 0);
			msm_otg_notify_charger(motg,
					IDEV_CHG_MIN - motg->mA_port);
		} else if (!test_bit(ID, &motg->inputs)) {
			msm_otg_notify_charger(motg, 0);
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		if (test_bit(A_WAIT_VFALL, &motg->tmouts)) {
			clear_bit(A_VBUS_VLD, &motg->inputs);
			otg->phy->state = OTG_STATE_A_IDLE;
			work = 1;
		}
		break;
	case OTG_STATE_A_VBUS_ERR:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_CLR_ERR, &motg->inputs)) {
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_notify_charger(motg, 0);
		}
		break;
	default:
		break;
	}
	if (work)
		queue_work(system_nrt_wq, &motg->sm_work);
}

static void msm_otg_suspend_work(struct work_struct *w)
{
	struct msm_otg *motg =
		container_of(w, struct msm_otg, suspend_work.work);

	/* This work is only for device bus suspend */
	if (test_bit(A_BUS_SUSPEND, &motg->inputs))
		msm_otg_sm_work(&motg->sm_work);
}

static irqreturn_t msm_otg_irq(int irq, void *data)
{
	struct msm_otg *motg = data;
	struct usb_otg *otg = motg->phy.otg;
	u32 otgsc = 0, usbsts, pc;
	bool work = 0;
	irqreturn_t ret = IRQ_HANDLED;

	if (atomic_read(&motg->in_lpm)) {
		pr_debug("OTG IRQ: %d in LPM\n", irq);
		disable_irq_nosync(irq);
		motg->async_int = irq;
		if (!atomic_read(&motg->pm_suspended))
			pm_request_resume(otg->phy->dev);
		return IRQ_HANDLED;
	}

	usbsts = readl(USB_USBSTS);
	otgsc = readl(USB_OTGSC);

	if (!(otgsc & OTG_OTGSTS_MASK) && !(usbsts & OTG_USBSTS_MASK))
		return IRQ_NONE;

	if ((otgsc & OTGSC_IDIS) && (otgsc & OTGSC_IDIE)) {
		if (otgsc & OTGSC_ID) {
			dev_dbg(otg->phy->dev, "ID set\n");
			set_bit(ID, &motg->inputs);
		} else {
			dev_dbg(otg->phy->dev, "ID clear\n");
			/*
			 * Assert a_bus_req to supply power on
			 * VBUS when Micro/Mini-A cable is connected
			 * with out user intervention.
			 */
			set_bit(A_BUS_REQ, &motg->inputs);
			clear_bit(ID, &motg->inputs);
			msm_chg_enable_aca_det(motg);
		}
		writel_relaxed(otgsc, USB_OTGSC);
		work = 1;
	} else if (otgsc & OTGSC_DPIS) {
		pr_debug("DPIS detected\n");
		writel_relaxed(otgsc, USB_OTGSC);
		set_bit(A_SRP_DET, &motg->inputs);
		set_bit(A_BUS_REQ, &motg->inputs);
		work = 1;
	} else if ((otgsc & OTGSC_BSVIE) && (otgsc & OTGSC_BSVIS)) {
		writel_relaxed(otgsc, USB_OTGSC);
		/*
		 * BSV interrupt comes when operating as an A-device
		 * (VBUS on/off).
		 * But, handle BSV when charger is removed from ACA in ID_A
		 */
		if ((otg->phy->state >= OTG_STATE_A_IDLE) &&
			!test_bit(ID_A, &motg->inputs))
			return IRQ_HANDLED;
		if (otgsc & OTGSC_BSV) {
			dev_dbg(otg->phy->dev, "BSV set\n");
			set_bit(B_SESS_VLD, &motg->inputs);
		} else {
			dev_dbg(otg->phy->dev, "BSV clear\n");
			clear_bit(B_SESS_VLD, &motg->inputs);
			clear_bit(A_BUS_SUSPEND, &motg->inputs);

			msm_chg_check_aca_intr(motg);
		}
		work = 1;
	} else if (usbsts & STS_PCI) {
		pc = readl_relaxed(USB_PORTSC);
		pr_debug("portsc = %x\n", pc);
		ret = IRQ_NONE;
		/*
		 * HCD Acks PCI interrupt. We use this to switch
		 * between different OTG states.
		 */
		work = 1;
		switch (otg->phy->state) {
		case OTG_STATE_A_SUSPEND:
			if (otg->host->b_hnp_enable && (pc & PORTSC_CSC) &&
					!(pc & PORTSC_CCS)) {
				pr_debug("B_CONN clear\n");
				clear_bit(B_CONN, &motg->inputs);
				msm_otg_del_timer(motg);
			}
			break;
		case OTG_STATE_A_PERIPHERAL:
			/*
			 * A-peripheral observed activity on bus.
			 * clear A_BIDL_ADIS timer.
			 */
			msm_otg_del_timer(motg);
			work = 0;
			break;
		case OTG_STATE_B_WAIT_ACON:
			if ((pc & PORTSC_CSC) && (pc & PORTSC_CCS)) {
				pr_debug("A_CONN set\n");
				set_bit(A_CONN, &motg->inputs);
				/* Clear ASE0_BRST timer */
				msm_otg_del_timer(motg);
			}
			break;
		case OTG_STATE_B_HOST:
			if ((pc & PORTSC_CSC) && !(pc & PORTSC_CCS)) {
				pr_debug("A_CONN clear\n");
				clear_bit(A_CONN, &motg->inputs);
				msm_otg_del_timer(motg);
			}
			break;
		case OTG_STATE_A_WAIT_BCON:
			if (TA_WAIT_BCON < 0)
				set_bit(A_BUS_REQ, &motg->inputs);
		default:
			work = 0;
			break;
		}
	} else if (usbsts & STS_URI) {
		ret = IRQ_NONE;
		switch (otg->phy->state) {
		case OTG_STATE_A_PERIPHERAL:
			/*
			 * A-peripheral observed activity on bus.
			 * clear A_BIDL_ADIS timer.
			 */
			msm_otg_del_timer(motg);
			work = 0;
			break;
		default:
			work = 0;
			break;
		}
	} else if (usbsts & STS_SLI) {
		ret = IRQ_NONE;
		work = 0;
		switch (otg->phy->state) {
		case OTG_STATE_B_PERIPHERAL:
			if (otg->gadget->b_hnp_enable) {
				set_bit(A_BUS_SUSPEND, &motg->inputs);
				set_bit(B_BUS_REQ, &motg->inputs);
				work = 1;
			}
			break;
		case OTG_STATE_A_PERIPHERAL:
			msm_otg_start_timer(motg, TA_BIDL_ADIS,
					A_BIDL_ADIS);
			break;
		default:
			break;
		}
	} else if ((usbsts & PHY_ALT_INT)) {
		writel_relaxed(PHY_ALT_INT, USB_USBSTS);
		if (msm_chg_check_aca_intr(motg))
			work = 1;
		ret = IRQ_HANDLED;
	}
	if (work)
		queue_work(system_nrt_wq, &motg->sm_work);

	return ret;
}
// Caller: otg_power_set_property_usb due to power present.
static void msm_otg_set_vbus_state(int online)
{
	static bool init;
	struct msm_otg *motg = the_msm_otg;

	if (online) {
		pr_debug("PMIC: BSV set\n");
		set_bit(B_SESS_VLD, &motg->inputs);
	} else {
		pr_debug("PMIC: BSV clear\n");
		clear_bit(B_SESS_VLD, &motg->inputs);
	}

	/* do not queue state m/c work if id is grounded */
	if (!test_bit(ID, &motg->inputs)) {
		/*
		 * state machine work waits for initial VBUS
		 * completion in UNDEFINED state.  Process
		 * the initial VBUS event in ID_GND state.
		 */
		if (init)
			return;
	}

	if (!init) {
		init = true;
		complete(&pmic_vbus_init);
		pr_debug("PMIC: BSV init complete\n");
		return;
	}

	if (test_bit(MHL, &motg->inputs) ||
			mhl_det_in_progress) {
		pr_debug("PMIC: BSV interrupt ignored in MHL\n");
		return;
	}

	if (atomic_read(&motg->pm_suspended))
		motg->sm_work_pending = true;
	else
		queue_work(system_nrt_wq, &motg->sm_work);
}

static void msm_pmic_id_status_w(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg,
						pmic_id_status_work.work);
	int work = 0;

	if (msm_otg_read_pmic_id_state(motg)) {
		if (!test_and_set_bit(ID, &motg->inputs)) {
			pr_debug("PMIC: ID set\n");
			work = 1;
		}
	} else {
		if (test_and_clear_bit(ID, &motg->inputs)) {
			pr_debug("PMIC: ID clear\n");
			set_bit(A_BUS_REQ, &motg->inputs);
			work = 1;
		}
	}

	if (work && (motg->phy.state != OTG_STATE_UNDEFINED)) {
		if (atomic_read(&motg->pm_suspended))
			motg->sm_work_pending = true;
		else
			queue_work(system_nrt_wq, &motg->sm_work);
	}

}

#define MSM_PMIC_ID_STATUS_DELAY	5 /* 5msec */
static irqreturn_t msm_pmic_id_irq(int irq, void *data)
{
	struct msm_otg *motg = data;

	if (test_bit(MHL, &motg->inputs) ||
			mhl_det_in_progress) {
		pr_debug("PMIC: Id interrupt ignored in MHL\n");
		return IRQ_HANDLED;
	}

	if (!aca_id_turned_on)
		/*schedule delayed work for 5msec for ID line state to settle*/
		queue_delayed_work(system_nrt_wq, &motg->pmic_id_status_work,
				msecs_to_jiffies(MSM_PMIC_ID_STATUS_DELAY));

	return IRQ_HANDLED;
}

static int msm_otg_mode_show(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;
	struct usb_otg *otg = motg->phy.otg;

	switch (otg->phy->state) {
	case OTG_STATE_A_HOST:
		seq_printf(s, "host\n");
		break;
	case OTG_STATE_B_PERIPHERAL:
		seq_printf(s, "peripheral\n");
		break;
	default:
		seq_printf(s, "none\n");
		break;
	}

	return 0;
}

static int msm_otg_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_mode_show, inode->i_private);
}

static ssize_t msm_otg_mode_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_otg *motg = s->private;
	char buf[16];
	struct usb_phy *phy = &motg->phy;
	int status = count;
	enum usb_mode_type req_mode;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		status = -EFAULT;
		goto out;
	}

	if (!strncmp(buf, "host", 4)) {
		req_mode = USB_HOST;
	} else if (!strncmp(buf, "peripheral", 10)) {
		req_mode = USB_PERIPHERAL;
	} else if (!strncmp(buf, "none", 4)) {
		req_mode = USB_NONE;
	} else {
		status = -EINVAL;
		goto out;
	}

	switch (req_mode) {
	case USB_NONE:
		switch (phy->state) {
		case OTG_STATE_A_HOST:
		case OTG_STATE_B_PERIPHERAL:
			set_bit(ID, &motg->inputs);
			clear_bit(B_SESS_VLD, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_PERIPHERAL:
		switch (phy->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_A_HOST:
			set_bit(ID, &motg->inputs);
			set_bit(B_SESS_VLD, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_HOST:
		switch (phy->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_B_PERIPHERAL:
			clear_bit(ID, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	default:
		goto out;
	}

	pm_runtime_resume(phy->dev);
	queue_work(system_nrt_wq, &motg->sm_work);
out:
	return status;
}

const struct file_operations msm_otg_mode_fops = {
	.open = msm_otg_mode_open,
	.read = seq_read,
	.write = msm_otg_mode_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_show_otg_state(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;
	struct usb_phy *phy = &motg->phy;

	seq_printf(s, "%s\n", otg_state_string(phy->state));
	return 0;
}

static int msm_otg_otg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_show_otg_state, inode->i_private);
}

const struct file_operations msm_otg_state_fops = {
	.open = msm_otg_otg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_show_chg_type(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;

	seq_printf(s, "%s\n", chg_to_string(motg->chg_type));
	return 0;
}

static int msm_otg_chg_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_show_chg_type, inode->i_private);
}

const struct file_operations msm_otg_chg_fops = {
	.open = msm_otg_chg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_aca_show(struct seq_file *s, void *unused)
{
	if (debug_aca_enabled)
		seq_printf(s, "enabled\n");
	else
		seq_printf(s, "disabled\n");

	return 0;
}

static int msm_otg_aca_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_aca_show, inode->i_private);
}

static ssize_t msm_otg_aca_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[8];

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6))
		debug_aca_enabled = true;
	else
		debug_aca_enabled = false;

	return count;
}

const struct file_operations msm_otg_aca_fops = {
	.open = msm_otg_aca_open,
	.read = seq_read,
	.write = msm_otg_aca_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_bus_show(struct seq_file *s, void *unused)
{
	if (debug_bus_voting_enabled)
		seq_printf(s, "enabled\n");
	else
		seq_printf(s, "disabled\n");

	return 0;
}

static int msm_otg_bus_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_bus_show, inode->i_private);
}

static ssize_t msm_otg_bus_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[8];
	struct seq_file *s = file->private_data;
	struct msm_otg *motg = s->private;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6)) {
		/* Do not vote here. Let OTG statemachine decide when to vote */
		debug_bus_voting_enabled = true;
	} else {
		debug_bus_voting_enabled = false;
		msm_otg_bus_vote(motg, USB_MIN_PERF_VOTE);
	}

	return count;
}

static int
otg_get_prop_usbin_voltage_now(struct msm_otg *motg)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (IS_ERR_OR_NULL(motg->vadc_dev)) {
		motg->vadc_dev = qpnp_get_vadc(motg->phy.dev, "usbin");
		if (IS_ERR(motg->vadc_dev))
			return PTR_ERR(motg->vadc_dev);
	}

	rc = qpnp_vadc_read(motg->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read usbin rc=%d\n", rc);
		return 0;
	} else {
		return results.physical;
	}
}

static int otg_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct msm_otg *motg = container_of(psy, struct msm_otg, usb_psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		if (motg->host_mode)
			val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		else
			val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = motg->voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = motg->current_max;
		break;
	/* Reflect USB enumeration */
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = motg->online;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = psy->type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = motg->usbin_health;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = otg_get_prop_usbin_voltage_now(motg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

//Caller: this is public API.
static int otg_power_set_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct msm_otg *motg = container_of(psy, struct msm_otg, usb_psy);

	switch (psp) {
	/* Process PMIC notification in PRESENT prop */
	case POWER_SUPPLY_PROP_PRESENT:
		msm_otg_set_vbus_state(val->intval);
		break;
	/* The ONLINE property reflects if usb has enumerated */
	case POWER_SUPPLY_PROP_ONLINE:
		motg->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		motg->voltage_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		motg->current_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		psy->type = val->intval;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		motg->usbin_health = val->intval;
		break;
	default:
		return -EINVAL;
	}

	power_supply_changed(&motg->usb_psy);
	return 0;
}

static int otg_power_property_is_writeable_usb(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static char *otg_pm_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property otg_pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

const struct file_operations msm_otg_bus_fops = {
	.open = msm_otg_bus_open,
	.read = seq_read,
	.write = msm_otg_bus_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *msm_otg_dbg_root;

static int msm_otg_debugfs_init(struct msm_otg *motg)
{
	struct dentry *msm_otg_dentry;

	msm_otg_dbg_root = debugfs_create_dir("msm_otg", NULL);

	if (!msm_otg_dbg_root || IS_ERR(msm_otg_dbg_root))
		return -ENODEV;
	msm_otg_dentry = debugfs_create_file("chg_type", S_IRUGO, msm_otg_dbg_root, motg, &msm_otg_chg_fops);
	msm_otg_dentry = debugfs_create_file("aca", S_IRUGO | S_IWUSR, msm_otg_dbg_root, motg, &msm_otg_aca_fops);
	msm_otg_dentry = debugfs_create_file("bus_voting", S_IRUGO | S_IWUSR, msm_otg_dbg_root, motg, &msm_otg_bus_fops);
	msm_otg_dentry = debugfs_create_file("otg_state", S_IRUGO, msm_otg_dbg_root, motg, &msm_otg_state_fops);
	return 0;
}

static void msm_otg_debugfs_cleanup(void)
{
	debugfs_remove_recursive(msm_otg_dbg_root);
}

#define MSM_OTG_CMD_ID		0x09
#define MSM_OTG_DEVICE_ID	0x04
#define MSM_OTG_VMID_IDX	0xFF
#define MSM_OTG_MEM_TYPE	0x02
struct msm_otg_scm_cmd_buf {
	unsigned int device_id;
	unsigned int vmid_idx;
	unsigned int mem_type;
} __attribute__ ((__packed__));

static void msm_otg_pnoc_errata_fix(struct msm_otg *motg)
{
	int ret;
	struct msm_otg_platform_data *pdata = motg->pdata;
	struct msm_otg_scm_cmd_buf cmd_buf;

	if (!pdata->pnoc_errata_fix)
		return;

	dev_dbg(motg->phy.dev, "applying fix for pnoc h/w issue\n");

	cmd_buf.device_id = MSM_OTG_DEVICE_ID;
	cmd_buf.vmid_idx = MSM_OTG_VMID_IDX;
	cmd_buf.mem_type = MSM_OTG_MEM_TYPE;

	ret = scm_call(SCM_SVC_MP, MSM_OTG_CMD_ID, &cmd_buf,
				sizeof(cmd_buf), NULL, 0);

	if (ret)
		dev_err(motg->phy.dev, "scm command failed to update VMIDMT\n");
}

static u64 msm_otg_dma_mask = DMA_BIT_MASK(64);
static struct platform_device *msm_otg_add_pdev(
		struct platform_device *ofdev, const char *name)
{
	struct platform_device *pdev;
	const struct resource *res = ofdev->resource;
	unsigned int num = ofdev->num_resources;
	int retval;
	struct ci13xxx_platform_data ci_pdata;
	struct msm_otg_platform_data *otg_pdata;

	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		retval = -ENOMEM;
		goto error;
	}

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &msm_otg_dma_mask;

	if (num) {
		retval = platform_device_add_resources(pdev, res, num);
		if (retval)
			goto error;
	}

	if (!strcmp(name, "msm_hsusb")) {
		otg_pdata =
			(struct msm_otg_platform_data *)
				ofdev->dev.platform_data;
		ci_pdata.log2_itc = otg_pdata->log2_itc;
		ci_pdata.usb_core_id = 0;
		ci_pdata.l1_supported = otg_pdata->l1_supported;
		ci_pdata.enable_ahb2ahb_bypass =
				otg_pdata->enable_ahb2ahb_bypass;
		retval = platform_device_add_data(pdev, &ci_pdata,
			sizeof(ci_pdata));
		if (retval)
			goto error;
	}

	retval = platform_device_add(pdev);
	if (retval)
		goto error;

	return pdev;

error:
	platform_device_put(pdev);
	return ERR_PTR(retval);
}

static int msm_otg_setup_devices(struct platform_device *ofdev,
		enum usb_mode_type mode, bool init)
{
	const char *gadget_name = "msm_hsusb";
	const char *host_name = "msm_hsusb_host";
	static struct platform_device *gadget_pdev;
	static struct platform_device *host_pdev;
	int retval = 0;

	if (!init) { // Mike, remove driver goes here!
		if (gadget_pdev)
			platform_device_unregister(gadget_pdev);
		if (host_pdev)
			platform_device_unregister(host_pdev);
		return 0;
	}

	switch (mode) {
		// Mike, it is OTG, so need to init gadget_pdev and host_pdev.
	case USB_OTG:
	case USB_PERIPHERAL:
		gadget_pdev = msm_otg_add_pdev(ofdev, gadget_name);
	case USB_HOST:
		host_pdev = msm_otg_add_pdev(ofdev, host_name);
	default:
		break;
	}

	return retval;
}

static int msm_otg_register_power_supply(struct platform_device *pdev,
					struct msm_otg *motg)
{
	int ret;

	ret = power_supply_register(&pdev->dev, &motg->usb_psy);
	if (ret < 0) {
		dev_err(motg->phy.dev,
			"%s:power_supply_register usb failed\n",
			__func__);
		return ret;
	}

	legacy_power_supply = false;
	return 0;
}

static int msm_otg_ext_chg_open(struct inode *inode, struct file *file)
{
	struct msm_otg *motg = the_msm_otg;

	pr_debug("msm_otg ext chg open\n");

	motg->ext_chg_opened = true;
	file->private_data = (void *)motg;
	return 0;
}

static long
msm_otg_ext_chg_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct msm_otg *motg = file->private_data;
	struct msm_usb_chg_info info = {0};
	int ret = 0, val;

	switch (cmd) {
	case MSM_USB_EXT_CHG_INFO:
		info.chg_block_type = USB_CHG_BLOCK_ULPI;
		info.page_offset = motg->io_res->start & ~PAGE_MASK;
		/* mmap() works on PAGE granularity */
		info.length = PAGE_SIZE;

		if (copy_to_user((void __user *)arg, &info, sizeof(info))) {
			pr_err("%s: copy to user failed\n\n", __func__);
			ret = -EFAULT;
		}
		break;
	case MSM_USB_EXT_CHG_BLOCK_LPM:
		if (get_user(val, (int __user *)arg)) {
			pr_err("%s: get_user failed\n\n", __func__);
			ret = -EFAULT;
			break;
		}
		pr_debug("%s: LPM block request %d\n", __func__, val);
		if (val) { /* block LPM */
			if (motg->chg_type == USB_DCP_CHARGER) {
				/*
				 * If device is already suspended, resume it.
				 * The PM usage counter is incremented in
				 * runtime resume method.  if device is not
				 * suspended, cancel the scheduled suspend
				 * and increment the PM usage counter.
				 */
				if (pm_runtime_suspended(motg->phy.dev))
					pm_runtime_resume(motg->phy.dev);
				else
					pm_runtime_get_sync(motg->phy.dev);
			} else {
				motg->ext_chg_active = false;
				complete(&motg->ext_chg_wait);
				ret = -ENODEV;
			}
		} else {
			motg->ext_chg_active = false;
			complete(&motg->ext_chg_wait);
			pm_runtime_put(motg->phy.dev);
		}
		break;
	case MSM_USB_EXT_CHG_VOLTAGE_INFO:
		if (get_user(val, (int __user *)arg)) {
			pr_err("%s: get_user failed\n\n", __func__);
			ret = -EFAULT;
			break;
		}

		if (val == USB_REQUEST_5V)
			pr_debug("%s:voting 5V voltage request\n", __func__);
		else if (val == USB_REQUEST_9V)
			pr_debug("%s:voting 9V voltage request\n", __func__);
		break;
	case MSM_USB_EXT_CHG_RESULT:
		if (get_user(val, (int __user *)arg)) {
			pr_err("%s: get_user failed\n\n", __func__);
			ret = -EFAULT;
			break;
		}

		if (!val)
			pr_debug("%s:voltage request successful\n", __func__);
		else
			pr_debug("%s:voltage request failed\n", __func__);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int msm_otg_ext_chg_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct msm_otg *motg = file->private_data;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	int ret;

	if (vma->vm_pgoff || vsize > PAGE_SIZE)
		return -EINVAL;

	vma->vm_pgoff = __phys_to_pfn(motg->io_res->start);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				 vsize, vma->vm_page_prot);
	if (ret < 0) {
		pr_err("%s: failed with return val %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int msm_otg_ext_chg_release(struct inode *inode, struct file *file)
{
	struct msm_otg *motg = file->private_data;

	pr_debug("msm_otg ext chg release\n");

	motg->ext_chg_opened = false;

	return 0;
}

static const struct file_operations msm_otg_ext_chg_fops = {
	.owner = THIS_MODULE,
	.open = msm_otg_ext_chg_open,
	.unlocked_ioctl = msm_otg_ext_chg_ioctl,
	.mmap = msm_otg_ext_chg_mmap,
	.release = msm_otg_ext_chg_release,
};

// probe() only. mike readed.
static int msm_otg_setup_ext_chg_cdev(struct msm_otg *motg)
{
	init_completion(&motg->ext_chg_wait);
	pr_debug("msm otg ext chg cdev setup success\n");
	return 0;
}

static ssize_t dpdm_pulldown_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct msm_otg *motg = the_msm_otg;
	struct msm_otg_platform_data *pdata = motg->pdata;

	return snprintf(buf, PAGE_SIZE, "%s\n", pdata->dpdm_pulldown_added ?
							"enabled" : "disabled");
}

static ssize_t dpdm_pulldown_enable_store(struct device *dev,
		struct device_attribute *attr, const char
		*buf, size_t size)
{
	struct msm_otg *motg = the_msm_otg;
	struct msm_otg_platform_data *pdata = motg->pdata;

	if (!strnicmp(buf, "enable", 6)) {
		pdata->dpdm_pulldown_added = true;
		return size;
	} else if (!strnicmp(buf, "disable", 7)) {
		pdata->dpdm_pulldown_added = false;
		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(dpdm_pulldown_enable, S_IRUGO | S_IWUSR,
		dpdm_pulldown_enable_show, dpdm_pulldown_enable_store);

// Caller: msm_otg_probe
struct msm_otg_platform_data *msm_otg_dt_to_pdata(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct msm_otg_platform_data *pdata;
	int len = 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	of_get_property(node, "qcom,hsusb-otg-phy-init-seq", &len);
	of_property_read_u32(node, "qcom,hsusb-otg-power-budget", &pdata->power_budget);
	of_property_read_u32(node, "qcom,hsusb-otg-mode", &pdata->mode);
	of_property_read_u32(node, "qcom,hsusb-otg-otg-control",&pdata->otg_control);
	of_property_read_u32(node, "qcom,hsusb-otg-default-mode", &pdata->default_mode);
	of_property_read_u32(node, "qcom,hsusb-otg-phy-type", &pdata->phy_type);

	pdata->disable_reset_on_disconnect = of_property_read_bool(node, "qcom,hsusb-otg-disable-reset");
	pdata->pnoc_errata_fix = of_property_read_bool(node, "qcom,hsusb-otg-pnoc-errata-fix");
	pdata->enable_lpm_on_dev_suspend = of_property_read_bool(node, "qcom,hsusb-otg-lpm-on-dev-suspend");
	pdata->core_clk_always_on_workaround = of_property_read_bool(node, "qcom,hsusb-otg-clk-always-on-workaround");
	pdata->delay_lpm_on_disconnect = of_property_read_bool(node, "qcom,hsusb-otg-delay-lpm");
	pdata->delay_lpm_hndshk_on_disconnect = of_property_read_bool(node, "qcom,hsusb-otg-delay-lpm-hndshk-on-disconnect");
	pdata->dp_manual_pullup = of_property_read_bool(node, "qcom,dp-manual-pullup");
	pdata->enable_sec_phy = of_property_read_bool(node, "qcom,usb2-enable-hsphy2");
	of_property_read_u32(node, "qcom,hsusb-log2-itc", &pdata->log2_itc);

	of_property_read_u32(node, "qcom,hsusb-otg-mpm-dpsehv-int", &pdata->mpm_dpshv_int);
	of_property_read_u32(node, "qcom,hsusb-otg-mpm-dmsehv-int", &pdata->mpm_dmshv_int);
	pdata->pmic_id_irq = platform_get_irq_byname(pdev, "pmic_id_irq"); // 288.
	if (pdata->pmic_id_irq < 0)
		pdata->pmic_id_irq = 0;

	pdata->l1_supported = of_property_read_bool(node, "qcom,hsusb-l1-supported");
	pdata->enable_ahb2ahb_bypass = of_property_read_bool(node, "qcom,ahb-async-bridge-bypass");
	pdata->disable_retention_with_vdd_min = of_property_read_bool(node, "qcom,disable-retention-with-vdd-min");

	return pdata;
}

static int __init msm_otg_probe(struct platform_device *pdev)
{
	int ret = 0;
	int len = 0;
	u32 tmp[3];
	struct resource *res;
	struct msm_otg *motg;
	struct usb_phy *phy;
	struct msm_otg_platform_data *pdata;


	if (1) {
		// Use device tree to initialize pdata.
		pdata = msm_otg_dt_to_pdata(pdev);
		pdev->dev.platform_data = pdata;
		msm_otg_setup_devices(pdev, pdata->mode, :init = true);
	}

	motg = devm_kzalloc(&pdev->dev, sizeof(struct msm_otg), GFP_KERNEL);

	motg->phy.otg = devm_kzalloc(&pdev->dev, sizeof(struct usb_otg), GFP_KERNEL);

	the_msm_otg = motg;
	motg->pdata = pdata;
	phy = &motg->phy;
	phy->dev = &pdev->dev;

	// Remove un-used code due to $otg_control is 'OTG_PMIC_CONTROL'

	/* initialize reset counter */ // Seems no use.
	motg->reset_counter = 0;

	// Remoce clock config.

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	motg->io_res = res;
	motg->regs = ioremap(res->start, resource_size(res));

	dev_info(&pdev->dev, "OTG regs = %p\n", motg->regs);

	motg->irq = platform_get_irq(pdev, 0); // 166

	msm_hsusb_config_vddcx(1);

	regulator_enable(hsusb_vdd);

	msm_hsusb_ldo_init(motg, 1);

	// Checked! NO!
	if (pdata->mhl_enable) {
	}

	ret = msm_hsusb_ldo_enable(motg, USB_PHY_REG_ON);

	// NO pnoc_errata_fix flag. Ignore it!

	writel(0, USB_USBINTR);
	writel(0, USB_OTGSC);

	msm_otg_mhl_register_callback(motg, msm_otg_mhl_notify_online);

	wake_lock_init(&motg->wlock, WAKE_LOCK_SUSPEND, "msm_otg");
	msm_otg_init_timer(motg);
	INIT_WORK(&motg->sm_work, msm_otg_sm_work);
	INIT_DELAYED_WORK(&motg->chg_work, msm_chg_detect_work);
	INIT_DELAYED_WORK(&motg->pmic_id_status_work, msm_pmic_id_status_w);
	INIT_DELAYED_WORK(&motg->suspend_work, msm_otg_suspend_work);
	setup_timer(&motg->id_timer, msm_otg_id_timer_func, (unsigned long) motg);
	setup_timer(&motg->chg_check_timer, msm_otg_chg_check_timer_func, (unsigned long) motg);

	request_irq(motg->irq, msm_otg_irq, IRQF_SHARED, "msm_otg", motg);

	motg->async_irq = platform_get_irq_byname(pdev, "async_irq"); //172
	// mike, checked, motg->async_irq is 172.
	if (motg->async_irq) {
		request_irq(motg->async_irq, msm_otg_irq,
					IRQF_TRIGGER_RISING, "msm_otg", motg);
		disable_irq(motg->async_irq);
	}

	// Remove codes due to $otg_control is  OTG_PMIC_CONTROL

	phy->init = msm_otg_reset; //API.
	phy->set_power = msm_otg_set_power; // API
	phy->set_suspend = msm_otg_set_suspend; // API

	phy->io_ops = &msm_otg_io_ops;

	phy->otg->phy = &motg->phy;
	phy->otg->set_host = msm_otg_set_host;
	phy->otg->set_peripheral = msm_otg_set_peripheral;
	phy->otg->start_hnp = msm_otg_start_hnp;
	phy->otg->start_srp = msm_otg_start_srp;

	// Yes, it is
	if (pdata->dp_manual_pullup)
		phy->flags |= ENABLE_DP_MANUAL_PULLUP;

	// Remove codes due to no enable_sec_phy flag.
	//	phy->flags |= ENABLE_SECONDARY_PHY;

	ret = usb_set_transceiver(&motg->phy);

	if (motg->pdata->mode == USB_OTG &&
		motg->pdata->otg_control == OTG_PMIC_CONTROL) {
		// YES! Mike.
		if (motg->pdata->pmic_id_irq) {
			// YES! Mike.
			ret = request_irq(motg->pdata->pmic_id_irq, msm_pmic_id_irq,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						"msm_otg", motg);
		}
	}

	msm_hsusb_mhl_switch_enable(motg, 1);

	platform_set_drvdata(pdev, motg);
	device_init_wakeup(&pdev->dev, 1);
	motg->mA_port = IUNIT;

	msm_otg_debugfs_init(motg);

	if (motg->pdata->phy_type == SNPS_28NM_INTEGRATED_PHY) {
		// YES! Mike.
		if (motg->pdata->otg_control == OTG_PMIC_CONTROL &&
			(!(motg->pdata->mode == USB_OTG) ||
			 motg->pdata->pmic_id_irq))
			motg->caps = ALLOW_PHY_POWER_COLLAPSE | ALLOW_PHY_RETENTION;
		// Remove by flag $otg_control is "OTG_PHY_CONTROL"
		// pdata->mpm_dpshv_int and pdata->mpm_dmshv_int are both 0.
	}

	// NO! Mike.
	if (motg->pdata->enable_lpm_on_dev_suspend)
		motg->caps |= ALLOW_LPM_ON_DEV_SUSPEND;

	// NO! Mike.
	if (motg->pdata->disable_retention_with_vdd_min) // it is 0!
		motg->caps |= ALLOW_VDD_MIN_WITH_RETENTION_DISABLED;

	// Mike, so the motg->caps ONLY has ALLOW_PHY_POWER_COLLAPSE + ALLOW_PHY_RETENTION

	wake_lock(&motg->wlock);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	// No flag: delay_lpm_on_disconnect

	motg->usb_psy.name = "usb";
	motg->usb_psy.type = POWER_SUPPLY_TYPE_USB;
	motg->usb_psy.supplied_to = otg_pm_power_supplied_to;
	motg->usb_psy.num_supplicants = ARRAY_SIZE(otg_pm_power_supplied_to);
	motg->usb_psy.properties = otg_pm_power_props_usb;
	motg->usb_psy.num_properties = ARRAY_SIZE(otg_pm_power_props_usb);
	motg->usb_psy.get_property = otg_power_get_property_usb;
	motg->usb_psy.set_property = otg_power_set_property_usb;
	motg->usb_psy.property_is_writeable = otg_power_property_is_writeable_usb;

	// checked!  Mike, it is legacy_power_supply <- false.
	if (!pm8921_charger_register_vbus_sn(NULL)) {
		// No!!!
	} else { 
		// YES.
		/* otherwise register our own power supply */
		if (!msm_otg_register_power_supply(pdev, motg))
			psy = &motg->usb_psy;
	}

	msm_otg_setup_ext_chg_cdev(motg);

	return 0;
}

static int __devexit msm_otg_remove(struct platform_device *pdev)
{
	struct msm_otg *motg = platform_get_drvdata(pdev);
	struct usb_phy *phy = &motg->phy;
	int cnt = 0;

	if (phy->otg->host || phy->otg->gadget)
		return -EBUSY;

	if (!motg->ext_chg_device) {
		device_destroy(motg->ext_chg_class, motg->ext_chg_dev);
		cdev_del(&motg->ext_chg_cdev);
		class_destroy(motg->ext_chg_class);
		unregister_chrdev_region(motg->ext_chg_dev, 1);
	}

	if (pdev->dev.of_node)
		msm_otg_setup_devices(pdev, motg->pdata->mode, false);
	if (motg->pdata->otg_control == OTG_PMIC_CONTROL)
		pm8921_charger_unregister_vbus_sn(0);
	msm_otg_mhl_register_callback(motg, NULL);
	msm_otg_debugfs_cleanup();
	cancel_delayed_work_sync(&motg->chg_work);
	cancel_delayed_work_sync(&motg->pmic_id_status_work);
	cancel_delayed_work_sync(&motg->suspend_work);
	cancel_work_sync(&motg->sm_work);

	pm_runtime_resume(&pdev->dev);

	device_init_wakeup(&pdev->dev, 0);
	pm_runtime_disable(&pdev->dev);
	wake_lock_destroy(&motg->wlock);

	msm_hsusb_mhl_switch_enable(motg, 0);
	if (motg->pdata->pmic_id_irq)
		free_irq(motg->pdata->pmic_id_irq, motg);
	usb_set_transceiver(NULL);
	free_irq(motg->irq, motg);

	if ((motg->pdata->phy_type == SNPS_28NM_INTEGRATED_PHY) &&
		(motg->pdata->mpm_dpshv_int || motg->pdata->mpm_dmshv_int))
			device_remove_file(&pdev->dev,
					&dev_attr_dpdm_pulldown_enable);
	if (motg->pdata->otg_control == OTG_PHY_CONTROL &&
		motg->pdata->mpm_otgsessvld_int)
		msm_mpm_enable_pin(motg->pdata->mpm_otgsessvld_int, 0);

	if (motg->pdata->mpm_dpshv_int)
		msm_mpm_enable_pin(motg->pdata->mpm_dpshv_int, 0);
	if (motg->pdata->mpm_dmshv_int)
		msm_mpm_enable_pin(motg->pdata->mpm_dmshv_int, 0);

	/*
	 * Put PHY in low power mode.
	 */
	ulpi_read(phy, 0x14);
	ulpi_write(phy, 0x08, 0x09);

	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);
	while (cnt < PHY_SUSPEND_TIMEOUT_USEC) {
		if (readl(USB_PORTSC) & PORTSC_PHCD)
			break;
		udelay(1);
		cnt++;
	}
	if (cnt >= PHY_SUSPEND_TIMEOUT_USEC)
		dev_err(phy->dev, "Unable to suspend PHY\n");

	clk_disable_unprepare(motg->pclk);
	clk_disable_unprepare(motg->core_clk);
	if (!IS_ERR(motg->xo_clk)) {
		clk_disable_unprepare(motg->xo_clk);
		clk_put(motg->xo_clk);
	} else {
		msm_xo_put(motg->xo_handle);
	}

	if (!IS_ERR(motg->sleep_clk))
		clk_disable_unprepare(motg->sleep_clk);

	msm_hsusb_ldo_enable(motg, USB_PHY_REG_OFF);
	msm_hsusb_ldo_init(motg, 0);
	regulator_disable(hsusb_vdd);
	regulator_set_voltage(hsusb_vdd,
		vdd_val[motg->vdd_type][VDD_NONE],
		vdd_val[motg->vdd_type][VDD_MAX]);

	iounmap(motg->regs);
	pm_runtime_set_suspended(&pdev->dev);

	clk_put(motg->pclk);
	if (!IS_ERR(motg->clk))
		clk_put(motg->clk);
	clk_put(motg->core_clk);

	if (motg->bus_perf_client) {
		msm_otg_bus_vote(motg, USB_NO_PERF_VOTE);
		msm_bus_scale_unregister_client(motg->bus_perf_client);
	}

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int msm_otg_runtime_idle(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);
	struct usb_phy *phy = &motg->phy;

	dev_dbg(dev, "OTG runtime idle\n");

	if (phy->state == OTG_STATE_UNDEFINED)
		return -EAGAIN;

	if (motg->ext_chg_active) {
		dev_dbg(dev, "Deferring LPM\n");
		/*
		 * Charger detection may happen in user space.
		 * Delay entering LPM by 3 sec.  Otherwise we
		 * have to exit LPM when user space begins
		 * charger detection.
		 *
		 * This timer will be canceled when user space
		 * votes against LPM by incrementing PM usage
		 * counter.  We enter low power mode when
		 * PM usage counter is decremented.
		 */
		pm_schedule_suspend(dev, 3000);
		return -EAGAIN;
	}

	return 0;
}

static int msm_otg_runtime_suspend(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime suspend\n");
	return msm_otg_suspend(motg);
}

static int msm_otg_runtime_resume(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime resume\n");
	pm_runtime_get_noresume(dev);
	motg->pm_done = 0;
	return msm_otg_resume(motg);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int msm_otg_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG PM suspend\n");

	atomic_set(&motg->pm_suspended, 1);
	ret = msm_otg_suspend(motg);
	if (ret)
		atomic_set(&motg->pm_suspended, 0);

	return ret;
}

static int msm_otg_pm_resume(struct device *dev)
{
	int ret = 0;
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG PM resume\n");

	motg->pm_done = 0;
	atomic_set(&motg->pm_suspended, 0);
	if (motg->async_int || motg->sm_work_pending) {
		pm_runtime_get_noresume(dev);
		ret = msm_otg_resume(motg);

		/* Update runtime PM status */
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);

		if (motg->sm_work_pending) {
			motg->sm_work_pending = false;
			queue_work(system_nrt_wq, &motg->sm_work);
		}
	}

	return ret;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops msm_otg_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_otg_pm_suspend, msm_otg_pm_resume)
	SET_RUNTIME_PM_OPS(msm_otg_runtime_suspend, msm_otg_runtime_resume,
				msm_otg_runtime_idle)
};
#endif

static struct of_device_id msm_otg_dt_match[] = {
	{	.compatible = "qcom,hsusb-otg",
	},
	{}
};

static struct platform_driver msm_otg_driver = {
	.remove = __devexit_p(msm_otg_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &msm_otg_dev_pm_ops,
#endif
		.of_match_table = msm_otg_dt_match,
	},
};

static int __init msm_otg_init(void)
{
	return platform_driver_probe(&msm_otg_driver, msm_otg_probe);
}

static void __exit msm_otg_exit(void)
{
	platform_driver_unregister(&msm_otg_driver);
}

module_init(msm_otg_init);
module_exit(msm_otg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM USB transceiver driver");
