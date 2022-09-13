// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI D-PHY controller driver.
 * Platform driver
 *
 * Author: Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include <dw-dphy-data.h>
#include <dw-csi-data.h>
#include "dw-dphy-rx.h"
#include "bm_csi_hw.h"

/* Global variable for compatibility mode, this could be override later */
static int phy_type = 1; //Changed to fit single phy 0 - single | 1 - 8 lanes phy

module_param(phy_type, int, 0664);
MODULE_PARM_DESC(phy_type, "Disable compatibility mode for D-PHY G128");

static struct phy_ops dw_dphy_ops = {
	.init = dw_dphy_init,
	.reset = dw_dphy_reset,
	.power_on = dw_dphy_power_on,
	.power_off = dw_dphy_power_off,
	.owner = THIS_MODULE,
};

static int get_resources(struct device *dev, struct dw_dphy_rx *dphy)
{
	int ret = 0;

    struct platform_device *pdev = to_platform_device(dev);
    struct bm_csi_drvdata *drvdata = platform_get_drvdata(pdev);
	struct dw_phy_pdata *pdata = &drvdata->dphy_pdata;

	dphy->dphy_freq = pdata->dphy_frequency;
	dphy->dphy_te_len = pdata->dphy_te_len;
	dphy->dphy_gen = pdata->dphy_gen;
    drvdata->dphy = dphy;

	return ret;
}

static int phy_register(struct device *dev)
{
	int ret = 0;
    struct platform_device *pdev = to_platform_device(dev);
    struct bm_csi_drvdata *drvdata = platform_get_drvdata(pdev);
    struct dw_dphy_rx *dphy = drvdata->dphy;
	struct dw_phy_pdata *pdata = &drvdata->dphy_pdata;

	ret = phy_create_lookup(dphy->phy,
					phys[pdata->id].name,
					csis[pdata->id].name);
	if (ret)
		dev_err(dev, "Failed to create dphy lookup\n");
	else
		dev_warn(dev, "Created dphy lookup [%s] --> [%s]\n",
		    phys[pdata->id].name, csis[pdata->id].name);


	return ret;
}

static void phy_unregister(struct device *dev)
{
	if (!dev->of_node) {

        struct platform_device *pdev = to_platform_device(dev);
        struct bm_csi_drvdata *drvdata = platform_get_drvdata(pdev);
        struct dw_dphy_rx *dphy = drvdata->dphy;
	    struct dw_phy_pdata *pdata = &drvdata->dphy_pdata;

		phy_remove_lookup(dphy->phy,
				  phys[pdata->id].name, csis[pdata->id].name);
		dev_warn(dev, "Removed dphy lookup [%s] --> [%s]\n",
			 phys[pdata->id].name, csis[pdata->id].name);
	}
}

#define REG_DPHY_OFFSET 0x40
int dw_dphy_rx_probe(struct platform_device *pdev, void __iomem *dphy1_if_addr)
{
	struct device *dev = &pdev->dev;
	struct dw_dphy_rx *dphy;
    struct bm_csi_drvdata *drvdata;

	dphy = devm_kzalloc(&pdev->dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;

	drvdata = platform_get_drvdata(pdev);
	dphy->base_address = drvdata->base + REG_DPHY_OFFSET;
    drvdata->dphy= dphy;
	dphy->dphy1_if_addr = dphy1_if_addr;

    if (IS_ERR(dphy->base_address)) {
		dev_err(&pdev->dev, "error requesting base address\n");
		return PTR_ERR(dphy->base_address);
	}

	if (get_resources(dev, dphy)) {
		dev_err(dev, "failed to parse PHY resources\n");
		return -EINVAL;
	}

	dphy->phy = devm_phy_create(dev, NULL, &dw_dphy_ops);
	if (IS_ERR(dphy->phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(dphy->phy);
	}

	phy_set_drvdata(dphy->phy, dphy);

	if (phy_register(dev)) {
		dev_err(dev, "failed to register PHY\n");
		return -EINVAL;
	}

	dphy->lp_time = 1000; /* 1000 ns */
	dphy->lanes_config = dw_dphy_setup_config(dphy);

	dev_info(&dphy->phy->dev, "Probing dphy finished\n");

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_create_capabilities_sysfs(pdev);
#endif

	return 0;
}

int dw_dphy_rx_remove(struct platform_device *pdev)
{
    struct bm_csi_drvdata *drvdata = platform_get_drvdata(pdev);
    struct dw_dphy_rx *dphy = drvdata->dphy;

	dev_info(&dphy->phy->dev, "phy removed\n");
	phy_unregister(&pdev->dev);
    dw_dphy_remove_capabilities_sysfs(pdev);

	return 0;
}

MODULE_DESCRIPTION("Synopsys DesignWare MIPI DPHY Rx driver");
MODULE_AUTHOR("Luis Oliveira <lolivei@synopsys.com>");
MODULE_LICENSE("GPL v2");
