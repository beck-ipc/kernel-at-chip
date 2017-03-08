/*
 * drivers/net/phy/tlk106l.c
 *
 * Driver for Texas Instruments TLK106L and compatible PHYs
 *
 * Author: Beck IPC GmbH
 *
 * Copyright (c) 2016-2017 Beck IPC GmbH
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>

#define TLK106L_REGCR           0x0D    /* PHY Register for Extended Addressing */
#define TLK106L_ADDAR           0x0E    /* PHY Register for Extended Addressing */
#define TLK106L_MLED			0x25	/* PHY Specific Multi LED Control Register */

#define TLK106L_PHY_ID_MASK	0xFFFFFFF0
#define PHY_ID_TLK106L		0x2000A210
#define PHY_ID_DP83822      0x2000A240

static int tlk106l_extended_write(struct phy_device *phydev, u16 regnum, u16 val)
{
	int err = phy_write(phydev, TLK106L_REGCR, 0x1F);
    err |= phy_write(phydev, TLK106L_ADDAR, regnum);
    err |= phy_write(phydev, TLK106L_REGCR, 0x401F);
    err |= phy_write(phydev, TLK106L_ADDAR, val);
	return err;
}

static int tlk106l_config_init(struct phy_device *phydev)
{
	/* Enable MLED on pin 29 with RX/TX activity functionality */
	return tlk106l_extended_write(phydev, TLK106L_MLED, 0x408);
}

static struct phy_driver tlk106l_driver[] = {
{
	.phy_id			= PHY_ID_TLK106L,
	.phy_id_mask	= TLK106L_PHY_ID_MASK,
	.name			= "Texas Instruments TLK106L PHY",
	.features		= (PHY_BASIC_FEATURES | SUPPORTED_Pause),
	.flags			= PHY_HAS_MAGICANEG,
	.config_init	= tlk106l_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.suspend		= genphy_suspend,
	.resume			= genphy_resume,
}
};

static int __init tlk106l_init(void)
{
	int ret;

	ret = phy_drivers_register(tlk106l_driver, ARRAY_SIZE(tlk106l_driver), THIS_MODULE);
	if (ret)
		phy_drivers_unregister(tlk106l_driver, ARRAY_SIZE(tlk106l_driver));
		
	return ret;
}

static void __exit tlk106l_exit(void)
{
	phy_drivers_unregister(tlk106l_driver, ARRAY_SIZE(tlk106l_driver));
}

module_init(tlk106l_init);
module_exit(tlk106l_exit);

MODULE_DESCRIPTION("TLK106L PHY driver");
MODULE_AUTHOR("Beck IPC GmbH");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused tlk106_tbl[] = {
	{ PHY_ID_TLK106L, TLK106L_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, tlk106l_tbl);
