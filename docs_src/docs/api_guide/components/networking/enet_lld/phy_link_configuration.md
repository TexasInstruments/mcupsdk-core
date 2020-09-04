Ethernet PHY Link Configuration {#enetphy_link_config_top}
=====================

[TOC]

# Link Configuration Guidelines {#enetphy_link_config_guidelines}

The success in establishing a reliable Ethernet link depends on different
factors including configuration parameters of the link partners like mode,
speed and duplexity, and also on choosing the right cabling for a given speed.

The following sections provide general guidelines to configure link partners
when establishing Ethernet links.


## Manual Mode {#enetphy_link_manual}

### Half-Duplex Mode {#enetphy_link_manual_half_duplex}

When either link partner is in manual mode with duplexity set to half-duplex,
the other link partner should be configured in any of the following modes:

- Manual mode with the exact same speed and duplexity.  Otherwise:
   + Speed mismatch can cause link not to be established at all.
   + Duplexity mismatch can result in an established link, but lower performance
     due to frame collisions.
- Auto-negotiation mode supporting same speed and duplexity.  The
  auto-negotiating partner always switches to half-duplex if FLPs are not
  being received from the other link partner.

### Full-Duplex Mode {#enetphy_link_manual_full_duplex}

When either link partner is in manual mode with duplexity set to full-duplex,
the other link partner should be configured in any of the following modes:

- Manual mode with the exact same speed and duplexity.  Otherwise:
   + Speed mismatch can cause link not to be established at all.
   + Duplexity mismatch can result in an established link, but lower performance
     due to frame collisions.
- Auto-negotiation mode is not recommended because link may be established if
  both devices are at the same speed but they will end with duplexity mismatch
  as the auto-negotiating device always switches to half-duplex if no FLPs are
  received from the other link partner.  This results in lower performance due
  to frame collisions.


## Auto-Negotiation Mode {#enetphy_link_autoneg}

When either link is in auto-negotiation mode, then other link partner can be
configured also in auto-negotiation mode or in manual mode following the
recommendations outlined in @ref enetphy_link_manual section.

Auto-negotiation mode should always succeed as long as the two link partners
have common capabilities.  The negotiated speed and duplexity is the highest
supported mode supported by both link partners.

It's worth mentioning that, as per standard, auto-negotiation is the only mode
to be used for 1 Gbps.  However, it's also possible to use manual mode if
both link partners are configured to the same speed and duplexity.


## Strapping {#enetphy_link_strapping}

PHY strapping allows certain settings to be configured at start-up time, for
instance, auto-negotiation enable/disable, advertise capabilities, auto-MDIX,
etc.

The Ethernet PHY abstraction layer supports PHYs which are strapped to enable
auto-negotiation and whose advertise abilities are also set via strap pins.
This mode is enabled when #EnetPhy_Cfg.isStrapped configuration parameter is
set to true.

In this mode, the driver follows a shorter path in its state machine. PHY reset
and auto-negotiation configuration related states are bypassed, directly jumping
to wait for link up.
