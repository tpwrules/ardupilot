# Small Fast Drone Project

The canonical ArduPilot README can be found [here](https://github.com/ArduPilot/ardupilot/blob/master/README.md)

[![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_copter.yml)

![SFD](sfd_logo2.png)

ArduPilot is the most advanced, full-featured, and reliable open source autopilot software available.
It has been under development since 2010 by a diverse team of professional engineers, computer scientists, and community contributors.
The autopilot software is capable of controlling almost any vehicle system imaginable, from conventional airplanes, quad planes, multi-rotors, and helicopters to rovers, boats, balance bots, and even submarines. It is continually being expanded to provide support for new emerging vehicle types.

However, the need to continously support all vehicle types constrains both flash usage and feature velocity. That might be fine if you are flying a 2 ton, $100k vehicle and absolute reliability is paramount - but for smaller vehicles these constraints can be prohibitive to forward progress - and yet at the same time you also don't want to be flying the master branch the whole time. For a while I have be maintaining branches off the latest stable branch of ArduPilot that also contain features - usually features that I have developed - that are either only available in master or as PRs. These features are particularly geared to the needs to smaller, faster copters - but are also can be applicable to any size of vehicle. Maintaining these
branches has become somewhat onerous, so I have instead started this new repo giving me greater flexibility in how I managed progress. The intent is:

- To be a derivative of the latest stable ArduPilot release (This branch is for 4.6.x)
- For all included features to be open source and eventually be available in either the main ArduPilot repository or one of the fossuav repositories
- For all additional features to have been flown
- For all additional features to be documented

## Included Features ##

The branch is based on ArduPilot 4.6.3. It also includes the following PRs and features:

- Fast rates (https://github.com/ArduPilot/ardupilot/pull/27029, https://github.com/ArduPilot/ardupilot/pull/27839, https://github.com/ArduPilot/ardupilot/pull/27841, https://github.com/ArduPilot/ardupilot/pull/29112, https://github.com/ArduPilot/ardupilot/pull/29749, https://github.com/ArduPilot/ardupilot/pull/30155, https://github.com/ArduPilot/ardupilot/pull/31395, https://github.com/ArduPilot/ardupilot/pull/27839, https://github.com/ArduPilot/ardupilot/pull/27842, https://github.com/ArduPilot/ardupilot/pull/27996, https://github.com/ArduPilot/ardupilot/pull/28984, https://github.com/ArduPilot/ardupilot/pull/30980)
[![Fast rates](https://img.youtube.com/vi/B8Dp2jwDamU/0.jpg)](https://www.youtube.com/playlist?list=PL_O9QDs-WAVyBpf7URQQgCmNQwv_aTcMf)
- Littlefs (https://github.com/ArduPilot/ardupilot/pull/28724, https://github.com/ArduPilot/ardupilot/pull/31227, https://github.com/ArduPilot/ardupilot/pull/29120, https://github.com/ArduPilot/ardupilot/pull/29295, https://github.com/ArduPilot/ardupilot/pull/29307, https://github.com/ArduPilot/ardupilot/pull/29308, https://github.com/ArduPilot/ardupilot/pull/29353, https://github.com/ArduPilot/ardupilot/pull/29354, https://github.com/ArduPilot/ardupilot/pull/29413, https://github.com/ArduPilot/ardupilot/pull/29472, https://github.com/ArduPilot/ardupilot/pull/31236, https://github.com/ArduPilot/ardupilot/pull/31244)
- Fence margins (https://github.com/ArduPilot/ardupilot/pull/28840, https://github.com/ArduPilot/ardupilot/pull/29744, https://github.com/ArduPilot/ardupilot/pull/30238, https://github.com/ArduPilot/ardupilot/pull/30240, https://github.com/ArduPilot/ardupilot/pull/31009, https://github.com/ArduPilot/ardupilot/pull/31012, https://github.com/ArduPilot/ardupilot/pull/31018, https://github.com/ArduPilot/ardupilot/pull/2581, https://github.com/ArduPilot/ardupilot/pull/31005)
[![Fence margins](https://img.youtube.com/vi/w21SylNejas/0.jpg)](www.youtube.com/watch?v=w21SylNejas)
- Dshot Cancel Failure (https://github.com/ArduPilot/ardupilot/pull/29409, a887bca91c85f5ae9a2e41a051e978fd917e2f2d)
- Harmonic Notches (https://github.com/ArduPilot/ardupilot/pull/30246, https://github.com/ArduPilot/ardupilot/pull/30994)
- Rate Acro (https://github.com/ArduPilot/ardupilot/pull/31171, https://github.com/ArduPilot/ardupilot/pull/31226)
- Flying Indoors (https://github.com/ArduPilot/ardupilot/pull/31155, https://github.com/ArduPilot/ardupilot/pull/31177, https://github.com/ArduPilot/ardupilot/pull/30490, https://github.com/ArduPilot/ardupilot/pull/29847)
- CRSF Scripted Menus (https://github.com/ArduPilot/ardupilot/pull/29368, https://github.com/ArduPilot/ardupilot/pull/31314)
[![CRSF Scripted Menus](https://img.youtube.com/vi/x5lsXGpmC9g/0.jpg)](www.youtube.com/watch?v=x5lsXGpmC9g)
- Motortest Errors (https://github.com/ArduPilot/ardupilot/pull/31274)
- CRSF Binding (https://github.com/ArduPilot/ardupilot/pull/31396)
- Idle-hook CPU Monitoring (https://github.com/ArduPilot/ardupilot/pull/30913)
- VTX Max Power (https://github.com/ArduPilot/ardupilot/pull/31500)
- Logging (https://github.com/ArduPilot/ardupilot/pull/30841, https://github.com/ArduPilot/ardupilot/pull/30842)
- CRSF Menus (https://github.com/fossuav/smallfastdrone)

[![CRSF Menus](https://img.youtube.com/vi/u8P8tj-KxXs/0.jpg)](www.youtube.com/watch?v=u8P8tj-KxXs)


## SmallFastDronev1 Target ##

There is a hardware target called SmallFastDronev1 that is designed to work optimally with this fork. The hardware itself is actually the TBS_LUCID_H7 v2, so if you get one of these flight controllers you can flash it with the target if you choose.

## The ArduPilot project is made up of: ##

- ArduCopter: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](https://ardupilot.org/copter/index.html)

## Developer Information ##

- Github repository: <https://github.com/fossuav/smallfastdrone>

## License ##

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)
