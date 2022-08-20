/* SPDX-License-Identifier: GPL-2.0 */
// Author: @AkiraNoSushi
// Adapted to k4.19 by Jprimero15

#ifndef _LINUX_SDM439_DEVICE_H
#define _LINUX_SDM439_DEVICE_H

enum SDM439_devices {
    DEVICE_UNKNOWN = -1,
    XIAOMI_PINE,
    XIAOMI_OLIVES
};

extern enum SDM439_devices sdm439_devices;

extern int sdm439_current_device;

#endif
