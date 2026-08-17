/*
 * Copyright (c) 2021 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DT_FREQ_H
#define __DT_FREQ_H

#define DT_FREQ_K(x) ((x) * 1000)
#define DT_FREQ_M(x) ((x) * 1000 * 1000)

/*
 * FR_PINMUX pack: port (A=0..D=3), pin (0..15), func (0..15) into uint32_t
 *   bits [15:12] = port index (0-3)
 *   bits [11:4]  = pin number
 *   bits [3:0]   = alt function number
 */
#define FR_PINMUX(port, pin, func) \
	(((((port) - 'A') & 0x3) << 12) | (((pin) & 0xff) << 4) | ((func) & 0xf))

#endif /* __DT_FREQ_H */
