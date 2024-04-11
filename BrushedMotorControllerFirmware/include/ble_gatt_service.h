/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 Maciej Baczmanski, Michal Kawiak, Jakub Mazur
 *
 * Copyright (c) 2015-2016 Intel Corporation
 *
 */

#if defined(CONFIG_BT_SUPPORT)
#define SERVICE_UUID    BT_UUID_128_ENCODE(0x491ee7f9, 0xb307, 0x4a19, 0x8932, 0x2a8bb514ac46)

int init_bt(void);
#endif
