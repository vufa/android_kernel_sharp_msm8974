/*
 * Copyright (C) 2011 Sharp.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* -------------------------------------------------------------------- */
#ifndef SH_KMOD_SHA256_HASH__
#define SH_KMOD_SHA256_HASH__

static const unsigned char kmod_sha256_hash[][32] = {
#include "shexfat.hash"
#include "adsp-loader.hash"
#include "shvmware.hash"
};

#endif /* SH_KMOD_SHA256_HASH__ */
