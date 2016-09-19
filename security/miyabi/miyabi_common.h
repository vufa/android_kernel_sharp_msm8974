/*
 * Copyright (C) 2013 Sharp.
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
#define LOCAL_MIYABI_PATH_APPPROCESS	"/system/bin/app_process"
#define LOCAL_MIYABI_SUFFIX_APK		".apk"
#define LOCAL_MIYABI_DIR_SYSTEM		"/system/"
#define LOCAL_MIYABI_DIR_FRAMEWORK		"/system/framework/"
#define LOCAL_MIYABI_DIR_SYSTEMBIN		"/system/bin/"
#define LOCAL_MIYABI_DIR_SYSTEMSHBIN	"/system/shbin/"
#define LOCAL_MIYABI_DIR_VENDORBIN		"/system/vendor/bin/"
#define LOCAL_MIYABI_DIR_SYSTEMAPP		"/system/app/"
#define LOCAL_MIYABI_DIR_VENDORAPP		"/system/vendor/app/"
#define LOCAL_MIYABI_DIR_SYSTEMLIB		"/system/lib/"
#define LOCAL_MIYABI_DIR_VENDORLIB		"/system/vendor/lib/"
#define LOCAL_MIYABI_DIR_DATADATA		"/data/data/"
#define LOCAL_MIYABI_DIR_DATAAPP		"/data/app/"
#define LOCAL_MIYABI_DIR_DATAAPPPRIVATE	"/data/app-private/"
/* -------------------------------------------------------------------- */
char* miyabi_detect_binary(struct task_struct* t, char* pathbuf);
char* miyabi_detect_package(struct task_struct* t, char* buffer);

