/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include "adsp.h"
#define VENDOR "AKM"
#define CHIP_ID "AK09970"
#define IDX_180_X 0
#define IDX_180_Y 1
#define IDX_180_Z 2
#define IDX_90_X 3
#define IDX_90_Y 4
#define IDX_90_Z 5
#define IDX_0_X 6
#define IDX_0_Y 7
#define IDX_0_Z 8
#define SPEC_180_X_IDX 3
#define SPEC_90_X_IDX 4
#define SPEC_0_X_IDX 5
#define SPEC_180_Y_IDX 6
#define SPEC_90_Y_IDX 7
#define SPEC_0_Y_IDX 8
#define SPEC_180_Z_IDX 9
#define SPEC_90_Z_IDX 10
#define SPEC_0_Z_IDX 11
#define SPEC_MIN_IDX 0
#define SPEC_MAX_IDX 1
#define PASS 0
#define FAIL (-1)
#define DEGREE_180 180
#define DEGREE_90 90
#define DEGREE_0 0
#define FAC_CAL_DATA_NUM 9

#define DIGITAL_HALL_FACTORY_CAL_PATH "/efs/FactoryApp/digital_hall_factory_cal"

static int32_t fac_cal[9] = { 0, };
static int32_t spec[12][2] = {
	{170, 180}, {80, 110}, {0, 10}, //ref angle 180, 90, 0
	{-39640, 39640}, {-39640, 39640}, {-39640, 39640}, // X 180, 90, 0
	{-39640, 39640}, {-39640, 39640}, {-39640, 39640}, // Y 180, 90, 0
	{-39640, 39640}, {-39640, 39640}, {-39640, 39640} // Z 180, 90, 0
};

static int32_t test_spec[6][2] = {
	{-39640, 39640}, {-39640, 39640}, // X 50 160
	{-39640, 39640}, {-39640, 39640}, // Y 50 160
	{-39640, 39640}, {-39640, 39640} // Z 50 160
};

int32_t curr_angle;

static int check_spec(int degree)
{
	int ret = PASS;

	switch(degree) {
	case DEGREE_180:
		if (fac_cal[IDX_180_X] < spec[SPEC_180_X_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_180_X] > spec[SPEC_180_X_IDX][SPEC_MAX_IDX] ||
			fac_cal[IDX_180_Y] < spec[SPEC_180_Y_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_180_Y] > spec[SPEC_180_Y_IDX][SPEC_MAX_IDX] ||
			fac_cal[IDX_180_Z] < spec[SPEC_180_Z_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_180_Z] > spec[SPEC_180_Z_IDX][SPEC_MAX_IDX]) {
			ret = FAIL;
			pr_info("[FACTORY] %s - Spec out at 180 degree\n", __func__);
		}
		break;
	case DEGREE_90:
		if (fac_cal[IDX_90_X] < spec[SPEC_90_X_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_90_X] > spec[SPEC_90_X_IDX][SPEC_MAX_IDX] ||
			fac_cal[IDX_90_Y] < spec[SPEC_90_Y_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_90_Y] > spec[SPEC_90_Y_IDX][SPEC_MAX_IDX] ||
			fac_cal[IDX_90_Z] < spec[SPEC_90_Z_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_90_Z] > spec[SPEC_90_Z_IDX][SPEC_MAX_IDX]) {
			ret = FAIL;
			pr_info("[FACTORY] %s - Spec out at 90 degree\n", __func__);
		}
		break;
	case DEGREE_0:
		if (fac_cal[IDX_0_X] < spec[SPEC_0_X_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_0_X] > spec[SPEC_0_X_IDX][SPEC_MAX_IDX] ||
			fac_cal[IDX_0_Y] < spec[SPEC_0_Y_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_0_Y] > spec[SPEC_0_Y_IDX][SPEC_MAX_IDX] ||
			fac_cal[IDX_0_Z] < spec[SPEC_0_Z_IDX][SPEC_MIN_IDX] ||
			fac_cal[IDX_0_Z] > spec[SPEC_0_Z_IDX][SPEC_MAX_IDX]) {
			ret = FAIL;
			pr_info("[FACTORY] %s - Spec out at 0 degree\n", __func__);
		}
		break;
	default:
		pr_info("[FACTORY] %s - wrong degree\n", __func__);
		ret = FAIL;
		break;
	}

	return ret;
}

int set_digital_hall_cal_data(bool first_booting)
{
	struct file *factory_cal_filp = NULL;
	mm_segment_t old_fs;
	int flag, ret = 0;
	umode_t mode = 0;
	char *write_buf = kzalloc(FILE_BUF_LEN, GFP_KERNEL);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (first_booting) {
		flag = O_TRUNC | O_RDWR | O_CREAT;
		mode = 0600;
	} else {
		flag = O_RDWR;
		mode = 0660;
	}

	factory_cal_filp = filp_open(DIGITAL_HALL_FACTORY_CAL_PATH, flag, mode);

	if (IS_ERR(factory_cal_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(factory_cal_filp);
		pr_err("[FACTORY] %s: open fail accel_factory_cal:%d\n",
			__func__, ret);
		kfree(write_buf);
		return ret;
	}

	snprintf(write_buf, FILE_BUF_LEN, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
		fac_cal[0], fac_cal[1], fac_cal[2], fac_cal[3], fac_cal[4],
		fac_cal[5], fac_cal[6], fac_cal[7], fac_cal[8]);


	ret = vfs_write(factory_cal_filp, (char *)write_buf,
		FILE_BUF_LEN * sizeof(char), &factory_cal_filp->f_pos);

	if (ret < 0)
		pr_err("[FACTORY] %s: fd write:%d\n", __func__, ret);

	filp_close(factory_cal_filp, current->files);
	set_fs(old_fs);
	kfree(write_buf);

	if (!first_booting) {
		adsp_unicast(fac_cal, sizeof(fac_cal),
			MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_SET_CAL_DATA);
	}
	return ret;
}

void digital_hall_factory_init_work(void)
{
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	int i;
	char *read_buf = kzalloc(FILE_BUF_LEN * sizeof(char), GFP_KERNEL);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(DIGITAL_HALL_FACTORY_CAL_PATH, O_RDONLY, 0440);
	if (PTR_ERR(cal_filp) == -ENOENT || PTR_ERR(cal_filp) == -ENXIO) {
		pr_info("[FACTORY] %s : no digital_hall_factory_cal file\n",
			__func__);
		set_fs(old_fs);
		set_digital_hall_cal_data(true);
	} else if (IS_ERR(cal_filp)) {
		pr_err("[FACTORY]: %s - filp_open error\n", __func__);
		set_fs(old_fs);
	} else {
		pr_info("[FACTORY] %s : already exist\n", __func__);

		ret = vfs_read(cal_filp, (char *)read_buf,
			FILE_BUF_LEN * sizeof(char), &cal_filp->f_pos);
		if (ret < 0) {
			pr_err("[FACTORY] %s: fd read fail:%d\n", __func__, ret);
			for (i = 0; i < FAC_CAL_DATA_NUM; i++)
				fac_cal[i] = 0;
			filp_close(cal_filp, current->files);
			set_fs(old_fs);
			kfree(read_buf);
			return;
		}

		ret = sscanf(read_buf, "%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d",
			&fac_cal[0], &fac_cal[1], &fac_cal[2], &fac_cal[3],
			&fac_cal[4], &fac_cal[5], &fac_cal[6], &fac_cal[7],
			&fac_cal[8]);

		if (ret != FAC_CAL_DATA_NUM) {
			pr_err("[FACTORY] %s: sscanf fail:%d\n", __func__, ret);
			filp_close(cal_filp, current->files);
			set_fs(old_fs);
			kfree(read_buf);
			return;
		}

		pr_info("[FACTORY] %s : %d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			__func__, fac_cal[0], fac_cal[1], fac_cal[2],
			fac_cal[3], fac_cal[4], fac_cal[5],
			fac_cal[6], fac_cal[7], fac_cal[8]);

		filp_close(cal_filp, current->files);
		set_fs(old_fs);
	}
	kfree(read_buf);

	adsp_unicast(fac_cal, sizeof(fac_cal),
		MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_SET_CAL_DATA);
}

static ssize_t digital_hall_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t digital_hall_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

static ssize_t digital_hall_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adsp_data *data = dev_get_drvdata(dev);
	uint8_t cnt = 0;

	pr_info("[FACTORY] %s - start", __func__);
	adsp_unicast(NULL, 0, MSG_DIGITAL_HALL, 0, MSG_TYPE_ST_SHOW_DATA);

	while (!(data->ready_flag[MSG_TYPE_ST_SHOW_DATA] & 1 << MSG_DIGITAL_HALL) &&
		cnt++ < TIMEOUT_CNT)
		msleep(20);

	data->ready_flag[MSG_TYPE_ST_SHOW_DATA] &= ~(1 << MSG_DIGITAL_HALL);

	if (cnt >= TIMEOUT_CNT) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
                return snprintf(buf, PAGE_SIZE, "-1,0,0,0,0,0,0,0,0,0,0\n");
	}

	pr_info("[FACTORY] SW_RST: %d, ST_RES: %d, min: %d/%d/%d, max: %d/%d/%d, avg: %d/%d/%d\n",
		data->msg_buf[MSG_DIGITAL_HALL][0], data->msg_buf[MSG_DIGITAL_HALL][1],
		data->msg_buf[MSG_DIGITAL_HALL][2], data->msg_buf[MSG_DIGITAL_HALL][3],
		data->msg_buf[MSG_DIGITAL_HALL][4], data->msg_buf[MSG_DIGITAL_HALL][5],
		data->msg_buf[MSG_DIGITAL_HALL][6], data->msg_buf[MSG_DIGITAL_HALL][7],
		data->msg_buf[MSG_DIGITAL_HALL][8], data->msg_buf[MSG_DIGITAL_HALL][9],
		data->msg_buf[MSG_DIGITAL_HALL][10]);

	return snprintf(buf, PAGE_SIZE,	"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		data->msg_buf[MSG_DIGITAL_HALL][0], data->msg_buf[MSG_DIGITAL_HALL][1],
		data->msg_buf[MSG_DIGITAL_HALL][2], data->msg_buf[MSG_DIGITAL_HALL][3],
		data->msg_buf[MSG_DIGITAL_HALL][4], data->msg_buf[MSG_DIGITAL_HALL][5],
		data->msg_buf[MSG_DIGITAL_HALL][6], data->msg_buf[MSG_DIGITAL_HALL][7],
		data->msg_buf[MSG_DIGITAL_HALL][8], data->msg_buf[MSG_DIGITAL_HALL][9],
		data->msg_buf[MSG_DIGITAL_HALL][10]);
}

static ssize_t digital_hall_spec_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		spec[0][0], spec[0][1], spec[1][0], spec[1][1], spec[2][0], spec[2][1],
		spec[3][0], spec[3][1], spec[4][0], spec[4][1], spec[5][0], spec[5][1],
		spec[6][0], spec[6][1], spec[7][0], spec[7][1], spec[8][0], spec[8][1],
		spec[9][0], spec[9][1], spec[10][0], spec[10][1], spec[11][0], spec[11][1]);
}

static ssize_t digital_hall_test_spec_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		spec[0][0], spec[0][1], spec[1][0], spec[1][1], spec[2][0], spec[2][1],
		spec[3][0], spec[3][1], spec[4][0], spec[4][1], spec[5][0], spec[5][1]);
}

static ssize_t digital_hall_ref_angle_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adsp_data *data = dev_get_drvdata(dev);
	uint8_t cnt = 0;
	int32_t min, max, result;

	min = curr_angle - 10;
	max = curr_angle + 10;
	result = PASS;

	adsp_unicast(NULL, 0, MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_GET_RAW_DATA);

	while (!(data->ready_flag[MSG_TYPE_GET_RAW_DATA] & 1 << MSG_DIGITAL_HALL_ANGLE) &&
		cnt++ < TIMEOUT_CNT)
		msleep(20);

	data->ready_flag[MSG_TYPE_GET_RAW_DATA] &= ~(1 << MSG_DIGITAL_HALL_ANGLE);

	if (cnt >= TIMEOUT_CNT) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
                return snprintf(buf, PAGE_SIZE, "-1\n");
	}

	pr_info("[FACTORY] %s - st %d/%d, akm %d/%d\n", __func__,
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][1],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][3]);

	if (data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0] < min ||
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0] > max)
		result = FAIL;
		
	return snprintf(buf, PAGE_SIZE,	"%d,%d\n",
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0], result);
}

static ssize_t digital_hall_read_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adsp_data *data = dev_get_drvdata(dev);
	uint8_t cnt = 0;

	adsp_unicast(NULL, 0, MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_GET_RAW_DATA);

	while (!(data->ready_flag[MSG_TYPE_GET_RAW_DATA] & 1 << MSG_DIGITAL_HALL_ANGLE) &&
		cnt++ < TIMEOUT_CNT)
		msleep(20);

	data->ready_flag[MSG_TYPE_GET_RAW_DATA] &= ~(1 << MSG_DIGITAL_HALL_ANGLE);

	if (cnt >= TIMEOUT_CNT) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
                return snprintf(buf, PAGE_SIZE, "-1\n");
	}

	pr_info("[FACTORY] %s - st %d/%d, akm %d/%d, lf %d/%d, hall %d/%d/%d(uT)\n",
		__func__, data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][1],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][3],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][4],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][5],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][6],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][7],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][8]);

		
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][1],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][3],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][4],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][5],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][6],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][7],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][8]);
}

static ssize_t digital_hall_test_read_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adsp_data *data = dev_get_drvdata(dev);
	uint8_t cnt = 0;
	int data_buf[3] = { 0, };
	int result = PASS;

	adsp_unicast(NULL, 0, MSG_DIGITAL_HALL, 0, MSG_TYPE_SET_CAL_DATA);

	while (!(data->ready_flag[MSG_TYPE_SET_CAL_DATA] & 1 << MSG_DIGITAL_HALL) &&
		cnt++ < TIMEOUT_CNT)
		msleep(20);

	data->ready_flag[MSG_TYPE_SET_CAL_DATA] &= ~(1 << MSG_DIGITAL_HALL);

	if (cnt >= TIMEOUT_CNT) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
                return snprintf(buf, PAGE_SIZE, "-1\n");
	}

	pr_info("[FACTORY] %s - digital_hal_data %d/%d/%d\n", __func__,
		data->msg_buf[MSG_DIGITAL_HALL][0],
		data->msg_buf[MSG_DIGITAL_HALL][1],
		data->msg_buf[MSG_DIGITAL_HALL][2]);

	data_buf[0] = data->msg_buf[MSG_DIGITAL_HALL][0];
	data_buf[1] = data->msg_buf[MSG_DIGITAL_HALL][1];
	data_buf[2] = data->msg_buf[MSG_DIGITAL_HALL][2];

	adsp_unicast(NULL, 0, MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_GET_RAW_DATA);

	while (!(data->ready_flag[MSG_TYPE_GET_RAW_DATA] & 1 << MSG_DIGITAL_HALL_ANGLE) &&
		cnt++ < TIMEOUT_CNT)
		msleep(20);

	data->ready_flag[MSG_TYPE_GET_RAW_DATA] &= ~(1 << MSG_DIGITAL_HALL_ANGLE);

	if (cnt >= TIMEOUT_CNT) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
                return snprintf(buf, PAGE_SIZE, "-1,-1,-1,-1,-1,-1\n");
	}

	pr_info("[FACTORY] %s - st %d/%d, akm %d/%d\n", __func__,
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][1],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][3]);

	if (data_buf[0] > test_spec[0][1] ||
		data_buf[0] < test_spec[0][0] ||
		data_buf[1] > test_spec[1][1] ||
		data_buf[1] < test_spec[1][0] ||
		data_buf[2] > test_spec[2][1] ||
		data_buf[2] < test_spec[2][0])
		result = FAIL;

	return snprintf(buf, PAGE_SIZE,	"%d,%d,%d,%d,%d,%d\n",
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
		data_buf[0], data_buf[1], data_buf[2],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2],
		result);
}

static ssize_t digital_hall_test_read_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (sysfs_streq(buf, "50")) {
		curr_angle = 50;
	} else if (sysfs_streq(buf, "160")) {
		curr_angle = 160;
	} else {
		pr_err("[FACTORY] %s - wrong degree !!!\n", __func__);
		return size;
	}

	pr_info("[FACTORY] %s - Test read at degree %d\n",
		__func__, curr_angle);

	return size;
}

static ssize_t digital_hall_fac_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adsp_data *data = dev_get_drvdata(dev);
	uint8_t cnt = 0;
	int ret = FAIL;

	pr_info("[FACTORY] %s\n", __func__);

	adsp_unicast(NULL, 0, MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_GET_RAW_DATA);

	while (!(data->ready_flag[MSG_TYPE_GET_RAW_DATA] & 1 << MSG_DIGITAL_HALL_ANGLE) &&
		cnt++ < TIMEOUT_CNT)
		msleep(20);

	data->ready_flag[MSG_TYPE_GET_RAW_DATA] &= ~(1 << MSG_DIGITAL_HALL_ANGLE);

	if (cnt >= TIMEOUT_CNT) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
                return snprintf(buf, PAGE_SIZE, "-1,-1,-1,-1,-1,-1\n");
	}

	pr_info("[FACTORY] %s - st %d/%d, akm %d/%d\n", __func__,
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][1],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2],
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][3]);

	ret = check_spec(curr_angle);
	pr_info("[FACTORY] %s - check_spec return %d\n", __func__, ret);

	pr_info("[FACTORY] %s - 180: %d/%d/%d, 90: %d/%d/%d, 0: %d/%d/%d\n", __func__,
		fac_cal[IDX_180_X], fac_cal[IDX_180_Y], fac_cal[IDX_180_Z],
		fac_cal[IDX_90_X], fac_cal[IDX_90_Y], fac_cal[IDX_90_Z],
		fac_cal[IDX_0_X], fac_cal[IDX_0_Y], fac_cal[IDX_0_Z]);

	switch(curr_angle) {
	case DEGREE_180:
		return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d\n",
			data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
			fac_cal[IDX_180_X], fac_cal[IDX_180_Y], fac_cal[IDX_180_Z],
			data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2], ret);
	case DEGREE_90:
		return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d\n",
			data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
			fac_cal[IDX_90_X], fac_cal[IDX_90_Y], fac_cal[IDX_90_Z],
			data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2], ret);
	case DEGREE_0:
		return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d\n",
			data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0],
			fac_cal[IDX_0_X], fac_cal[IDX_0_Y], fac_cal[IDX_0_Z],
			data->msg_buf[MSG_DIGITAL_HALL_ANGLE][2], ret);
	}

	return snprintf(buf, PAGE_SIZE, "-1,-1,-1,-1,-1,-1\n");
}

static ssize_t digital_hall_factory_cal_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct adsp_data *data = dev_get_drvdata(dev);
	uint8_t cnt = 0;

	if (sysfs_streq(buf, "0")) {
		curr_angle = 0;
	} else if (sysfs_streq(buf, "90")) {
		curr_angle = 90;
	} else if (sysfs_streq(buf, "180")) {
		curr_angle = 180;
	} else {
		pr_err("[FACTORY] %s - wrong degree !!!\n", __func__);
		return size;
	}

	pr_info("[FACTORY] %s - Calibration at degree %d\n",
		__func__, curr_angle);

	adsp_unicast(&curr_angle, sizeof(int32_t), MSG_DIGITAL_HALL,
		0, MSG_TYPE_SET_CAL_DATA);

	while (!(data->ready_flag[MSG_TYPE_SET_CAL_DATA] & 1 << MSG_DIGITAL_HALL) &&
		cnt++ < TIMEOUT_CNT)
		msleep(20);

	data->ready_flag[MSG_TYPE_SET_CAL_DATA] &= ~(1 << MSG_DIGITAL_HALL);

	if (cnt >= TIMEOUT_CNT) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
		return size;
	}

	pr_info("[FACTORY] %s - [%d] %d, %d, %d\n", __func__, curr_angle,
		data->msg_buf[MSG_DIGITAL_HALL][0],
		data->msg_buf[MSG_DIGITAL_HALL][1],
		data->msg_buf[MSG_DIGITAL_HALL][2]);

	if (curr_angle == 180) {
		fac_cal[IDX_180_X] = data->msg_buf[MSG_DIGITAL_HALL][0];
		fac_cal[IDX_180_Y] = data->msg_buf[MSG_DIGITAL_HALL][1];
		fac_cal[IDX_180_Z] = data->msg_buf[MSG_DIGITAL_HALL][2];
	} else if (curr_angle == 90) {
		fac_cal[IDX_90_X] = data->msg_buf[MSG_DIGITAL_HALL][0];
		fac_cal[IDX_90_Y] = data->msg_buf[MSG_DIGITAL_HALL][1];
		fac_cal[IDX_90_Z] = data->msg_buf[MSG_DIGITAL_HALL][2];
	} else if (curr_angle == 0) {
		fac_cal[IDX_0_X] = data->msg_buf[MSG_DIGITAL_HALL][0];
		fac_cal[IDX_0_Y] = data->msg_buf[MSG_DIGITAL_HALL][1];
		fac_cal[IDX_0_Z] = data->msg_buf[MSG_DIGITAL_HALL][2];
	}

	if (fac_cal[IDX_180_X] != 0 && fac_cal[IDX_180_Y] != 0 &&
		fac_cal[IDX_180_Z] != 0 && fac_cal[IDX_90_X] != 0 &&
		fac_cal[IDX_90_Y] != 0 && fac_cal[IDX_90_Z] != 0 &&
		fac_cal[IDX_0_X] != 0 && fac_cal[IDX_0_Y] != 0 &&
		fac_cal[IDX_0_Z] != 0)
		set_digital_hall_cal_data(false);

	return size;
}

static ssize_t reset_auto_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adsp_data *data = dev_get_drvdata(dev);
	int reset_data[58] = { 0, };
	uint8_t cnt = 0;

	/* reset */
	adsp_unicast(reset_data, sizeof(reset_data),
		MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_SET_REGISTER);

	/* read */
	adsp_unicast(NULL, 0, MSG_DIGITAL_HALL_ANGLE, 0, MSG_TYPE_GET_CAL_DATA);

	while (!(data->ready_flag[MSG_TYPE_GET_CAL_DATA] & 1 << MSG_DIGITAL_HALL_ANGLE) &&
		cnt++ < 3)
		msleep(30);

	data->ready_flag[MSG_TYPE_GET_CAL_DATA] &= ~(1 << MSG_DIGITAL_HALL_ANGLE);

	if (cnt >= 3) {
		pr_err("[FACTORY] %s: Timeout!!!\n", __func__);
		return snprintf(buf, PAGE_SIZE, "-1\n");
	}

	pr_info("[FACTORY] %s: flg_update=%d\n", __func__, data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0]);

		
	return snprintf(buf, PAGE_SIZE,	"%d\n",
		data->msg_buf[MSG_DIGITAL_HALL_ANGLE][0]);
}

static DEVICE_ATTR(name, 0444, digital_hall_name_show, NULL);
static DEVICE_ATTR(vendor, 0444, digital_hall_vendor_show, NULL);
static DEVICE_ATTR(selftest, 0440, digital_hall_selftest_show, NULL);
static DEVICE_ATTR(spec, 0440, digital_hall_spec_show, NULL);
static DEVICE_ATTR(test_spec, 0440, digital_hall_test_spec_show, NULL);
static DEVICE_ATTR(ref_angle, 0440, digital_hall_ref_angle_show, NULL);
static DEVICE_ATTR(read_data, 0440, digital_hall_read_data_show, NULL);
static DEVICE_ATTR(test_read, 0660,
	digital_hall_test_read_show, digital_hall_test_read_store);
static DEVICE_ATTR(fac_cal, 0660,
	digital_hall_fac_cal_show, digital_hall_factory_cal_store);
static DEVICE_ATTR(reset_auto_cal, 0440, reset_auto_cal_show, NULL);

static struct device_attribute *digital_hall_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_selftest,
	&dev_attr_spec,
	&dev_attr_test_spec,
	&dev_attr_ref_angle,
	&dev_attr_read_data,
	&dev_attr_test_read,
	&dev_attr_fac_cal,
	&dev_attr_reset_auto_cal,
	NULL,
};

static int __init ak09970_factory_init(void)
{
	adsp_factory_register(MSG_DIGITAL_HALL, digital_hall_attrs);

	pr_info("[FACTORY] %s\n", __func__);

	return 0;
}

static void __exit ak09970_factory_exit(void)
{
	adsp_factory_unregister(MSG_DIGITAL_HALL);

	pr_info("[FACTORY] %s\n", __func__);
}

module_init(ak09970_factory_init);
module_exit(ak09970_factory_exit);
