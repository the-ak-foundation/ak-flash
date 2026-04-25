/**
 ******************************************************************************
 * @author: GaoKong
 * @date:   15/09/2017
 ******************************************************************************
**/
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "firmware.h"
#include "app_dbg.h"

int firmware_get_info(firmware_header_t* fh, const char* bin_file_path) {
	int index;
	uint32_t temp_data;
	uint32_t check_sum = 0;
	struct stat file_info;
	int binary_file = -1;

	binary_file = open(bin_file_path, O_RDONLY);
	if (binary_file < 0) {
		APP_DBG(DEBUG_LEVEL_ERROR, "open firmware failed: path=%s errno=%d (%s)\n", bin_file_path, errno, strerror(errno));
		return -1;
	}

	if (fstat(binary_file, &file_info) < 0) {
		APP_DBG(DEBUG_LEVEL_ERROR, "fstat firmware failed: path=%s errno=%d (%s)\n", bin_file_path, errno, strerror(errno));
		close(binary_file);
		return -1;
	}

	for (index = 0; index < file_info.st_size; index += sizeof(uint32_t)) {
		temp_data = 0;
		if (pread(binary_file, &temp_data, sizeof(uint32_t), index) < 0) {
			APP_DBG(DEBUG_LEVEL_ERROR, "pread firmware failed: path=%s offset=%d errno=%d (%s)\n", bin_file_path, index, errno, strerror(errno));
			close(binary_file);
			return -1;
		}
		check_sum += temp_data;
	}

	close(binary_file);

	fh->bin_len = file_info.st_size;
	fh->checksum = (check_sum & 0xFFFF);

	return 0;
}

int firmware_read(uint8_t* data, uint32_t cursor, uint32_t size, const char* bin_file_path) {
	struct stat file_info;
	int binary_file = -1;

	binary_file = open(bin_file_path, O_RDONLY);
	if (binary_file < 0) {
		APP_DBG(DEBUG_LEVEL_ERROR, "open firmware failed: path=%s errno=%d (%s)\n", bin_file_path, errno, strerror(errno));
		return -1;
	}

	if (fstat(binary_file, &file_info) < 0) {
		APP_DBG(DEBUG_LEVEL_ERROR, "fstat firmware failed: path=%s errno=%d (%s)\n", bin_file_path, errno, strerror(errno));
		close(binary_file);
		return -1;
	}

	if ((cursor + size) > (uint32_t)file_info.st_size) {
		APP_DBG(DEBUG_LEVEL_ERROR, "firmware read out of range: cursor=%u size=%u file_size=%u\n", cursor, size, (uint32_t)file_info.st_size);
		close(binary_file);
		return -1;
	}

	if (pread(binary_file, data, size, cursor) < 0) {
		APP_DBG(DEBUG_LEVEL_ERROR, "pread firmware failed: path=%s cursor=%u size=%u errno=%d (%s)\n", bin_file_path, cursor, size, errno, strerror(errno));
		close(binary_file);
		return -1;
	}

	close(binary_file);

	return 0;
}
