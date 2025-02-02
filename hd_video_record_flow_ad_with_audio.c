/**
	@brief Sample code of video record from proc and encode to bs.\n

	@file video_record_flow.c

	@author Boyan Huang

	@ingroup mhdal

	@note This file is modified from video_liveview.c and video_record.c.

	Copyright Novatek Microelectronics Corp. 2018.  All rights reserved.
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "hdal.h"
#include "hd_debug.h"
#include "vendor_videocapture.h"

// platform dependent
#if defined(__LINUX)
#include <pthread.h>			//for pthread API
//#include <kwrap/util.h>		//for sleep API
//#define msleep(x)    			vos_util_delay_ms(x)
//#define usleep(x)   			vos_util_delay_us(x)
#define MAIN(argc, argv) 		int main(int argc, char** argv)
#define GETCHAR()				getchar()
#else
#include <FreeRTOS_POSIX.h>
#include <FreeRTOS_POSIX/pthread.h> //for pthread API
#include <kwrap/util.h>		//for sleep API
#define sleep(x)    			vos_util_delay_ms(1000*(x))
#define msleep(x)    			vos_util_delay_ms(x)
#define usleep(x)   			vos_util_delay_us(x)
#include <kwrap/examsys.h> 	//for MAIN() API
#define MAIN(argc, argv) 		EXAMFUNC_ENTRY(hd_video_record_flow, argc, argv)
#define GETCHAR()				NVT_EXAMSYS_GETCHAR()
#endif

#define DEBUG_MENU 		1

#define AUDIO_OUT_ENABLE
#define AD_ENABLE 1 

#define CHKPNT			printf("\033[37mCHK: %s, %s: %d\033[0m\r\n",__FILE__,__func__,__LINE__)
#define DBGH(x)			printf("\033[0;35m%s=0x%08X\033[0m\r\n", #x, x)
#define DBGD(x)			printf("\033[0;35m%s=%d\033[0m\r\n", #x, x)


#define MOVIE_BRC_MODE      0 //1: Movie BRC mode , 0: fix quality mode
#ifdef AUDIO_OUT_ENABLE
#define BITSTREAM_SIZE      12800
#define FRAME_SAMPLES       1024
#define AUD_BUFFER_CNT      5
#endif
#define TIME_DIFF(new_val, old_val)     ((int)(new_val) - (int)(old_val))

///////////////////////////////////////////////////////////////////////////////

//header
#define DBGINFO_BUFSIZE()	(0x200)

//RAW
#define VDO_RAW_BUFSIZE(w, h, pxlfmt)   (ALIGN_CEIL_4((w) * HD_VIDEO_PXLFMT_BPP(pxlfmt) / 8) * (h))
//NRX: RAW compress: Only support 12bit mode
#define RAW_COMPRESS_RATIO 59
#define VDO_NRX_BUFSIZE(w, h)           (ALIGN_CEIL_4(ALIGN_CEIL_64(w) * 12 / 8 * RAW_COMPRESS_RATIO / 100 * (h)))
//CA for AWB
#define VDO_CA_BUF_SIZE(win_num_w, win_num_h) ALIGN_CEIL_4((win_num_w * win_num_h << 3) << 1)
//LA for AE
#define VDO_LA_BUF_SIZE(win_num_w, win_num_h) ALIGN_CEIL_4((win_num_w * win_num_h << 1) << 1)

//YUV
#define VDO_YUV_BUFSIZE(w, h, pxlfmt)	(ALIGN_CEIL_4((w) * HD_VIDEO_PXLFMT_BPP(pxlfmt) / 8) * (h))
//NVX: YUV compress
#define YUV_COMPRESS_RATIO 75
#define VDO_NVX_BUFSIZE(w, h, pxlfmt)	(VDO_YUV_BUFSIZE(w, h, pxlfmt) * YUV_COMPRESS_RATIO / 100)

///////////////////////////////////////////////////////////////////////////////

#ifdef AD_ENABLE  // YUV 
#define SEN_OUT_FMT		HD_VIDEO_PXLFMT_YUV422
#define CAP_OUT_FMT		HD_VIDEO_PXLFMT_YUV420
#define SHDR_CAP_OUT_FMT	HD_VIDEO_PXLFMT_NRX12_SHDR2
#define PRC_OUT_FMT		HD_VIDEO_PXLFMT_YUV420_NVX2
#else // RAW INPUT 
#define SEN_OUT_FMT		HD_VIDEO_PXLFMT_RAW12
#define CAP_OUT_FMT		HD_VIDEO_PXLFMT_RAW12
#define SHDR_CAP_OUT_FMT	HD_VIDEO_PXLFMT_NRX12_SHDR2
#define PRC_OUT_FMT		HD_VIDEO_PXLFMT_YUV420_NVX2
#endif 
#define CA_WIN_NUM_W		32
#define CA_WIN_NUM_H		32
#define LA_WIN_NUM_W		32
#define LA_WIN_NUM_H		32
#define VA_WIN_NUM_W		16
#define VA_WIN_NUM_H		16
#define YOUT_WIN_NUM_W	128
#define YOUT_WIN_NUM_H	128
#define ETH_8BIT_SEL		0 //0: 2bit out, 1:8 bit out
#define ETH_OUT_SEL		1 //0: full, 1: subsample 1/2

#define VDO_SIZE_W      	1920
#define VDO_SIZE_H      	1080
#define REC_SIZE_W		1920
#define REC_SIZE_H		1080

#if MOVIE_BRC_MODE
#define REC_TBR			(2 * 1024 * 1024) //bits-per-sec
#else
#define REC_TBR			(((3840 * 2160 * 3/2) /5) * 8) //fix-quality 70, assume compress ratio = 1/5, for max resolution 3840x2160
#endif
#define REC_FIX_TBR(w,h)	((((w) * (h) * 3/2) /5) * 8) //fix-quality 70, assume compress ratio = 1/5, for max resolution w x h
#define REC_BUF			10000 //ms

#define VCAP_ALG_FUNC 	HD_VIDEOCAP_FUNC_AE | HD_VIDEOCAP_FUNC_AWB
#define VPRC_ALG_FUNC 	0

#define g_shdr_mode       0

///////////////////////////////////////////////////////////////////////////////


static HD_RESULT mem_init(void)
{
	HD_RESULT              ret;
	HD_COMMON_MEM_INIT_CONFIG mem_cfg = {0};

	// config common pool (cap)
	mem_cfg.pool_info[0].type = HD_COMMON_MEM_COMMON_POOL;
	mem_cfg.pool_info[0].blk_size = DBGINFO_BUFSIZE()+VDO_RAW_BUFSIZE(REC_SIZE_W, REC_SIZE_H,CAP_OUT_FMT)
        													+VDO_CA_BUF_SIZE(CA_WIN_NUM_W, CA_WIN_NUM_H)
        													+VDO_LA_BUF_SIZE(LA_WIN_NUM_W, LA_WIN_NUM_H);
	mem_cfg.pool_info[0].blk_cnt = 2;
	mem_cfg.pool_info[0].ddr_id = DDR_ID0;
	// config common pool (liveview)
	mem_cfg.pool_info[1].type = HD_COMMON_MEM_COMMON_POOL;
	mem_cfg.pool_info[1].blk_size = DBGINFO_BUFSIZE()+VDO_YUV_BUFSIZE(VDO_SIZE_W, VDO_SIZE_H, HD_VIDEO_PXLFMT_YUV420);
	mem_cfg.pool_info[1].blk_cnt = 3;
	mem_cfg.pool_info[1].ddr_id = DDR_ID0;
	// config common pool (record)
	mem_cfg.pool_info[2].type = HD_COMMON_MEM_COMMON_POOL;
	mem_cfg.pool_info[2].blk_size = DBGINFO_BUFSIZE()+VDO_YUV_BUFSIZE(REC_SIZE_W, REC_SIZE_H, HD_VIDEO_PXLFMT_YUV420);
	mem_cfg.pool_info[2].blk_cnt = 3;
	mem_cfg.pool_info[2].ddr_id = DDR_ID0;

#ifdef AUDIO_OUT_ENABLE
	/* user buffer for bs pushing in */
	mem_cfg.pool_info[3].type = HD_COMMON_MEM_USER_POOL_BEGIN;
	mem_cfg.pool_info[3].blk_size = 0x100000;
	mem_cfg.pool_info[3].blk_cnt = 1;
	mem_cfg.pool_info[3].ddr_id = DDR_ID0;	
#endif
	ret = hd_common_mem_init(&mem_cfg);
	return ret;
}

static HD_RESULT mem_exit(void)
{
	HD_RESULT ret = HD_OK;
	hd_common_mem_uninit();
	return ret;
}

///////////////////////////////////////////////////////////////////////////////

static HD_RESULT get_cap_caps(HD_PATH_ID video_cap_ctrl, HD_VIDEOCAP_SYSCAPS *p_video_cap_syscaps)
{
	HD_RESULT ret = HD_OK;
	hd_videocap_get(video_cap_ctrl, HD_VIDEOCAP_PARAM_SYSCAPS, p_video_cap_syscaps);
	return ret;
}
#ifndef AD_ENABLE
static HD_RESULT set_cap_cfg(HD_PATH_ID *p_video_cap_ctrl)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOCAP_DRV_CONFIG cap_cfg = {0};
	HD_PATH_ID video_cap_ctrl = 0;
	HD_VIDEOCAP_CTRL iq_ctl = {0};
	char *chip_name = getenv("NVT_CHIP_ID");

	snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_imx290");
	printf("Using nvt_sen_imx290\n");
	
    if(1){
	    cap_cfg.sen_cfg.sen_dev.if_type = HD_COMMON_VIDEO_IN_MIPI_CSI;
	    cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x220; //PIN_SENSOR_CFG_MIPI
	    printf("MIPI interface\n");
		printf("Using imx290\n");
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0xf01;//0xf01;//PIN_MIPI_LVDS_CFG_CLK2 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
	}

	if (chip_name != NULL && strcmp(chip_name, "CHIP_NA51089") == 0) {
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x01;//PIN_I2C_CFG_CH1, //56x
	} else {
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x10;//PIN_I2C_CFG_CH2 // 52x
	}

	cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
	ret = hd_videocap_open(0, HD_VIDEOCAP_0_CTRL, &video_cap_ctrl); //open this for device control

	if (ret != HD_OK) {
		return ret;
	}

	if (g_shdr_mode == 1) {
		cap_cfg.sen_cfg.shdr_map = HD_VIDEOCAP_SHDR_MAP(HD_VIDEOCAP_HDR_SENSOR1, (HD_VIDEOCAP_0|HD_VIDEOCAP_1));
	}

	ret |= hd_videocap_set(video_cap_ctrl, HD_VIDEOCAP_PARAM_DRV_CONFIG, &cap_cfg);
	iq_ctl.func = VCAP_ALG_FUNC;

	if (g_shdr_mode == 1) {
		iq_ctl.func |= HD_VIDEOCAP_FUNC_SHDR;
	}
	ret |= hd_videocap_set(video_cap_ctrl, HD_VIDEOCAP_PARAM_CTRL, &iq_ctl);

	*p_video_cap_ctrl = video_cap_ctrl;
	return ret;
}
#else

static HD_RESULT set_cap_cfg_ad(HD_PATH_ID *p_video_cap_ctrl)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOCAP_DRV_CONFIG cap_cfg = {0};
	HD_PATH_ID video_cap_ctrl = 0;
	HD_VIDEOCAP_CTRL iq_ctl = {0};
	UINT32 ad_map;

	printf("cap cfg tp2863");
	snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tp2863");
	cap_cfg.sen_cfg.sen_dev.if_type = HD_COMMON_VIDEO_IN_MIPI_CSI;

	cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
	cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0xF02;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3		
	cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x01;//PIN_I2C_CFG_CH1
	cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
	cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;		

	ret = hd_videocap_open(0, HD_VIDEOCAP_0_CTRL, &video_cap_ctrl); //open this for device control
	if (ret != HD_OK) {
		return ret;
	}

	// ad_map = VENDOR_VIDEOCAP_AD_MAP(p_stream->chip_id, p_stream->vin_id, p_stream->vcap_id_bit);
	ad_map = VENDOR_VIDEOCAP_AD_MAP(0, 0, 1 << 0 );
	vendor_videocap_set(video_cap_ctrl, VENDOR_VIDEOCAP_PARAM_AD_MAP, &ad_map);
	ret |= hd_videocap_set(video_cap_ctrl, HD_VIDEOCAP_PARAM_DRV_CONFIG, &cap_cfg);
	iq_ctl.func = 0;
	ret |= hd_videocap_set(video_cap_ctrl, HD_VIDEOCAP_PARAM_CTRL, &iq_ctl);
	*p_video_cap_ctrl = video_cap_ctrl;	
	return ret;
}
#endif 
static HD_RESULT set_cap_param(HD_PATH_ID video_cap_path, HD_DIM *p_dim, BOOL is_pull)
{
	HD_RESULT ret = HD_OK;
	{//select sensor mode, manually or automatically
		HD_VIDEOCAP_IN video_in_param = {0};
		video_in_param.sen_mode = HD_VIDEOCAP_SEN_MODE_AUTO; //auto select sensor mode by the parameter of HD_VIDEOCAP_PARAM_OUT
		video_in_param.frc = HD_VIDEO_FRC_RATIO(30,1);
		video_in_param.dim.w = p_dim->w;
		video_in_param.dim.h = p_dim->h;
		video_in_param.pxlfmt = SEN_OUT_FMT;
		video_in_param.out_frame_num = HD_VIDEOCAP_SEN_FRAME_NUM_1;
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_IN, &video_in_param);
		//printf("set_cap_param MODE=%d\r\n", ret);
		if (ret != HD_OK) {
			return ret;
		}
	}

#ifdef AD_ENABLE
{
	// VENDOR_VIDEOCAP_CCIR_INFO ccir_info;
	VENDOR_VIDEOCAP_CCIR_INFO ccir_info = {0};
	ccir_info.interlace = 0; //  plug_info.interlace;
	ccir_info.field_sel = VENDOR_VIDEOCAP_FIELD_DISABLE; // plug_info.interlace ? VENDOR_VIDEOCAP_FIELD_EN_0 : VENDOR_VIDEOCAP_FIELD_DISABLE;
	ccir_info.fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601; //  p_stream->module_info->ccir_fmt;
	ccir_info.mux_data_index = 0; // p_stream->ccir_mux_index;
		ret = vendor_videocap_set(video_cap_path, VENDOR_VIDEOCAP_PARAM_CCIR_INFO, &ccir_info);
		//printf("set_cap_param MODE=%d\r\n", ret);
		if (ret != HD_OK) {
			return ret;
		}
}		
#endif 
	#if 1 //no crop, full frame
	{
		HD_VIDEOCAP_CROP video_crop_param = {0};

		video_crop_param.mode = HD_CROP_OFF;
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_IN_CROP, &video_crop_param);
		//printf("set_cap_param CROP NONE=%d\r\n", ret);
	}
	#else //HD_CROP_ON
	{
		HD_VIDEOCAP_CROP video_crop_param = {0};

		video_crop_param.mode = HD_CROP_ON;
		video_crop_param.win.rect.x = 0;
		video_crop_param.win.rect.y = 0;
		video_crop_param.win.rect.w = 1920/2;
		video_crop_param.win.rect.h= 1080/2;
		video_crop_param.align.w = 4;
		video_crop_param.align.h = 4;
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_IN_CROP, &video_crop_param);
		//printf("set_cap_param CROP ON=%d\r\n", ret);
	}
	#endif
	{
		HD_VIDEOCAP_OUT video_out_param = {0};

		//without setting dim for no scaling, using original sensor out size
		video_out_param.pxlfmt = CAP_OUT_FMT;
		video_out_param.dir = HD_VIDEO_DIR_NONE;
		video_out_param.depth = is_pull; //set 1 to allow pull
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_OUT, &video_out_param);
		//printf("set_cap_param OUT=%d\r\n", ret);
	}

	{ // Set MIPI Data Lane
		int lane= 4; 
		ret = vendor_videocap_set(video_cap_path, VENDOR_VIDEOCAP_PARAM_DATA_LANE, &lane);
	}

	return ret;
}

///////////////////////////////////////////////////////////////////////////////
#ifndef AD_ENABLE
static HD_RESULT set_proc_cfg(HD_PATH_ID *p_video_proc_ctrl, HD_DIM* p_max_dim, HD_VIDEO_PXLFMT fmt, HD_OUT_ID _out_id, BOOL is_cap)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOPROC_DEV_CONFIG video_cfg_param = {0};
	HD_VIDEOPROC_CTRL video_ctrl_param = {0};
	HD_PATH_ID video_proc_ctrl = 0;

	ret = hd_videoproc_open(0, _out_id, &video_proc_ctrl); //open this for device control
	if (ret != HD_OK)
		return ret;

	{
		if (is_cap)
			video_cfg_param.pipe = HD_VIDEOPROC_PIPE_RAWCAP;
		else
			video_cfg_param.pipe = HD_VIDEOPROC_PIPE_RAWALL;
		if ((HD_CTRL_ID)_out_id == HD_VIDEOPROC_0_CTRL) {
			video_cfg_param.isp_id = 0x00000000;
		} else {
			video_cfg_param.isp_id = 0x00010000;
		}
		video_cfg_param.ctrl_max.func = VPRC_ALG_FUNC;
		video_cfg_param.in_max.func = 0;
		video_cfg_param.in_max.dim.w = (p_max_dim != NULL ) ? p_max_dim->w : 0;
		video_cfg_param.in_max.dim.h = (p_max_dim != NULL ) ? p_max_dim->h : 0;
		video_cfg_param.in_max.pxlfmt = (fmt != 0) ? fmt : 0;
		video_cfg_param.in_max.frc = HD_VIDEO_FRC_RATIO(1,1);
		ret = hd_videoproc_set(video_proc_ctrl, HD_VIDEOPROC_PARAM_DEV_CONFIG, &video_cfg_param);
		if (ret != HD_OK) {
			return HD_ERR_NG;
		}
	}

	video_ctrl_param.func = VPRC_ALG_FUNC;
	ret = hd_videoproc_set(video_proc_ctrl, HD_VIDEOPROC_PARAM_CTRL, &video_ctrl_param);

	*p_video_proc_ctrl = video_proc_ctrl;

	return ret;
}
#else
static HD_RESULT set_proc_cfg_ad(HD_PATH_ID *p_video_proc_ctrl, HD_DIM* p_max_dim, HD_VIDEO_PXLFMT fmt, HD_OUT_ID _out_id, BOOL is_cap)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOPROC_DEV_CONFIG video_cfg_param = {0};
	HD_VIDEOPROC_CTRL video_ctrl_param = {0};
	HD_PATH_ID video_proc_ctrl = 0;

	ret = hd_videoproc_open(0, _out_id, &video_proc_ctrl); //open this for device control
	if (ret != HD_OK)
		return ret;

	{
		video_cfg_param.pipe = HD_VIDEOPROC_PIPE_YUVALL;
		video_cfg_param.isp_id = _out_id;
		video_cfg_param.ctrl_max.func = 0;
		video_cfg_param.in_max.func = 0;
		video_cfg_param.in_max.dim.w = (p_max_dim != NULL ) ? p_max_dim->w : 0;
		video_cfg_param.in_max.dim.h = (p_max_dim != NULL ) ? p_max_dim->h : 0;
		video_cfg_param.in_max.pxlfmt = (fmt != 0) ? fmt : 0;	
		video_cfg_param.in_max.frc = HD_VIDEO_FRC_RATIO(1,1);
		ret = hd_videoproc_set(video_proc_ctrl, HD_VIDEOPROC_PARAM_DEV_CONFIG, &video_cfg_param);
		if (ret != HD_OK) {
			return HD_ERR_NG;
		}
	}

	video_ctrl_param.func = VPRC_ALG_FUNC;
	ret = hd_videoproc_set(video_proc_ctrl, HD_VIDEOPROC_PARAM_CTRL, &video_ctrl_param);

	*p_video_proc_ctrl = video_proc_ctrl;

	return ret;
}
#endif 
static HD_RESULT set_proc_param(HD_PATH_ID video_proc_path, HD_DIM* p_dim, BOOL is_pull)
{
	HD_RESULT ret = HD_OK;

	if (p_dim != NULL) { //if videoproc is already binding to dest module, not require to setting this!
		HD_VIDEOPROC_OUT video_out_param = {0};
		video_out_param.func = 0;
		video_out_param.dim.w = p_dim->w;
		video_out_param.dim.h = p_dim->h;
		video_out_param.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
		video_out_param.dir = HD_VIDEO_DIR_NONE;
		video_out_param.frc = HD_VIDEO_FRC_RATIO(1,1);
		video_out_param.depth = is_pull; //set 1 to allow pull
		ret = hd_videoproc_set(video_proc_path, HD_VIDEOPROC_PARAM_OUT, &video_out_param);
	} else {
		HD_VIDEOPROC_OUT video_out_param = {0};
		video_out_param.func = 0;
		video_out_param.dim.w = 0;
		video_out_param.dim.h = 0;
		video_out_param.pxlfmt = 0;
		video_out_param.dir = HD_VIDEO_DIR_NONE;
		video_out_param.frc = HD_VIDEO_FRC_RATIO(1,1);
		video_out_param.depth = is_pull; //set 1 to allow pull
		ret = hd_videoproc_set(video_proc_path, HD_VIDEOPROC_PARAM_OUT, &video_out_param);
	}

	return ret;
}

///////////////////////////////////////////////////////////////////////////////

static HD_RESULT set_out_cfg(HD_CTRL_ID ctrl_id,HD_PATH_ID *p_video_out_ctrl, UINT32 out_type, HD_VIDEOOUT_HDMI_ID hdmi_id)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOOUT_MODE videoout_mode = {0};
	HD_PATH_ID video_out_ctrl = 0;

	ret = hd_videoout_open(0, ctrl_id, &video_out_ctrl); //open this for device control
	if (ret != HD_OK) {
		return ret;
	}
	printf("out_type=%d\r\n", out_type);
	videoout_mode.output_type = HD_COMMON_VIDEO_OUT_LCD;
	videoout_mode.input_dim = HD_VIDEOOUT_IN_AUTO;
	videoout_mode.output_mode.lcd = HD_VIDEOOUT_LCD_0;
	if (out_type != 1) {
		printf("520 only support LCD\r\n");
	}

	ret = hd_videoout_set(video_out_ctrl, HD_VIDEOOUT_PARAM_MODE, &videoout_mode);

	*p_video_out_ctrl=video_out_ctrl ;
	return ret;
}

static HD_RESULT get_out_caps(HD_PATH_ID video_out_ctrl,HD_VIDEOOUT_SYSCAPS *p_video_out_syscaps)
{
	HD_RESULT ret = HD_OK;
    HD_DEVCOUNT video_out_dev = {0};

	ret = hd_videoout_get(video_out_ctrl, HD_VIDEOOUT_PARAM_DEVCOUNT, &video_out_dev);
	if (ret != HD_OK) {
		return ret;
	}
	printf("##devcount %d\r\n", video_out_dev.max_dev_count);

	ret = hd_videoout_get(video_out_ctrl, HD_VIDEOOUT_PARAM_SYSCAPS, p_video_out_syscaps);
	if (ret != HD_OK) {
		return ret;
	}
	return ret;
}

static HD_RESULT set_out_param(HD_PATH_ID video_out_path, HD_DIM *p_dim)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOOUT_IN video_out_param={0};

	video_out_param.dim.w = p_dim->w;
	video_out_param.dim.h = p_dim->h;
	video_out_param.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
	video_out_param.dir = HD_VIDEO_DIR_NONE;
	ret = hd_videoout_set(video_out_path, HD_VIDEOOUT_PARAM_IN, &video_out_param);
	if (ret != HD_OK) {
		return ret;
	}
	memset((void *)&video_out_param,0,sizeof(HD_VIDEOOUT_IN));
	ret = hd_videoout_get(video_out_path, HD_VIDEOOUT_PARAM_IN, &video_out_param);
	if (ret != HD_OK) {
		return ret;
	}
	printf("##video_out_param w:%d,h:%d %x %x\r\n", video_out_param.dim.w, video_out_param.dim.h, video_out_param.pxlfmt, video_out_param.dir);

	return ret;
}

///////////////////////////////////////////////////////////////////////////////

static HD_RESULT set_enc_cfg(HD_PATH_ID video_enc_path, HD_DIM *p_max_dim, UINT32 enc_type, UINT32 max_bitrate, UINT32 buf_ms)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOENC_PATH_CONFIG video_path_config = {0};

	if (p_max_dim != NULL) {

		//--- HD_VIDEOENC_PARAM_PATH_CONFIG ---
		if (enc_type == 0) {
			video_path_config.max_mem.codec_type = HD_CODEC_TYPE_H265;
		} else if (enc_type == 1) {
			video_path_config.max_mem.codec_type = HD_CODEC_TYPE_H264;
		} else if (enc_type == 2) {
			video_path_config.max_mem.codec_type = HD_CODEC_TYPE_JPEG;
		} else {
			printf("not support enc_type\r\n");
			return HD_ERR_NG;
		}
		video_path_config.max_mem.max_dim.w  = p_max_dim->w;
		video_path_config.max_mem.max_dim.h  = p_max_dim->h;
		video_path_config.max_mem.bitrate    = max_bitrate;
		video_path_config.max_mem.enc_buf_ms = buf_ms;
		video_path_config.max_mem.svc_layer  = HD_SVC_4X;
		video_path_config.max_mem.ltr        = TRUE;
		video_path_config.max_mem.rotate     = FALSE;
		video_path_config.max_mem.source_output   = FALSE;
		video_path_config.isp_id             = 0;
		ret = hd_videoenc_set(video_enc_path, HD_VIDEOENC_PARAM_PATH_CONFIG, &video_path_config);
		if (ret != HD_OK) {
			printf("set_enc_path_config = %d\r\n", ret);
			return HD_ERR_NG;
		}
	}

	return ret;
}

static HD_RESULT set_enc_param(HD_PATH_ID video_enc_path, HD_DIM *p_dim, UINT32 enc_type, UINT32 bitrate)
{
	HD_RESULT ret = HD_OK;
	HD_VIDEOENC_IN  video_in_param = {0};
	HD_VIDEOENC_OUT video_out_param = {0};
	HD_H26XENC_RATE_CONTROL rc_param = {0};

	if (p_dim != NULL) {

		//--- HD_VIDEOENC_PARAM_IN ---
		video_in_param.dir           = HD_VIDEO_DIR_NONE;
		video_in_param.pxl_fmt = HD_VIDEO_PXLFMT_YUV420;
		video_in_param.dim.w   = p_dim->w;
		video_in_param.dim.h   = p_dim->h;
		video_in_param.frc     = HD_VIDEO_FRC_RATIO(1,1);
		ret = hd_videoenc_set(video_enc_path, HD_VIDEOENC_PARAM_IN, &video_in_param);
		if (ret != HD_OK) {
			printf("set_enc_param_in = %d\r\n", ret);
			return ret;
		}

		printf("enc_type=%d\r\n", enc_type);

		if (enc_type == 0) {

			//--- HD_VIDEOENC_PARAM_OUT_ENC_PARAM ---
			video_out_param.codec_type         = HD_CODEC_TYPE_H265;
			video_out_param.h26x.profile       = HD_H265E_MAIN_PROFILE;
			video_out_param.h26x.level_idc     = HD_H265E_LEVEL_5;
			video_out_param.h26x.gop_num       = 15;
			video_out_param.h26x.ltr_interval  = 0;
			video_out_param.h26x.ltr_pre_ref   = 0;
			video_out_param.h26x.gray_en       = 0;
			video_out_param.h26x.source_output = 0;
			video_out_param.h26x.svc_layer     = HD_SVC_DISABLE;
			video_out_param.h26x.entropy_mode  = HD_H265E_CABAC_CODING;
			ret = hd_videoenc_set(video_enc_path, HD_VIDEOENC_PARAM_OUT_ENC_PARAM, &video_out_param);
			if (ret != HD_OK) {
				printf("set_enc_param_out = %d\r\n", ret);
				return ret;
			}

			//--- HD_VIDEOENC_PARAM_OUT_RATE_CONTROL ---
			rc_param.rc_mode             = HD_RC_MODE_CBR;
			rc_param.cbr.bitrate         = bitrate;
			rc_param.cbr.frame_rate_base = 30;
			rc_param.cbr.frame_rate_incr = 1;
			rc_param.cbr.init_i_qp       = 26;
			rc_param.cbr.min_i_qp        = 10;
			rc_param.cbr.max_i_qp        = 45;
			rc_param.cbr.init_p_qp       = 26;
			rc_param.cbr.min_p_qp        = 10;
			rc_param.cbr.max_p_qp        = 45;
			rc_param.cbr.static_time     = 4;
			rc_param.cbr.ip_weight       = 0;
			ret = hd_videoenc_set(video_enc_path, HD_VIDEOENC_PARAM_OUT_RATE_CONTROL, &rc_param);
			if (ret != HD_OK) {
				printf("set_enc_rate_control = %d\r\n", ret);
				return ret;
			}
		} else if (enc_type == 1) {

			//--- HD_VIDEOENC_PARAM_OUT_ENC_PARAM ---
			video_out_param.codec_type         = HD_CODEC_TYPE_H264;
			video_out_param.h26x.profile       = HD_H264E_HIGH_PROFILE;
			video_out_param.h26x.level_idc     = HD_H264E_LEVEL_5_1;
			video_out_param.h26x.gop_num       = 15;
			video_out_param.h26x.ltr_interval  = 0;
			video_out_param.h26x.ltr_pre_ref   = 0;
			video_out_param.h26x.gray_en       = 0;
			video_out_param.h26x.source_output = 0;
			video_out_param.h26x.svc_layer     = HD_SVC_DISABLE;
			video_out_param.h26x.entropy_mode  = HD_H264E_CABAC_CODING;
			ret = hd_videoenc_set(video_enc_path, HD_VIDEOENC_PARAM_OUT_ENC_PARAM, &video_out_param);
			if (ret != HD_OK) {
				printf("set_enc_param_out = %d\r\n", ret);
				return ret;
			}

			//--- HD_VIDEOENC_PARAM_OUT_RATE_CONTROL ---
			rc_param.rc_mode             = HD_RC_MODE_CBR;
			rc_param.cbr.bitrate         = bitrate;
			rc_param.cbr.frame_rate_base = 30;
			rc_param.cbr.frame_rate_incr = 1;
			rc_param.cbr.init_i_qp       = 26;
			rc_param.cbr.min_i_qp        = 10;
			rc_param.cbr.max_i_qp        = 45;
			rc_param.cbr.init_p_qp       = 26;
			rc_param.cbr.min_p_qp        = 10;
			rc_param.cbr.max_p_qp        = 45;
			rc_param.cbr.static_time     = 4;
			rc_param.cbr.ip_weight       = 0;
			ret = hd_videoenc_set(video_enc_path, HD_VIDEOENC_PARAM_OUT_RATE_CONTROL, &rc_param);
			if (ret != HD_OK) {
				printf("set_enc_rate_control = %d\r\n", ret);
				return ret;
			}

		} else if (enc_type == 2) {

			//--- HD_VIDEOENC_PARAM_OUT_ENC_PARAM ---
			video_out_param.codec_type         = HD_CODEC_TYPE_JPEG;
			video_out_param.jpeg.retstart_interval = 0;
			video_out_param.jpeg.image_quality = 70;
			ret = hd_videoenc_set(video_enc_path, HD_VIDEOENC_PARAM_OUT_ENC_PARAM, &video_out_param);
			if (ret != HD_OK) {
				printf("set_enc_param_out = %d\r\n", ret);
				return ret;
			}

		} else {

			printf("not support enc_type\r\n");
			return HD_ERR_NG;
		}
	}

	return ret;
}

#ifdef AUDIO_OUT_ENABLE
// set_aout_cfg
static HD_RESULT set_aout_cfg(HD_PATH_ID *p_audio_out_ctrl, HD_AUDIO_SR sample_rate)
{
	HD_RESULT ret = HD_OK;
	HD_AUDIOOUT_DEV_CONFIG audio_cfg_param = {0};
	HD_AUDIOOUT_DRV_CONFIG audio_driver_cfg_param = {0};
	HD_PATH_ID audio_out_ctrl = 0;

	ret = hd_audioout_open(0, HD_AUDIOOUT_0_CTRL, &audio_out_ctrl); //open this for device control
	if (ret != HD_OK) {
		return ret;
	}

	/* set audio out maximum parameters */
	audio_cfg_param.out_max.sample_rate = sample_rate;
	audio_cfg_param.out_max.sample_bit = HD_AUDIO_BIT_WIDTH_16;
	audio_cfg_param.out_max.mode = HD_AUDIO_SOUND_MODE_STEREO;
	audio_cfg_param.frame_sample_max = 1024;
	audio_cfg_param.frame_num_max = 10;
	audio_cfg_param.in_max.sample_rate = 0;
	ret = hd_audioout_set(audio_out_ctrl, HD_AUDIOOUT_PARAM_DEV_CONFIG, &audio_cfg_param);
	if (ret != HD_OK) {
		return ret;
	}

	/* set audio out driver parameters */
	audio_driver_cfg_param.mono = HD_AUDIO_MONO_LEFT;
	audio_driver_cfg_param.output = HD_AUDIOOUT_OUTPUT_SPK;
	ret = hd_audioout_set(audio_out_ctrl, HD_AUDIOOUT_PARAM_DRV_CONFIG, &audio_driver_cfg_param);

	*p_audio_out_ctrl = audio_out_ctrl;

	return ret;
}

static HD_RESULT set_aout_param(HD_PATH_ID audio_out_ctrl, HD_PATH_ID audio_out_path, HD_AUDIO_SR sample_rate)
{
	HD_RESULT ret;
	HD_AUDIOOUT_OUT audio_out_out_param = {0};
	HD_AUDIOOUT_VOLUME audio_out_vol = {0};
	HD_AUDIOOUT_IN audio_out_in_param = {0};

	// set hd_audioout output parameters
	audio_out_out_param.sample_rate = sample_rate;
	audio_out_out_param.sample_bit = HD_AUDIO_BIT_WIDTH_16;
	audio_out_out_param.mode = HD_AUDIO_SOUND_MODE_STEREO;
	ret = hd_audioout_set(audio_out_path, HD_AUDIOOUT_PARAM_OUT, &audio_out_out_param);
	if (ret != HD_OK) {
		return ret;
	}

	// set hd_audioout volume
	audio_out_vol.volume = 100;
	ret = hd_audioout_set(audio_out_ctrl, HD_AUDIOOUT_PARAM_VOLUME, &audio_out_vol);
	if (ret != HD_OK) {
		return ret;
	}


	// set hd_audioout input parameters
	audio_out_in_param.sample_rate = 0;
	ret = hd_audioout_set(audio_out_path, HD_AUDIOOUT_PARAM_IN, &audio_out_in_param);

	return ret;
}

#endif


///////////////////////////////////////////////////////////////////////////////

typedef struct _VIDEO_RECORD_SIZE {
	UINT32 w;
	UINT32 h;
} VIDEO_RECORD_SIZE;

VIDEO_RECORD_SIZE rec_size[3] = {
	{1920, 1080}, //2M
	{1280, 720},
	{864, 480},
};

typedef struct _VIDEO_LIVEVIEW {

	// (1)
	HD_VIDEOCAP_SYSCAPS cap_syscaps;
	HD_PATH_ID cap_ctrl;
	HD_PATH_ID cap_path;

	HD_DIM  cap_dim;
	HD_DIM  proc_max_dim;

	// (2)
	HD_VIDEOPROC_SYSCAPS proc_syscaps;
	HD_PATH_ID proc_ctrl;
	HD_PATH_ID proc_path;

	HD_DIM  out_max_dim;
	HD_DIM  out_dim;

	// (3)
	HD_VIDEOOUT_SYSCAPS out_syscaps;
	HD_PATH_ID out_ctrl;
	HD_PATH_ID out_path;
    HD_VIDEOOUT_HDMI_ID hdmi_id;

	HD_DIM  out1_max_dim;
	HD_DIM  out1_dim;

	// (4)
	HD_VIDEOOUT_SYSCAPS out1_syscaps;
	HD_PATH_ID out1_ctrl;
	HD_PATH_ID out1_path;

} VIDEO_LIVEVIEW;


typedef struct _VIDEO_RECORD {

	// (1)
	HD_VIDEOCAP_SYSCAPS cap_syscaps;
	HD_PATH_ID cap_ctrl;
	HD_PATH_ID cap_path;

	HD_DIM  cap_dim;
	HD_DIM  proc_max_dim;

	// (2)
	HD_VIDEOPROC_SYSCAPS proc_syscaps;
	HD_PATH_ID proc_ctrl;
	HD_PATH_ID proc_path;  //for record to venc
	HD_DIM  record_dim;

	HD_DIM  enc_max_dim;
	HD_DIM  enc_dim;
	UINT32	enc_type;

	// (3)
	HD_VIDEOENC_SYSCAPS enc_syscaps;
	HD_PATH_ID enc_path;

	// (4) user pull
	UINT32 	sel_rec_size; //to select rec_size[]

	pthread_t  save_thread_id;
	UINT32	save_enter;
	UINT32	save_exit;
	UINT32	save_count;

	pthread_t  flow_thread_id;
	UINT32	flow_run;
	UINT32	flow_quit;
	UINT32	flow_state;

} VIDEO_RECORD;


#ifdef AUDIO_OUT_ENABLE
typedef struct _AUDIO_OUTONLY {
	HD_AUDIO_SR sample_rate_max;
	HD_AUDIO_SR sample_rate;

	HD_PATH_ID out_ctrl;
	HD_PATH_ID out_path;

	UINT32 out_exit;
	UINT32 out_pause;
} AUDIO_OUTONLY;
#endif

static HD_RESULT init_module(void)
{
	HD_RESULT ret;
	if ((ret = hd_videocap_init()) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_init()) != HD_OK)
		return ret;
	if ((ret = hd_videoout_init()) != HD_OK)
		return ret;
    if ((ret = hd_videoenc_init()) != HD_OK)
		return ret;
#ifdef AUDIO_OUT_ENABLE		
	if((ret = hd_audioout_init()) != HD_OK)
		return ret;		
#endif 		
	return HD_OK;
}

static HD_RESULT open_module(VIDEO_LIVEVIEW *p_stream, HD_DIM* p_proc_max_dim, HD_VIDEO_PXLFMT fmt, UINT32 out_type)
{
	HD_RESULT ret;
	// set videocap config
#ifdef AD_ENABLE
	ret = set_cap_cfg_ad(&p_stream->cap_ctrl);
#else	
	ret = set_cap_cfg(&p_stream->cap_ctrl);
#endif 	
	if (ret != HD_OK) {
		printf("set cap-cfg fail=%d\n", ret);
		return HD_ERR_NG;
	}
	// set videoproc config
#ifdef AD_ENABLE
	ret = set_proc_cfg_ad(&p_stream->proc_ctrl, p_proc_max_dim, fmt, HD_VIDEOPROC_0_CTRL, FALSE); //open for liveview
#else	
	ret = set_proc_cfg(&p_stream->proc_ctrl, p_proc_max_dim, fmt, HD_VIDEOPROC_0_CTRL, FALSE); //open for liveview
#endif 	
	if (ret != HD_OK) {
		printf("set proc-cfg fail=%d\n", ret);
		return HD_ERR_NG;
	}
	// set videoout config
	ret = set_out_cfg(HD_VIDEOOUT_0_CTRL,&p_stream->out_ctrl, out_type, p_stream->hdmi_id);
	if (ret != HD_OK) {
		printf("set out-cfg fail=%d\n", ret);
		return HD_ERR_NG;
	}
	if ((ret = hd_videocap_open(HD_VIDEOCAP_0_IN_0, HD_VIDEOCAP_0_OUT_0, &p_stream->cap_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_open(HD_VIDEOPROC_0_IN_0, HD_VIDEOPROC_0_OUT_0, &p_stream->proc_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoout_open(HD_VIDEOOUT_0_IN_0, HD_VIDEOOUT_0_OUT_0, &p_stream->out_path)) != HD_OK)
		return ret;

	return HD_OK;
}

static HD_RESULT open_module_2(VIDEO_RECORD *p_stream, HD_DIM* p_proc_max_dim, HD_VIDEO_PXLFMT fmt)
{
	HD_RESULT ret;

    if ((ret = hd_videoproc_open(HD_VIDEOPROC_0_IN_0, HD_VIDEOPROC_0_OUT_1, &p_stream->proc_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoenc_open(HD_VIDEOENC_0_IN_0, HD_VIDEOENC_0_OUT_0, &p_stream->enc_path)) != HD_OK)
		return ret;
	return HD_OK;
}

#ifdef AUDIO_OUT_ENABLE
static HD_RESULT open_module_audio(AUDIO_OUTONLY *p_outonly)
{
	HD_RESULT ret;
	ret = set_aout_cfg(&p_outonly->out_ctrl, p_outonly->sample_rate_max);
	if (ret != HD_OK) {
		printf("set out-cfg fail\n");
		return HD_ERR_NG;
	}
	if((ret = hd_audioout_open(HD_AUDIOOUT_0_IN_0, HD_AUDIOOUT_0_OUT_0, &p_outonly->out_path)) != HD_OK)
		return ret;
	return HD_OK;
}
#endif 

static HD_RESULT close_module(VIDEO_LIVEVIEW *p_stream)
{
	HD_RESULT ret;
	if ((ret = hd_videocap_close(p_stream->cap_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_close(p_stream->proc_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoout_close(p_stream->out_path)) != HD_OK)
		return ret;
	return HD_OK;
}

static HD_RESULT close_module_2(VIDEO_RECORD *p_stream)
{
	HD_RESULT ret;
	if ((ret = hd_videoproc_close(p_stream->proc_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoenc_close(p_stream->enc_path)) != HD_OK)
		return ret;
	return HD_OK;
}
#ifdef AUDIO_OUT_ENABLE
static HD_RESULT close_module_audio(AUDIO_OUTONLY *p_outonly)
{
	HD_RESULT ret;
	if((ret = hd_audioout_close(p_outonly->out_path)) != HD_OK)
		return ret;
	return HD_OK;
}
#endif 
static HD_RESULT exit_module(void)
{
	HD_RESULT ret;
	if ((ret = hd_videocap_uninit()) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_uninit()) != HD_OK)
		return ret;
	if ((ret = hd_videoout_uninit()) != HD_OK)
		return ret;
	if ((ret = hd_videoenc_uninit()) != HD_OK)
		return ret;
#ifdef AUDIO_OUT_ENABLE
	if((ret = hd_audioout_uninit()) != HD_OK)
		return ret;
#endif 
	return HD_OK;
}

#define FLOW_ON_OPEN		1
#define FLOW_ON_REC			2
#define FLOW_ON_CLOSE		3
#define FLOW_ON_STOP		4

static void *save_thread(void *arg)
{
	VIDEO_RECORD* p_stream0 = (VIDEO_RECORD *)arg;
	HD_RESULT ret = HD_OK;
	HD_VIDEOENC_BS  data_pull;
	UINT32 j;

	UINT32 vir_addr_main;
	HD_VIDEOENC_BUFINFO phy_buf_main;
	char file_path_main[64] = {0};
	FILE *f_out_main;
	#define PHY2VIRT_MAIN(pa) (vir_addr_main + (pa - phy_buf_main.buf_info.phy_addr))

	p_stream0->save_exit = 0;
	//------ wait flow_start ------
	while (p_stream0->save_enter == 0) usleep(100);

	// query physical address of bs buffer ( this can ONLY query after hd_videoenc_start() is called !! )
	hd_videoenc_get(p_stream0->enc_path, HD_VIDEOENC_PARAM_BUFINFO, &phy_buf_main);

	// mmap for bs buffer (just mmap one time only, calculate offset to virtual address later)
	vir_addr_main = (UINT32)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, phy_buf_main.buf_info.phy_addr, phy_buf_main.buf_info.buf_size);
	if (vir_addr_main == 0) {
		printf("mmap error !!\r\n\r\n");
		return 0;
	}

	if (p_stream0->enc_type == 0) {
		snprintf(file_path_main, 64, "/mnt/sd/rec_%lux%lu_%lu_h265.mp4",
			p_stream0->record_dim.w, p_stream0->record_dim.h,
			p_stream0->save_count+1);
	} else if (p_stream0->enc_type == 1) {
		snprintf(file_path_main, 64, "/mnt/sd/rec_%lux%lu_%lu_h264.mp4",
			p_stream0->record_dim.w, p_stream0->record_dim.h,
			p_stream0->save_count+1);
	} else if (p_stream0->enc_type == 2) {
		snprintf(file_path_main, 64, "/mnt/sd/rec_%lux%lu_%lu_mjpg.mp4",
			p_stream0->record_dim.w, p_stream0->record_dim.h,
			p_stream0->save_count+1);
	} else {
		printf("not support enc_type\r\n");
		return 0;
	}
	printf("save %d (%s) .... ", p_stream0->save_count+1, file_path_main);

	//----- open output files -----
	if ((f_out_main = fopen(file_path_main, "wb")) == NULL) {
		HD_VIDEOENC_ERR("open file (%s) fail....\r\n\r\n", file_path_main);
			goto skip3;
	}

	//--------- pull data test ---------
	while (p_stream0->save_exit == 0) {

		//printf("enc_pull ....\r\n");
		ret = hd_videoenc_pull_out_buf(p_stream0->enc_path, &data_pull, -1); // -1 = blocking mode
		if (ret != HD_OK) {
			if (ret != HD_ERR_UNDERRUN)
			printf("enc_pull error=%d !!\r\n\r\n", ret);
    			goto skip3;
		}

		for (j=0; j< data_pull.pack_num; j++) {
			UINT8 *ptr = (UINT8 *)PHY2VIRT_MAIN(data_pull.video_pack[j].phy_addr);
			UINT32 len = data_pull.video_pack[j].size;
			if (f_out_main) fwrite(ptr, 1, len, f_out_main);
			if (f_out_main) fflush(f_out_main);
		}

		//printf("enc_release ....\r\n");
		ret = hd_videoenc_release_out_buf(p_stream0->enc_path, &data_pull);
		if (ret != HD_OK) {
			printf("enc_release error=%d !!\r\n", ret);
		}

		if ((p_stream0->flow_run == FLOW_ON_STOP)) {
			// let save_thread stop loop and exit
			p_stream0->save_exit = 1;
		}
skip3:
		usleep(100); //sleep for getchar()
	}

	// mummap for bs buffer
	ret = hd_common_mem_munmap((void *)vir_addr_main, phy_buf_main.buf_info.buf_size);
	if (ret != HD_OK) {
		printf("mnumap error !!\r\n\r\n");
	}

	// close output file
	fclose(f_out_main);

	printf("ok\r\n");

	p_stream0->save_count++;
	//printf("save count = %d\r\n", p_stream0->save_count);

	return 0;
}

static void *flow_thread(void *arg)
{
	VIDEO_LIVEVIEW* p_stream0 = (VIDEO_LIVEVIEW *)((UINT32*)arg)[0];
	VIDEO_RECORD* p_stream2 = (VIDEO_RECORD *)((UINT32*)arg)[1];
	UINT32 enc_type;

	while (p_stream2->flow_quit == 0 ) {

		if (p_stream2->flow_run == FLOW_ON_OPEN) {
			HD_RESULT ret;
            p_stream2->flow_state = FLOW_ON_OPEN;
			// get videocap capability
			ret = get_cap_caps(p_stream0->cap_ctrl, &p_stream0->cap_syscaps);
			if (ret != HD_OK) {
				printf("get cap-caps fail=%d\n", ret);
				goto exit2;
			}

			// get videoout capability
			ret = get_out_caps(p_stream0->out_ctrl, &p_stream0->out_syscaps);
			if (ret != HD_OK) {
				printf("get out-caps fail=%d\n", ret);
				goto exit2;
			}
			p_stream0->out_max_dim = p_stream0->out_syscaps.output_dim;

			// set videocap parameter
			p_stream0->cap_dim.w = VDO_SIZE_W; //assign by user
			p_stream0->cap_dim.h = VDO_SIZE_H; //assign by user
			ret = set_cap_param(p_stream0->cap_path, &p_stream0->cap_dim, FALSE);
			if (ret != HD_OK) {
				printf("set cap fail=%d\n", ret);
				goto exit2;
			}

			// set videoproc parameter (liveview)
			ret = set_proc_param(p_stream0->proc_path, NULL, FALSE);
			if (ret != HD_OK) {
				printf("set proc fail=%d\n", ret);
				goto exit2;
			}

			// set videoout parameter (liveview)
			p_stream0->out_dim.w = p_stream0->out_max_dim.w; //using device max dim.w
			p_stream0->out_dim.h = p_stream0->out_max_dim.h; //using device max dim.h
			ret = set_out_param(p_stream0->out_path, &p_stream0->out_dim);
			if (ret != HD_OK) {
				printf("set out fail=%d\n", ret);
				goto exit2;
			}

			// bind video_liveview modules (liveview)
			hd_videocap_bind(HD_VIDEOCAP_0_OUT_0, HD_VIDEOPROC_0_IN_0);
			hd_videoproc_bind(HD_VIDEOPROC_0_OUT_0, HD_VIDEOOUT_0_IN_0);

			// start video_liveview modules (liveview)
			hd_videocap_start(p_stream0->cap_path);
			hd_videoproc_start(p_stream0->proc_path);
			// just wait ae/awb stable for auto-test, if don't care, user can remove it
			sleep(1);
			hd_videoout_start(p_stream0->out_path);
		}
		if (p_stream2->flow_run == FLOW_ON_CLOSE) {
            p_stream2->flow_state = FLOW_ON_CLOSE;
			// stop video_liveview modules (liveview)
			hd_videocap_stop(p_stream0->cap_path);
			hd_videoproc_stop(p_stream0->proc_path);
			hd_videoout_stop(p_stream0->out_path);

			// unbind video_liveview modules (liveview)
			hd_videocap_unbind(HD_VIDEOCAP_0_OUT_0);
			hd_videoproc_unbind(HD_VIDEOPROC_0_OUT_0);
		}
		if (p_stream2->flow_run == FLOW_ON_REC) {
			HD_RESULT ret = HD_OK;
            p_stream2->flow_state = FLOW_ON_REC;
			//printf("stop liveview - begin\n");
			hd_videocap_stop(p_stream0->cap_path);
			hd_videoproc_stop(p_stream0->proc_path);
			//printf("stop liveview - end\n");

			// create save_thread (pull_out bitstream, save bitstream)
			ret = pthread_create(&p_stream2->save_thread_id, NULL, save_thread, (void *)p_stream2);
			if (ret < 0) {
				printf("create save thread failed");
				goto exit2;
			}

			printf("start record.\r\n");
			//printf("start record - begin\n");
			// set videocap parameter (record)
			p_stream2->cap_path = p_stream0->cap_path; //using the same cap_path
			p_stream2->cap_dim.w = REC_SIZE_W; //assign by user
			p_stream2->cap_dim.h = REC_SIZE_H; //assign by user
			ret = set_cap_param(p_stream2->cap_path, &p_stream2->cap_dim, FALSE);
			if (ret != HD_OK) {
				printf("set cap fail=%d\n", ret);
			}
			// set videoproc parameter (record)
			p_stream2->record_dim.w = rec_size[p_stream2->sel_rec_size].w; //assign by user
			p_stream2->record_dim.h = rec_size[p_stream2->sel_rec_size].h; //assign by user
			ret = set_proc_param(p_stream2->proc_path, &p_stream2->record_dim, FALSE);
			if (ret != HD_OK) {
				printf("set proc2 record fail=%d\n", ret);
				goto exit2;
			}
			if ((ret = hd_videoenc_close(p_stream2->enc_path)) != HD_OK) {
				printf("close enc record fail=%d\n", ret);
				goto exit2;
			}
			if ((ret = hd_videoenc_open(HD_VIDEOENC_0_IN_0, HD_VIDEOENC_0_OUT_0, &p_stream2->enc_path)) != HD_OK)	 {
				printf("open enc record fail=%d\n", ret);
				goto exit2;
			}
			// set videoenc config (record)
			enc_type = p_stream2->enc_type;
			p_stream2->enc_max_dim.w = REC_SIZE_W; //assign by user
			p_stream2->enc_max_dim.h = REC_SIZE_H; //assign by user
			ret = set_enc_cfg(p_stream2->enc_path, &p_stream2->enc_max_dim, enc_type, REC_TBR, REC_BUF);
			if (ret != HD_OK) {
				printf("set enc-cfg fail=%d\n", ret);
			}
			// set videoenc parameter (record)
			p_stream2->enc_dim.w = rec_size[p_stream2->sel_rec_size].w; //assign by user
			p_stream2->enc_dim.h = rec_size[p_stream2->sel_rec_size].h; //assign by user
			ret = set_enc_param(p_stream2->enc_path, &p_stream2->enc_dim, enc_type, REC_FIX_TBR(p_stream2->enc_dim.w, p_stream2->enc_dim.h));
			if (ret != HD_OK) {
				printf("set enc fail=%d\n", ret);
				goto exit2;
			}
			hd_videoproc_bind(HD_VIDEOPROC_0_OUT_1, HD_VIDEOENC_0_IN_0);
			hd_videocap_start(p_stream2->cap_path);
			hd_videoproc_start(p_stream0->proc_path);
			hd_videoproc_start(p_stream2->proc_path);
			hd_videoenc_start(p_stream2->enc_path);
			// let first thread start to work
			p_stream2->save_enter = 1;
			//printf("start record - end\n");

			//printf("wait ...\n");
			// destroy save thread
			pthread_join(p_stream2->save_thread_id, NULL);
			printf("stop record.\r\n");
            p_stream2->flow_state = FLOW_ON_STOP;
			//printf("stop record - begin\n");
			p_stream2->save_enter = 0;
			hd_videocap_stop(p_stream2->cap_path);
			hd_videoproc_stop(p_stream0->proc_path);
			hd_videoproc_stop(p_stream2->proc_path);
			hd_videoenc_stop(p_stream2->enc_path);
			hd_videoproc_unbind(HD_VIDEOPROC_0_OUT_1);
			//printf("stop record - end\n");

			//printf("start liveview - begin\n");
			// set videocap parameter (liveview)
			p_stream0->cap_dim.w = VDO_SIZE_W; //assign by user
			p_stream0->cap_dim.h = VDO_SIZE_H; //assign by user
			ret = set_cap_param(p_stream0->cap_path, &p_stream0->cap_dim, FALSE);
			if (ret != HD_OK) {
				printf("set cap fail=%d\n", ret);
				goto exit2;
			}
			// set videoproc parameter (liveview)
			ret = set_proc_param(p_stream0->proc_path, NULL, FALSE);
			if (ret != HD_OK) {
				printf("set proc fail=%d\n", ret);
				goto exit2;
			}
			hd_videocap_start(p_stream0->cap_path);
			hd_videoproc_start(p_stream0->proc_path);
			//printf("start liveview - end\n");
		}
exit2:
		p_stream2->flow_run = 0;
		usleep(100);
	}
	return 0;
}

#ifdef AUDIO_OUT_ENABLE
static void *playback_thread(void *arg)
{
	INT ret, bs_size, result;
	CHAR filename[50];
	FILE *bs_fd, *len_fd;
	HD_AUDIO_FRAME  bs_in_buf = {0};
	HD_COMMON_MEM_VB_BLK blk;
	UINT32 pa, va;
	UINT32 blk_size = 0x100000;
	HD_COMMON_MEM_DDR_ID ddr_id = DDR_ID0;
	UINT32 bs_buf_start, bs_buf_curr, bs_buf_end;
	INT au_frame_ms, elapse_time, au_buf_time, timestamp;
	UINT start_time, data_time;
	AUDIO_OUTONLY *p_out_only = (AUDIO_OUTONLY *)arg;

	/* read test pattern */
	snprintf(filename, sizeof(filename), "/mnt/sd/audio_bs_%d_%d_%d_pcm.dat", HD_AUDIO_BIT_WIDTH_16, HD_AUDIO_SOUND_MODE_STEREO, p_out_only->sample_rate);
	bs_fd = fopen(filename, "rb");
	if (bs_fd == NULL) {
		printf("[ERROR] Open %s failed!!\n", filename);
		return 0;
	}
	printf("play file: [%s]\n", filename);

	snprintf(filename, sizeof(filename), "/mnt/sd/audio_bs_%d_%d_%d_pcm.len", HD_AUDIO_BIT_WIDTH_16, HD_AUDIO_SOUND_MODE_STEREO, p_out_only->sample_rate);
	len_fd = fopen(filename, "rb");
	if (len_fd == NULL) {
		printf("[ERROR] Open %s failed!!\n", filename);
		goto play_fclose;
	}
	printf("len file: [%s]\n", filename);

	au_frame_ms = FRAME_SAMPLES * 1000 / p_out_only->sample_rate - 5; // the time(in ms) of each audio frame
	start_time = hd_gettime_ms();
	data_time = 0;

	/* get memory */
	blk = hd_common_mem_get_block(HD_COMMON_MEM_USER_POOL_BEGIN, blk_size, ddr_id); // Get block from mem pool
	if (blk == HD_COMMON_MEM_VB_INVALID_BLK) {
		printf("get block fail, blk = 0x%x\n", blk);
		goto play_fclose;
	}
	pa = hd_common_mem_blk2pa(blk); // get physical addr
	if (pa == 0) {
		printf("blk2pa fail, blk(0x%x)\n", blk);
		goto rel_blk;
	}
	if (pa > 0) {
		va = (UINT32)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, pa, blk_size); // Get virtual addr
		if (va == 0) {
			printf("get va fail, va(0x%x)\n", blk);
			goto rel_blk;
		}
		/* allocate bs buf */
		bs_buf_start = va;
		bs_buf_curr = bs_buf_start;
		bs_buf_end = bs_buf_start + blk_size;
		printf("alloc bs_buf: start(0x%x) curr(0x%x) end(0x%x) size(0x%x)\n", bs_buf_start, bs_buf_curr, bs_buf_end, blk_size);
	}

	while (1) {
retry:
		if (p_out_only->out_exit == 1) {
			break;
		}

		if (p_out_only->out_pause == 1) {
			usleep(10000);
			goto retry;
		}

		elapse_time = TIME_DIFF(hd_gettime_ms(), start_time);
		au_buf_time = data_time - elapse_time;
		if (au_buf_time > AUD_BUFFER_CNT * au_frame_ms) {
			//usleep(au_frame_ms);
			//goto retry;
		}

		/* get bs size */
		if (fscanf(len_fd, "%d %d\n", &bs_size, &timestamp) == EOF) {
			// reach EOF, read from the beginning
			fseek(bs_fd, 0, SEEK_SET);
			fseek(len_fd, 0, SEEK_SET);
			if (fscanf(len_fd, "%d %d\n", &bs_size, &timestamp) == EOF) {
				printf("[ERROR] fscanf error\n");
				continue;
			}
		}
		if (bs_size == 0 || bs_size > BITSTREAM_SIZE) {
			printf("Invalid bs_size(%d)\n", bs_size);
			continue;
		}

		/* check bs buf rollback */
		if ((bs_buf_curr + bs_size) > bs_buf_end) {
			bs_buf_curr = bs_buf_start;
		}

		/* read bs from file */
		result = fread((void *)bs_buf_curr, 1, bs_size, bs_fd);
		if (result != bs_size) {
			printf("reading error\n");
			continue;
		}
		bs_in_buf.sign = MAKEFOURCC('A','F','R','M');
		bs_in_buf.phy_addr[0] = pa + (bs_buf_curr - bs_buf_start); // needs to add offset
		bs_in_buf.size = bs_size;
		bs_in_buf.ddr_id = ddr_id;
		bs_in_buf.timestamp = hd_gettime_us();
		bs_in_buf.bit_width = HD_AUDIO_BIT_WIDTH_16;
		bs_in_buf.sound_mode = HD_AUDIO_SOUND_MODE_STEREO;
		bs_in_buf.sample_rate = p_out_only->sample_rate;

		/* push in buffer */
		data_time += au_frame_ms;
resend:
		ret = hd_audioout_push_in_buf(p_out_only->out_path, &bs_in_buf, -1);
		if (ret != HD_OK) {
			//printf("hd_audioout_push_in_buf fail, ret(%d)\n", ret);
			usleep(10000);
			goto resend;
		}

		bs_buf_curr += ALIGN_CEIL_4(bs_size); // shift to next
	}

	/* release memory */
	hd_common_mem_munmap((void*)va, blk_size);
rel_blk:
	ret = hd_common_mem_release_block(blk);
	if (HD_OK != ret) {
		printf("release blk fail, ret(%d)\n", ret);
		goto play_fclose;
	}

play_fclose:
	if (bs_fd != NULL) { fclose(bs_fd);}
	if (len_fd != NULL) { fclose(len_fd);}

	return 0;
}
#endif 


MAIN(argc, argv)
{
	HD_RESULT ret;
	INT key;
	VIDEO_LIVEVIEW stream[1] = {0}; //0: liveview stream
	VIDEO_RECORD stream2[1] = {0}; //0: record stream
	UINT32 stream_list[2] = {((UINT32)&stream[0]), ((UINT32)&stream2[0])};
	UINT32 out_type = 1;
	UINT32 enc_type = 0;


#ifdef AUDIO_OUT_ENABLE
	pthread_t out_thread_id;
	AUDIO_OUTONLY outonly = {0};
#endif

	// query program options
	if (argc >= 2) {
		out_type = atoi(argv[1]);
		printf("out_type %d\r\n", out_type);
		if(out_type > 2) {
			printf("error: not support out_type!\r\n");
			return 0;
		}
	}
    stream[0].hdmi_id=HD_VIDEOOUT_HDMI_1920X1080I60;//default
	if (argc >= 3) {
		enc_type = atoi(argv[2]);
		printf("enc_type %d\r\n", enc_type);
		if(enc_type > 2) {
			printf("error: not support enc_type!\r\n");
			return 0;
		}
	}
    stream2[0].enc_type = enc_type;

	// init hdal
	ret = hd_common_init(0);
	if (ret != HD_OK) {
		printf("common fail=%d\n", ret);
		goto exit;
	}

	// init memory
	ret = mem_init();
	if (ret != HD_OK) {
		printf("mem fail=%d\n", ret);
		goto exit;
	}

	// init all modules
	ret = init_module();
	if (ret != HD_OK) {
		printf("init fail=%d\n", ret);
		goto exit;
	}

	// open video_liveview modules (liveview)
	stream[0].proc_max_dim.w = VDO_SIZE_W; //assign by user
	stream[0].proc_max_dim.h = VDO_SIZE_H; //assign by user
	ret = open_module(&stream[0], &stream[0].proc_max_dim, CAP_OUT_FMT, out_type);
	if (ret != HD_OK) {
		printf("open fail=%d\n", ret);
		goto exit;
	}

	// open video_record modules (record)
	stream2[0].proc_max_dim.w = REC_SIZE_W; //assign by user
	stream2[0].proc_max_dim.h = REC_SIZE_H; //assign by user
	ret = open_module_2(&stream2[0], &stream2[0].proc_max_dim, CAP_OUT_FMT);
	if (ret != HD_OK) {
		printf("open fail=%d\n", ret);
		goto exit;
	}
#ifdef AUDIO_OUT_ENABLE
	//open output module
	outonly.sample_rate_max = HD_AUDIO_SR_48000; //assign by user
	ret = open_module_audio(&outonly);
	if(ret != HD_OK) {
		printf("open fail=%d\n", ret);
		goto exit;
	}

	//set audioout parameter
	outonly.sample_rate = HD_AUDIO_SR_48000; //assign by user
	ret = set_aout_param(outonly.out_ctrl, outonly.out_path, outonly.sample_rate);
	if (ret != HD_OK) {
		printf("set out fail=%d\n", ret);
		goto exit;
	}	

	//create output thread
	ret = pthread_create(&out_thread_id, NULL, playback_thread, (void *)&outonly);
	if (ret < 0) {
		printf("create playback thread failed");
		goto exit;
	}

	//start output module
	hd_audioout_start(outonly.out_path);	
#endif



	// create flow_thread
	ret = pthread_create(&stream2[0].flow_thread_id, NULL, flow_thread, (void *)stream_list);
	if (ret < 0) {
		printf("create flow thread failed");
		goto exit;
	}

	// query user key
	printf("Enter q to exit\n");
	printf("\r\nif you want to record 1, enter \"s\" to trigger !!\r\n");

	stream2[0].flow_run = FLOW_ON_OPEN;
	while (stream2[0].flow_run != 0) usleep(100); //wait unitl flow idle

	stream2[0].save_count = 0;


#if 1



if(1)
{
	stream2[0].sel_rec_size = 0;
	stream2[0].enc_type = 0;
	stream2[0].flow_run = FLOW_ON_REC; //start record
	while (stream2[0].flow_state != FLOW_ON_REC) usleep(100); //wait unitl flow record		
}
	while (1) {
		key = GETCHAR();
		if (key == 's') {
			if (stream2[0].flow_run == 0) { //flow is idle
				stream2[0].flow_run = FLOW_ON_REC; //start record
	            while (stream2[0].flow_state != FLOW_ON_REC) usleep(100); //wait unitl flow record
			} else { //flow is still under record
				stream2[0].flow_run = FLOW_ON_STOP; //stop record
	            while (stream2[0].flow_state != FLOW_ON_STOP) usleep(100); //wait unitl flow stop
	            while (stream2[0].flow_run != 0) usleep(100); //wait unitl flow idle
			}
		}

		if (key == 'q' || key == 0x3) {
#ifdef AUDIO_OUT_ENABLE
			outonly.out_exit = 1;
#endif
			// quit thread
			if((stream2[0].flow_state == FLOW_ON_STOP)||(stream2[0].flow_state == FLOW_ON_OPEN)){
    			break;
            } else{
                printf("stop record first\r\n");
            }
		}
#ifdef AUDIO_OUT_ENABLE


		if (key == 'p') {
			outonly.out_pause = 1;
		}
		if (key == 'r') {
			outonly.out_pause = 0;
		}		
#endif
		#if (DEBUG_MENU == 1)
		if (key == 'd') {
			// enter debug menu
			hd_debug_run_menu();
			printf("\r\nEnter q to exit, Enter d to debug\r\n");
		}
		#endif
	}

#else


#endif 	
	while (stream2[0].flow_run != 0) usleep(100); //wait unitl flow idle
	stream2[0].flow_run = FLOW_ON_CLOSE;
	while (stream2[0].flow_state != FLOW_ON_CLOSE) usleep(100); //wait unitl flow idle

	stream2[0].flow_quit = 1;
	// destroy save flow_thread
	pthread_join(stream2[0].flow_thread_id, NULL);

	//stop output module
	hd_audioout_stop(outonly.out_path);

exit:
	// close video_liveview modules (liveview)
	ret = close_module(&stream[0]);
	if (ret != HD_OK) {
		printf("close fail=%d\n", ret);
	}

	// close video_record modules (record)
	ret = close_module_2(&stream2[0]);
	if (ret != HD_OK) {
		printf("close fail=%d\n", ret);
	}

#ifdef AUDIO_OUT_ENABLE
	//close audio_output module
	ret = close_module_audio(&outonly);
	if(ret != HD_OK) {
		printf("close fail=%d\n", ret);
	}
#endif
	// uninit all modules
	ret = exit_module();
	if (ret != HD_OK) {
		printf("exit fail=%d\n", ret);
	}

	// uninit memory
	ret = mem_exit();
	if (ret != HD_OK) {
		printf("mem fail=%d\n", ret);
	}

	// uninit hdal
	ret = hd_common_uninit();
	if (ret != HD_OK) {
		printf("common fail=%d\n", ret);
	}

	return 0;
}

