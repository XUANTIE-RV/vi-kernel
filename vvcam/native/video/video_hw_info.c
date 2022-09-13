
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/reset.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include "video.h"
#include "video_kernel_defs.h"
#include "video.h"


typedef struct describe_comm{
	unsigned id;
	char *desc;
}describe_comm_t;

static int get_id_by_desc( describe_comm_t *,const char* );

#define ENUM_DESC_MAPS_DECLATR(type) static describe_comm_t describe_map_##type[]={
#define ENUM_DESC_MAPS_ADD(str)  {str,#str},
#define ENUM_DESC_MAPS_END {-1,"end"}};
#define GET_ENUM_BY_DESC(type,str) get_id_by_desc(describe_map_##type,str)

/****vedio_ip_type_e  string to enum map*/
ENUM_DESC_MAPS_DECLATR(subdev)
    {SENSOR, "vivcam"},
    {VIPRE, "vipre"},
    {ISP, "isp"},
    {DW, "dw"},
    {DSP, "dsp"},
    {RY, "ry"},
ENUM_DESC_MAPS_END

/****sensor_path_type_e  string to enum map*/
ENUM_DESC_MAPS_DECLATR(sensor)
ENUM_DESC_MAPS_ADD(SENSOR_VGA_RAW12_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_1080P_RAW12_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_4K_RAW12_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_VGA_RAW10_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_1080P_RAW10_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_4K_RAW10_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_2592x1944_RAW10_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_1280x720_RAW10_LINER)
ENUM_DESC_MAPS_ADD(SENSOR_VGA_RAW12_HDR_2DOL)
ENUM_DESC_MAPS_ADD(SENSOR_VGA_RAW12_HDR_3DOL)
ENUM_DESC_MAPS_END

/****isp_path_type_e  string to enum map*/
ENUM_DESC_MAPS_DECLATR(isp)
ENUM_DESC_MAPS_ADD(ISP_MI_PATH_MP)
ENUM_DESC_MAPS_ADD(ISP_MI_PATH_SP)
ENUM_DESC_MAPS_ADD(ISP_MI_PATH_SP2_BP)
#ifdef ISP_MI_MCM_WR
ENUM_DESC_MAPS_ADD(ISP_MI_MCM_WR0)
ENUM_DESC_MAPS_ADD(ISP_MI_MCM_WR1)
#endif
ENUM_DESC_MAPS_ADD(ISP_MI_PATH_PP)
ENUM_DESC_MAPS_ADD(ISP_MI_HDR_L)
ENUM_DESC_MAPS_ADD(ISP_MI_HDR_S)
ENUM_DESC_MAPS_ADD(ISP_MI_HDR_VS)
ENUM_DESC_MAPS_END

/****vipre_path_type_e  string to enum map*/
ENUM_DESC_MAPS_DECLATR(vipre)
ENUM_DESC_MAPS_ADD(VIPRE_CSI0_DDR)
ENUM_DESC_MAPS_ADD(VIPRE_CSI1_DDR)
ENUM_DESC_MAPS_ADD(VIPRE_CSI2_DDR)
ENUM_DESC_MAPS_ADD(VIPRE_CSI0_ISP0)
ENUM_DESC_MAPS_ADD(VIPRE_CSI1_ISP0)
ENUM_DESC_MAPS_ADD(VIPRE_CSI2_ISP0)
ENUM_DESC_MAPS_ADD(VIPRE_CSI0_ISP1)
ENUM_DESC_MAPS_ADD(VIPRE_CSI1_ISP1)
ENUM_DESC_MAPS_ADD(VIPRE_CSI2_ISP1)
ENUM_DESC_MAPS_ADD(VIPRE_CSI0_LOW_COAST_HDR_ISP0)
ENUM_DESC_MAPS_ADD(VIPRE_CSI1_LOW_COAST_HDR_ISP0)
ENUM_DESC_MAPS_ADD(VIPRE_CSI2_LOW_COAST_HDR_ISP0)
ENUM_DESC_MAPS_END

/****ry_path_type_e  string to enum map*/
ENUM_DESC_MAPS_DECLATR(ry)
ENUM_DESC_MAPS_ADD(ISP_RY_MI_PATH_MP)
ENUM_DESC_MAPS_ADD(ISP_RY_MI_PATH_SP)
ENUM_DESC_MAPS_ADD(ISP_RY_MI_PATH_SP2_BP)
ENUM_DESC_MAPS_END

/****dsp_path_type_e  string to enum map*/

ENUM_DESC_MAPS_DECLATR(dsp)
ENUM_DESC_MAPS_ADD(DSP_PATH_ISP_RY)
ENUM_DESC_MAPS_ADD(DSP_PATH_ISP_CPU)
ENUM_DESC_MAPS_ADD(DSP_PATH_VIPRE_DDR)
ENUM_DESC_MAPS_ADD(DSP_PATH_VIPRE_EVEN)
ENUM_DESC_MAPS_ADD(DSP_PATH_VIPRE_ODD)
ENUM_DESC_MAPS_ADD(DSP_PATH_VIPRE_RY)
ENUM_DESC_MAPS_END

ENUM_DESC_MAPS_DECLATR(dw)
ENUM_DESC_MAPS_ADD(DW_DWE_VSE0)
ENUM_DESC_MAPS_ADD(DW_DWE_VSE1)
ENUM_DESC_MAPS_ADD(DW_DWE_VSE2)
ENUM_DESC_MAPS_END
static int get_id_by_desc(describe_comm_t* desc_list,const char* str)
 {
     int i=0;
     while(0!=strcmp(desc_list[i].desc,"end") )
     {
         if(0==strcmp(desc_list[i].desc,str))
         {
             return desc_list[i].id;
         }
         i++;
     }
     return -1;
 }

vedio_ip_type_e convert_subdev_name_to_id(const char* name)
{
     return GET_ENUM_BY_DESC(subdev,name);
}

int vedio_get_path_type(const char*subdev_name,const char* path_name)
{
    vedio_ip_type_e ip_type = GET_ENUM_BY_DESC(subdev,subdev_name);
    switch(ip_type)
    {
        case SENSOR:
                return GET_ENUM_BY_DESC(sensor,path_name);
        case VIPRE:
                return GET_ENUM_BY_DESC(vipre,path_name);
        case ISP:
                return GET_ENUM_BY_DESC(isp,path_name);
        case DW:
                return GET_ENUM_BY_DESC(dw,path_name);
        case DSP:
                return GET_ENUM_BY_DESC(dsp,path_name);
        case RY:
                return GET_ENUM_BY_DESC(ry,path_name);
        default:
            // printk("No match sub device name\n");
            return -1;
    }
}





