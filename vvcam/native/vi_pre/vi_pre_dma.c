#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
//#include <sys/types.h>

#include "vi_pre.h"
#include "vi_pre_ioctl.h"

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)


#define img_buf 0xf0000000

typedef struct {
   u32 dma_ctl0;
   u32 dma_ctl1;
   u32 dma_ctl2;
   u32 dma_ctl3;
   u32 dma_ctl4;
   u32 dma_ctl5;
   u32 dma_ctl6;
   u32 dma_ctl7;
   u32 dma_ctl8;
   u32 dma_ctl9;
   u32 dma_ctl10;
   u32 dma_ctl11;
   u32 dma_ctl12;
   u32 dma_ctl13;
   u32 dma_ctl14;
   u32 dma_ctl15;
   u32 dma_ctl16;
   u32 dma_ctl17;
   u32 dma_ctl18;
   u32 dma_ctl19;
   u32 dma_ctl20;
   u32 dma_ctl21;
   u32 dma_ctl22;
   u32 dma_ctl23;
   u32 dma_ctl24;
   u32 dma_ctl25;
   u32 dma_ctl26;
   u32 dma_ctl27;
   u32 dma_ctl28;
   u32 dma_ctl29;
   u32 dma_ctl30;
   u32 dma_ctl31;
   u32 dma_ctl32;
   u32 dma_ctl33;
   u32 dma_ctl34;
   u32 dma_ctl35;
   u32 dma_ctl36;
   u32 dma_ctl37;
   u32 dma_ctl38;
   u32 dma_ctl39;
   u32 dma_ctl40;
   u32 dma_ctl41;
   u32 dma_ctl42;
   u32 dma_ctl43;
   u32 dma_ctl44;
   u32 dma_ctl45;
   u32 dma_ctl46;
   u32 dma_ctl47;
   u32 dma_ctl48;
   u32 dma_ctl49;
   u32 dma_ctl50;
   u32 dma_ctl51;
} vi_dma_reg_t;

typedef enum {
    VI_PRE_RAW_6BIT,
    VI_PRE_RAW_7BIT,
    VI_PRE_RAW_8BIT,
    VI_PRE_RAW_10BIT_16ALIGN,
    VI_PRE_RAW_12BIT,
    VI_PRE_RAW_10BIT,
} vi_pre_data_width_t;

typedef enum {
    VI_PRE_M_FRAME,
    VI_PRE_N_LANE,
} vi_pre_dma_mode_t;

typedef struct {
    int vc_num;         //1~3
    vi_pre_data_width_t width;
    vi_pre_dma_mode_t mode;
    u32 resolution_h;
    u32 resolution_v;
    u32 line_size; //Data size of one line, unit byte; Must be 256-byte aligned
    u32 num;       //n line num, or m frame num(m frme max is 4)
    u32 n_line_int;//only used for nline mode.(Generate an interrupt every time when n lines are completed). n_line_int <= num
    u32 buf_size;
    u8 *buf;
} vi_pre_dma_cfg_t;

static vi_dma_reg_t dma_reg = {
    .dma_ctl0 = 0x198,
    .dma_ctl1 = 0x19c,
    .dma_ctl2 = 0x1a0,
    .dma_ctl3 = 0x1a4,
    .dma_ctl4 = 0x1a8,
    .dma_ctl5 = 0x1ac,
    .dma_ctl6 = 0x1b0,
    .dma_ctl7 = 0x1b4,
    .dma_ctl8 = 0x1b8,
    .dma_ctl9 = 0x1bc,

    .dma_ctl10 = 0x1c0,
    .dma_ctl11 = 0x1c4,
    .dma_ctl12 = 0x1c8,
    .dma_ctl13 = 0x1cc,
    .dma_ctl14 = 0x1d0,
    .dma_ctl15 = 0x1d4,
    .dma_ctl16 = 0x1d8,
    .dma_ctl17 = 0x1dc,
    .dma_ctl18 = 0x1e0,
    .dma_ctl19 = 0x1e4,

    .dma_ctl20 = 0x1e8,
    .dma_ctl21 = 0x1ec,
    .dma_ctl22 = 0x1f0,
    .dma_ctl23 = 0x1f4,
    .dma_ctl24 = 0x1f8,
    .dma_ctl25 = 0x1fc,
    .dma_ctl26 = 0x200,
    .dma_ctl27 = 0x204,
    .dma_ctl28 = 0x208,
    .dma_ctl29 = 0x20c,

    .dma_ctl30 = 0x210,
    .dma_ctl31 = 0x214,
    .dma_ctl32 = 0x218,
    .dma_ctl33 = 0x21c,
    .dma_ctl34 = 0x220,
    .dma_ctl35 = 0x224,
    .dma_ctl36 = 0x228,
    .dma_ctl37 = 0x22c,
    .dma_ctl38 = 0x230,
    .dma_ctl39 = 0x234,

    .dma_ctl40 = 0x238,
    .dma_ctl41 = 0x23c,
    .dma_ctl42 = 0x240,
    .dma_ctl43 = 0x244,
    .dma_ctl44 = 0x248,
    .dma_ctl45 = 0x24c,
    .dma_ctl46 = 0x250,
    .dma_ctl47 = 0x254,
    .dma_ctl48 = 0x258,
    .dma_ctl49 = 0x25c,

    .dma_ctl50 = 0x260,
    .dma_ctl51 = 0x264,
};

static inline void vi_pre_dma_write(struct vi_pre_dev *dev,
                     u32 address, u32 data)
{
    writel(data, dev->reg_base + address);
}

static inline u32 vi_pre_dma_read(struct vi_pre_dev *dev,
                     u32 address)
{
    return readl(dev->reg_base + address);
}

static void set_dma_mode(struct vi_pre_dev *dev, vi_pre_dma_mode_t mode)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl0);

    if(mode == VI_PRE_M_FRAME) {
        reg_val &= ~1;
    } else {
        reg_val |= 1;
    }

    vi_pre_dma_write(dev, dma_reg.dma_ctl0, reg_val);
}

static void set_frame_num(struct vi_pre_dev *dev, u32 num)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl0);
    num -= 1;
    reg_val &= ~(3 << 1);
    reg_val |= (num << 1);

    vi_pre_dma_write(dev, dma_reg.dma_ctl0, reg_val);
}

static int get_frame_num(struct vi_pre_dev *dev)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl0);
    reg_val &= (3 << 2);
    return (reg_val >> 2) + 1;
}


/*big align ro little align*/
static void set_bit_mode(struct vi_pre_dev *dev, bool is_big)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl0);

    if (is_big) {
        reg_val |= (1 << 3);
    } else {
        reg_val &= ~(1 << 3);
    }

    vi_pre_dma_write(dev, dma_reg.dma_ctl0, reg_val);
}

static void set_dma_data_width(struct vi_pre_dev *dev, vi_pre_data_width_t width)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl0);
    reg_val &= ~(0x7 <<4);
    reg_val |= (width << 4);
    vi_pre_dma_write(dev, dma_reg.dma_ctl0, reg_val);
}

/*length is 4, 8, 16*/
static void set_dma_burst_length(struct vi_pre_dev *dev, u8 len)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl1);
    len -=1;
    reg_val &= ~(0xff << 16);
    reg_val |= (len << 16);
    reg_val |= 0x3ff;
    vi_pre_dma_write(dev, dma_reg.dma_ctl1, reg_val);
}

static u32 read_dma_burst_length(struct vi_pre_dev *dev)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl1);
    return ((reg_val >> 16) & 0xff) + 1;
}

static void set_dma_resolution(struct vi_pre_dev *dev, u16 h, u16 v)
{
    u32 reg_val = 0;
    h &= 0x1fff;
    v &= 0x1fff;
    reg_val = h | (v << 16);

    vi_pre_dma_write(dev, dma_reg.dma_ctl3, reg_val);
}

static int set_nline_oneline_size(struct vi_pre_dev *dev,  vi_pre_dma_cfg_t *cfg)
{
    u32 reg_val = 0;
    u32 stream_min = 0;
    u32 cnt_128 = 0;
    u32 read_num = 0;
    u32 burstrem = 0;

    stream_min = cfg->resolution_h;
    if (cfg->width == VI_PRE_RAW_12BIT ||
        cfg->width == VI_PRE_RAW_10BIT_16ALIGN) {
        stream_min = cfg->resolution_h * 2;
    }

    if (stream_min > cfg->line_size) {
        return -1;
    }

    if (stream_min % 16) {
        stream_min += 16 - (stream_min % 16);
    }

    cnt_128 = stream_min / 16;
    read_num = cnt_128 / read_dma_burst_length(dev);
    burstrem = cnt_128 % read_dma_burst_length(dev);

    if (cnt_128 % read_dma_burst_length(dev)) {
        read_num +=1;
    }

    if (burstrem == 0) {
        burstrem = read_dma_burst_length(dev);
    }

    reg_val = cnt_128 | (read_num << 16) | (burstrem << 27);
    vi_pre_dma_write(dev, dma_reg.dma_ctl4, reg_val);
    vi_pre_dma_write(dev, dma_reg.dma_ctl6, cfg->line_size);
    return 0;
}

static void set_nline_period(struct vi_pre_dev *dev, u32 num)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl5);
    reg_val &= ~0xfff;
    reg_val |= num;
    printk("vipre nline period num %d, reg_val 0x%x\n", num, reg_val);
    vi_pre_dma_write(dev, dma_reg.dma_ctl5, reg_val);
}

static void set_nline_int_period(struct vi_pre_dev *dev, u32 num)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl5);
    reg_val &= ~(0x7ff << 16);
    reg_val |= (num << 16);
    printk("vipre nline int period num %d, reg_val 0x%x\n", num, reg_val);
    vi_pre_dma_write(dev, dma_reg.dma_ctl5, reg_val);
}

static u16 get_nline_period(struct vi_pre_dev *dev)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl5);

    return reg_val & 0xfff;
}

static u16 get_nline_int_period(struct vi_pre_dev *dev)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl5);

    return (reg_val >> 16) & 0xfff;
}

static int set_mfram_oneline_range(struct vi_pre_dev *dev, int frame_id, u32 size)
{
    u32 aligned = 0;
    u32 reg_val = 0;
    u32 reg = 0;

    aligned = read_dma_burst_length(dev) * 16;
    /*set buf addr*/
    if(size % aligned) {
        printk("<0>""###########error !!!!!!!!! %s, %d\n", __func__, __LINE__);
        return -1;
    }

    reg = frame_id / 2 * 4 + dma_reg.dma_ctl7;
    reg_val = vi_pre_dma_read(dev, reg);
    if (frame_id % 2 == 0) {
        reg_val &= ~(0xffff << 16);
        reg_val |= (size << 16);
    } else {
        reg_val &= ~0xffff;
        reg_val |= size;
    }

    vi_pre_dma_write(dev, reg, reg_val);
    return 0;
}

static int set_mframe_oneline_group(struct vi_pre_dev *dev, vi_pre_dma_cfg_t *cfg)
{
    u32 reg_val = 0;
    u32 stream_min = 0;
    u32 cnt_128 = 0;
    u32 read_num = 0;
    u32 burstrem = 0;

    stream_min = cfg->resolution_h;
    if (cfg->width == VI_PRE_RAW_12BIT ||
        cfg->width == VI_PRE_RAW_10BIT_16ALIGN) {
        stream_min = cfg->resolution_h * 2;
    }

    if (stream_min > cfg->line_size) {
        return -1;
    }

    if (stream_min % 16) {
        stream_min += 16 - (stream_min % 16);
    }

    cnt_128 = stream_min / 16;
    read_num = cnt_128 / read_dma_burst_length(dev);
    burstrem = cnt_128 % read_dma_burst_length(dev);

    if (cnt_128 % read_dma_burst_length(dev)) {
        read_num +=1;
    }

    if (burstrem == 0) {
        burstrem = read_dma_burst_length(dev);
    }

    reg_val = cnt_128 | (read_num << 16) | (burstrem << 27);
    vi_pre_dma_write(dev, dma_reg.dma_ctl4, reg_val);

    return 0;
}

static void dma_reset(struct vi_pre_dev *dev)
{
    vi_pre_dma_write(dev, dma_reg.dma_ctl9, 1);
    while(vi_pre_dma_read(dev, dma_reg.dma_ctl9) == 1);
}

static void dma_enable(struct vi_pre_dev *dev, int en)
{
    if(en) {
        vi_pre_dma_write(dev, dma_reg.dma_ctl10, 1);
    } else {
        vi_pre_dma_write(dev, dma_reg.dma_ctl10, 0);
    }
}

static void dma_period_clear(struct vi_pre_dev *dev, int ch)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl11);
    if (reg_val & (1 << ch)) {
        reg_val &= ~(1 << ch);
    } else {
        reg_val |= (1 << ch);
    }

    vi_pre_dma_write(dev, dma_reg.dma_ctl11, reg_val);
}

static void nline_mode_set_line_position(struct vi_pre_dev *dev, int ch, int p)
{
    u32 reg = dma_reg.dma_ctl12 + ch * 4;
    vi_pre_dma_write(dev, reg, p);
}

static void nline_mode_set_start_addr(struct vi_pre_dev *dev, int ch, unsigned long addr)
{
    u32 reg = dma_reg.dma_ctl15 + ch * 8;
    u8 h = addr >> 32;
    u32 l = addr & 0xffffffff;
    vi_pre_dma_write(dev, reg, h);
    vi_pre_dma_write(dev, reg + 4, l);
}

static u16 nline_mode_period_cnt(struct vi_pre_dev *dev, int ch)
{
    u32 reg = dma_reg.dma_ctl21 + ch / 2 * 4;

    if(ch % 2) {
        return (vi_pre_dma_read(dev, reg) >> 16) & 0xfff;
    } else {
        return vi_pre_dma_read(dev, reg) & 0xfff;
    }
}

static void mframe_mode_set_position(struct vi_pre_dev *dev, int ch, int f, int l)
{
    u32 reg = dma_reg.dma_ctl23 + ch / 2 * 4;
    u32 reg_val = vi_pre_dma_read(dev, reg);

    f &= 3;
    l &= 0xfff;

    if (ch % 2) {
        reg_val &=~(0xffff << 16);
        reg_val |= (l << 16) | (f << 29);
    } else {
        reg_val &=~0xffff;
        reg_val |= l | (f << 13);
    }

    vi_pre_dma_write(dev, reg, reg_val);
}

static void mframe_set_start_addr(struct vi_pre_dev *dev, int frame_id, int vc_ch,  unsigned long addr)
{
    u32 reg = 0;
    u8 h = addr >> 32;
    u32 l = addr & 0xffffffff;

    if (frame_id <= 2) {
        reg = dma_reg.dma_ctl25 + frame_id * 24 + vc_ch * 8;
    } else {
        reg = dma_reg.dma_ctl45 + vc_ch * 8;
    }


    vi_pre_dma_write(dev, reg, h);
    vi_pre_dma_write(dev, reg + 4, l);
}

static u32 mframe_frame_done_flag(struct vi_pre_dev *dev)
{
    return vi_pre_dma_read(dev, dma_reg.dma_ctl43) & 0xfff;
}

static void mframe_frame_done_flag_clear(struct vi_pre_dev *dev, u32 mask)
{

     u32 reg_val  = vi_pre_dma_read(dev, dma_reg.dma_ctl43);
     reg_val |= mask;

     vi_pre_dma_write(dev, dma_reg.dma_ctl43, mask);
}


static void dma_interrupt_set(struct vi_pre_dev *dev, u32 mask, int en)
{
    u32 reg_val = vi_pre_dma_read(dev, dma_reg.dma_ctl44);
    if (en) {
        reg_val &= ~(mask << 16);
    } else {
        reg_val |= (mask << 16);
    }

    vi_pre_dma_write(dev, dma_reg.dma_ctl44, reg_val);
}

static u16 dma_interrupt_status(struct vi_pre_dev *dev)
{
    return vi_pre_dma_read(dev, dma_reg.dma_ctl44) & 0xffff;
}

static void dma_clear_interrupt_status(struct vi_pre_dev *dev, u32 sta)
{

    u32 reg = vi_pre_dma_read(dev, dma_reg.dma_ctl44);
    reg |= sta;
    return vi_pre_dma_write(dev, dma_reg.dma_ctl44, reg);
}

static int vi_pre_mframe_config(struct vi_pre_dev *dev, vi_pre_dma_cfg_t *cfg)
{
    unsigned long buf_base = (unsigned long)cfg->buf;
    int ret = 0;
    int i = 0;
    int j = 0;

    if (cfg->num > 4) {
        return -1;
    }

    if (cfg->buf_size < cfg->line_size * cfg->resolution_v * cfg->num) {
        return -1;
    }

    set_frame_num(dev, cfg->num);

    ret = set_mframe_oneline_group(dev, cfg);
    if (ret != 0) {
        printk("<0>""!!!!!!!set mframe oneline size error!!!!!!!!!!!!!!!!\n");
        return -1;
    }

    for(i = 0; i < cfg->num; i++) {
        ret = set_mfram_oneline_range(dev, i, cfg->line_size);
        if (ret != 0) {
            printk("<0>""!!!!!!!set mframe oneline range error!!!!!!!!!!!!!!!!\n");
            return -1;
        }
        for (j = 0; j < cfg->vc_num; j++) {
            mframe_set_start_addr(dev, i, j, buf_base);
            buf_base += cfg->line_size * cfg->resolution_v;
        }
    }

    return 0;
}

static int vi_pre_nline_config(struct vi_pre_dev *dev, vi_pre_dma_cfg_t *cfg)
{
    unsigned long buf_base = (unsigned long)cfg->buf;
    int i = 0;

    if (cfg->line_size % 256) {
        return -1;
    }

    if (cfg->buf_size < cfg->line_size * cfg->num) {
        return -1;
    }

    if (set_nline_oneline_size(dev, cfg) != 0) {
        return -1;
    }

    set_nline_period(dev, cfg->num);
    set_nline_int_period(dev, cfg->n_line_int);
    for(i = 0; i < cfg->vc_num; i++) {
        nline_mode_set_start_addr(dev, i, buf_base);
        buf_base += cfg->line_size * cfg->resolution_v;
    }

    return 0;
}

int vi_pre_dma_init(struct vi_pre_dev *pdriver_dev)
{
    return 0;
}

int vi_pre_dma_uninit(struct platform_device *pdev)
{
    struct vi_pre_dev *pdriver_dev;
    pdriver_dev = platform_get_drvdata(pdev);
    memset(pdriver_dev->cnt, 0, sizeof(pdriver_dev->cnt));
    return 0;
}

static int dma_config(struct vi_pre_dev *pdriver_dev, vi_pre_dma_cfg_t *cfg)
{
    int ret = 0;
    set_dma_burst_length(pdriver_dev, 16);
    set_dma_data_width(pdriver_dev, cfg->width);
    set_bit_mode(pdriver_dev, 1);
    set_dma_mode(pdriver_dev, cfg->mode);
    set_dma_resolution(pdriver_dev, cfg->resolution_h, cfg->resolution_v);

    if (cfg->mode == VI_PRE_M_FRAME) {
        ret = vi_pre_mframe_config(pdriver_dev, cfg);
        pdriver_dev->is_mframe_mode  = true;
    } else if (cfg->mode == VI_PRE_N_LANE) {
        ret = vi_pre_nline_config(pdriver_dev, cfg);
        pdriver_dev->is_mframe_mode  = false;
    }

    return ret;
}

int vi_pre_dma_config(struct vi_pre_dev *pdriver_dev, void *arg)
{
    vi_pre_dma_cfg_t cfg;
	check_retval(copy_from_user(&cfg, arg, sizeof(cfg)));
    //cfg.buf_size = cfg.num * cfg.vc_num * cfg.line_size * cfg.resolution_v;
    //cfg.buf = (u8 *)img_buf;

    return dma_config(pdriver_dev, &cfg);
}

#define VIPRE_BUS_ERR (1 << 5)
#define VIPRE_FIFO_OVER (1 << 4)
#define VIPRE_IDLE_DONE (1 << 3)
#define VIPRE_NMOVERFLOW (1 << 2)
#define VIPRE_LINE_DONE (1 << 1)
#define VIPRE_FRAME_DONE (1 << 0)

int vi_pre_dma_start(struct vi_pre_dev *pdriver_dev)
{
    u32 mask =  VIPRE_BUS_ERR | VIPRE_FIFO_OVER | VIPRE_NMOVERFLOW;

    if (pdriver_dev->is_mframe_mode) {
        mask |= VIPRE_FRAME_DONE;
    } else {
        mask |= VIPRE_LINE_DONE;
    }

    dma_interrupt_set(pdriver_dev, mask, 1);
    dma_enable(pdriver_dev, 1);

    return 0;
}

int vi_pre_dma_stop(struct vi_pre_dev *pdriver_dev)
{
    u32 mask =  VIPRE_BUS_ERR | VIPRE_FIFO_OVER | VIPRE_NMOVERFLOW;
    dma_interrupt_set(pdriver_dev, mask, 0);
    dma_enable(pdriver_dev, 0);
    udelay(1000);
    return 0;
}

static int vipre_event = 0;

int *vi_pre_event(void)
{
    return &vipre_event;
}

void vi_pre_dma_interrupt_handler(struct vi_pre_dev *pdriver_dev)
{
    u32 status = 0;
    u32 f_done_sta = 0;
    unsigned long flags = 0;
    //int i = 0;

    status = dma_interrupt_status(pdriver_dev);
    //printk("<0>""vipre int sta %x\n", status );
    if (status & (1 << 4)) {
        printk("<0>""vipre err int sta 0x%x\n", status);
    }
    dma_clear_interrupt_status(pdriver_dev, status);
    spin_lock_irqsave(&pdriver_dev->slock, flags);
    if (status & VIPRE_FRAME_DONE) {
        pdriver_dev->cnt++;
        if (pdriver_dev->cnt >= 4) {
            pdriver_dev->cnt = 0;
            vipre_event = 1;
        }
        f_done_sta = mframe_frame_done_flag(pdriver_dev);
        //printk("<0>""f_done_sta int sta 0x%x\n", f_done_sta);
        mframe_frame_done_flag_clear(pdriver_dev, f_done_sta);
        //vi_pre_dma_stop(pdriver_dev);
        /*TODO send event*/
        //printk("frame done %x\n", f_done_sta);
    } else if (status & VIPRE_LINE_DONE) {
        //u32 nline_done_sta =;
        //pdriver_dev->cnt++;
        //printk("line done %x\n", f_done_sta);
        vipre_event = 1;
    }

#if 0
    //u32 line_cnt = pdriver_dev->cnt * get_nline_int_period(pdriver_dev);

    if (pdriver_dev->is_mframe_mode) {
        for(i = 0; i < 3; i++) {
            if(pdriver_dev->cnt[i] < get_frame_num(pdriver_dev)) {
                continue;
            }
            pdriver_dev->cnt[i] = 0;
            dma_period_clear(pdriver_dev, i);
        }
    }
    else (pdriver_dev->is_mframe_mode == 0 && line_cnt >= get_nline_period(pdriver_dev)){
        dma_period_clear(struct vi_pre_dev *dev, int ch)
    }
#endif
    spin_unlock_irqrestore(&pdriver_dev->slock, flags);
}

int vi_pore_vc_bind(struct platform_device *pdev, u8 buf_id, u8 vc_id)
{

    return 0;
}

int vi_pore_attach_callback(struct platform_device *pdev)
{

    return 0;
}

int vi_pore_deattach_callback(struct platform_device *pdev)
{

    return 0;
}

/////////////////demo///////////////////////
void test(struct vi_pre_dev *pdev)
{
    vi_pre_dma_cfg_t cfg;
    cfg.vc_num = 1;
    cfg.width = VI_PRE_RAW_12BIT;
    cfg.mode = VI_PRE_M_FRAME;
    cfg.resolution_h = 1944;  //1920;//3840;
    cfg.resolution_v = 1100;  //1080;//2160;
    cfg.line_size = 2048 * 2; //必须256byte对齐
    cfg.num = 4;              //n line num, or m frame num(m frme max is 4)
    cfg.buf_size = 2048 * 1100 * 2 * 10;
    cfg.buf = (u8 *)img_buf;

    dma_config(pdev, &cfg);
    vi_pre_dma_start(pdev);
}
