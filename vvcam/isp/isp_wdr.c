/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************
 *
 * The GPL License (GPL)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 *****************************************************************************
 *
 * Note: This software is released under dual MIT and GPL licenses. A
 * recipient may use this file under the terms of either the MIT license or
 * GPL License. If you wish to use only one license not the other, you can
 * indicate your decision by deleting one of the above license notices in your
 * version of this file.
 *
 *****************************************************************************/
#include "isp_wdr.h"
#include "mrv_all_bits.h"
#include "isp_ioctl.h"
#include "isp_types.h"
#include "ic_dev.h"

extern MrvAllRegister_t *all_regs;


int isp_s_wdr(struct isp_ic_dev *dev)
{
    isp_wdr_context_t* wdr = &dev->wdr;

    pr_info(" enter %s\n", __func__);

    /*update wdr configuration  after frame end when isp enable*/
    if (!is_isp_enable(dev) || wdr->changed) {
        uint32_t isp_wdr_offset, isp_wdr_ctrl;

        isp_wdr_offset = isp_read_reg(dev, REG_ADDR(isp_wdr_offset));
        REG_SET_SLICE( isp_wdr_offset, MRV_WDR_LUM_OFFSET, wdr->LumOffset );
        REG_SET_SLICE( isp_wdr_offset, MRV_WDR_RGB_OFFSET, wdr->RgbOffset );
    	isp_write_reg(dev, REG_ADDR(isp_wdr_offset), isp_wdr_offset);

        isp_wdr_ctrl = isp_read_reg(dev, REG_ADDR(isp_wdr_ctrl));
    	REG_SET_SLICE(isp_wdr_ctrl, MRV_WDR_ENABLE, wdr->enabled);
    	isp_write_reg(dev, REG_ADDR(isp_wdr_ctrl), isp_wdr_ctrl);
        wdr->changed = false;
    } else {
        wdr->changed = true;
    }
    pr_info(" %s wdr.changed %d \n", __func__, wdr->changed);
   	return 0;
}

int isp_s_wdr_curve(struct isp_ic_dev *dev)
{

    isp_wdr_context_t* wdr = &dev->wdr;

    int i, j;
    uint32_t dYi = 0U;
    for ( i=0; i<4; i++ )
    {
        for ( j=8; j>0; j-- )
        {
            dYi <<= 4;
            dYi += wdr->dY[ (i*8 + j) ];
        }

        if (i == 0)
        {

            isp_write_reg(dev, REG_ADDR(isp_wdr_tonecurve_1), dYi);
        }
        else if (i == 1)
        {
            isp_write_reg(dev, REG_ADDR(isp_wdr_tonecurve_2), dYi);
        }
        else if (i == 2)
        {
            isp_write_reg(dev, REG_ADDR(isp_wdr_tonecurve_3), dYi);
        }
        else  /* if (i == 3) */
        {
            isp_write_reg(dev, REG_ADDR(isp_wdr_tonecurve_4), dYi);
        }
    }

    for ( i=0; i<33; i++ )
    {
        isp_write_reg(dev, REG_ADDR(wdr_tone_mapping_curve_y_block_arr[i]), wdr->Ym[i]);
    }

    dYi = 0x00000000;

    isp_write_reg(dev, REG_ADDR(isp_wdr_offset), dYi);
    isp_write_reg(dev, REG_ADDR(isp_wdr_deltamin), 0x00100000);

    return 0;

}


