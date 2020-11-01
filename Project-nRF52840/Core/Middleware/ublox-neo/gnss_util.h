/*
 * MIT License
 *
 * Copyright (c) 2020 <Duy Lion Tran>. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @file GNSS_Util.h
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#ifndef __GNSS_UTIL_H__
#define __GNSS_UTIL_H__

#ifdef __cplusplus
extern "C" {
#endif  

#include <string.h>
#include "gnss_type.h"

void gnss_parse_rmc(char *str_rmc, rmc_t *rmc_st); 
void gnss_parse_gns(char *str_gns, gns_t *gns_st);
void gnss_parse_gga(char *str_gga, gga_t *gga_st);
void gnss_parse_gll(char *str_gll, gll_t *gll_st);


#ifdef __cplusplus
}
#endif

#endif /* __GNSS_UTIL_H__ */