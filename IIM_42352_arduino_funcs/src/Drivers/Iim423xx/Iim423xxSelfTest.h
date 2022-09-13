/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/** @defgroup DriverIim423xxSelfTest Iim423xx selftest
 *  @brief Low-level function to run selftest on a Iim423xx device
 *  @ingroup  DriverIim423xx
 *  @{
 */

/** @file Iim423xxSelfTest.h
 * Low-level function to run selftest on a Iim423xx device
 */

#ifndef _INV_IIM423XX_SELFTEST_H_
#define _INV_IIM423XX_SELFTEST_H_

#include "InvExport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
struct inv_iim423xx;

/**
*  @brief      Perform hardware self-test for Accel
*  @param[in]  result containing ACCEL_SUCCESS<<1 so 2
*  @return     0 if success, error code if failure
*/
int inv_iim423xx_run_selftest(struct inv_iim423xx * s, int * result);

/**
*  @brief      Retrieve bias collected by self-test.
*  @param[out] st_bias bias scaled by 2^16, accel is gee.
*                      The buffer will be filled as below.
*                      Accel LN mode X,Y,Z
*  @return     0 if success, error code if failure
*/
int inv_iim423xx_get_st_bias(struct inv_iim423xx * s, int st_bias[6]);

/**
*  @brief      Apply bias.
*  @param[in] st_bias bias scaled by 2^16, accel is gee.
*                      The buffer must be filled as below.
*                      Accel LN mode X,Y,Z
*  @return     0 if success, error code if failure
*/
int inv_iim423xx_set_st_bias(struct inv_iim423xx * s, const int st_bias[6]);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IIM423XX_SELFTEST_H_ */

/** @} */
