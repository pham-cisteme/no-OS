/***************************************************************************//**
 *   @file   common_data.h
 *   @brief  Defines common data to be used by the swiot1l project.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __COMMON_DATA_H__
#define __COMMON_DATA_H__

#include "platform_includes.h"
#include "ad74413r.h"
#include "swiot.h"
#include "max14906.h"
#include "adin1110.h"
#include "adt75.h"
#include "iio_trigger.h"

#define AD74413R_GPIO_TRIG_NAME "ad74413r-dev0"

extern struct no_os_uart_init_param uart_ip;
extern struct adin1110_init_param adin1110_ip;
extern const struct no_os_spi_init_param adin1110_spi_ip;
extern const struct no_os_spi_init_param ad74413r_spi_ip;
extern struct no_os_spi_init_param max14906_spi_ip;
extern struct swiot_iio_desc_init_param swiot_ip;
extern struct max14906_init_param max14906_ip;
extern struct ad74413r_init_param ad74413r_ip;
extern struct adt75_init_param adt75_ip;
extern const struct no_os_gpio_init_param adin1110_int_gpio_ip;
extern const struct no_os_gpio_init_param tx_perf_gpio_ip;
extern const struct no_os_gpio_init_param rx_perf_gpio_ip;
extern const struct no_os_gpio_init_param adin1110_rst_gpio_ip;
extern const struct no_os_gpio_init_param adin1110_swpd_ip;
extern const struct no_os_gpio_init_param adin1110_tx2p4_ip;
extern const struct no_os_gpio_init_param adin1110_mssel_ip;
extern const struct no_os_gpio_init_param adin1110_cfg1_ip;
extern const struct no_os_gpio_init_param adin1110_cfg0_ip;
extern const struct no_os_gpio_init_param adin1110_int_ip;
extern const struct no_os_gpio_init_param swiot_led1_ip;
extern const struct no_os_gpio_init_param swiot_led2_ip;
extern const struct no_os_gpio_init_param psu_gpio_ip;
extern struct no_os_gpio_init_param ad74413r_reset_ip;
extern const struct no_os_gpio_init_param ad74413r_ldac_ip;
extern const struct no_os_gpio_init_param ad74413r_irq_ip;
extern const struct no_os_gpio_init_param max14906_en_ip;
extern const struct no_os_gpio_init_param max14906_d1_ip;
extern const struct no_os_gpio_init_param max14906_d2_ip;
extern const struct no_os_gpio_init_param max14906_d3_ip;
extern const struct no_os_gpio_init_param max14906_d4_ip;
extern const struct no_os_gpio_init_param max14906_synch_ip;
extern const struct no_os_irq_init_param ad74413r_nvic_ip;

extern struct iio_trigger ad74413r_iio_trig_desc;
extern struct iio_trigger swiot_trigger;
extern struct iio_hw_trig_init_param ad74413r_gpio_trig_ip;
extern struct no_os_irq_init_param ad74413r_gpio_irq_ip;
extern struct iio_trigger ad74413r_iio_trig_desc;

#endif /* __COMMON_DATA_H__ */
