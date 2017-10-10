/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <arch/byteorder.h>
#include <arch/board/board.h>

#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/power/pm.h>
#include <nuttx/time.h>
#include <nuttx/wqueue.h>
#include <arch/board/mods.h>

#include "hdk.h"
#include "stm32_exti.h"

#define LED_ON              0
#define LED_OFF             1

#ifdef CONFIG_STM32_ADC1
/* The number of ADC channels in the conversion list */
#define ADC1_NCHANNELS 1

/* Identifying number of each ADC channel
 * Temperature output is available on PC3, which corresponds
 * to ADC1 channel 4 (GPIO_ADC1_IN4).
 */
static const uint8_t  g_chanlist[ADC1_NCHANNELS] = {4};

/* Configurations of pins used by each ADC channel */
static const uint32_t g_pinlist[ADC1_NCHANNELS]  = {GPIO_ADC1_IN4};
#endif

static struct work_s data_report_work; /* work queue for data reporting */

struct temp_raw_info {
    struct device *gDevice;         /* device handle for this driver */
    uint16_t interval;              /* reporting interval */
    raw_send_callback gCallback;    /* callback to send back messages */
    uint8_t client_verified;        /* flag to indicate client was verified */
};

struct project_info {
    struct device *gDevice;
    raw_send_callback gCallback;
};

/**
 * Initialize ADC1 interface and register the ADC driver.
 */
static int adc_devinit(void)
{
#ifdef CONFIG_STM32_ADC1
    static bool initialized = false;
    struct adc_dev_s *adc;
    int ret;
    int i;

    /* Check if we have already initialized */
    if (!initialized)
    {
        /* Configure the pins as analog inputs for the selected channels */
        for (i = 0; i < ARRAY_SIZE(g_pinlist); i++)
        {
            stm32_configgpio(g_pinlist[i]);
        }

        /* Call stm32_adcinitialize() to get an instance of the ADC interface */
        adc = stm32_adcinitialize(1, g_chanlist, ARRAY_SIZE(g_chanlist));
        if (adc == NULL)
        {
            adbg("ERROR: Failed to get ADC interface\n");
            return -ENODEV;
        }

        /* Register the ADC driver */
        ret = adc_register(TEMP_RAW_ADC_DEVPATH, adc);
        if (ret < 0)
        {
            adbg("adc_register failed: %d\n", ret);
            return ret;
        }

        /* Now we are initialized */
        adbg("adc initialized: %s\n", TEMP_RAW_ADC_DEVPATH);
        initialized = true;
    }

    return OK;
#else
    return -ENOSYS;
#endif
}

static void led_toggle(void)
{
    uint8_t new_val;

    new_val = gpio_get_value(GPIO_MODS_LED_DRV_3) ^ 1;
    gpio_set_value(GPIO_MODS_LED_DRV_3, new_val);

}

static int security_sleep(int mi)
{

    struct timespec t10u, rem;
    t10u.tv_sec = 0;
    t10u.tv_nsec = mi*1000;
    return nanosleep(&t10u, &rem);

}

static int raw_send(struct device *dev, uint32_t len, uint8_t data[])
{
    
    struct project_info *info = NULL;

    uint8_t *resp_msg;

    if (len == 0) return -EINVAL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    // allocate memory for response message 
    resp_msg = zalloc(len);
    if (!resp_msg) {
        return -ENOMEM;
    }

    // copy payload data and send the message
    memcpy((void *)resp_msg, data, len);

    if (info->gCallback) {
        info->gCallback(info->gDevice, len, resp_msg);
    }
    free(resp_msg); 
    return OK;
}

static int readADC(void) {
    int ret;
    int adc_fd;
    int command;
    struct adc_msg_s sample;

    adc_fd = open(TEMP_RAW_ADC_DEVPATH, O_RDONLY);
    if (adc_fd < 0)
    {
        dbg("open %s failed: %d\n", TEMP_RAW_ADC_DEVPATH, errno);
    }
    ret = ioctl(adc_fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
    {
        dbg("ANIOC_TRIGGER ioctl failed: %d\n", errno);
    }
    read(adc_fd, &sample, sizeof(sample));
    close(adc_fd);
    command = (int) sample.am_data;
    return command;
}

static void main_worker(void *arg)
{
    struct temp_raw_info *info = NULL;
    info = arg;

    /* cancel any work and reset ourselves */
    if (!work_available(&data_report_work))
        work_cancel(LPWORK, &data_report_work);
    uint8_t sensorMSG[6];   // Message containing information about sensor measurements
    union {
        int CMint;
        uint8_t CMbyte[2];
    } dist;

    /* SENSOR 1 */
    gpio_set_value(GPIO_PI_CAM_GPIO1, 0);   // PA9
    gpio_set_value(GPIO_PI_CAM_GPIO0, 1);   // PC9
    security_sleep(5);
    dist.CMint = readADC();
    gpio_set_value(GPIO_PI_CAM_GPIO1, 1);   // PA9
    gpio_set_value(GPIO_PI_CAM_GPIO0, 0);   // PC9
    sensorMSG[0] = dist.CMbyte[0];
    sensorMSG[1] = dist.CMbyte[1];

    /* SENSOR 2 */
    gpio_set_value(GPIO_FACT_DISP_RST1_N, 0);    // PC8
    gpio_set_value(GPIO_FACT_DISP_PWR1_EN, 1);    // PA10
    security_sleep(5);
    dist.CMint = readADC();
    gpio_set_value(GPIO_FACT_DISP_RST1_N, 1);    // PC8
    gpio_set_value(GPIO_FACT_DISP_PWR1_EN, 0);    // PA10
    sensorMSG[2] = dist.CMbyte[0];
    sensorMSG[3] = dist.CMbyte[1];

    /* SENSOR 3 */
    gpio_set_value(GPIO_FACT_DISP_PWR4_EN, 0);    // PA2
    gpio_set_value(GPIO_FACT_DISP_PWR3_EN, 1);    // PA0
    security_sleep(5);
    dist.CMint = readADC();
    gpio_set_value(GPIO_FACT_DISP_PWR4_EN, 1);    // PA2
    gpio_set_value(GPIO_FACT_DISP_PWR3_EN, 0);    // PA0
    sensorMSG[4] = dist.CMbyte[0];
    sensorMSG[5] = dist.CMbyte[1];

    /* Sending message */
    raw_send(info->gDevice, 6, sensorMSG);

    /* schedule work */
    work_queue(LPWORK, &data_report_work,
                main_worker, info, MSEC2TICK(500)); // 500 ms
/*errout:
    return;*/
}
static int walkassist_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    dbg("Entering recv...\n");
    if (len == 0)
        return -EINVAL;

    struct temp_raw_info *info = NULL;
    info = device_get_private(dev);
    
    if (len == 1) {
        dbg("cmd: Toggle - Collect data\n");
        led_toggle();
        union {
            int CMint;
            uint8_t CMbyte[2];
        } dist;
        dist.CMint = readADC();
        raw_send(info->gDevice, 2, dist.CMbyte);
    }
    /* Activate/Deactivate worker to auto-collect data */
    if (len == 3) {

        if (data[2] == 0 || data[2] == '0') {
            lldbg("cmd: Deactivate worker");
            if (!work_available(&data_report_work))
                work_cancel(LPWORK, &data_report_work);
        } else if (data[2] == 1 || data[2] == '1') {
            lldbg("cmd: Activate worker");
            if (!work_available(&data_report_work))
                work_cancel(LPWORK, &data_report_work);
            //  schedule work 
            work_queue(LPWORK, &data_report_work,
                        main_worker, info, 0);
        }
    }

    return 0;
}


/**
 * register the send callback function.
 */
static int walkassist_register_callback(struct device *dev, raw_send_callback callback)
{
    struct project_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    info->gCallback = callback;
    return 0;
}

/**
 * unregister the send callback function.
 */
static int walkassist_unregister_callback(struct device *dev)
{
    struct project_info *info = NULL;
    
    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    info->gCallback = NULL;
    return 0;
}

static int walkassist_probe(struct device *dev)
{

    /* SENSOR 1 */
    gpio_direction_out(GPIO_PI_CAM_GPIO1, 1);    // PA9
    gpio_direction_out(GPIO_PI_CAM_GPIO0, 0);    // PC9

    /* SENSOR 2 */
    gpio_direction_out(GPIO_FACT_DISP_RST1_N, 1);    // PC8
    gpio_direction_out(GPIO_FACT_DISP_PWR1_EN, 0);    // PA10
    
    /* SENSOR 3 */
    gpio_direction_out(GPIO_FACT_DISP_PWR4_EN, 1);    // PA2
    gpio_direction_out(GPIO_FACT_DISP_PWR3_EN, 0);    // PA0
    
    gpio_direction_out(GPIO_MODS_LED_DRV_3, LED_OFF);
    struct project_info *info;
    dbg("Entering probe...\n");
    if (!dev) {
        return -EINVAL;
    }
    adc_devinit();
    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->gDevice = dev;
    device_set_private(dev, info);

    return 0;
}

/**
 * remove function called when device is unregistered.
 */
static void walkassist_remove(struct device *dev)
{
    struct project_info *info = NULL;
    dbg("REMOOOOVE!!\n");
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    free(info);
    device_set_private(dev, NULL);
}

static struct device_raw_type_ops walkassist_raw_ops = {
    .recv = walkassist_recv,
    .register_callback = walkassist_register_callback,
    .unregister_callback = walkassist_unregister_callback,
};

static struct device_driver_ops walkassist_driver_ops = {
    .probe = walkassist_probe,
    .remove = walkassist_remove,
    .type_ops = &walkassist_raw_ops,
};

struct device_driver mods_raw_walkassist_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_walkassist",
    .desc = "Walk Assist Raw Interface",
    .ops = &walkassist_driver_ops,
};
