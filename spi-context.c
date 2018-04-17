/*
 * generic SPI functions
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "spi-context.h"

#include "logging.h"
#include "miner.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h>

struct spi_ctx *spi_init(struct spi_config *config)
{
    char dev_fname[PATH_MAX];
    struct spi_ctx *ctx;

    if (config == NULL)
        return NULL;

    memset(dev_fname, 0, sizeof(dev_fname));
    sprintf(dev_fname, SPI_DEVICE_TEMPLATE, config->bus, config->cs_line);

    int fd = open(dev_fname, O_RDWR);
    if (fd < 0) {
        applog(LOG_ERR, "SPI: Can not open SPI device %s", dev_fname);
        return NULL;
    }

    if ((ioctl(fd, SPI_IOC_WR_MODE, &config->mode) < 0) ||
        (ioctl(fd, SPI_IOC_RD_MODE, &config->mode) < 0) ||
        (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config->bits) < 0) ||
        (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &config->bits) < 0) ||
        (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config->speed) < 0) ||
        (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &config->speed) < 0)) 
    {
        applog(LOG_ERR, "SPI: ioctl error on SPI device %s", dev_fname);
        close(fd);
        return NULL;
    }

    ctx = malloc(sizeof(*ctx));
    assert(ctx != NULL);

    ctx->fd = fd;
    ctx->config = *config;
    mutex_init(&ctx->spi_lock);
    applog(LOG_WARNING, "SPI '%s': mode=%hhu, bits=%hhu, speed=%u",
           dev_fname, ctx->config.mode, ctx->config.bits,
           ctx->config.speed);
    return ctx;
}


extern void spi_exit(struct spi_ctx *ctx)
{
    if (NULL == ctx)
    {
        return;
    }

    close(ctx->fd);
    free(ctx);
}

extern bool spi_write_data(struct spi_ctx *ctx, uint8_t *txbuf, int len)
{
    mutex_lock(&(ctx->spi_lock));
    
    if((len <= 0) || (txbuf == NULL))
    {
        mutex_unlock(&ctx->spi_lock);
        applog(LOG_ERR, "SPI: write para error");
        return false;
    }

    if(write(ctx->fd, txbuf, len) <= 0)
    {
        mutex_unlock(&ctx->spi_lock);
        applog(LOG_ERR, "SPI: write data error");
        return false;
    }

    mutex_unlock(&ctx->spi_lock);
    return true;
}


extern bool spi_read_data(struct spi_ctx *ctx, uint8_t *rxbuf, int len)
{
    mutex_lock(&(ctx->spi_lock));
    
    if((len <= 0) || (rxbuf == NULL))
    {
        mutex_unlock(&ctx->spi_lock);
        applog(LOG_ERR, "SPI: read para error");
        return false;
    }

    if(read(ctx->fd, rxbuf, len) <= 0)
    {
        mutex_unlock(&ctx->spi_lock);
        applog(LOG_ERR, "SPI: read data error");        
        return false;
    }

    mutex_unlock(&ctx->spi_lock);
    return true;
}


extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf, uint8_t *rxbuf, int len)
{
    if(!spi_write_data(ctx, txbuf, len))
        return false;
    
    //usleep(100);

    if(!spi_read_data(ctx, rxbuf, len))
        return false;

    return true;    
}

