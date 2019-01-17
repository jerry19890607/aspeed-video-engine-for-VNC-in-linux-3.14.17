/*
 * 8250_dma.c - DMA Engine API support for 8250.c
 *
 * Copyright (C) 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <mach/hardware.h>
#include <linux/delay.h>

#include "8250.h"

//#define CONFIG_UART_DMA_DEBUG

#ifdef CONFIG_UART_DMA_DEBUG
    #define UART_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
    #define UART_DBG(fmt, args...)
#endif

#define UART_UDMA_TIMER_DEFAULT_VALUE 0x171

#define AST_UART_TX_DMA_BUFFER_SIZE                        UDMA_BUFF_SIZE_4KB 
#if AST_UART_TX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_1KB
  #define AST_UART_TX_DMA_BUFFER_SIZE_VAL                  1024
#elif AST_UART_TX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_4KB
  #define AST_UART_TX_DMA_BUFFER_SIZE_VAL                  4*1024
#elif AST_UART_TX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_16KB
  #define AST_UART_TX_DMA_BUFFER_SIZE_VAL                  16*1024
#else /*AST_UART_TX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_64KB*/
  #define AST_UART_TX_DMA_BUFFER_SIZE_VAL                  64*1024
#endif


#define AST_UART_RX_DMA_BUFFER_SIZE                        UDMA_BUFF_SIZE_64KB 
#if AST_UART_RX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_1KB
  #define AST_UART_RX_DMA_BUFFER_SIZE_VAL                  1024
#elif AST_UART_RX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_4KB
  #define AST_UART_RX_DMA_BUFFER_SIZE_VAL                  4*1024
#elif AST_UART_RX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_16KB
  #define AST_UART_RX_DMA_BUFFER_SIZE_VAL                  16*1024
#else /*AST_UART_RX_DMA_BUFFER_SIZE==UDMA_BUFF_SIZE_64KB*/
  #define AST_UART_RX_DMA_BUFFER_SIZE_VAL                  64*1024
#endif

#define SCU_HARDWARE_STRAP_REQ                      0x70
#define SCU_HW_STRAP_REQ_SEL_UART_DEBUG_PORT        (1 << 29)

static int dma_count=0;
u32 locount=0;
u32 errcount=0;

static inline void ast_udma_bufffdone(struct uart_8250_port *p, u8 dir);
static irqreturn_t ast_uart_udma_irq(int irq, void *dev_id);
static int ast_uart_tx_udma_update(struct uart_8250_port *p, u16 point);
static int ast_uart_tx_udma_enqueue(struct uart_8250_port *p);
static int ast_uart_rx_udma_enqueue(struct uart_8250_port *p);
static int ast_uart_rx_udma_ctrl(struct uart_8250_port *p, enum ast_uart_chan_op op);
static int ast_uart_tx_udma_ctrl(struct uart_8250_port *p, enum ast_uart_chan_op op);

static void ast_uart_udma_write(u32 value, u32 offset)
{
    iowrite32( value, (void * __iomem)AST_UART_UDMA_VA_BASE + offset );
}

static u32 ast_uart_udma_read(u32 offset)
{
    return( ioread32( (void * __iomem)AST_UART_UDMA_VA_BASE + offset ) );
}

int serial8250_tx_dma(struct uart_8250_port *p)
{
    struct circ_buf *xmit = &p->port.state->xmit;
    UART_DBG("line [%d]  \n", p->dma->tx.dma_ch);

    ast_uart_tx_udma_ctrl(p, AST_UART_DMAOP_PAUSE);
    dma_sync_single_for_device(p->port.dev,
                               p->dma->tx.dma_virt_addr,
                               AST_UART_TX_DMA_BUFFER_SIZE_VAL,
                               DMA_TO_DEVICE);
    //update xmit->head -->fot tx 
    ast_uart_tx_udma_update(p, xmit->head);
    ast_uart_tx_udma_ctrl(p, AST_UART_DMAOP_TRIGGER);
    return 0;
}
EXPORT_SYMBOL_GPL(serial8250_tx_dma);

int serial8250_rx_dma(struct uart_8250_port *p, unsigned int iir)
{
    return 0;
}
EXPORT_SYMBOL_GPL(serial8250_rx_dma);

int serial8250_request_dma(struct uart_8250_port *p)
{
    struct uart_8250_dma *dma = p->dma;

    dma->rx.dma_buf.head = 0;
    dma->rx.dma_buf.tail = 0;
    dma->rx.dma_buf.buf = (unsigned char *)dma_alloc_coherent(NULL, AST_UART_RX_DMA_BUFFER_SIZE_VAL, &dma->rx.dma_virt_addr, GFP_KERNEL);
    UART_DBG("RX buff vir = %p, phy = %x \n", dma->rx.dma_buf.buf, dma->rx.dma_virt_addr);
    ast_uart_rx_udma_ctrl(p, AST_UART_DMAOP_STOP);
    ast_uart_rx_udma_enqueue(p);
    mdelay(300);
    ast_uart_rx_udma_ctrl(p, AST_UART_DMAOP_TRIGGER);

    dma->tx.dma_buf.head = 0;
    dma->tx.dma_buf.tail = 0;
    dma->tx.dma_buf.buf = p->port.state->xmit.buf;
    dma->tx.dma_virt_addr = dma_map_single(p->port.dev,
                                           dma->tx.dma_buf.buf,
                                           UART_XMIT_SIZE,
                                           DMA_TO_DEVICE);
                        
    ast_uart_tx_udma_ctrl(p, AST_UART_DMAOP_STOP);
    ast_uart_tx_udma_enqueue(p);
    return 0;
}
EXPORT_SYMBOL_GPL(serial8250_request_dma);

void serial8250_release_dma(struct uart_8250_port *p)
{
    ast_uart_rx_udma_ctrl(p, AST_UART_DMAOP_STOP);
    ast_uart_tx_udma_ctrl(p, AST_UART_DMAOP_STOP);
    UART_DBG("free TX , RX buffer \n");
    dma_free_coherent(p->port.dev,
                      AST_UART_RX_DMA_BUFFER_SIZE_VAL,
                      p->dma->rx.dma_buf.buf,
                      p->dma->rx.dma_virt_addr);
    dma_unmap_single(p->port.dev, p->dma->tx.dma_virt_addr,
                     AST_UART_TX_DMA_BUFFER_SIZE_VAL,
                     DMA_TO_DEVICE);
}
EXPORT_SYMBOL_GPL(serial8250_release_dma);

static inline void ast_udma_bufffdone(struct uart_8250_port *p, u8 dir)
{
    u32 len=0;
    int ch = (dir==DMA_TO_DEVICE?p->dma->tx.dma_ch:p->dma->rx.dma_ch);
    struct ast_uart_dma_info *udma = (dir==DMA_TO_DEVICE?(&(p->dma->tx)):(&(p->dma->rx)));

    if(udma->enable == 0) {
//        printk("udma Please check \n");
        return;
    }

    if(dir==DMA_TO_DEVICE) {
        len = ast_uart_udma_read(UART_TX_R_POINT(ch)) ;
        UART_DBG("tx rp %x , wp %x \n", ast_uart_udma_read(UART_TX_R_POINT(ch)), ast_uart_udma_read(UART_TX_W_POINT(ch)));
        
    }

    UART_DBG("<dma dwn>: ch[(%d:%d)] : %s ,len : %d \n", p->port.line, ch, (dir==DMA_TO_DEVICE ? "tx" : "rx"), len);

    if (udma->callback_fn != NULL)
        (udma->callback_fn)(p, len);
}

static irqreturn_t ast_uart_udma_irq(int irq, void *dev_id)
{
    struct uart_8250_port *p = (struct uart_8250_port *)dev_id;
    int count=0;

    u32 tx_sts = (u16) ast_uart_udma_read(UART_TX_UDMA_ISR);
    u32 rx_sts = (u16) ast_uart_udma_read(UART_RX_UDMA_ISR);

    UART_DBG("tx sts : %x, rx sts : %x \n",tx_sts, rx_sts);

    if((tx_sts == 0) && (rx_sts == 0)) {
        printk("UDMA IRQ ERROR !!!\n");
        return IRQ_HANDLED;    
    }

    while (rx_sts) {
        /* BMC DEBUG PORT doesn't SUPPORT DMA mode */
        if (p->dma==NULL) p++;
        if (rx_sts & (1 << count)) {
            /* clear the interrupt status */
            ast_uart_udma_write((1 << count), UART_RX_UDMA_ISR);
            ast_udma_bufffdone(p, DMA_FROM_DEVICE);
        }
        rx_sts &= ~(1 << count);
        count++;
        p++;
    }


    count=0;
    p = (struct uart_8250_port *)dev_id;

    while (tx_sts) {
        /* BMC DEBUG PORT doesn't SUPPORT DMA mode */
        if (p->dma==NULL) p++;
        if (tx_sts & (1 << count)) {
            /* clear the interrupt status */
            ast_uart_udma_write((1 << count), UART_TX_UDMA_ISR);
            ast_udma_bufffdone(p, DMA_TO_DEVICE);
        }
        tx_sts &= ~(1 << count);
        count++;
        p++;
    }

    return IRQ_HANDLED;
}

int ast_uart_udma_init(struct uart_8250_port *p)
{
    int ret;

    ast_uart_udma_write(0x400, UART_UDMA_TIMER);
    ast_uart_udma_write(0xfff, UART_TX_UDMA_ISR);
    ast_uart_udma_write(0, UART_TX_UDMA_IER);
    ast_uart_udma_write(0xfff, UART_RX_UDMA_ISR);
    ast_uart_udma_write(0, UART_RX_UDMA_IER);
    ast_uart_udma_write(UART_UDMA_TIMER_DEFAULT_VALUE, UART_UDMA_TIMER);

    ret = request_irq(IRQ_UART_UDMA_INT,
                      ast_uart_udma_irq, IRQF_DISABLED, 
                      "ast_uart_udma", p);
    if (ret) {
        printk (KERN_ERR "Request UART UDMA IRQ Fail\n");
        return -1;
    }

    ast_uart_udma_write(UDMA_SET_TX_BUFF_SIZE(AST_UART_TX_DMA_BUFFER_SIZE) | UDMA_SET_RX_BUFF_SIZE(AST_UART_RX_DMA_BUFFER_SIZE), UART_UDMA_CONF);

    return 0;
}                                                                              
EXPORT_SYMBOL(ast_uart_udma_init);

static int ast_uart_tx_udma_update(struct uart_8250_port *p, u16 point)
{
    unsigned long flags;
    int ch = p->dma->tx.dma_ch;
    UART_DBG("TX DMA CTRL [ch (%d:%d)] \n", p->port.line, ch);

    local_irq_save(flags);
    ast_uart_udma_write(point, UART_TX_W_POINT(ch));
    local_irq_restore(flags);
    return 0;
}

void ast_uart_rx_udma_tasklet_func(unsigned long data)
{
    struct uart_8250_port *up = (struct uart_8250_port *)data;
    struct circ_buf *rx_ring = &up->dma->rx.dma_buf;
    struct tty_port *port = &(up->port.state->port);

    u32 h=0,t=0;
    u32 len=0;
    int ch = up->dma->rx.dma_ch;
    UART_DBG("rx rp %x , wp %x \n", ast_uart_udma_read(UART_RX_R_POINT(ch)), ast_uart_udma_read(UART_RX_W_POINT(ch)));
    spin_lock_irq(&up->dma->rx.lock);
    ast_uart_rx_udma_ctrl(up, AST_UART_DMAOP_TRIGGER);
    h=ast_uart_udma_read(UART_RX_W_POINT(ch));
    t=ast_uart_udma_read(UART_RX_R_POINT(ch));
    ast_uart_udma_write(h,UART_RX_R_POINT(ch)) ;
    if (t > h) {
        len=(AST_UART_RX_DMA_BUFFER_SIZE_VAL-t)+h+1;
    } else {
        len=h-t;
    }

    if ((rx_ring->head+len) > AST_UART_RX_DMA_BUFFER_SIZE_VAL)
        rx_ring->head=(rx_ring->head+len) - AST_UART_RX_DMA_BUFFER_SIZE_VAL -1;
    else
        rx_ring->head+= len;

    {
        if (rx_ring->head != rx_ring->tail) {
            if (rx_ring->head < rx_ring->tail) {
                tty_insert_flip_string(port, &rx_ring->buf[rx_ring->tail], AST_UART_RX_DMA_BUFFER_SIZE_VAL-rx_ring->tail);
                spin_lock(&up->port.lock);
                tty_flip_buffer_push(port);
                spin_unlock(&up->port.lock);
                rx_ring->tail = 0;
            }

            if (rx_ring->head != rx_ring->tail) {
                tty_insert_flip_string(port, &rx_ring->buf[rx_ring->tail], rx_ring->head-rx_ring->tail);
                rx_ring->tail = rx_ring->head;
            }
        }
    }
    spin_unlock_irq(&up->dma->rx.lock);

    spin_lock(&up->port.lock);
    tty_flip_buffer_push(port);
    spin_unlock(&up->port.lock);
}
EXPORT_SYMBOL_GPL(ast_uart_rx_udma_tasklet_func);

void ast_uart_rx_buffdone(void *dev_id, u16 len)
{
    struct uart_8250_port *up = (struct uart_8250_port *)dev_id;

    UART_DBG("line [(%d:%d)],head = %d, len : %d\n",up->port.line,up->dma->rx.dma_ch,up->dma->rx.dma_buf.head, len);
    tasklet_schedule(&up->dma->rx.tasklet);
}
EXPORT_SYMBOL_GPL(ast_uart_rx_buffdone);

void ast_uart_tx_buffdone(void *dev_id, u16 len)
{
    struct uart_8250_port *up = (struct uart_8250_port *) dev_id;
    struct circ_buf *xmit = &up->port.state->xmit;

    UART_DBG("line [(%d:%d)] : tx len = %d \n", up->port.line, up->dma->tx.dma_ch, len);    

    spin_lock(&up->port.lock);
    //-->get tail for update len 
    xmit->tail = len;
    UART_DBG("???? line [%d], xmit->head =%d, xmit->tail = %d\n",up->dma->tx.dma_ch,xmit->head, xmit->tail);

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
            uart_write_wakeup(&up->port);

    if(xmit->head != xmit->tail) {
        ast_uart_tx_udma_ctrl(up, AST_UART_DMAOP_PAUSE);
        dma_sync_single_for_device(up->port.dev,
                                   up->dma->tx.dma_virt_addr,
                                   AST_UART_TX_DMA_BUFFER_SIZE_VAL,
                                   DMA_TO_DEVICE);
        //update xmit->head -->fot tx 
        ast_uart_tx_udma_update(up, xmit->head);
        ast_uart_tx_udma_ctrl(up, AST_UART_DMAOP_TRIGGER);
    }

    spin_unlock(&up->port.lock);

}
EXPORT_SYMBOL_GPL(ast_uart_tx_buffdone);

int ast_uart_tx_udma_request(struct uart_8250_port *p, ast_uart_dma_cbfn_t rtn, void *id)
{
    unsigned long flags;
    int ch = p->dma->tx.dma_ch;

    UART_DBG("TX DMA REQUEST ch = (%d:%d) \n", p->port.line, ch);

    local_irq_save(flags);

    p->dma->tx.priv = id;
    p->dma->tx.callback_fn = rtn;

    //DMA IRQ En
    ast_uart_udma_write(ast_uart_udma_read(UART_TX_UDMA_IER) | (1 << ch),
                        UART_TX_UDMA_IER);

    local_irq_restore(flags);

    return 0;

}

EXPORT_SYMBOL(ast_uart_tx_udma_request);

int ast_uart_rx_udma_request(struct uart_8250_port *p, ast_uart_dma_cbfn_t rtn, void *id)
{
    unsigned long flags;
    int ch = p->dma->rx.dma_ch;

    UART_DBG("RX DMA REQUEST ch = (%d:%d) \n", p->port.line,ch);

    local_irq_save(flags);

    p->dma->rx.priv = id;
    p->dma->rx.callback_fn = rtn;

    //DMA IRQ En
    ast_uart_udma_write(ast_uart_udma_read(UART_RX_UDMA_IER) | (1 << ch),
                        UART_RX_UDMA_IER);

    local_irq_restore(flags);

    return 0;

}

EXPORT_SYMBOL(ast_uart_rx_udma_request);


static int ast_uart_tx_udma_ctrl(struct uart_8250_port *p, enum ast_uart_chan_op op)
{
    unsigned long flags;
    int ch = p->dma->tx.dma_ch;
    UART_DBG("TX DMA CTRL [ch (%d:%d)] \n", p->port.line, ch);

    local_irq_save(flags);

    switch (op) {
        case AST_UART_DMAOP_TRIGGER:
            UART_DBG("Trigger \n");
            p->dma->tx.enable = 1;
            //set enable 
            ast_uart_udma_write(ast_uart_udma_read(UART_TX_UDMA_EN) | (0x1 << ch), UART_TX_UDMA_EN);
            break;
        case AST_UART_DMAOP_STOP:
            UART_DBG("STOP \n");
            p->dma->tx.enable = 0;
            //disable engine 
            ast_uart_udma_write(ast_uart_udma_read(UART_TX_UDMA_EN) & ~(0x1 << ch), UART_TX_UDMA_EN);

            //set reset 
            ast_uart_udma_write(ast_uart_udma_read(UART_TX_UDMA_REST) | (0x1 << ch), UART_TX_UDMA_REST);
            ast_uart_udma_write(ast_uart_udma_read(UART_TX_UDMA_REST) & ~(0x1 << ch), UART_TX_UDMA_REST);
            break;
        case AST_UART_DMAOP_PAUSE:
            //disable engine
            ast_uart_udma_write(ast_uart_udma_read(UART_TX_UDMA_EN) & ~(0x1 << ch), UART_TX_UDMA_EN);
    }

    local_irq_restore(flags);
    return 0;
}

static int ast_uart_tx_udma_enqueue(struct uart_8250_port *p)
{
    unsigned long flags;
    int ch = p->dma->tx.dma_ch;

    UART_DBG("ch = (%d:%d), rx buff = %x, len = %d \n", p->port.line, ch, p->dma->tx.dma_virt_addr, AST_UART_TX_DMA_BUFFER_SIZE_VAL);

    local_irq_save(flags);

    ast_uart_udma_write(p->dma->tx.dma_virt_addr, UART_TX_UDMA_ADDR(ch));

    ast_uart_udma_write(0, UART_TX_W_POINT(ch));

    local_irq_restore(flags);

    return 0;
}

static int ast_uart_rx_udma_ctrl(struct uart_8250_port *p, enum ast_uart_chan_op op)
{
    unsigned long flags;
    int ch = p->dma->rx.dma_ch;
    UART_DBG("RX DMA CTRL [ch %d] \n", ch);

    local_irq_save(flags);

    switch (op) {
        case AST_UART_DMAOP_TRIGGER:
            UART_DBG("Trigger \n");
            p->dma->rx.enable = 1;
            //set enable 
            ast_uart_udma_write(ast_uart_udma_read(UART_RX_UDMA_EN) | (0x1 << ch), UART_RX_UDMA_EN);
            break;
        case AST_UART_DMAOP_STOP:
            //disable engine 
            UART_DBG("STOP \n");
            p->dma->rx.enable = 0;
            ast_uart_udma_write(ast_uart_udma_read(UART_RX_UDMA_EN) & ~(0x1 << ch), UART_RX_UDMA_EN);

            //set reset 
            ast_uart_udma_write(ast_uart_udma_read(UART_RX_UDMA_REST) | (0x1 << ch), UART_RX_UDMA_REST);
            ast_uart_udma_write(ast_uart_udma_read(UART_RX_UDMA_REST) & ~(0x1 << ch), UART_RX_UDMA_REST);
            break;
        case AST_UART_DMAOP_PAUSE:
            //disable engine
            ast_uart_udma_write(ast_uart_udma_read(UART_RX_UDMA_EN) & ~(0x1 << ch), UART_RX_UDMA_EN);
            break;
    }

    local_irq_restore(flags);
    return 0;
}

static int ast_uart_rx_udma_enqueue(struct uart_8250_port *p)
{
    unsigned long flags;
    int ch = p->dma->rx.dma_ch;

    UART_DBG("ch = %d, rx buff = %x, len = %d \n", ch, p->dma->rx.dma_virt_addr, AST_UART_RX_DMA_BUFFER_SIZE_VAL);

    local_irq_save(flags);

    ast_uart_udma_write(p->dma->rx.dma_virt_addr, UART_RX_UDMA_ADDR(ch));

    local_irq_restore(flags);

    return 0;
}

int serial8250_dma_config(struct uart_8250_port *p)
{
    struct uart_8250_dma *dma=NULL;
    int ch = p->port.line;

    if (ch == 4) {
        p->dma = NULL;
        //Select UART5 as BMC Console
        return -EPERM;
    }

    dma = kmalloc(sizeof(struct uart_8250_dma), GFP_KERNEL);
    if (!dma)
        return -ENOMEM;

    dma->rx.enable = 0;
    dma->rx.dma_ch=dma_count;

    dma->tx.enable = 0;
    dma->tx.dma_ch=dma_count;
    dma_count++;
    p->dma = dma;
    ast_uart_rx_udma_ctrl(p, AST_UART_DMAOP_STOP);
    ast_uart_tx_udma_ctrl(p, AST_UART_DMAOP_STOP);
    return 0;
}
EXPORT_SYMBOL_GPL(serial8250_dma_config);
