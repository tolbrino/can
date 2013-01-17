/****** BEGIN COPYRIGHT *******************************************************
 *
 * Copyright (C) 2012 Feuerlabs, Inc. All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 ****** END COPYRIGHT ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <fcntl.h>

// CROSS COMPILE
// when missing can.h files add symbolic links to host ...
// cd /usr/local/arm/arm-2007q1/arm-none-linux-gnueabi/libc/usr/include/linux
// sudo ln -s /usr/include/linux/can.h
// sudo ln -s /usr/include/can
//

#include <linux/can.h>
#include "can_drv.h"
#include "can_mcp2515.h"

#define MCP2515_CAN_DEVICE "can0"
#define MCP2515_CAN_PATH "/dev/can0"

#define ATOM(NAME) am_ ## NAME
#define INIT_ATOM(NAME) am_ ## NAME = driver_mk_atom(#NAME)

#include "dthread.h"

typedef struct _drv_ctx_t
{
    ErlDrvPort     port;        // port controling the thread
    ErlDrvTermData dport;       // the port identifier as DriverTermData
    ErlDrvTermData owner;       // owner process pid
    ErlDrvEvent    desc;        // CAN
} drv_ctx_t;


static int  can_mcp2515_drv_init(void);
static void can_mcp2515_drv_finish(void);
static void can_mcp2515_drv_stop(ErlDrvData);
static void can_mcp2515_drv_output(ErlDrvData, char*, ErlDrvSizeT);
static void can_mcp2515_drv_ready_input(ErlDrvData, ErlDrvEvent);
static void can_mcp2515_drv_ready_output(ErlDrvData data, ErlDrvEvent event);
static ErlDrvData can_mcp2515_drv_start(ErlDrvPort, char* command);
static ErlDrvSSizeT can_mcp2515_drv_ctl(ErlDrvData,unsigned int,char*,ErlDrvSizeT,char**, ErlDrvSizeT);
static void can_mcp2515_drv_timeout(ErlDrvData);
static void can_mcp2515_drv_stop_select(ErlDrvEvent, void*);

static ErlDrvEntry can_mcp2515_drv_entry;

static ErlDrvTermData am_ok;
static ErlDrvTermData am_error;
static ErlDrvTermData am_can_frame;
static ErlDrvTermData am_data;

/* general control reply function */
static ErlDrvSSizeT ctl_reply(int rep, char* buf, ErlDrvSizeT len,
			      char** rbuf, ErlDrvSizeT rsize)
{
    char* ptr;

    if ((len+1) > rsize) {
	ErlDrvBinary* bin;
	if ((bin = driver_alloc_binary(len+1)) == NULL)
	    return -1;
	ptr = bin->orig_bytes;
	*rbuf = (char*) bin;
    }
    else
	ptr = *rbuf;
    *ptr++ = rep;
    memcpy(ptr, buf, len);
    return len+1;
}

static ErlDrvSSizeT ctl_reply_ok(char** rbuf, ErlDrvSizeT rsize)
{
    return ctl_reply(CTL_OK,"",0,rbuf,rsize);
}

static ErlDrvSSizeT ctl_reply_error(int err, char** rbuf, ErlDrvSizeT rsize)
{
    char* errid = erl_errno_id(err);
    ErlDrvSizeT len = strlen(errid);
    return ctl_reply(CTL_ERROR,errid,len,rbuf,rsize);
}

static ErlDrvSSizeT ctl_reply_u32(uint32_t v, char** rbuf, ErlDrvSizeT rsize)
{
    char buf[4];
    put_uint32(buf, v);
    return ctl_reply(CTL_UINT32,buf,sizeof(buf),rbuf,rsize);
}

static int can_mcp2515_drv_init(void)
{
    dlib_set_debug(DLOG_DEFAULT);
    DEBUGF("can_mcp2515_drv_init");
    dthread_lib_init();

    INIT_ATOM(ok);
    INIT_ATOM(error);
    INIT_ATOM(can_frame);
    INIT_ATOM(data);

    dlib_set_debug(DLOG_DEFAULT);
    return 0;
}

static void can_mcp2515_drv_finish(void)
{
    // cleanup global stuff!
    dthread_lib_finish();
}

static ErlDrvData can_mcp2515_drv_start(ErlDrvPort port, char* command)
{
    (void) command;
    drv_ctx_t* ctx = NULL;
    int desc;

    INFOF("memory allocated: %ld", dlib_allocated());
    INFOF("total memory allocated: %ld", dlib_total_allocated());

    if ((desc = open(MCP2515_CAN_PATH, O_RDWR)) < 0)
	return ERL_DRV_ERROR_ERRNO;

    set_port_control_flags(port, PORT_CONTROL_FLAG_BINARY);

    ctx = DZALLOC(sizeof(drv_ctx_t));
    ctx->port = port;
    ctx->dport = driver_mk_port(port);
    ctx->owner = driver_connected(port);
    ctx->desc = (ErlDrvEvent)desc;
    return (ErlDrvData) ctx;
}

static void can_mcp2515_drv_stop(ErlDrvData d)
{
    drv_ctx_t* ctx = (drv_ctx_t*) d;

    DEBUGF("can_mcp2515_drv_stop: called");
    driver_select(ctx->port,ctx->desc,ERL_DRV_USE,0);
    close((int) ctx->desc);
    DFREE(ctx);
    INFOF("memory allocated: %ld", dlib_allocated());
    INFOF("total memory allocated: %ld", dlib_total_allocated());
}

static int send_frame(drv_ctx_t* ctx, mcp2515_can_frame* frame)
{
    return write(DTHREAD_EVENT(ctx->desc), frame, sizeof(mcp2515_can_frame));
}


static ErlDrvSSizeT can_mcp2515_drv_ctl(ErlDrvData d,
				     unsigned int cmd, char* buf,
				     ErlDrvSizeT len,
				     char** rbuf, ErlDrvSizeT rsize)
{
    drv_ctx_t* ctx = (drv_ctx_t*) d;

    DEBUGF("can_mcp2515_drv: ctl: cmd=%u(%s), len=%d",
	   cmd, format_command(cmd), len);

    switch(cmd) {
    case CAN_DRIVER_CMD_IFNAME: {
	return ctl_reply(CTL_STRING,MCP2515_CAN_DEVICE, strlen(MCP2515_CAN_DEVICE), rbuf, rsize);
    }

    case CAN_DRIVER_CMD_IFINDEX: {
	return ctl_reply_u32(0, rbuf, rsize);
    }

    case CAN_DRIVER_CMD_SET_ERROR_FILTER: {
	return ctl_reply_ok(rbuf, rsize); // Not supported by MCP
    }

    case CAN_DRIVER_CMD_SET_LOOPBACK: {
	int value;
	int r;

	if (len != 1)
	    return ctl_reply_error(EINVAL, rbuf, rsize);
	value = buf[0];

	// Two different ops to set and clear a bit. Not stupid at all...
	if (value)
	    r = ioctl((int)ctx->desc, MCP2515_CAN_IOCTLOOPBACKMODE, 1);
	else
	    r = ioctl((int) ctx->desc, MCP2515_CAN_IOCTNORMALMODE, 1);

	if (r < 0)
	    return ctl_reply_error(errno, rbuf, rsize);
	else
	    return ctl_reply_ok(rbuf, rsize);
    }

    case CAN_DRIVER_CMD_RECV_OWN_MESSAGES: {
	return ctl_reply_ok(rbuf, rsize); // Not supported by MCP
    }

    case CAN_DRIVER_CMD_BIND: {
	// Only one interface makes binding to specific can bus if index not necessary
	driver_select(ctx->port, ctx->desc ,ERL_DRV_READ,1);
	return ctl_reply_ok(rbuf, rsize);
    }

    case CAN_DRIVER_CMD_SEND: {
//	int       index;
	uint32_t  id;
	uint8_t   flen;
	char*     fptr;
	mcp2515_can_frame frame;

	if (len != 25)
	    return ctl_reply_error(EINVAL, rbuf, rsize);
//	index = (int) get_uint32(buf); // Not used
	id    = get_uint32(buf+4);
	flen   = get_uint8(buf+8);
	fptr   = buf+9;  // this are is always 8 bytes!
	// intf  = (int) get_uint32(buf+17);
	// ts    = (int) get_uint32(buf+21);

	if (flen > 8)
	    return ctl_reply_error(EINVAL, rbuf, rsize);

	// Unused members.
	frame.header.srr = 0;
	frame.header.rb0 = 0;
	frame.header.rb1 = 0;

	// Do some bit checking on the id and set the corresponding fields
	// in the mcp2515_can_frame struct (which has a different memory layout)

	// Can error frame type does not seem to be supported by the driver.

	frame.header.id = id & CAN_SFF_MASK; // Lower 11 bits.
	frame.header.rtr = ((id & CAN_RTR_FLAG) == CAN_RTR_FLAG)?1:0;
	frame.header.dlc = flen & 0xF;

	if ((id & CAN_EFF_FLAG) == CAN_EFF_FLAG) {
	    frame.header.ide = 1;
	    frame.header.eid = (id & CAN_EFF_MASK) >> 11; // Upper 18 bits. Shifted.
	} else {
	    frame.header.ide = 0;
	    frame.header.eid = 0;
	}

	memcpy(frame.data, fptr, 8);

	// FIXME: drop packets when full! (deq old, enq new)
	if (driver_sizeq(ctx->port) == 0) {
	    int r = send_frame(ctx, &frame);
	    if ((r < 0) && (errno == EAGAIN)) {
		driver_enq(ctx->port, (char*)&frame, sizeof(frame));
		driver_select(ctx->port, ctx->desc,ERL_DRV_WRITE,1);
		return ctl_reply_ok(rbuf, rsize);
	    }
	    else if (r < 0)
		return ctl_reply_error(errno, rbuf, rsize);
	}
	else {
	    driver_enq(ctx->port, (char*) &frame, sizeof(frame));
	}
	return ctl_reply_ok(rbuf, rsize);
    }
    default:
	return ctl_reply_error(EINVAL, rbuf, rsize);
    }
}

static void can_mcp2515_drv_output(ErlDrvData d, char* buf, ErlDrvSizeT len)
{
    (void) d;
    DEBUGF("can_mcp2515_drv: output");
}


static void can_mcp2515_drv_ready_input(ErlDrvData d, ErlDrvEvent e)
{
    drv_ctx_t* ctx = (drv_ctx_t*) d;
    dterm_t t;
    dterm_mark_t m1;
    dterm_mark_t m2;
    dterm_mark_t m3;
    mcp2515_can_frame frame;
    unsigned int header_id = 0;

    DEBUGF("can_mcp2515_drv: ready_input called");

    memset(&frame, 0, sizeof(frame));
    if (read(DTHREAD_EVENT(ctx->desc), &frame, sizeof(frame)) != sizeof(frame))
	return;

    dterm_init(&t);

    DEBUGF("can_mcp2515_drv: ready_input got frame");
    // Format as: {Port,{data,#can_frame{}}}
    dterm_tuple_begin(&t, &m1); {
	dterm_port(&t, ctx->dport);
	dterm_tuple_begin(&t, &m2); {
	    dterm_atom(&t, ATOM(data));
	    dterm_tuple_begin(&t, &m3); {
		dterm_atom(&t, ATOM(can_frame));

		if (frame.header.ide)
		    header_id = frame.header.id | (frame.header.eid << 11) | CAN_EFF_FLAG;
		else
		    header_id = frame.header.id;

		if (frame.header.rtr)
		    header_id |= CAN_RTR_FLAG;

		dterm_uint(&t, header_id);

		dterm_uint(&t, frame.header.dlc);
		// check rtr?
		dterm_buf_binary(&t, (char*) frame.data,
				 (frame.header.dlc & 0xf));

		dterm_int(&t,  0); // Index is not used.
		// fixme timestamp, if requested
		dterm_int(&t, -1);
	    }
	    dterm_tuple_end(&t, &m3);
	}
	dterm_tuple_end(&t, &m2);
    }
    dterm_tuple_end(&t, &m1);
    // dterm_dump(stderr, dterm_data(&t), dterm_used_size(&t));
    driver_output_term(ctx->port, dterm_data(&t), dterm_used_size(&t));
    dterm_finish(&t);
}

static void can_mcp2515_drv_ready_output(ErlDrvData d, ErlDrvEvent e)
{
    drv_ctx_t* ctx = (drv_ctx_t*) d;
    (void) e;
    DEBUGF("can_mcp2515_drv: ready_output");
    ErlIOVec ev;
    int n;

    // FIXME: send N frames?
    if ((n=driver_peekqv(ctx->port, &ev)) >= (int)sizeof(mcp2515_can_frame)) {
	mcp2515_can_frame frame;
	int r;

	driver_vec_to_buf(&ev, (char*)&frame, sizeof(frame));
	r = send_frame(ctx, &frame);
	if ((r < 0) && (errno == EAGAIN))
	    return;
	if (r < (int)sizeof(frame))
	    return;
	driver_deq(ctx->port, sizeof(frame));
	n -= r;
    }
    if (n == 0)
	driver_select(ctx->port, ctx->desc,ERL_DRV_WRITE,0);
}

// operation timed out
static void can_mcp2515_drv_timeout(ErlDrvData d)
{
    (void) d;
    DEBUGF("can_mcp2515_drv: timeout");
}

static void can_mcp2515_drv_stop_select(ErlDrvEvent event, void* arg)
{
    (void) arg;
    DEBUGF("can_mcp2515_drv: stop_select event=%d", DTHREAD_EVENT(event));
    DTHREAD_CLOSE_EVENT(event);
}


DRIVER_INIT(can_mcp2515_drv)
{
    ErlDrvEntry* ptr = &can_mcp2515_drv_entry;

    DEBUGF("driver_init");

    ptr->init  = can_mcp2515_drv_init;
    ptr->start = can_mcp2515_drv_start;
    ptr->stop  = can_mcp2515_drv_stop;
    ptr->output = can_mcp2515_drv_output;
    ptr->ready_input  = can_mcp2515_drv_ready_input;
    ptr->ready_output = can_mcp2515_drv_ready_output;
    ptr->finish = can_mcp2515_drv_finish;
    ptr->driver_name = "can_mcp2515_drv";
    ptr->control = can_mcp2515_drv_ctl;
    ptr->timeout = can_mcp2515_drv_timeout;
    ptr->extended_marker = ERL_DRV_EXTENDED_MARKER;
    ptr->major_version = ERL_DRV_EXTENDED_MAJOR_VERSION;
    ptr->minor_version = ERL_DRV_EXTENDED_MINOR_VERSION;
    ptr->driver_flags = ERL_DRV_FLAG_USE_PORT_LOCKING;
    ptr->process_exit = 0;
    ptr->stop_select = can_mcp2515_drv_stop_select;
    return ptr;
}

