/**** BEGIN COPYRIGHT ********************************************************
 *
 * Copyright (C) 2007 - 2012, Rogvall Invest AB, <tony@rogvall.se>
 *
 * This software is licensed as described in the file COPYRIGHT, which
 * you should have received as part of this distribution. The terms
 * are also available at http://www.rogvall.se/docs/copyright.txt.
 *
 * You may opt to use, copy, modify, merge, publish, distribute and/or sell
 * copies of the Software, and permit persons to whom the Software is
 * furnished to do so, under the terms of the COPYRIGHT file.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 **** END COPYRIGHT **********************************************************/

#ifndef __ERL_CAN_DRV_H__
#define __ERL_CAN_DRV_H__

#define ATOM(NAME) am_ ## NAME
#define INIT_ATOM(NAME) am_ ## NAME = driver_mk_atom(#NAME)

#define CTL_OK     0
#define CTL_ERROR  1
#define CTL_UINT32 2
#define CTL_STRING 3

// Driver commands to be supported by all port drivers implementing the interface
#define CAN_DRIVER_CMD_IFNAME 1
#define CAN_DRIVER_CMD_IFINDEX 2
#define CAN_DRIVER_CMD_SET_ERROR_FILTER 3
#define CAN_DRIVER_CMD_SET_LOOPBACK 4
#define CAN_DRIVER_CMD_RECV_OWN_MESSAGES 5
#define CAN_DRIVER_CMD_BIND 6
#define CAN_DRIVER_CMD_SEND 7
#define CAN_DRIVER_CMD_SET_FILTER 8


// Some common funcs to inline into the including code
static inline uint32_t get_uint32(char* ptr)
{
    uint8_t* p = (uint8_t*) ptr;
    uint32_t value = (p[0]<<24) | (p[1]<<16) | (p[2]<<8) | (p[3]<<0);
    return value;
}

static inline uint16_t get_uint16(char* ptr)
{
    uint8_t* p = (uint8_t*) ptr;
    uint16_t value = (p[0]<<8) | (p[1]<<0);
    return value;
}

static inline uint8_t get_uint8(char* ptr)
{
    return ((uint8_t*)ptr)[0];
}

static inline void put_uint16(char* ptr, uint16_t v)
{
    uint8_t* p = (uint8_t*) ptr;
    p[0] = v>>8;
    p[1] = v;
}

static inline void put_uint32(char* ptr, uint32_t v)
{
    uint8_t* p = (uint8_t*) ptr;
    p[0] = v>>24;
    p[1] = v>>16;
    p[2] = v>>8;
    p[3] = v;
}

static char* format_command(int cmd)
{
    switch(cmd) {
    case CAN_DRIVER_CMD_IFNAME: return "ifname";
    case CAN_DRIVER_CMD_IFINDEX: return "ifindex";
    case CAN_DRIVER_CMD_SET_ERROR_FILTER: return "set_error_filter";
    case CAN_DRIVER_CMD_SET_LOOPBACK: return "set_loopback";
    case CAN_DRIVER_CMD_RECV_OWN_MESSAGES: return "revc_own_messages";
    case CAN_DRIVER_CMD_BIND:  return "bind";
    case CAN_DRIVER_CMD_SEND:  return "send";
    case CAN_DRIVER_CMD_SET_FILTER: return "set_filter";
    default: return "????";
    }
}

#endif /*! __ERL_CAN_DRV_H__ */
