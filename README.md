can
=====

### CAN binding for Erlang

CAN or Controller Area Network for short, is a two wire serial protcol
for industrial applications.

This implementation currently supports three different backends:

* can_usb: CANUSB is a USB dongle from [LAWICEL AB](http://www.canusb.com).
* can_udp: This is my own invention. A simple repackaging of CAN frames into UDP/IP datagrams sent over local multicast channel.
* can_sock: A binding to linux SocketCAN interface.

Any number of backend interfaces may be started and attached to the
can\_router, which is the main interface to receice and send CAN frames.<br/>
An application will typically call can_router:attach() and then 
receive CAN frames from any of the interfaces. To send a frame then
simple call can:send/n, this will pass the CAN frame to all the
interfaces and connected local applications in the Erlang node.

### Dependencies

To build can you will need a working installation of Erlang R15B (or
later).<br/>
Information on building and installing [Erlang/OTP](http://www.erlang.org)
can be found [here](https://github.com/erlang/otp/wiki/Installation)
([more info](https://github.com/erlang/otp/blob/master/INSTALL.md)).

can is built using rebar that can be found [here](https://github.com/basho/rebar), with building instructions [here](https://github.com/basho/rebar/wiki/Building-rebar).

can also requires the following applications to be installed:
<ul>
<li>sl - https://github.com/tonyrog/sl</li>
<li>eapi - https://github.com/tonyrog/eapi</li>
</ul>


### Downloading

Clone the repository in a suitable location:

```sh
$ git clone git://github.com/tonyrog/can.git
```
### Configurating
#### Concepts

...

#### Files

...

### Building

Rebar will compile all needed dependencies.<br/>
Compile:

```sh
$ cd can
$ rebar compile
...
==> can (compile)
```

### Running

```sh
$ erl -pa <path>/can/ebin
>can_router:start().
```
(Instead of specifing the path to the ebin directory you can set the environment ERL_LIBS.)

Stop:

```sh
>halt().

