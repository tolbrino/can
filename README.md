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
<li>dthread - https://github.com/tonyrog/dthread</li>
<li>uart - https://github.com/tonyrog/uart</li>
</ul>

### Downloading

Clone the repository in a suitable location:

```sh
$ git clone git://github.com/tonyrog/can.git
```
### Configurating

Interfaces can be added and remove dynamically, but can also
be initialized in the environment like:

    {can, [{interfaces,
             [{can_udp, 1, []},
              {can_udp, 2, [{ttl,0}]},
	      {can_usb, 1, [{device, "/dev/tty.usbserial-LWQ6UYOM"},
                            {bitrate, 125000}]},
              {can_usb, 2, [{device, "/dev/tty.usbserial-LWQ8CA1K"},
                            {bitrate, 250000}]},
              {can_sock, "can0", []},
              {can_sock, "vcan0", []}]}]}
	   
The interfaces in the environment will get under supervision.
		     
#### Concepts

### Linux virtual can driver

#### load the driver
    $ sudo modprobe vcan

#### Create a virtual CAN network interface called 'vcan0'
    $ sudo ip link add dev vcan0 type vcan

#### Set the bitrate of the CAN network interface
This must be done prior to bringing the interface up.

    $ sudo ip link set vcan0 type can bitrate 125000

#### Activate a virtual CAN network interface called 'vcan0'
    $ sudo ifconfig vcan0 up

#### Remove a (virtual) CAN network interface 'vcan0'
    $ sudp ip link del vcan0

#### Create a virtual CAN network interface
    $ sudo ip link add type vcan

 triple-sampling on
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
(Instead of specifying the path to the ebin directory you can set the environment variable ERL_LIBS.)

Stop:

```sh
>halt().


