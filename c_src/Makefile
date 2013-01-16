#
# We assume that dthread has been checked out as well into an
# overlaid directory structure by
# meta-exosense/recipes-exodev/can/erlang-can.bb
#

PRIVDIR := ../priv

all : $(PRIVDIR)/can_sock_drv.so

override CFLAGS += -DDLOG_DEFAULT=DLOG_NONE -fpic -shared -I ../../../erlang-dthread-*/git/include/


$(PRIVDIR)/can_sock_drv.so : can_sock_drv.o can_mcp2515_drv.c
	$(CC) -shared -fpic $(LDFLAGS) 	\
	../../../erlang-dthread-*/git/c_src/dthread.o \
	../../../erlang-dthread-*/git/c_src/dlib.o \
	../../../erlang-dthread-*/git/c_src/dterm.o \
	-lpthread -o  $@ $^

clean:
	$(RM) -f $(PRIVDIR)/can_sock_drv.so can_sock_drv.o

