#
# We assume that eapi has been checked out as well into an 
# overlaid directory structure by 
# meta-exosense/recipes-exodev/can/erlang-can.bb
#

PRIVDIR := ../priv

all : $(PRIVDIR)/can_sock_drv.so $(PRIVDIR)/eapi_drv.so

override CFLAGS += -fpic -shared

$(PRIVDIR)/can_sock_drv.so : can_sock_drv.o can_sock_drv_impl.o
	$(CC) -shared -fpic $(LDFLAGS) -o $@ $^

$(PRIVDIR)/eapi_drv.so : eapi_drv.o eapi_drv_lib.o
	$(CC) -shared -fpic $(LDFLAGS) -o $@ $^

clean:
	$(RM) -f $(PRIVDIR)/can_sock_drv.so can_sock_drv.o can_sock_drv_impl.o \
		$(PRIVDIR)/eapi_drv.so eapi_drv.o eapi_drv_lib.o
