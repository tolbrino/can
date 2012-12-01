#
# We assume that eapi has been checked out as well into an 
# overlaid directory structure by 
# meta-exosense/recipes-exodev/can/erlang-can.bb
#

PRIVDIR := ../priv

all : $(PRIVDIR)/can_sock_drv.so 

override CFLAGS += -fpic -shared -I ../../../erlang-eapi-*/git/include/


$(PRIVDIR)/can_sock_drv.so : can_sock_drv.o can_sock_drv_impl.o
	$(CC) -shared -fpic $(LDFLAGS) ../../../erlang-eapi-*/git/include/eapi_drv.so -o $@ $^ 

clean:
	$(RM) -f $(PRIVDIR)/can_sock_drv.so can_sock_drv.o can_sock_drv_impl.o 

