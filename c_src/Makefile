#
# We assume that eapi has been checked out as well into an 
# overlaid directory structure by 
# meta-exosense/recipes-exodev/can/erlang-can.bb
#

PRIVDIR := ../priv

all : $(PRIVDIR)/can_sock_drv.so 

override CFLAGS += -fpic -shared -I ../../../erlang-dthread-*/git/include/


$(PRIVDIR)/can_sock_drv.so : can_sock_drv.o
	$(CC) -shared -fpic $(LDFLAGS) ../../../erlang-dthread-*/git/priv/dthread_drv.so -o $@ $^ 

clean:
	$(RM) -f $(PRIVDIR)/can_sock_drv.so can_sock_drv.o

