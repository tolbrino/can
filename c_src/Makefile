PRIVDIR := ../priv

all : $(PRIVDIR)/can_sock_drv.so

override CFLAGS + -fpic -shared

$(PRIVDIR)/can_sock_drv.so : can_sock_drv.o can_sock_drv_impl.o
	$(CC) -shared -fpic $(LDFLAGS) -o $@ $^

clean:
	$(RM) -f $(PRIVDIR)/can_sock_drv.so can_sock_drv.o can_sock_drv_impl.o
