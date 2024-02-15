PKG_CONFIG ?= pkg-config

CFLAGS=-Wall -Ofast -fomit-frame-pointer -funroll-loops \
 -I/opt/vc/include \
 -I/opt/vc/include/interface/vcos/pthreads \
 -I/opt/vc/include/interface/vmcs_host \
 -I/opt/vc/include/interface/vmcs_host/linux \
 -L/opt/vc/lib

LIBS=-pthread -lrt -lm -lbcm_host -lgpiod

# Host deps
WAYLAND_FLAGS = $(shell $(PKG_CONFIG) wayland-client --cflags --libs)

# Build deps
WAYLAND_SCANNER = $(shell $(PKG_CONFIG) --variable=wayland_scanner wayland-scanner)

HEADERS=wlr-screencopy-unstable-v1.h xdg-output-unstable-v1.h
SOURCES=wlr-screencopy-unstable-v1.c xdg-output-unstable-v1.c fbx2.c


all: fbx2

fbx2: $(HEADERS) $(SOURCES)
	cc $(CFLAGS) $(SOURCES) -o fbx2 $(LIBS) $(WAYLAND_FLAGS)
	strip fbx2

wlr-screencopy-unstable-v1.h:
	$(WAYLAND_SCANNER) client-header protocols/wlr-screencopy-unstable-v1.xml wlr-screencopy-unstable-v1.h

wlr-screencopy-unstable-v1.c:
	$(WAYLAND_SCANNER) private-code protocols/wlr-screencopy-unstable-v1.xml wlr-screencopy-unstable-v1.c

xdg-output-unstable-v1.h:
	$(WAYLAND_SCANNER) client-header protocols/xdg-output-unstable-v1.xml xdg-output-unstable-v1.h

xdg-output-unstable-v1.c:
	$(WAYLAND_SCANNER) private-code protocols/xdg-output-unstable-v1.xml xdg-output-unstable-v1.c

.PHONY: clean
clean:
	rm -f fbx2
