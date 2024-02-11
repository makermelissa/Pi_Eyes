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
WAYLAND_PROTOCOLS_DIR = $(shell $(PKG_CONFIG) wayland-protocols --variable=protocols)

# Build deps
WAYLAND_SCANNER = $(shell pkg-config --variable=wayland_scanner wayland-scanner)

all: fbx2

fbx2: fbx2.c
	$(WAYLAND_SCANNER) client-header protocols/wlr-screencopy-unstable-v1.xml wlr-screencopy-unstable-v1.h
	$(WAYLAND_SCANNER) private-code protocols/wlr-screencopy-unstable-v1.xml wlr-screencopy-unstable-v1.c
	cc $(CFLAGS) fbx2.c -o fbx2 $(LIBS)
	strip fbx2

clean:
	rm -f fbx2
