// Framebuffer-copy-to-two-SPI-screens utility for "Pi Eyes" project.
// Requires a Raspberry Pi Model A+, B+, Zero, Pi 2 or Pi 3 (older models
// lack the auxiliary SPI port and will not work).  Uses two RGB screens
// with SPI interface, either:
//  - SSD1351 OLED   www.adafruit.com/products/1431  -or-
//  - ST7789 IPS TFT www.adafruit.com/products/3787
//  - ST7735 TFT LCD www.adafruit.com/products/2088 ("green tab" version)
// NOT COMPATIBLE WITH OTHER DISPLAYS, PERIOD.

// Enable both the primary and auxiliary SPI devices in /boot/config.txt
// and enable device tree overlay for the aux port (recent Raspbian Jessie
// releases include this overlay, but not enabled by default):
//     dtparam=spi=on
//     dtparam=spi1=on
//     dtoverlay=spi1-3cs
// If a project uses an I2C analog-to-digital converter, also enable that
// interface in /boot/config.txt:
//     dtparam=i2c_arm=on
// Increase the spidev buffer size to 8K by appending to /boot/cmdline.txt:
//     spidev.bufsiz=8192
// The latter improves frame rates and avoids some spi1 (2nd eye) glitching!
// THE ABOVE STEPS ARE ALL HANDLED BY THE pi-eyes.sh INSTALLER SCRIPT HERE:
// https://github.com/adafruit/Raspberry-Pi-Installer-Scripts

// Must be run as root (e.g. sudo fbx2), because hardware.  Options:
// -o, -t or -i to select OLED, TFT or IPS display
// -b ### to specify bitrate (default is based on screen type)
// -w ### to specify number of frames between pixel window writes (default 1)
// -s to print FPS while running (default is silent)

// This code works regardless of screen resolution and aspect ratio, but
// ideally should be set for 640x480 pixels for 128x128 screens (OLED or
// TFT), or 1280x720 for 240x240 screens (IPS) for optimal interpolation.
// Do this even if no monitor attached; this still configures the
// framebuffer.  In /boot/config.txt:
//     hdmi_force_hotplug=1
//     hdmi_group=2
//     hdmi_mode=87
//     hdmi_cvt=640 480 60 1 0 0 0
// or
//     hdmi_cvt=1280 720 60 1 0 0 0
// (Again, the pi-eyes.sh installer script takes care of this.)

// This code runs in the background for an accompanying eye rendering
// application.  This separation allows for new and different custom eye
// renderers to be written in whatever language or library of choice.

// To determine regions copied to each SPI screen: picture the screen
// divided in half, two equal regions side-by-side.  Centered within each
// region, a 256x256 pixel square (for OLED and TFT) is scaled to 50%
// (with 2x2 area filtering) to produce 128x128 bitmaps send to each
// screen.  For IPS screens, the squares are 480x480 pixels and scaled to
// 240x240.  The framebuffer size and the eye-rendering code both need
// some configuration to work properly with all this; it's not entirely
// automagic (earlier versions tried that, but certain screen sizes caused
// problems with the dispmanx functions, so it needs to be a little more
// manual for now).  The 2x2 filtering provides additional antialiasing
// for OpenGL, which offers at most 4X multisampling (2x2) on Raspberry Pi,
// so effectively now 16X (4x4).  Don't bother with higher res; this yields
// lesser quality downsampling!

// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.
// Insights from Tasanakorn's fbcp tool: github.com/tasanakorn/rpi-fbcp

#include <gpiod.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <bcm_host.h>
#include <wayland-client.h>
#include <assert.h>
#include <png.h>
#include "wlr-screencopy-unstable-v1.h"

// CONFIGURATION AND GLOBAL STUFF ------------------------------------------

#define DC_PIN    5             // These pins connect
#define RESET_PIN 6             // to BOTH screens
#define DCMASK    (1 << DC_PIN) // GPIO pin bitmasks of reset
#define	CONSUMER  "fbx2"        // For use with gpiod

// Main and auxiliary SPI buses are used concurrently, thus MOSI and SCLK
// are unique to each screen, plus CS as expected.  First screen ("right
// eye" -- positioned on the left from observer's POV) connects to SPI0,
// which is on Broadcom GPIO pins #10 (MOSI), #11 (SCLK) and #8 (CE0).
// Second screen ("left eye" -- positioned on right) connects to SPI1, on
// GPIO #20 (MOSI), #21 (SCLK) and #16 (CE2).  CE2 (rather than CE1) is
// used for 2nd screen as it simplified PCB routing

#define DEBUG            // Uncomment to enable debug output
#define SCREEN_OLED      0 // Compatible screen types
#define SCREEN_TFT_GREEN 1 // Just these three
#define SCREEN_IPS       2 // Other types WILL NOT WORK!

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
uint8_t screenType = SCREEN_OLED;

// Screen initialization commands and data.  Derived from Adafruit Arduino
// libraries, stripped bare here...see corresponding original libraries for
// a more in-depth explanation of each screen command.
static const uint8_t initOLED[] = {
  // OLED initialization distilled from Adafruit SSD1351 Arduino library
  0xFD,  1, 0x12,             // Command lock setting, unlock 1/2
  0xFD,  1, 0xB1,             // Command lock setting, unlock 2/2
  0xAE,  0,                   // Display off
  0xB3,  1, 0xF0,             // Clock div (F1=typical, F0=faster refresh)
  0xCA,  1, 0x7F,             // Duty cycle (128 lines)
  0xA2,  1, 0x00,             // Display offset (0)
  0xA1,  1, 0x00,             // Start line (0)
  0xA0,  1, 0x74,             // Set remap, color depth (5/6/5)
  0xB5,  1, 0x00,             // Set GPIO (disable)
  0xAB,  1, 0x01,             // Function select (internal regulator)
  0xB4,  3, 0xA0, 0xB5, 0x55, // Set VSL (external)
  0xC1,  3, 0xFF, 0xA3, 0xFF, // Contrast A/B/C
  0xC7,  1, 0x0F,             // Contrast master (reset)
  0xB1,  1, 0x32,             // Set precharge & discharge
  0xBB,  1, 0x07,             // Precharge voltage of color A/B/C
  0xB2,  3, 0xA4, 0x00, 0x00, // Display enhanvement
  0xB6,  1, 0x01,             // Precharge period
  0xBE,  1, 0x05,             // Set VcomH (0.82 x Vcc)
  0xA6,  0,                   // Normal display
  0xAF,  0,                   // Display on
  0xB8, 64,                   // Gamma table, 64 values, no delay
    0x00, 0x08, 0x0D, 0x12, 0x17, 0x1B, 0x1F, 0x22,
    0x26, 0x2A, 0x2D, 0x30, 0x34, 0x37, 0x3A, 0x3D,
    0x40, 0x43, 0x46, 0x49, 0x4C, 0x4F, 0x51, 0x54,
    0x57, 0x59, 0x5C, 0x5F, 0x61, 0x64, 0x67, 0x69,
    0x6C, 0x6E, 0x71, 0x73, 0x76, 0x78, 0x7B, 0x7D,
    0x7F, 0x82, 0x84, 0x86, 0x89, 0x8B, 0x8D, 0x90,
    0x92, 0x94, 0x97, 0x99, 0x9B, 0x9D, 0x9F, 0xA2,
    0xA4, 0xA6, 0xA8, 0xAA, 0xAD, 0xAF, 0xB1, 0xB3,
  0x00 },                     // EOD
initTFT[] = {
  // TFT initialization from Adafruit ST7735 Arduino library ('green tab')
  0x01, 0x80, 150,            // Software reset, 0 args, w/150ms delay
  0x11, 0x80, 255,            // Out of sleep mode, 0 args, w/500ms delay
  0xB1,    3,                 // Frame rate ctrl - normal mode, 3 args:
    0x01, 0x2C, 0x2D,         // Rate = fosc/(1x2+40) * (LINE+2C+2D)
  0xB2,    3,                 // Frame rate control - idle mode, 3 args:
    0x01, 0x2C, 0x2D,         // Rate = fosc/(1x2+40) * (LINE+2C+2D)
  0xB3,    6,                 // Frame rate ctrl - partial mode, 6 args:
    0x01, 0x2C, 0x2D,         // Dot inversion mode
    0x01, 0x2C, 0x2D,         // Line inversion mode
  0xB4, 1, 0x07,              // Display inversion ctrl: no inversion
  0xC0,    3,                 // Power control 1, 3 args, no delay:
    0xA2, 0x02, 0x84,         // -4.6V, AUTO mode
  0xC1,    1, 0xC5,           // Pwr ctrl 2: VGH25=2.4C VGSEL=-10 VGH=3*AVDD
  0xC2,    2, 0x0A, 0x00,     // Pwr ctrl 3: opamp current small, boost freq
  0xC3,    2, 0x8A, 0x2A,     // Pwr ctrl 4: BCLK/2, Opamp small & med low
  0xC4,    2, 0x8A, 0xEE,     // Power control 5, 2 args, no delay
  0xC5,    1, 0x0E,           // Power control, 1 arg, no delay
  0x20,    0,                 // Don't invert display, no args, no delay
  0x36,    1, 0xC8,           // MADCTL: row addr/col addr, bottom-to-top
  0x3A,    1, 0x05,           // set color mode, 1 arg: 16-bit color
  0x2A,    4,                 // Column addr set, 4 args, no delay:
    0x00, 0x00, 0x00, 0x7F,   // XSTART = 0, XEND = 127
  0x2B,    4,                 // Row addr set, 4 args, no delay:
    0x00, 0x00, 0x00, 0x7F,   // XSTART = 0, XEND = 127
  0xE0,   16,                 // ???, 16 args, no delay:
    0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
    0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
  0xE1,   16,                 // ???, 16 args, no delay:
    0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
    0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
  0x13, 0x80, 10,             // Normal display on, no args, w/10ms delay
  0x29, 0x80, 100,            // Main screen turn on, no args w/100ms delay
  0x00 },                     // EOD
initIPS[] = {
  // IPS initialization
  0x01, 0x80,       150,      // Soft reset, no args, 150 ms delay
  0x11, 0x80,       255,      // Out of sleep, no args, 500 ms delay
  0x3A, 0x81, 0x55,  10,      // COLMOD, 1 arg, 10ms delay
  0x36,    1, 0x00,           // MADCTL, 1 arg (RGB), no delay,
  0x26,    1, 0x02,           // GAMSET, 1 arg (curve 2 (G1.8)), no delay
  0xBA,    1, 0x04,           // DGMEN, 1 arg (enable gamma), no delay,
  0x21, 0x80,        10,      // INVON, no args, 10 ms delay
  0x13, 0x80,        10,      // NORON, no args, 10 ms delay
  0x29, 0x80,       255,      // DISPON, no args, 500 ms delay
  0x00 },
winOLED[] = {
  0x15, 2, 0x00, 0x7F,         // Column range
  0x75, 2, 0x00, 0x7F,         // Row range
  0x5C,                        // Write to display RAM
  0x00 },                      // EOD
winTFT[] = {
  0x2A, 4, 0, 2, 0, 129,       // Column set, xtart, xend (MSB first)
  0x2B, 4, 0, 3, 0, 130,       // Row set, ystart, yend (MSB first)
  0x2C,                        // RAM write
  0x00 },                      // EOD
winIPS[] = {
  0x2A, 4, 0, 0, 0, 239,       // CASET (column set) xstart, xend (MSB first)
  0x2B, 4, 0, 0, 0, 239,       // RASET (row set) ystart, yend (MSB first)
  0x2C,                        // RAMWR (RAM write)
  0x00 };                      // EOD

// Further data specific to each screen type: pixel dimensions, maximum
// stable SPI bitrate, pointer to initialization commands above.
// Datasheet figures for SPI screen throughput don't always match reality;
// factors like wire length and quality of connections, phase of the moon
// and other mysterious influences play a part...run them too fast and the
// screen will exhibit visual glitches or just not initialize correctly.
// You may need to use the -b command-line option to set the bitrate.
static const struct {
	const int      width;   // Width in pixels
	const int      height;  // Height in pixels
	const int      bitrate; // Default stable SPI bitrate
	const uint8_t *init;    // Pointer to initialization command list
	const uint8_t *win;     // Pointer to window command list
} screen[] = {
  { 128, 128, 10000000, initOLED, winOLED },
  { 128, 128, 12000000, initTFT , winTFT  },
  { 240, 240, 80000000, initIPS , winIPS  } };

// The concurrent nature of this code plus the eye renderer (which may be
// performing heavy math) can be taxing, mostly on single-core systems; a
// balance must be established or one task or the other will suffer (and
// frame rates with it).  Limiting the peak frame rate of this code can
// be accomplished by selecting a lower SPI bitrate.

// Per-eye structure
static struct {
	int        fd;                // SPI file descriptor
	uint16_t  *buf[2];            // Double-buffered eye data 16 BPP
	pthread_t  thread;            // Thread ID of eye's spiThreadFunc()
	struct spi_ioc_transfer xfer; // ioctl() transfer struct
} eye[2];

static struct {
	struct wl_buffer *wl_buffer;
	void *data;
	enum wl_shm_format format;
	int width, height, stride;
	bool y_invert;
} buffer;

struct format {
	enum wl_shm_format wl_format;
	bool is_bgr;
};

// wl_shm_format describes little-endian formats, libpng uses big-endian
// formats (so Wayland's ABGR is libpng's RGBA).
static const struct format formats[] = {
	{WL_SHM_FORMAT_XRGB8888, true},
	{WL_SHM_FORMAT_ARGB8888, true},
	{WL_SHM_FORMAT_XBGR8888, false},
	{WL_SHM_FORMAT_ABGR8888, false},
};

static pthread_barrier_t barr;          // For thread synchronization
static uint8_t           bufIdx = 0;    // Double-buffering index
static int               bufsiz = 4096; // SPI block xfer size (4K default)
static struct spi_ioc_transfer xfer = {
  .rx_buf        = 0, // ioctl() transfer structure for issuing
  .delay_usecs   = 0, // commands (not pixel data) to both screens.
  .bits_per_word = 8,
  .pad           = 0,
  .tx_nbits      = 0,
  .rx_nbits      = 0,
  .cs_change     = 0 };

int32_t output_width, output_height;

enum screencopy_status {
	SCREENCOPY_STOPPED = 0,
	SCREENCOPY_IN_PROGRESS,
	SCREENCOPY_FAILED,
	SCREENCOPY_FATAL,
	SCREENCOPY_DONE,
};

struct gpiod_chip *chip = NULL;
struct gpiod_line *reset_line = NULL;
struct gpiod_line *dc_line = NULL;

struct zwlr_screencopy_manager_v1 *manager;
struct zwlr_screencopy_frame_v1 *frame;

struct wl_display* display = NULL;   // Wayland Display
struct wl_registry* registry = NULL; // Wayland Global Registry
struct wl_output* output = NULL;     // Wayland Output
struct wl_shm* shm = NULL;           // Wayland Shared Memory

bool buffer_copy_done = false;
enum screencopy_status status;

// UTILITY FUNCTIONS -------------------------------------------------------

#define COMMAND 0 // Values for last argument
#define DATA    1 // to dcX2() function below

// Issue data or command to both SPI displays:
static void dcX2(uint8_t x, uint8_t dc) {
    gpiod_line_set_value(dc_line, dc ? 1 : 0);
	xfer.tx_buf = (uintptr_t)&x; // Uses global xfer struct,
	xfer.len    = 1;            // as most elements don't change
	(void)ioctl(eye[0].fd, SPI_IOC_MESSAGE(1), &xfer);
	(void)ioctl(eye[1].fd, SPI_IOC_MESSAGE(1), &xfer);
}

// Issue a list of commands (and arguments, delays) to both displays:
static void commandList(const uint8_t *ptr) { // pass in -> command list
	int i, j, ms;
	for(i=0; (j=ptr[i++]);) {                // 0 = EOD
		dcX2(j, COMMAND);                // First byte = command
		j  = ptr[i++];                   // Delay flag | num args
		ms = j & 0x80;                   // Mask delay flag
		j &= ~0x80;                      // Mask arg count
		while(j--) dcX2(ptr[i++], DATA); // Issue args (data)
		if(ms) {                         // Delay flag set?
			ms = ptr[i++];           // Next byte = milliseconds
			if(ms == 255) ms = 500;  // If 255, make it 500
			usleep(ms * 1000);
		}
	}
}

// Each eye's SPI transfers are handled by a separate thread, to provide
// concurrent non-blocking transfers to both displays while the main thread
// processes the next frame.  Same function is used for both eyes, each in
// its own thread; eye index is passed in.
void *spiThreadFunc(void *data) {
	int      i = *(uint8_t *)data; // Pass in eye index
	uint32_t bytesThisPass, bytesToGo, screenBytes =
	  screen[screenType].width * screen[screenType].height * 2;

	for(;;) {
		// POSIX thread "barriers" are used to sync the main thread
		// with the SPI transfer threads.  This needs to happen at
		// two points: just after finishing the pixel data transfer,
		// and just before starting the next, so that the screen-
		// rectangle commands (which fiddle the shared 'DC' pin)
		// don't corrupt the transfer.  Both barrier waits occur at
		// the *top* of this function to match up with the way the
		// main() loop is entered; it processes a frame before
		// waiting for prior transfers to finish.
		pthread_barrier_wait(&barr); // This is the 'after' wait
		pthread_barrier_wait(&barr); // And the 'before' wait

		eye[i].xfer.tx_buf = (uintptr_t)eye[i].buf[bufIdx];
		bytesToGo = screenBytes;
		do {
			bytesThisPass = bytesToGo;
			if(bytesThisPass > bufsiz) bytesThisPass = bufsiz;
			eye[i].xfer.len = bytesThisPass;
			(void)ioctl(eye[i].fd, SPI_IOC_MESSAGE(1),
			  &eye[i].xfer);
			eye[i].xfer.tx_buf += bytesThisPass;
			bytesToGo          -= bytesThisPass;
		} while(bytesToGo > 0);
	}
	return NULL;
}

static struct wl_buffer *create_shm_buffer(enum wl_shm_format fmt, int width, int height, int stride, void **data_out) {
	int size = stride * height;

	const char shm_name[] = "/fbcp-screencopy";
	int fd = shm_open(shm_name, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
	if (fd < 0) {
		fprintf(stderr, "shm_open failed\n");
		return NULL;
	}
	shm_unlink(shm_name);

	int ret;
	while ((ret = ftruncate(fd, size)) == EINTR) {
		// No-op
	}
	if (ret < 0) {
		close(fd);
		fprintf(stderr, "ftruncate failed\n");
		return NULL;
	}

	void *data = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (data == MAP_FAILED) {
		perror("mmap failed");
		close(fd);
		return NULL;
	}

	struct wl_shm_pool *pool = wl_shm_create_pool(shm, fd, size);
	close(fd);
	struct wl_buffer *buffer = wl_shm_pool_create_buffer(pool, 0, width, height,
		stride, fmt);
	wl_shm_pool_destroy(pool);

	*data_out = data;
	return buffer;
}

static void write_image(char *filename, enum wl_shm_format wl_fmt, int width,
		int height, int stride, bool y_invert, png_bytep data) {
	const struct format *fmt = NULL;
	for (size_t i = 0; i < sizeof(formats) / sizeof(formats[0]); ++i) {
		if (formats[i].wl_format == wl_fmt) {
			fmt = &formats[i];
			break;
		}
	}
	if (fmt == NULL) {
		fprintf(stderr, "unsupported format %"PRIu32"\n", wl_fmt);
		exit(EXIT_FAILURE);
	}

	FILE *f = fopen(filename, "wb");
	if (f == NULL) {
		fprintf(stderr, "failed to open output file\n");
		exit(EXIT_FAILURE);
	}

	png_structp png =
		png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	png_infop info = png_create_info_struct(png);

	png_init_io(png, f);

	png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGBA,
		PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
		PNG_FILTER_TYPE_DEFAULT);

	if (fmt->is_bgr) {
		png_set_bgr(png);
	}

	png_write_info(png, info);

	for (size_t i = 0; i < (size_t)height; ++i) {
		png_bytep row;
		if (y_invert) {
			row = data + (height - i - 1) * stride;
		} else {
			row = data + i * stride;
		}
		png_write_row(png, row);
	}

	png_write_end(png, NULL);

	png_destroy_write_struct(&png, &info);

	fclose(f);
}

static void deinit(void) {
    if (buffer.wl_buffer != NULL) {
	    wl_buffer_destroy(buffer.wl_buffer);
	    munmap(buffer.data, buffer.stride * buffer.height);     // Memory Unmap
    }
    if (output) {
        wl_output_destroy(output);
    }
    if (shm) {
        wl_shm_destroy(shm);
    }
    if (manager) {
        zwlr_screencopy_manager_v1_destroy(manager);
    }
    if (registry) {
        wl_registry_destroy(registry);
    }
    if (display) {
        wl_display_disconnect(display);
    }
    if (reset_line) {
        gpiod_line_release(reset_line);
    }
    if (dc_line) {
        gpiod_line_release(dc_line);
    }
    if (chip) {
        gpiod_chip_close(chip);
    }
    pthread_barrier_destroy(&barr);
}

// Crude error handler (prints message, exits program with status code)
static int err(int code, char *string) {
    deinit();
	(void)puts(string);
	exit(code);
}

// EVENT HANDLERS ----------------------------------------------------------

static void registry_add(void *data, struct wl_registry *registry, uint32_t id, const char *interface, uint32_t version) {
#ifdef DEBUG
    printf("Loading Interface: %s\n", interface);
#endif
	if (strcmp(interface, wl_output_interface.name) == 0 && output == NULL) {
		output = wl_registry_bind(registry, id, &wl_output_interface, 3);
        return;
    }
    if (strcmp(interface, wl_shm_interface.name) == 0) {
		shm = wl_registry_bind(registry, id, &wl_shm_interface, 1);
        return;
	}
    if (strcmp(interface, zwlr_screencopy_manager_v1_interface.name) == 0) {
        manager = wl_registry_bind(registry, id, &zwlr_screencopy_manager_v1_interface, MIN(3, version));
	}
}

static void registry_remove(void *data, struct wl_registry *registry, uint32_t name) {
	// Not used for this
}

static void output_handle_geometry(void *data, struct wl_output *wl_output,
				   int32_t x, int32_t y, int32_t phys_width,
				   int32_t phys_height, int32_t subpixel,
				   const char* make, const char* model,
				   int32_t transform) {
}

static void output_handle_mode(void *data, struct wl_output *wl_output,
			       uint32_t flags, int32_t width, int32_t height,
			       int32_t refresh) {
    output_width = width;
    output_height = height;
#ifdef DEBUG
	printf("Width: %d, Height: %d\n", width, height);
#endif
}

static void output_handle_done(void *data, struct wl_output *wl_output) {
}

static void output_handle_scale(void *data, struct wl_output *wl_output, int32_t factor) {
}

static void frame_handle_buffer(void* data,
			      struct zwlr_screencopy_frame_v1* frame,
			      enum wl_shm_format format, uint32_t width,
			      uint32_t height, uint32_t stride) {
	buffer.format = format;
	buffer.width = width;
	buffer.height = height;
	buffer.stride = stride;

	// Make sure the buffer is not allocated
	assert(!buffer.wl_buffer);
	buffer.wl_buffer = create_shm_buffer(format, width, height, stride, &buffer.data);
	if (buffer.wl_buffer == NULL) {
		err(9, "failed to create buffer\n");
	}

    zwlr_screencopy_frame_v1_copy(frame, buffer.wl_buffer);
}

static void frame_handle_flags(void* data,
			     struct zwlr_screencopy_frame_v1* frame,
			     uint32_t flags) {

	buffer.y_invert = !!(flags & ZWLR_SCREENCOPY_FRAME_V1_FLAGS_Y_INVERT);
}

static void frame_handle_ready(void* data,
			     struct zwlr_screencopy_frame_v1* frame,
			     uint32_t sec_hi, uint32_t sec_lo, uint32_t nsec) {

	buffer_copy_done = true;
}

static void frame_handle_failed(void* data,
			      struct zwlr_screencopy_frame_v1* frame) {
    err(8, "Frame copy failed");
}

static void frame_handle_linux_dmabuf(void* data,
			      struct zwlr_screencopy_frame_v1* frame,
			      uint32_t format, uint32_t width, uint32_t height)
{
}

static void frame_handle_buffer_done(void* data,
			      struct zwlr_screencopy_frame_v1* frame)
{
}

static void frame_handle_damage(void* data,
			      struct zwlr_screencopy_frame_v1* frame,
			      uint32_t x, uint32_t y,
			      uint32_t width, uint32_t height)
{

	//wv_buffer_damage_rect(self->front, x, y, width, height);
}

// INIT AND MAIN LOOP ------------------------------------------------------

int main(int argc, char *argv[]) {

	uint8_t showFPS   = 0;
	int     bitrate   = 0, // If 0, use default
	        winFrames = 1, // How often to reset pixel window
	        i, j,
            ret;
    const char* display_name = "wayland-1";
	while((i = getopt(argc, argv, "otib:w:sd:")) != -1) {
		switch(i) {
		   case 'o': // Select OLED screen type
			screenType = SCREEN_OLED;
			break;
		   case 't': // Select TFT screen type
			screenType = SCREEN_TFT_GREEN;
			break;
		   case 'i': // Select IPS screen type
			screenType = SCREEN_IPS;
			break;
		   case 'b': // SPI bitrate
			bitrate = strtol(optarg, NULL, 0);
			break;
		   case 'w': // Number of frames between window sync
			winFrames = strtol(optarg, NULL, 0);
			break;
		   case 's': // Show FPS
			showFPS = 1;
			break;
           case 'd': // Display name
            display_name = optarg;
            break;
		}
	}

	if(!bitrate) bitrate = screen[screenType].bitrate;

	// Get SPI buffer size from sysfs.  Default is 4K.
	FILE *fp;
	if((fp = fopen("/sys/module/spidev/parameters/bufsiz", "r"))) {
		if(fscanf(fp, "%d", &i) == 1) bufsiz = i;
		fclose(fp);
	}

	// GPIO AND SCREEN INIT --------------------------------------------

    bool is_pi5 = false;
	if((fp = fopen("/dev/gpiochip4", "r"))) {   // Pi 5 uses gpiochip4, the older ones use gpiochip0
		is_pi5 = true;
		fclose(fp);
	}
    char *chipname = "gpiochip0";
    if (is_pi5) {
        chipname = "gpiochip4";
    }

	chip = gpiod_chip_open_by_name(chipname);
	if (!chip) {
		err(1, "Open chip failed\n");
	}

    dc_line = gpiod_chip_get_line(chip, DC_PIN);
    if (!dc_line) {
        err(2, "Get dc line failed\n");
    }

	ret = gpiod_line_request_output(dc_line, CONSUMER, 0);
	if (ret < 0) {
		err(3, "Setting dc line as output failed\n");
	}

	reset_line = gpiod_chip_get_line(chip, RESET_PIN);
	if (!reset_line) {
        err(4, "Get reset line failed\n");
	}

	ret = gpiod_line_request_output(reset_line, CONSUMER, 0);
	if (ret < 0) {
		err(5, "Setting reset line as output failed\n");
	}

	if((eye[0].fd = open("/dev/spidev0.0", O_WRONLY|O_NONBLOCK)) < 0)
		err(6, "Can't open spidev0.0, is SPI enabled?");
	if((eye[1].fd = open("/dev/spidev1.2", O_WRONLY|O_NONBLOCK)) < 0)
		err(7, "Can't open spidev1.2, is spi1-3cs overlay enabled?");

	xfer.speed_hz = bitrate;
	uint8_t  mode = SPI_MODE_0;
	for(i=0; i<2; i++) {
		ioctl(eye[i].fd, SPI_IOC_WR_MODE, &mode);
		ioctl(eye[i].fd, SPI_IOC_WR_MAX_SPEED_HZ, bitrate);
		memcpy(&eye[i].xfer, &xfer, sizeof(xfer));
		for(j=0; j<2; j++) {
			if(NULL == (eye[i].buf[j] = (uint16_t *)malloc(
			  screen[screenType].width *
			  screen[screenType].height * sizeof(uint16_t)))) {
				err(5, "Eye buffer malloc failed");
			}
		}
	}

	// INITIALIZE SPI SCREENS ------------------------------------------

	gpiod_line_set_value(reset_line, 1); usleep(5); // Reset high,
	gpiod_line_set_value(reset_line, 0); usleep(5); // low,
	gpiod_line_set_value(reset_line, 1); usleep(5); // high

	commandList(screen[screenType].init); // Send init commands

    // WAYLAND SCREEN CAPTURE INIT ------------------------------------

    // Insights gained from wayvnc:
    // https://github.com/any1/wayvnc
	// Rather than copying framebuffer-to-framebuffer, this code
	// issues screen data directly and concurrently to two 'raw'
	// SPI displays (no secondary framebuffer device / driver).

    bool overlay_cursor = false;
	uint16_t                  *pixelBuf;

    static const struct wl_registry_listener registry_listener = {
        .global = registry_add,
        .global_remove = registry_remove,
    };

    static const struct wl_output_listener output_listener = {
        .geometry = output_handle_geometry,
        .mode = output_handle_mode,
        .done = output_handle_done,
        .scale = output_handle_scale,
    };

	static const struct zwlr_screencopy_frame_v1_listener frame_listener = {
		.buffer = frame_handle_buffer,
		.flags = frame_handle_flags,
		.ready = frame_handle_ready,
		.failed = frame_handle_failed,
		.linux_dmabuf = frame_handle_linux_dmabuf,
		.buffer_done = frame_handle_buffer_done,
		.damage = frame_handle_damage,
	};

    display = wl_display_connect(display_name);  // TODO: Look at starting detached and then attaching later if this doesn't work
	if (!display) {
    	err(6, "Failed to connect to display. Ensure wayland is running with that display name.");
	}

    registry = wl_display_get_registry(display);
	if (!registry) {
		err(7, "Could not locate the wayland compositor object registry");
	}

	wl_registry_add_listener(registry, &registry_listener, NULL);
	wl_display_roundtrip(display); // Initialize the global registry interfaces

	if (!shm) {
		err(8, "Compositor is missing wl_shm\n");
	}

    if (!manager) {
        err(9, "Compositor doesn't support wlr-screencopy-unstable-v1. Exiting.");
    }

	if (output == NULL) {
		err(10, "no output available\n");
	}

    // Set up the output listener to get the geometry
    wl_output_add_listener(output, &output_listener, NULL);

    // Set up the frame listener
    frame = zwlr_screencopy_manager_v1_capture_output(manager, overlay_cursor, output);
    zwlr_screencopy_frame_v1_add_listener(frame, &frame_listener, NULL);

	while (!buffer_copy_done && wl_display_dispatch(display) != -1) {
		// Wait for buffer copy to finish
        // Needed for dimensions of the first frame
	}

	// output_width and output_height are primary display dimensions.
	// Create a 16-bit (5/6/5) offscreen resource that's 1/2 the display
	// size.  GPU bilinear interpolation then yields 2x2 pixel averaging.
	// Note that some resource constraints exist but possibly not
	// documented -- appears that width needs to be a multiple of 32?
	// Not entirely sure.  Point is, while the framebuffer resolution
	// is extremely flexible, not all resolutions will work here, and
	// it may require some configuration, testing and reboots.

	int width  = (output_width  + 1) / 2, // Resource dimensions
	    height = (output_height + 1) / 2;

	// Also determine positions of upper-left corners for the two
	// SPI screens, and corresponding offsets into pixelBuf[].
	// Rendering application will need to observe similar size and
	// position constraints to produce desired results.
	int x, y,
	    offset0 = width * ((height - screen[screenType].height) / 2) +
	             (width / 2 - screen[screenType].width) / 2,
	    offset1 = offset0 + width / 2;

	// screen_resource is an intermediary between framebuffer and
	// main RAM -- VideoCore will copy the primary framebuffer
	// contents to this resource while providing interpolated
	// scaling plus 8/8/8 -> 5/6/5 dithering.
    /*
	if(!(screen_resource = vc_dispmanx_resource_create(
	  VC_IMAGE_RGB565, width, height, &handle))) {
		vc_dispmanx_display_close(display);
		err(8, "Can't create screen buffer");
	}
	vc_dispmanx_rect_set(&rect, 0, 0, width, height);
*/

	// Create a buffer in RAM w/same dimensions as offscreen
	// resource, 16 bits per pixel.
	if(!(pixelBuf = (uint16_t *)malloc(width * height * 2))) {
		err(11, "Can't malloc pixelBuf");
	}

    // Save the buffer as a PNG file as proof we are grabbing the display
	write_image("wayland-screenshot.png", buffer.format, buffer.width,
		buffer.height, buffer.stride, buffer.y_invert, buffer.data);

	// Initialize SPI transfer threads and synchronization barrier
	pthread_barrier_init(&barr, NULL, 3);
	uint8_t aa = 0, bb = 1;
	pthread_create(&eye[0].thread, NULL, spiThreadFunc, &aa);
	pthread_create(&eye[1].thread, NULL, spiThreadFunc, &bb);

	// MAIN LOOP -------------------------------------------------------

	uint32_t  frames=0, t, prevTime = time(NULL);
	uint16_t *src0, *dst0, *src1, *dst1;
	int       winCount = 0,
	          w = screen[screenType].width,
	          h = screen[screenType].height;
	for(;;) {

		// Framebuffer -> scale & dither to intermediary
		//vc_dispmanx_snapshot(display, screen_resource, 0);

		// Intermediary -> main RAM (tried doing some stuff with
		// an 'optimal' bounding rect but it just crashed & burned,
		// so doing full-screen transfers for now).
		//vc_dispmanx_resource_read_data(screen_resource, &rect, pixelBuf, width * 2);

		// Crop & transfer rects to eye buffers, flip hi/lo bytes
		j    = 1 - bufIdx; // Render to 'back' buffer
		src0 = &pixelBuf[offset0];
		src1 = &pixelBuf[offset1];
		dst0 = eye[0].buf[j];
		dst1 = eye[1].buf[j];
		for(y=0; y<h; y++) {
			for(x=0; x<w; x++) {
				dst0[x] = __builtin_bswap16(src0[x]);
				dst1[x] = __builtin_bswap16(src1[x]);
			}
			src0 += width;
			src1 += width;
			dst0 += w;
			dst1 += w;
		}

		// Sync up all threads; wait for prior transfers to finish
		pthread_barrier_wait(&barr);

		// Before pushing data to SPI screens, the pixel 'window'
		// is periodically reset to force screen data pointer back
		// to (0,0).  The pointer automatically 'wraps' when the end
		// of the screen is reached, but a periodic reset provides
		// extra insurance in case of SPI glitches (which would
		// put one or both screens out of sync for all subsequent
		// frames).  Default behavior is to reset on every frame
		// (performance difference is negligible).
		if(++winCount >= winFrames) {
			commandList(screen[screenType].win);
            gpiod_line_set_value(dc_line, 1); // DC high (data)
			winCount = 0;
		}

		// With screen commands now issued, sync up the
		// threads again, they'll start pushing data...
		bufIdx   = 1 - bufIdx;       // Swap buffers
		pthread_barrier_wait(&barr); // Activates data-write thread

		if(showFPS) {
			// Show approx. frames-per-second once per second.
			// This is the update speed of fbx2 alone and is
			// disengaged from the eye-rendering application,
			// which operates at its own unrelated refresh rate.
			frames++;
			if((t = time(NULL)) != prevTime) {
				(void)printf("%d fps\n", frames);
				frames   = 0;
				prevTime = t;
			}
		}
	}

    deinit();
	return 0;
}
