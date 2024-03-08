# Pi_Eyes


## Building fbx2
run: `sudo apt update && sudo apt upgrade -y`

### Install the following dependencies:
* libgpiod-dev
* wayland-protocols
* libwayland-dev
* libraspberrypi-dev
* raspberrypi-kernel-headers

### Enable SPI for Both Displays
Make the following changes to /boot/firmware/config.txt

Uncomment:
`dtparam=spi=on`

Add:
```
dtparam=spi1=on
hdmi_force_hotplug=1  # required for cases when HDMI is not plugged in!
dtoverlay=spi1-3cs
```

Then reboot.

### Build
Copy **Pi_Eyes** folder to root folder and run:
```
cd Pi_Eyes
make
```

### Run

For 240x240 IPS displays, run:
`./fbx2 -i`
