#!/bin/bash
# script MUST be executed inside of root terminal run ($ sudo su) to get there
# the script MUST be executed from the directory it resides in (ExarKernelDriver)
# ie
# uname@machinename:.../ExarKernelDriver$
if [ "$($PWD | sed 's/^.*\///g')" = "ExarKernelDriver" ] ; then
    echo "please navigate root terminal to the ExarKernelDriver directory and rerun code"
    exit
fi

# unload the xr module if it is already loaded in the kernel
# this allows us to use our modified module
# if the module is already loaded this method doesn't affect anything
modprobe -r xr_usb_serial_common

# Run the driver makefile
make

# Copy Driver Module into Modules
cp xr_usb_serial_common.ko /lib/modules/"$(uname -r)"/kernel/drivers/usb/serial

# Refresh Modules
depmod

# make module load at boot time by adding "xr_usb_serial_common" on its own line to /etc/modules
echo "xr_usb_serial_common" | tee -a /etc/modules

# Load the module into Kernel
insmod /lib/modules/"$(uname -r)"/kernel/drivers/usb/serial/xr_usb_serial_common.ko

# Check if SC4 Device Connected
BUS_ID=""
if grep -q v2890p0213 /sys/bus/usb/devices/*/modalias; then
	# If Device connected check what driver is bound to it
    # this gets the driver numbers just choose 1

    BUS_ID=$(grep v2890p0213 /sys/bus/usb/devices/*/modalias | head -1 | sed 's/\/sys\/bus\/usb\/devices\///g;s/\/.*//g')
    if [ -d "/sys/bus/usb/drivers/cdc_acm/$BUS_ID" ] ; then
        # need to unbind from acm
        echo -n "$BUS_ID" > /sys/bus/usb/drivers/cdc_acm/unbind
        
    elif [ -d "/sys/bus/usb/drivers/cdc_xr_usb_serial/$BUS_ID" ]; then
        echo SC4 already bound to XR no need to rebind
    elif [ -d "/sys/bus/usb/devices/$BUS_ID/driver" ]; then
        echo "$BUS_ID" Appears to be bound to another unrecognized driver
        echo "please unbind from this driver then reattempt this process"
    fi
    
else
	# we not connected
    read -n 1 -s -r -p "Please Insert the SC HUB if you haven't already and press any key to continue"
fi

if  [ ! -d "/sys/bus/usb/drivers/cdc_xr_usb_serial/$BUS_ID" ] ; then
    # bind hub to the loaded xr driver
    echo -n "$BUS_ID" > /sys/bus/usb/drivers/cdc_xr_usb_serial/bind
fi

cd ..
if [ ! -d "/sys/bus/usb/drivers/cdc_xr_usb_serial/$BUS_ID" ]; then
    # one last check
    echo Something went wrong it does not appear as though the SC-HUB is bound to the XR Driver
else
    echo "Teknic Driver Install appears to have succeeded"
fi
