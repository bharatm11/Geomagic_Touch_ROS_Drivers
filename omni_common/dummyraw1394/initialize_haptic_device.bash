#!/bin/bash
# This script compiles the dummyraw1394 module and insert it in the
# kernel so we can find the phantom_omni.

#~ Locate the dummyraw1394 folder and compile the dummyraw1394
OMNI_PKG=`rospack find omni_common`
RAW_FOLDER=$OMNI_PKG/dummyraw1394
cd $RAW_FOLDER
make all

#~ Insert the module
tput setaf 4
echo "Creating link: /dev/fw1 --> /dev/raw1394"
tput sgr0
`sudo ln /dev/fw1 /dev/raw1394`
tput setaf 4
echo "Allowing access to firewire interface"
tput sgr0
`sudo chmod 0777 /dev/fw1`
`sudo chmod 0777 /dev/raw1394`
tput setaf 4
echo "Inserting dummyraw1394 in the kernel"
tput sgr0
`sudo insmod $RAW_FOLDER/raw1394.ko`
cd -
