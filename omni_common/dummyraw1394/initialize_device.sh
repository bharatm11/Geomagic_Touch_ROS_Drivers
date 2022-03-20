#!/bin/bash

FILENAME=$(basename "$0")
usage()
{
cat << EOF

usage: rosrun omni_common $FILENAME [options]

This script compiles (if required) the dummyraw1394 module and insert it in the kernel so we can find the phantom_omni.

OPTIONS:
   -h     Show this message
   -c     Compile the dummyraw. Required for new kernels.
   -v     Verbose (TODO)
   
EOF
}

COMPILE=0
VERBOSE=0
while getopts hc opts; do
  case ${opts} in
    h)
      usage
      exit 1 ;;
    c) 
      COMPILE=1 ;;
    v) 
      VERBOSE=1 ;;
  esac
done

# Locate the dummyraw1394 folder
OMNI_PKG=`rospack find omni_common`
RAW_FOLDER=$OMNI_PKG/dummyraw1394

if [ $COMPILE = 1 ]; then
  # Compile the dummyraw1394  
  cd $RAW_FOLDER
  make all
  cd -
fi

if [[ ! -f $RAW_FOLDER/raw1394.ko ]]; then
  tput setaf 1
  echo "$RAW_FOLDER/raw1394.ko not found"
  tput setaf 4
  echo "Compile the dummyraw: rosrun omni_common $FILENAME -c"
  tput sgr0
  exit 1
fi

# Insert the module in the kernel
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
