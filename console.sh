#!/bin/bash

NODE=/dev/ttyACM0
BAUDRATE=115200
OUTPUTMAP=crlf
INPUTMAP=lfcrlf
EXTRAPARAMS='--quiet'

if [ -n "$1" ]; then NODE=$1; fi

echo "Press CTRL+A+Q to exit"
picocom $NODE --baud $BAUDRATE --omap $OUTPUTMAP --imap $INPUTMAP $EXTRAPARAMS


