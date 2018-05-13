#!/bin/bash
export PATH=$PATH:/home/lukasz/Desktop/work/AMU/tools/stm32flash/OUTPUT/bin/


IMAGE_HEX=$1
if [ -n "$IMAGE_HEX" ]; then
#	stm32flash -w $IMAGE_HEX -v /dev/ttyUSB -b115200
	stm32flash -w $IMAGE_HEX -v /dev/ttyUSB0 -b460800

	if [ $? -eq 0 ]; then
		echo "[RESULT] Flashing succesfull!"
	else
		echo "[RESULT] Errors during flashing process..."
	fi
else
	echo "Image file not given!"
fi

