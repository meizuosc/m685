#!/system/bin/sh

#  ****************************************************************************
function Setup
{
	export fpc_sensor_device=/dev/fpc1020
	export fpc_spi_path=/sys/bus/spi/devices/spi1.0

	export fpc_multi_capture=3

	export fpc_image_dir=/data/fpc_capture
	mkdir -p ${fpc_image_dir}
}

#  ****************************************************************************

