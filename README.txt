Just a few scripts to compile the software needed for the
ADALM Pluto SDR from Analog Devices.

buils.sh :
	builds (and installs to /opt/pluto) the iio-libraries and gnuradio
	blocks. You need to have created /opt/pluto with write permissions
	for your user.

install.sh :
	create symlinks for python libraries, copy the udev rules file
	to the appropriate place, add /opt/pluto/lib to ld.so.conf.d.


Before you run gnuradio, add the following to ~/.gnuradio/config.conf:

	[grc]
	local_blocks_path = /opt/pluto/share/gnuradio/grc/blocks
