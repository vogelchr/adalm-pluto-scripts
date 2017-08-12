#!/bin/sh
set -e

# see https://www.plutosdr.com/viewtopic.php?t=5

if ! [ -d libiio ] ; then
	git clone https://github.com/analogdevicesinc/libiio.git
fi

if ! [ -d bd.libiio ] ; then
	(
		mkdir bd.libiio
		cd bd.libiio
		cmake \
			-D CMAKE_BUILD_TYPE=RELEASE \
			-D CMAKE_INSTALL_PREFIX=/opt/pluto \
			-D INSTALL_UDEV_RULE=OFF \
			../libiio
		make
		make install
	)
fi

if ! [ -d libad9361-iio ] ; then
	git clone https://github.com/analogdevicesinc/libad9361-iio.git
fi

if ! [ -d bd.libad9361-iio ] ; then
	(
		mkdir bd.libad9361-iio
		cd bd.libad9361-iio
		cmake \
			-D CMAKE_BUILD_TYPE=RELEASE \
			-D CMAKE_INSTALL_PREFIX=/opt/pluto \
			../libad9361-iio
		make
		make install
	)
fi

if ! [ -d gr-iio ] ; then
	git clone https://github.com/analogdevicesinc/gr-iio.git
fi

if ! [ -d bd.gr-iio ] ; then
	(
		mkdir bd.gr-iio
		cd bd.gr-iio
		cmake \
			-D CMAKE_BUILD_TYPE=RELEASE \
			-D CMAKE_INSTALL_PREFIX=/opt/pluto \
			../gr-iio
		make -j4
		make install
	)
fi

###
# then symlink /opt/pluto/lib/python2.7/site-packages/gnuradio/iio to
#              /usr/lib/python2.7/site-packages/gnuradio/iio
#
# and in .gnuradio/grc.conf set your local blocks path to /opt/pluto/..... (tbd)
###