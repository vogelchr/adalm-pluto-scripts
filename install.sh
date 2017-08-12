#!/bin/sh

python_lib=/lib/python2.7/site-packages/gnuradio/iio

if ! [ -e "/etc/ld.so.conf.d/opt_pluto_lib.conf" ] ; then
	echo "Adding /opt/pluto/lib, lib64 to /etc/ld.so.conf.d/opt_pluto_lib.conf."
	cat <<EOF >/etc/ld.so.conf.d/opt_pluto_lib.conf
/opt/pluto/lib
/opt/pluto/lib64
EOF
	ldconfig
fi

if ! [ -e "/usr${python_lib}" ] ; then
	echo "Creating symlink for /usr/lib/python2.7/site-packages/gnuradio/iio."
	ln -s "/opt/pluto${python_lib}" "/usr${python_lib}"
fi

if ! [ -e "/etc/udev/rules.d/99-adalm-pluto-sdr.rules" ] ; then
	echo "Copying 99-adalm-pluto-sdr.rules to /etc/udev/rules.d."
	install -m644 99-adalm-pluto-sdr.rules /etc/udev/rules.d

	echo "Reloading rules, disconnect and reconnect your ADALM Pluto, then"
	echo "check with /opt/pluto/bin/iio_info -s."
	udevadm control -R
fi