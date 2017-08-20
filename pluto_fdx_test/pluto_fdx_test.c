#include <iio.h>
#include <ad9361.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

#define BANDWIDTH 200000

struct iio_context;
struct iio_channel;
struct iio_buffer;
struct iio_device;

/* devices are 0=RX; 1=TX */
enum pluto_devices  { DEV_RX, DEV_TX, DEV_PHY, DEV_NDEV };
enum pluto_buffers  { BUF_RX, BUF_TX, BUF_NBUF };
/* channes are numbered 0,1 = RX, 2,3 = TX each I and Q */
enum pluto_channels {
	CH_RX_I,
	CH_RX_Q,
	CH_TX_I,
	CH_TX_Q,
	CH_PHY_RX,
	CH_PHY_TX,
	CH_LO_RX,
	CH_LO_TX,
	CH_NCHAN
};

/* devices are identified by their name */
struct iio_dev_info {
	char * nicename;  /* human readable, not used otherwise */
	char * name;      /* device name */
};

static struct iio_dev_info iio_dev_info[DEV_NDEV] = {
	/* [devnum]= ... */
	[DEV_RX]  = { "Rx",     "cf-ad9361-lpc"          },
	[DEV_TX]  = { "Tx",     "cf-ad9361-dds-core-lpc" },
	[DEV_PHY] = { "Phy/LO", "ad9361-phy"            }
};


/* channels are identified by their name and a direction */
struct iio_chan_info {
	char               *nicename;  /* human readable, not used otherwise */
	enum pluto_devices  devnum;    /* which device to connect, see above */
	char               *name;      /* channel name */
	bool                is_tx;     /* true: this is a sink, false: source */
};

static struct iio_chan_info iio_chan_info[CH_NCHAN] = {
	/* [chnum] = ... */
	[CH_RX_I]   = { "Rx-I",  DEV_RX,  "voltage0", false  },
	[CH_RX_Q]   = { "Rx-Q",  DEV_RX,  "voltage1", false  },
	[CH_TX_I]   = { "Tx-I",  DEV_TX,  "voltage0", true   },
	[CH_TX_Q]   = { "Tx-Q",  DEV_TX,  "voltage1", true   },

	/* PHY is always chid 0, always voltage, is_tx accdg. to rx/tx */
	[CH_PHY_RX] = { "PhyRx", DEV_PHY, "voltage0", false },
	[CH_PHY_TX] = { "PhyTx", DEV_PHY, "voltage0", true  },

	/* LO is chid 0 for RX, chid 1 for TX, always altvoltage, always output */
	[CH_LO_RX]  = { "LO-Rx", DEV_PHY, "altvoltage0", true  },
	[CH_LO_TX]  = { "LO-Tx", DEV_PHY, "altvoltage1", true  }
};

/* buffers have to be created for each devices, they will stream a number
   of channels which have to be activated previously. */

struct iio_buf_info {
	char               *nicename;     /* human readable name for the buffer  */
	enum pluto_devices  devnum;        /* device on which the buffer operates */
	size_t              nsamples;     /* number of samples                   */
	enum pluto_channels ch[CH_NCHAN]; /* channels which are to be activated  */
};

static struct iio_buf_info iio_buf_info[BUF_NBUF] = {
	[BUF_RX] = { "BufRx", DEV_RX, 1<<12, { CH_RX_I, CH_RX_Q, CH_NCHAN } },
	[BUF_TX] = { "BufTx", DEV_TX, 1<<12, { CH_TX_I, CH_TX_Q, CH_NCHAN } }
};


struct iio_context *
plutodev_open_usb()
{
	struct iio_context_info **info;
	struct iio_scan_context *ctx;
	struct iio_context *ret;
	const char *uri;
	int i;

	ctx = iio_create_scan_context("usb", 0);
	if (!ctx) {
		fprintf(stderr, "[IIO] iio_create_scan_context(): failed.\n");
		return NULL;
	}

	i = iio_scan_context_get_info_list(ctx, &info);
	if (i < 0) {
		iio_scan_context_destroy(ctx);
		return NULL;
	}

	if (i <= 0) {
		iio_context_info_list_free(info);
		iio_scan_context_destroy(ctx);
		fprintf(stderr, "[IIO] no ADALM PLUTO found on usb!\n");
		return NULL;
	}                                                                       

	if (i > 1) {
		printf("More than one Pluto found:\n");
		for (unsigned int i = 0; i < (size_t) ret; i++) {
			printf("\t%d: %s [%s]\n", i,
				iio_context_info_get_description(info[i]),
				iio_context_info_get_uri(info[i]));
		}
		printf("We will use the first one.\n");
	}

	uri = iio_context_info_get_uri(info[0]);
	ret = iio_create_context_from_uri(uri);
	iio_context_info_list_free(info);
	iio_scan_context_destroy(ctx);

	return ret;
}

struct pluto_bufmgmt {
	size_t      sample_sz;
	void       *p, *end;
	ptrdiff_t   inc;
};

static long long int           sample_rate_Hz;
static long long int           rx_freq_Hz;
static long long int           tx_freq_Hz;
static struct iio_context     *iio_ctx;
static struct iio_device      *iio_dev[DEV_NDEV];
static struct iio_channel     *iio_ch[CH_NCHAN];
static struct iio_buffer      *iio_buf[BUF_NBUF];
static struct pluto_bufmgmt    iio_buf_mgmt[BUF_NBUF];

static int
plutodev_set_attr_str(enum pluto_channels ch, const char *name,
	const char *value
) {
	int ret;
	const struct iio_chan_info *info = &iio_chan_info[ch];
	const struct iio_dev_info *devinfo = &iio_dev_info[info->devnum];

	ret = iio_channel_attr_write(iio_ch[ch], name, value);
	if (ret < 0) {
		fprintf(stderr, "[IIO] channel %s cannot set attribute %s to value %s\n.",
			info->nicename, name, value);
	} else {
		fprintf(stderr, "[IIO] channel %s attribute %s set to %s.\n",
			info->nicename, name, value);
	}
	return ret;
}

static int
plutodev_set_attr_lli(enum pluto_channels ch, const char *name,
	long long int value
) {
	int ret;
	const struct iio_chan_info *info = &iio_chan_info[ch];
	const struct iio_dev_info *devinfo = &iio_dev_info[info->devnum];

	ret = iio_channel_attr_write_longlong(iio_ch[ch], name, value);
	if (ret < 0) {
		fprintf(stderr, "[IIO] channel %s cannot set attribute %s to value %lld\n.",
			info->nicename, name, value);
	} else {
		fprintf(stderr, "[IIO] channel %s attribute %s set to %lld.\n",
			info->nicename, name, value);
	}
	return ret;
}


static int
plutodev_open()
{
	int i,k;

	iio_ctx = plutodev_open_usb();
	if (!iio_ctx)
		return -1;

	/* open all channels and devices */
	for (i=0; i<CH_NCHAN; i++) {
		const struct iio_chan_info *info = &iio_chan_info[i];
		int d = info->devnum;
		const struct iio_dev_info *devinfo = &iio_dev_info[d];

		if (!iio_dev[d]) { /* need to open a device */
			iio_dev[d] = iio_context_find_device(iio_ctx, devinfo->name);
			if (!iio_dev[d]) {
				fprintf(stderr, "Cannot find %s iio device.\n",
					devinfo->nicename);
				return -1;			
			}
			fprintf(stderr, "[IIO] %s device %s opened.\n",
				devinfo->nicename, devinfo->name);
		}

		iio_ch[i] = iio_device_find_channel(iio_dev[info->devnum],
			info->name, info->is_tx);
		if (!iio_ch[i]) {
			fprintf(stderr, "[IIO] Cannot open channel %s.\n",
				info->nicename);
			return -1;				
		}

		fprintf(stderr, "[IIO]  -> channel %s (%s) tx=%d opened.\n",
			info->nicename, devinfo->name, info->is_tx);
	}

	/* set sample rate, using libad9361 */
	if(ad9361_set_bb_rate(iio_dev[DEV_PHY], sample_rate_Hz) < 0) {
		fprintf(stderr, "[IIO] cannot set RX sample rate.\n");
		return -1;
	}

	/* TX-Phy */
	if(plutodev_set_attr_str(CH_PHY_TX, "rf_port_select", "A")<0)
		return -1;
	if(plutodev_set_attr_lli(CH_PHY_TX, "rf_bandwidth", BANDWIDTH)<0)
		return -1;
	if(plutodev_set_attr_lli(CH_PHY_TX, "hardwaregain", -30)<0)
		return -1;

	/* RX-Phy */
	if(plutodev_set_attr_str(CH_PHY_RX, "rf_port_select", "A_BALANCED")<0)
		return -1;
	if(plutodev_set_attr_lli(CH_PHY_RX, "rf_bandwidth", BANDWIDTH)<0)
		return -1;

	/* local oscillator */
	if(plutodev_set_attr_lli(CH_LO_RX, "frequency", rx_freq_Hz)<0)
		return -1;
	if(plutodev_set_attr_lli(CH_LO_TX, "frequency", tx_freq_Hz)<0)
		return -1;

	return 0;
}

static int
plutodev_start() {
	for (int i=0; i<BUF_NBUF; i++) {
		ssize_t s;
		const struct iio_buf_info *info = &iio_buf_info[i];
		const struct iio_dev_info *devinfo = &iio_dev_info[info->devnum];

		if (iio_buf[i]) {
			iio_buffer_cancel(iio_buf[i]);
			iio_buffer_destroy(iio_buf[i]);
		}
		iio_buf_mgmt[i].p = NULL;

		for (int k=0; k<CH_NCHAN; k++) {
			const struct iio_chan_info *chinfo;
			if (info->ch[k] == CH_NCHAN)
				break;

			chinfo = &iio_chan_info[info->ch[k]];

			iio_channel_enable(iio_ch[info->ch[k]]);
			if (!iio_channel_is_enabled(iio_ch[info->ch[k]])) {
				fprintf(stderr, "[IIO] Buf %s: Cannot enable channel %s/%s.\n",
					info->nicename, devinfo->nicename, chinfo->nicename);
				return -1;
			}
		}
		s = iio_device_get_sample_size(iio_dev[DEV_RX]);
		if (s<0) {
			fprintf(stderr, "[IIO] cannot get sample size for %s.\n",
				devinfo->nicename);
			return -1;
		}
		iio_buf_mgmt[i].sample_sz = s;
		iio_buf[i] = iio_device_create_buffer(
			iio_dev[info->devnum], info->nsamples, false /*cyclic*/);
		if (!iio_buf[i]) {
			fprintf(stderr, "[IIO] cannot create buffer %s.\n",
				info->nicename);
			return -1;			
		}

		fprintf(stderr, "[IIO] buffer %s created, %d samples each %d bytes.\n",
			info->nicename, info->nsamples, iio_buf_mgmt[i].sample_sz);
		iio_buf_mgmt[i].inc = iio_buffer_step(iio_buf[BUF_RX]);
	}

	return 0;
}



int
main(int argc, char **argv)
{
	ssize_t s;
	void *p, *end;
	ptrdiff_t inc;
#if 0
	size_t len;
	int fd;
#endif
	double phase = 0.0;
	double pstep = M_PI/5.0; /* one cycle every 10 samples */
	double scale = (float)((1<<14) - 1);
	float p1, navg, i_avg, q_avg, i_rms, q_rms, angle;
	float rms;

	sample_rate_Hz = 1000000;
	rx_freq_Hz = 830000000;
	tx_freq_Hz = 830000000;

	if (plutodev_open())
		exit(1);
	if (plutodev_start())
		exit(1);

#if 0
	fd = open("dummy.txt", O_CREAT|O_WRONLY, 0777);
	if (fd == -1) {
		perror("open");
		exit(1);
	}
#endif

	i_avg = q_avg = navg = 0.0;
	i_rms = q_rms = 0.0;

	while (1) {

		/* ===== RECEIVE DATA ===== */

		s = iio_buffer_refill(iio_buf[BUF_RX]);
		if (s<0) {
			fprintf(stderr, "iio_buffer_refill()=%d\n", s);
			break;
		}

		if (s != iio_buf_info[BUF_RX].nsamples * iio_buf_mgmt[BUF_RX].inc) {
			fprintf(stderr, "iio_buffer_refill(): incomplete read.\n");
		}


		p = iio_buffer_start(iio_buf[BUF_RX]);
		end = iio_buffer_end(iio_buf[BUF_RX]);
		inc = iio_buffer_step(iio_buf[BUF_RX]);

#if 0
		len = end-p;
		write(fd, p, len);
#endif

		p1 = phase;

		while (p != end) {
			float   i_lo = cos(p1);
			float   q_lo = -sin(p1);
			float   i_rx = ((int16_t*)p)[0];
			float   q_rx = ((int16_t*)p)[1];

			float i_mix = i_lo*i_rx - q_lo*q_rx;
			float q_mix = i_lo*q_rx + q_lo*i_rx;

			i_avg += i_mix;
			q_avg += q_mix;
			navg += 1;

			i_rms += i_rx * i_rx;
			q_rms += q_rx * q_rx;

			p1 += pstep;
			if (p1 > (M_PI*2.0))
				p1 -= (M_PI*2.0);
			p = (void*)(((char*)p) + inc);
		}

		if (navg > 100000) {

			i_avg /= navg;
			q_avg /= navg;
			i_rms = sqrtf(i_rms/navg);
			q_rms = sqrtf(q_rms/navg);
			angle = atan2(q_avg, i_avg)*(180.0/M_PI);

			fprintf(stderr, "I=%9.2f Q=%9.2f angle=%4.1f Irms=%9.2f Qrms=%9.2f (%f)\n",
				i_avg, q_avg, angle, i_rms, q_rms, navg);

			i_avg = q_avg = navg = 0.0;
			i_rms = q_rms = 0.0;
		}

		/* ===== TRANSMIT DATA ===== */

		p = iio_buffer_start(iio_buf[BUF_TX]);
		end = iio_buffer_end(iio_buf[BUF_TX]);
		inc = iio_buffer_step(iio_buf[BUF_TX]);

		while (p != end) {
			int16_t i = lrint(cos(phase)*(scale));
			int16_t q = lrint(sin(phase)*(scale));

			phase += pstep;
			if (phase > (M_PI*2.0))
				phase -= (M_PI*2.0);

			((int16_t*)p)[0] = i;
			((int16_t*)p)[1] = q;
			p = (void*)(((char*)p) + inc);
		}

		s = iio_buffer_push(iio_buf[BUF_TX]);
		if (s<0) {
			fprintf(stderr, "iio_buffer_push()=%d\n", s);
			break;
		}

		if (s != iio_buf_info[BUF_TX].nsamples * iio_buf_mgmt[BUF_TX].inc) {
			fprintf(stderr, "push(): incomplete read.\n");
		}


	}

	iio_buffer_cancel(iio_buf[BUF_RX]);
	iio_buffer_destroy(iio_buf[BUF_RX]);

	iio_buffer_cancel(iio_buf[BUF_TX]);
	iio_buffer_destroy(iio_buf[BUF_TX]);
}