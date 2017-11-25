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
#include <fftw3.h>
#include <pthread.h>

#include "pluto_fdx_dsp.h"


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
    [BUF_RX] = { "BufRx", DEV_RX, -1, { CH_RX_I, CH_RX_Q, CH_NCHAN } },
    [BUF_TX] = { "BufTx", DEV_TX, -1, { CH_TX_I, CH_TX_Q, CH_NCHAN } }
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

    if (i >= 1) {
        printf("Pluto devices found:\n");
        for (unsigned int j = 0; j < (unsigned int)i; j++) {
            printf("\t%d: %s [uri=\"%s\"]\n", j,
                iio_context_info_get_description(info[j]),
                iio_context_info_get_uri(info[j]));
        }
        if (i>1)
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
static long long int           rf_bandwidth_Hz;
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
        const struct iio_data_format *fmt;

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

        fmt = iio_channel_get_data_format(iio_ch[i]);
        if (fmt && fmt->is_fully_defined) {
            fprintf(stderr, "[IIO]  -> channel %s (%s) tx=%d (%s %d/%d bits, <<%d) opened.\n",
                info->nicename, devinfo->name, info->is_tx,
                fmt->is_be?"BE":"LE",fmt->bits, fmt->length, fmt->shift);
        } else {
            fprintf(stderr, "[IIO]  -> channel %s (%s) tx=%d opened.\n",
                info->nicename, devinfo->name, info->is_tx);
        }
    }

    /* set sample rate, using libad9361 */
    if(ad9361_set_bb_rate(iio_dev[DEV_PHY], sample_rate_Hz) < 0) {
        fprintf(stderr, "[IIO] cannot set RX sample rate.\n");
        return -1;
    }

    /* TX-Phy */
    if(plutodev_set_attr_str(CH_PHY_TX, "rf_port_select", "A")<0)
        return -1;
    if(plutodev_set_attr_lli(CH_PHY_TX, "rf_bandwidth", rf_bandwidth_Hz)<0)
        return -1;
    if(plutodev_set_attr_lli(CH_PHY_TX, "hardwaregain", -60)<0)
        return -1;

    /* RX-Phy */
/*
    $ /opt/pluto/bin/iio_attr -u "usb:4.8.5" -c "ad9361-phy"  "voltage0"
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'hardwaregain', value '73.000000 dB'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'rssi', value '108.75 dB'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'rf_port_select', value 'A_BALANCED'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'gain_control_mode', value 'slow_attack'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'rf_port_select_available', value 'A_BALANCED B_BALANCED C_BALANCED A_N A_P B_N B_P C_N C_P TX_MONITOR1 TX_MONITOR2 TX_MONITOR1_2'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'rf_bandwidth', value '1875000'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'rf_dc_offset_tracking_en', value '1'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'quadrature_tracking_en', value '1'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'sampling_frequency', value '2499999'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'gain_control_mode_available', value 'manual fast_attack slow_attack hybrid'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'filter_fir_en', value '1'
    dev 'ad9361-phy', channel 'voltage0' (input), attr 'bb_dc_offset_tracking_en', value '1'
    dev 'ad9361-phy', channel 'voltage0' (output), attr 'rf_port_select', value 'A'
    dev 'ad9361-phy', channel 'voltage0' (output), attr 'hardwaregain', value '-60.000000 dB'
    dev 'ad9361-phy', channel 'voltage0' (output), attr 'rssi', value '0.00 dB'
    dev 'ad9361-phy', channel 'voltage0' (output), attr 'rf_port_select_available', value 'A B'
    dev 'ad9361-phy', channel 'voltage0' (output), attr 'filter_fir_en', value '1'
    dev 'ad9361-phy', channel 'voltage0' (output), attr 'sampling_frequency', value '2499999'
    dev 'ad9361-phy', channel 'voltage0' (output), attr 'rf_bandwidth', value '1875000'
*/
    if(plutodev_set_attr_str(CH_PHY_RX, "rf_port_select", "A_BALANCED")<0)
        return -1;
    if(plutodev_set_attr_lli(CH_PHY_RX, "rf_bandwidth", rf_bandwidth_Hz)<0)
        return -1;
    if(plutodev_set_attr_str(CH_PHY_RX, "gain_control_mode", "manual")<0)
        return -1;
    if(plutodev_set_attr_lli(CH_PHY_RX, "hardwaregain", 0)<0)
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

        fprintf(stderr, "[IIO] buffer %s created, %ld samples each %ld bytes.\n",
            info->nicename, info->nsamples, iio_buf_mgmt[i].sample_sz);
        iio_buf_mgmt[i].inc = iio_buffer_step(iio_buf[BUF_RX]);
    }

    return 0;
}

static volatile uint32_t cycles_tx;

void *
tx_thread(void *arg) {
    uint32_t val;
    int ret;
    ssize_t s;
    ssize_t buf_sz;
    void *buf_start,*buf_end;

    buf_start = iio_buffer_start(iio_buf[BUF_RX]);
    buf_end   = iio_buffer_end(iio_buf[BUF_RX]);
    buf_sz   = buf_end - buf_start;  /* used below */

    memset(buf_start, '\0', buf_sz);

    if ((ret = pluto_fdx_dsp_setup_tx(tx_freq_Hz, sample_rate_Hz, iio_buf[BUF_TX]) < 0) != 0) {
        fprintf(stderr, "[TX] pluto_fdx_dsp_setup_tx() returned %d (!=0).\n", ret);
        return (void*)(intptr_t)ret;
    }

    while (1) {
        ret = iio_device_reg_read(iio_dev[DEV_TX],0x80000088, &val);
        if (ret) {
            fprintf(stderr, "Cannot read TX status register.\n");
            break;
        }

        if ((ret = pluto_fdx_dsp_cb_tx() < 0) != 0) {
            fprintf(stderr, "[TX] pluto_fdx_dsp_cb_tx() returned %d (!=0).\n", ret);
            return (void*)(intptr_t)ret;
        }

        s = iio_buffer_push(iio_buf[BUF_TX]);

        if (s<0) {
            fprintf(stderr, "iio_buffer_push()=%ld\n", s);
            break;
        }

        if (s != (ssize_t)(iio_buf_info[BUF_TX].nsamples * iio_buf_mgmt[BUF_TX].inc)) {
            fprintf(stderr, "push(): incomplete write.\n");
        }

        if (val) {
            fprintf(stderr, "[TX] Status register: 0x%08x\n", val);
            ret = iio_device_reg_write(iio_dev[DEV_TX],0x80000088, val);
            if (ret) {
                fprintf(stderr, "Cannot write TX status register.\n");
                break;
            }
        }

        cycles_tx++;
    }

    return NULL;
}

static volatile uint32_t cycles_rx;

void *
rx_thread(void *arg) {
    uint32_t val;
    int ret;
    ssize_t s;
    ssize_t buf_sz;
    void *buf_start,*buf_end;

    buf_start = iio_buffer_start(iio_buf[BUF_RX]);
    buf_end   = iio_buffer_end(iio_buf[BUF_RX]);
    buf_sz   = buf_end - buf_start;  /* used below */

    fprintf(stderr, "[RX] thread starting.\n");

    if ((ret = pluto_fdx_dsp_setup_rx(rx_freq_Hz, sample_rate_Hz, iio_buf[BUF_RX]) < 0) != 0) {
        fprintf(stderr, "[RX] pluto_fdx_dsp_setup_rx() returned %d (!=0).\n", ret);
        return (void*)(intptr_t)ret;
    }

    while (1) {
        ret = iio_device_reg_read(iio_dev[DEV_RX],0x80000088, &val);
        if (ret) {
            fprintf(stderr, "Cannot read RX status register.\n");
            break;
        }

        if (val) {
            fprintf(stderr, "[RX] Status register: 0x%08x\n", val);
            ret = iio_device_reg_write(iio_dev[DEV_RX],0x80000088, val);
            if (ret) {
                fprintf(stderr, "Cannot write RX status register.\n");
                break;
            }
        }

        s = iio_buffer_refill(iio_buf[BUF_RX]);
        if (s<0) {
            fprintf(stderr, "iio_buffer_qrefill()=%ld\n", s);
            break;
        }

        if (s != buf_sz) {
            fprintf(stderr, "iio_buffer_refill(): incomplete read.\n");
            continue;
        }

        if ((ret = pluto_fdx_dsp_cb_rx() < 0) != 0) {
            fprintf(stderr, "[RX] pluto_fdx_dsp_cb_rx() returned %d (!=0).\n", ret);
            return (void*)(intptr_t)ret;
        }
        cycles_rx++;
    }
    return (void*)0;
}

void
usage(char *argv0)
{
    fprintf(stderr, "%s [options]\n", argv0);
    fprintf(stderr, "   -f hz       Rx Frequency (def: 869.0 MHz)\n");
    fprintf(stderr, "   -F hz       Tx Frequency (def: same as Rx)\n");
    fprintf(stderr, "   -s hz       Sample Rate (def: 2.5 MHz)\n");
    fprintf(stderr, "   -b bw_hz    Rx & Tx Bandwidht (def: 3/4 sampe rate)\n");
    fprintf(stderr, "   -n N        number of samples/block (def: 65536)\n");
}

int
main(int argc, char **argv)
{
    pthread_t thr_rx, thr_tx;
    int i;
    int nsamples = 1UL << 16;

    rx_freq_Hz = 869000000;   /* 869 MHz */
    tx_freq_Hz = -1;          /* set to value of rx_freq_Hz */
    rf_bandwidth_Hz = -1;     /* set to 3/4 of sample rate */
    sample_rate_Hz = 2500000; /* 2.5 MHz */
    nsamples = 1ULL<<16;

    while ((i=getopt(argc, argv, "f:F:b:s:n:h")) != -1) {
        switch (i) {
            case 'f': /* RX Frequency (or both) */
                rx_freq_Hz = atoll(optarg);
                break;
            case 'F': /* TX Frequency */
                tx_freq_Hz = atoll(optarg);
                break;
            case 'b' : /* bandwidth */
                rf_bandwidth_Hz = atoll(optarg);
                break;
            case 's': /* sample rate */
                sample_rate_Hz = atoll(optarg);
                break;
            case 'n': /* blocksize */
                nsamples = atoi(optarg);
                break;
            case 'h':
                usage(argv[0]);
                exit(1);
        }
    }

    if (tx_freq_Hz == -1)
        tx_freq_Hz = rx_freq_Hz;
    if (rf_bandwidth_Hz == -1)
        rf_bandwidth_Hz = 3 * (sample_rate_Hz / 4);

    iio_buf_info[BUF_TX].nsamples = nsamples;
    iio_buf_info[BUF_RX].nsamples = nsamples;


    if (plutodev_open())
        exit(1);
    if (plutodev_start())
        exit(1);

    if (pthread_create(&thr_rx, NULL, rx_thread, NULL)) {
        perror("pthread_create(rx_thread)");
        exit(1);
    }

    if (pthread_create(&thr_tx, NULL, tx_thread, NULL)) {
        perror("pthread_create(rx_thread)");
        exit(1);
    }

    while (1) {
        sleep(1);
        fprintf(stderr, "RX: %9u TX: %9u\n", cycles_rx, cycles_tx);
    }

    pthread_join(thr_rx, NULL);
    pthread_join(thr_tx, NULL);

    iio_buffer_cancel(iio_buf[BUF_RX]);
    iio_buffer_destroy(iio_buf[BUF_RX]);

    iio_buffer_cancel(iio_buf[BUF_TX]);
    iio_buffer_destroy(iio_buf[BUF_TX]);
}