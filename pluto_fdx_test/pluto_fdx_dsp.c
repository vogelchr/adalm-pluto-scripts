#include "pluto_fdx_dsp.h"
#include <iio.h>
#include <stdio.h>

#define MHZ(v) ((v)*1e-6)

/*
 * ================== RECEIVE ==================
 */

int
pluto_fdx_dsp_setup_rx(float f_LO_Hz, float f_S_Hz, struct iio_buffer *ch)
{
    fprintf(stderr, "[RX] fLO=%8.3f MHz fS=%5.3f MHz\n", MHZ(f_LO_Hz), MHZ(f_S_Hz));
    return 0;
}

int
pluto_fdx_dsp_cb_rx() {
    return 0;
}

/*
 * =============== TRANSMIT ==================
 */

int
pluto_fdx_dsp_setup_tx(float f_LO_Hz, float f_S_Hz, struct iio_buffer *ch)
{
    fprintf(stderr, "[TX] fLO=%8.3f MHz fS=%5.3f MHz\n",
        MHZ(f_LO_Hz), MHZ(f_S_Hz));
    return 0;
}


int
pluto_fdx_dsp_cb_tx() {
    return 0;
}


#if 0
void *
rx_thread(void *arg) {
    int16_t *buf_start, *buf_end, *p;
    size_t    buf_size;
    ssize_t s;
    size_t ns,i;
    uint32_t val;
    int ret;
    int fd =-1;

    float rms, rms_scale, maxv;
    float fft_maxv;
    int fft_maxbin, j;
    float fmax_Hz;

    uint16_t sample_number;
    float long_average = 0.0;
    float short_average = 0.0;

    fftw_complex *fft_in,*fft_out, *q;
    fftw_plan fft_plan;

    fd = open("dummy.raw", O_CREAT|O_RDWR, 0777);

    fprintf(stderr, "RX thread starting.\n");

    /* receive from SDR */
    buf_start = iio_buffer_start(iio_buf[BUF_RX]);
    buf_end   = iio_buffer_end(iio_buf[BUF_RX]);
    buf_size  = (void*)buf_end - (void*)buf_start;

    if (iio_buffer_step(iio_buf[BUF_RX]) != sizeof(uint16_t)*2) {
        fprintf(stderr, "Fatal error, buffer inc must be 2x uint16_t!\n");
        exit(1);
    }

    ns = iio_buf_info[BUF_RX].nsamples;
    fprintf(stderr, "[FFTW] Alloc buffers and create wisdom (%d samples).\n",
        (int)ns);

    fft_in = fftw_malloc(sizeof(fftw_complex) * ns);
    fft_out = fftw_malloc(sizeof(fftw_complex) * ns);
    fft_plan = fftw_plan_dft_1d(ns, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
    fprintf(stderr, "fft_in=%p fft_out=%p fft_plan=%p\n", fft_in, fft_out, fft_plan);

    rms_scale = 1.0/ns;
    fprintf(stderr, "rms_scale=%f\n", rms_scale);

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

        /* ===== RECEIVE DATA ===== */
        s = iio_buffer_refill(iio_buf[BUF_RX]);
        if (s<0) {
            fprintf(stderr, "iio_buffer_qrefill()=%ld\n", s);
            break;
        }

        if (s != (ssize_t)buf_size) {
            fprintf(stderr, "iio_buffer_refill(): incomplete read.\n");
            continue;
        }

        rms = 0.0, maxv=0.0;
        /* 2 samples per iteration */
        for (q=fft_in, p=buf_start; p!=buf_end; p+=2) {
            fftw_complex c = { p[0], p[1] };
            float magnitude = c[1]*c[1]+c[0]*c[0];

            /* max, rms */
            if (fabs(c[0]) > maxv)
                maxv = fabs(c[0]);
            if (fabs(c[1]) > maxv)
                maxv = fabs(c[1]);
            rms += magnitude;

            (*q)[0] = c[0];
            (*q)[1] = c[1];
            q++;
        }

#if 0
        if (fd != -1)
            write(fd, buf_start, (void*)buf_end - (void*)buf_start);
        if (fd != -1)
            write(fd, fft_in, sizeof(fftw_complex) * ns);
#endif

        rms *= rms_scale;
        rms = sqrt(rms);

        fftw_execute(fft_plan);

        /* find peak in fft */
        fft_maxbin=0, fft_maxv=0;
        for (j=1; j<(int)ns; j++) { /* start at 1, skip DC! */
            fftw_complex c;

            c[0] = fft_out[j][0];
            c[1] = fft_out[j][1];

            float magn = c[0]*c[0] + c[1]*c[1];
            if (magn > fft_maxv) {
                fft_maxv = magn;
                fft_maxbin = j;
            }
        }

        fft_maxv *= rms_scale;

        /*
           fftw layout of frequencies is:
             [    0] = DC
             [    1] = 1*fS/N
             ...
             [N/2-1] = (N/2-1) * (fS/N)
             [  N/2] = (N/2)*(fS/N) = fS/2 = fNyq
             [N/2+1] = -(N/2-1)* (fS/N)
              ...
             [  N-1] = -1*Fs/N
        */

        if (fft_maxbin > (int)(ns/2)) {
            fft_maxbin = -(ns/2) + (fft_maxbin - ns/2) - 1;
        }

        fmax_Hz = rx_freq_Hz + ((float)sample_rate_Hz/(float)ns)*fft_maxbin;
        fprintf(stderr, "rms=%8.2f max=%5.0f fft_max=%9.2g @ %5d f = %9f Hz\n", rms, maxv, fft_maxv, fft_maxbin, fmax_Hz);


        cycles_rx++;
    }

    fftw_destroy_plan(fft_plan);
    fftw_free(fft_in);
    fftw_free(fft_out);

    return NULL;
}

#endif
