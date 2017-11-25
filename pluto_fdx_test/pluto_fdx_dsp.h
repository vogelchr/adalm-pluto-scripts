#ifndef PLUTO_FDX_DSP_H
#define PLUTO_FDX_DSP_H

struct iio_buffer;

/* setup, called once, gets buffer info */
extern int pluto_fdx_dsp_setup_rx(float f_LO_rx, float f_S, struct iio_buffer *ch);
extern int pluto_fdx_dsp_setup_tx(float f_LO_rx, float f_S, struct iio_buffer *ch);

/* callback, called once for every block */
extern int pluto_fdx_dsp_cb_rx();
extern int pluto_fdx_dsp_cb_tx();

#endif