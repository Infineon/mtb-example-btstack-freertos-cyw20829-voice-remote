#ifndef _dvi_adpcm_h
#define _dvi_adpcm_h
/*
** adpcm.h - include file for adpcm coder.
**
** Version 1.0, 7-Jul-92.
*/

#include "stdint.h"

/* vat uses four bytes for state, so we do, too, to stay compatible */
typedef struct __attribute__((__packed__))
{
  int16_t valpred;    /* Previous predicted value */
  uint8_t index;     /* Index into stepsize table */
}dvi_adpcm_state_t;

int dvi_adpcm_encode(void *in_buf, int in_size,
                     void *out_buf, int *out_size, void *state, int hflag);
int dvi_adpcm_decode(void *in_buf, int in_size,
                     void *out_buf, int *out_size, void *state);
void dvi_adpcm_init(dvi_adpcm_state_t *);
#endif
