#include "FIRFilter.h"

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {
		    -0.0003476979f,
		     0.0017823400f,
		     0.0056038913f,
		     0.0104789343f,
		     0.0131028742f,
		     0.0093120346f,
		    -0.0030871602f,
		    -0.0212259032f,
		    -0.0360810906f,
		    -0.0350615829f,
		    -0.0079120835f,
		     0.0467480011f,
		     0.1180692986f,
		     0.1854413003f,
		     0.2264017761f,
		     0.2264017761f,
		     0.1854413003f,
		     0.1180692986f,
		     0.0467480011f,
		    -0.0079120835f,
		    -0.0350615829f,
		    -0.0360810906f,
		    -0.0212259032f,
		    -0.0030871602f,
		     0.0093120346f,
		     0.0131028742f,
		     0.0104789343f,
		     0.0056038913f,
		     0.0017823400f,
		    -0.0003476979f
		};


//Taken from Phil's lab implementation

void FIRFilter_Init(FIRFilter *fir) {

	/* Clear filter buffer */
	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		fir->buf[n] = 0.0f;

	}

	/* Reset buffer index */
	fir->bufIndex = 0;

	/* Clear filter output */
	fir->out = 0.0f;

}

float FIRFilter_Update(FIRFilter *fir, float inp) {

	/* Store latest sample in buffer */
	fir->buf[fir->bufIndex] = inp;

	/* Increment buffer index and wrap around if necessary */
	fir->bufIndex++;

	if (fir->bufIndex == FIR_FILTER_LENGTH) {

		fir->bufIndex = 0;

	}

	/* Compute new output sample (via convolution) */
	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		/* Decrement index and wrap if necessary */
		if (sumIndex > 0) {

			sumIndex--;

		} else {

			sumIndex = FIR_FILTER_LENGTH - 1;

		}

		/* Multiply impulse response with shifted input sample and add to output */
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	/* Return filtered output */
	//printf("%.8f, %.8f \r\n", inp, fir->out);
	return fir->out;

}
