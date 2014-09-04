/************************************************
* FFT code from the book Numerical Recipes in C *
* Visit www.nr.com for the licence.             *
************************************************/

// The following line must be defined before including math.h to correctly define M_PI
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <stdlib.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"


#define M_PI   (3.1415926)
#define PI     M_PI   /* pi to machine precision, defined in math.h */
#define TWOPI  (2.0*PI)


void fft_four1 ( double data[], int nn, int isign );
int  fft_calc  ( void );


/* FFT/IFFT routine. (see pages 507-508 of Numerical Recipes in C)
Inputs:
   data[] : array of complex* data points of size 2*NFFT+1.
      data[0] is unused,
      * the n'th complex number x(n), for 0 <= n <= length (x)-1, is stored as:
         data[2*n+1] = real(x(n))
         data[2*n+2] = imag(x(n))
      if length(Nx) < NFFT, the remainder of the array must be padded with zeros

   nn : FFT order NFFT. This MUST be a power of 2 and >= length(x).
   isign:  if set to 1,
            computes the forward FFT
         if set to -1,
            computes Inverse FFT - in this case the output values have
            to be manually normalized by multiplying with 1/NFFT.
Outputs:
   data[] : The FFT or IFFT results are stored in data, overwriting the input.*/

void fft_four1(double data[], int nn, int isign) {
	int n, mmax, m, j, istep, i;
	double wtemp, wr, wpr, wpi, wi, theta;
	double tempr, tempi;

	n = nn << 1;
	j = 1;
	for (i = 1; i < n; i += 2) {
		if (j > i) {
		   tempr = data[j];     data[j] = data[i];     data[i] = tempr;
		   tempr = data[j+1]; data[j+1] = data[i+1]; data[i+1] = tempr;
		}
		m = n >> 1;
		while (m >= 2 && j > m) {
		   j -= m;
		   m >>= 1;
		}
		j += m;
	}
	mmax = 2;
	while (n > mmax) {
		istep = 2*mmax;
		theta = TWOPI/(isign*mmax);
		wtemp = sin(0.5*theta);
		wpr = -2.0*wtemp*wtemp;
		wpi = sin(theta);
		wr = 1.0;
		wi = 0.0;
		for (m = 1; m < mmax; m += 2) {
			for (i = m; i <= n; i += istep) {
				j =i + mmax;
				tempr = wr*data[j]   - wi*data[j+1];
				tempi = wr*data[j+1] + wi*data[j];
				data[j]   = data[i]   - tempr;
				data[j+1] = data[i+1] - tempi;
				data[i] += tempr;
				data[i+1] += tempi;
			}
			wr = (wtemp = wr)*wpr - wi*wpi + wr;
			wi = wi*wpr + wtemp*wpi + wi;
		}
		mmax = istep;
	}
}


#define  FFT_NUM  (32)

// FOR TEST - SINUS-PERIOD + NOISE_EMULATION
double  buf_in[FFT_NUM] // 360/32 = 11.25" (step)
	= { 0, 		0.195,	0.382,	0.555,	0.707,	0.831,	0.923,	0.980,
		1.0,	0.980,	0.923,	0.831,	0.707,	0.555,	0.382,	0.195,
		0,		-0.195,	-0.382,	-0.555,	-0.707,	-0.831,	-0.923,	-0.980,
		-1.0,	-0.980,	-0.923,	-0.831,	-0.707,	-0.555,	-0.382,	-0.195  };

double  buf_out[2*FFT_NUM];


/********************************************************
* The following is a test routine that generates a ramp *
* with 10 elements, finds their FFT, and then finds the *
* original sequence using inverse FFT                   *
********************************************************/
//int fft_main(int argc, char * argv[])
int fft_calc ( void )
{
   int i;
   int Nx;
   int NFFT;
   //uint32_t *x;
   //double *x;
   //double *X;

   /* generate a ramp with FFT_NUM (16) numbers */
   Nx = FFT_NUM;
   //printf("Nx = %d\n", Nx);

   //x = (double *) malloc( Nx * sizeof(double) );
   //x = malloc( Nx );

   //if ( buf_in==NULL ) return -1;

   /*for(i=0; i<Nx; i++) {
      x[i] = i;
   }*/

   /* calculate NFFT as the next higher power of 2 >= Nx */
   NFFT = (int)pow(2.0, ceil(log((double)Nx)/log(2.0)));
   //printf("NFFT = %d\n", NFFT);

   /* allocate memory for NFFT complex numbers (note the +1) */
   //buf_out = (double *) malloc((2*NFFT+1) * sizeof(double));

   /* Storing x(n) in a complex array to make it work with four1.
   This is needed even though x(n) is purely real in this case. */
   for(i=0; i<Nx; i++) {
	  buf_out[2*i+1] = buf_in[i];
      buf_out[2*i+2] = 0.0;
   }

   /* pad the remainder of the array with zeros (0 + 0 j) */
   for(i=Nx; i<NFFT; i++) {
	  buf_out[2*i+1] = 0.0;
      buf_out[2*i+2] = 0.0;
   }

   /*printf("\nInput complex sequence (padded to next highest power of 2):\n");
   for(i=0; i<NFFT; i++) {
      printf("x[%d] = (%.2f + j %.2f)\n", i, X[2*i+1], X[2*i+2]);
   }*/

   /* calculate FFT */
   /*fft_four1(X, NFFT, 1);

   printf("\nFFT:\n");
   for(i=0; i<NFFT; i++) {
      printf("X[%d] = (%.2f + j %.2f)\n", i, X[2*i+1], X[2*i+2]);
   }*/

   //-------------------------------------------------------------------------
	// FOR TEST
	/*for(i=4; i<13; i++) {
		buf_out[2*i+1] = 0;//x[i];
		buf_out[2*i+2] = 0.0;
	}*/

	/*for(i=9; i<15; i++) {
	   X[2*i+1] = 0;//x[i];
	   X[2*i+2] = 0.0;
	}*/
   //-------------------------------------------------------------------------

   /*printf("\n XXXX :\n");
   for(i=0; i<NFFT; i++)
   {
      printf("X[%d] = (%.2f + j %.2f)\n", i, X[2*i+1], X[2*i+2]);
   }*/
   //-------------------------------------------------------------------------

   
   
   /* calculate IFFT */
   fft_four1(buf_out, NFFT, -1);

   /* normalize the IFFT */
   for(i=0; i<NFFT; i++) {
      buf_out[2*i+1] /= NFFT;
      buf_out[2*i+2] /= NFFT;
   }

   /*printf("\nComplex sequence reconstructed by IFFT:\n");
   for(i=0; i<NFFT; i++)
   {
      printf("x[%d] = (%.2f + j %.2f)\n", i, X[2*i+1], X[2*i+2]);
   }

   getchar();*/

   return 0;
}
