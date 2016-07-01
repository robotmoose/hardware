// gccphat.cpp
// Implements the GCC-PHAT algorithm for determining the delay between two signals. Has better
//     performance than a normal cross correlation in reverberant environments.
// Ryker Dial
// UAF ITEST
// Created: June 23, 2016
// Last Modified: June 29, 2016

// Cross correlation implementation originally from https://github.com/dMaggot/libxcorr.

#include "../include/gccphat.h"
#include <string.h>

class GccphatManager {
    public:
        GccphatManager(int);
        ~GccphatManager();
        friend std::pair<double, int> gccphat(fftw_complex * signala, fftw_complex * signalb, fftw_complex * result);
    private:
        fftw_complex * signala_ext;
        fftw_complex * signalb_ext;
        fftw_complex * out_shifted;
        fftw_complex * outa;
        fftw_complex * outb;
        fftw_complex * out;
        int numsaples_fft;
        fftw_plan fft;
        fftw_plan ifft;
};

GccphatManager::GccphatManager(int N) : 
    signala_ext( (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (2 * N - 1)) ),
    signalb_ext( (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (2 * N - 1)) ),
    out_shifted( (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (2 * N - 1)) ),
    outa( (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (2 * N - 1)) ),
    outb( (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (2 * N - 1)) ),
    out( (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (2 * N - 1)) ),
    numsaples_fft(N)
    {
        fft = fftw_plan_dft_1d(2 * N - 1, signala_ext, out, FFTW_FORWARD, FFTW_MEASURE);
        ifft = fftw_plan_dft_1d(2 * N - 1, out, signala_ext, FFTW_BACKWARD, FFTW_MEASURE);
    }

GccphatManager::~GccphatManager() {
    fftw_free(signala_ext);
    fftw_free(signalb_ext);
    fftw_free(out_shifted);
    fftw_free(out);
    fftw_free(outa);
    fftw_free(outb);
    fftw_destroy_plan(fft);
    fftw_destroy_plan(ifft);
    fftw_cleanup();
}

GccphatManager * gccphat_manager;

void gccphatInit(int N) {
    gccphat_manager = new GccphatManager(N);
}

void printGccphatResult(std::pair<double, int> max_corr_pair, int signum1, int signum2, double fs) {
    std::printf("Max correlation of signals %d and %d is %f at a lag of %d.\n", signum1, signum2, max_corr_pair.first, max_corr_pair.second);
    if(max_corr_pair.second > 0) 
        std::printf("Signal %d leads signal %d by %f seconds.\n", signum1, signum2, max_corr_pair.second/fs);
    else if(max_corr_pair.second < 0)
        std::printf("Signal %d lags signal %d by %f seconds.\n", signum1, signum2, max_corr_pair.second/fs);
    else
        std::printf("Signals %d and %d are fully synchronized.\n", signum1, signum2);
}

std::pair<double, int> gccphat(fftw_complex * signala, fftw_complex * signalb, fftw_complex * result) {
    fftw_complex * signala_ext = gccphat_manager -> signala_ext;
    fftw_complex * signalb_ext = gccphat_manager -> signalb_ext;
    fftw_complex * out_shifted = gccphat_manager -> out_shifted;
    fftw_complex * outa = gccphat_manager -> outa;
    fftw_complex * outb = gccphat_manager -> outb;
    fftw_complex * out = gccphat_manager -> out;
    int N = gccphat_manager -> numsaples_fft;

    //zeropadding
    memset (signala_ext, 0, sizeof(fftw_complex) * (N - 1));
    memcpy (signala_ext + (N - 1), signala, sizeof(fftw_complex) * N);
    memcpy (signalb_ext, signalb, sizeof(fftw_complex) * N);
    memset (signalb_ext + N, 0, sizeof(fftw_complex) * (N - 1));

    fftw_execute_dft(gccphat_manager -> fft, signala_ext, outa);
    fftw_execute_dft(gccphat_manager -> fft, signalb_ext, outb);


   // double scale = 1.0/(2 * N -1);
    std::complex<double> * out_cmplx = reinterpret_cast<std::complex<double> *>(out);
    std::complex<double> * outa_cmplx = reinterpret_cast<std::complex<double> *>(outa);
    std::complex<double> * outb_cmplx = reinterpret_cast<std::complex<double> *>(outb);

    for (int i = 0; i < 2 * N - 1; i++)
        out_cmplx[i] = outa_cmplx[i] * conj(outb_cmplx[i])/(abs(outa_cmplx[i] * conj(outb_cmplx[i])));
    // *****

    fftw_execute_dft(gccphat_manager -> ifft, out, result);

    std::pair<double, int> max_corr_pair = std::make_pair(result[0][0],-N+1);
    for(int i=0; i<2*N-1; ++i) {
        if(result[i][0] > max_corr_pair.first) {
            max_corr_pair.first = result[i][0];
            max_corr_pair.second = i-N+1; 
        }
    }

    return max_corr_pair;
}
