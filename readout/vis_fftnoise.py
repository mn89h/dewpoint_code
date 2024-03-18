# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 12:46:09 2024

@author: malte
"""

import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 0.48
if PLOTOPTION == 'pgf':
    mpl.use('pgf')

import csv
import matplotlib.pyplot as plt
import numpy as np
import warnings
import StyleHelper as SH
from numpy import log10
SH.update_pyplot_rcParams()

def signaltonoise(a, axis=0, ddof=0):
    a = np.asanyarray(a)
    m = a.mean(axis)
    sd = a.std(axis=axis, ddof=ddof)
    return np.where(sd == 0, 0, m/sd)

def signaltonoise_dB(a, axis=0, ddof=0):
    a = np.asanyarray(a)
    m = a.mean(axis)
    sd = a.std(axis=axis, ddof=ddof)
    return 20*np.log10(abs(np.where(sd == 0, 0, m/sd)))

def fftPlot(sig, dt=None, plot=True, lim=(0,0), name="noise"):
    # Here it's assumes analytic signal (real signal...) - so only half of the axis is required

    if dt is None:
        dt = 1
        t = np.arange(0, sig.shape[-1])
        xLabel = 'samples'
    else:
        t = np.arange(0, sig.shape[-1]) * dt
        xLabel = 'frequency [\\unit{\\Hz}]'

    if sig.shape[0] % 2 != 0:
        warnings.warn("signal preferred to be even in size, autoFixing it...")
        t = t[0:-1]
        sig = sig[0:-1]

    sigFFT = np.fft.fft(sig) / t.shape[0]  # Divided by size t for coherent magnitude

    freq = np.fft.fftfreq(t.shape[0], d=dt)

    # Plot analytic signal - right half of frequence axis needed only...
    firstNegInd = np.argmax(freq < 0)
    freqAxisPos = freq[0:firstNegInd]
    sigFFTPos = 2 * sigFFT[0:firstNegInd]  # *2 because of magnitude of analytic signal

    MAG_dB = 20*log10(np.abs(sigFFTPos)/max(np.abs(sigFFTPos)));
    
    if plot:
                            
        fig, ax = plt.subplots(figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE, 1, (1,1), 1), layout='constrained')

        if not lim==(0,0):
            ax.set_ylim(lim)
        ax.plot(freqAxisPos, MAG_dB)
        ax.margins(0.01, 0.04)
        ax.relim()
        plt.xlabel(xLabel)
        plt.grid()
        # plt.yscale('log')
        # plt.xscale('log')
        plt.ylabel('magnitude [\\unit{\\dB}]')
        
        if PLOTOPTION == 'pgf':
            plt.savefig('lolza.pgf', format = 'pgf')
        else:
            plt.savefig(name + '.pdf', bbox_inches='tight')
            plt.show()

    return sigFFTPos, freqAxisPos


name = "noise_cap"
col = -3
samples = []
with open('stability_test_25C_55RH.csv') as csvfile:
    reader = csv.reader(csvfile)
    # next(reader) # skip header row
    for row in reader:
        samples.append(float(row[col]))
samples = np.array(samples)
# fftPlot(samples, 0.2, True, (0,0), name)

# # PSD 
# # https://stackoverflow.com/questions/15382076/plotting-power-spectrum-in-python
# # https://www.analog.com/media/en/technical-documentation/dsp-book/dsp_book_Ch2.pdf

# p = 20*np.log10(np.abs(np.fft.rfft(samples)))
# f = np.linspace(0, 5/2, len(p))
# plt.plot(f, p)
# plt.show()

print(signaltonoise_dB(samples[:150]))