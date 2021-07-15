package com.stuypulse.robot.util;

import com.stuypulse.stuylib.streams.IStream;

import com.stuypulse.stuylib.streams.filters.HighPassFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

/**
 * A "fuser" combins a low frequency input stream and a 
 * high frequency input stream using a low pass and high pass filter.
 * 
 * This fuser is implemented as a stream itself, and takes in two others.
 * 
 * TOOD: move to stuylib?
 * 
 * @author Myles Pasetsky (@selym3)
 */
public class IFuser implements IStream {

    /**
     * Two input streams: one being low frequency updates and the other 
     * being high frequency updates.
     * 
     * The low and high frequency streams are passed through a low pass
     * and high pass filter respectively.
     */
    private IStream lowFreq, highFreq;

    /**
     * Creates an input fuser stream
     * 
     * @param rc window size value given to both the high and low pass filter
     * @param lowFrequencyData an input stream of low frequency data 
     * @param highFrequencyData an input stream of high frequency data
     */
    public IFuser(Number rc, IStream lowFrequencyData, IStream highFrequencyData) {
        lowFreq = lowFrequencyData.filtered(new LowPassFilter(rc));
        highFreq = highFrequencyData.filtered(new HighPassFilter(rc));
    }

    /**
     * Returns the combined low pass and high pass data.
     * 
     * Provides the IStream interface for an IFuser using 
     * two filtered IStreams.
     * 
     * @return the combined low pass and high pass data
     */
    public double get() {
        return lowFreq.get() + highFreq.get();
    }

}
