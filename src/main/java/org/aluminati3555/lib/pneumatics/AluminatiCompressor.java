/**
 * Copyright (c) 2019 Team 3555
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.aluminati3555.lib.pneumatics;

import edu.wpi.first.wpilibj.Compressor;

/**
 * This is the wrapper class of the wpilib compressor. It automatically sets
 * closed loop control to true and does not allow it to be changed.
 * 
 * @author Caleb Heydon
 */
public class AluminatiCompressor extends Compressor {
    private int pcm;

    /**
     * Returns the pcm in use
     */
    public int getPCM() {
        return pcm;
    }

    /**
     * Returns a usefule string about the compressor
     */
    @Override
    public String toString() {
        return "[Compressor:" + pcm + "] running: " + this.enabled();
    }

    /**
     * This is a placeholder method that prevents the value from being changed
     */
    @Override
    public void setClosedLoopControl(boolean value) {

    }

    /**
     * This constructor sets closed loop control to true
     */
    public AluminatiCompressor() {
        super();

        this.setClosedLoopControl(true);
    }

    public AluminatiCompressor(int pcm) {
        super(pcm);

        this.setClosedLoopControl(true);
    }
}
