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

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * This is a wrapper class for the usual DoubleSolenoid wpilib class. It allows
 * for a simpler interface which avoid certain issues with the usual class. It
 * also allows access to all wpilib methods to allow for backwards compatibility
 * with code examples and tutorials.
 * 
 * @author Caleb Heydon
 */

public class AluminatiDoubleSolenoid extends DoubleSolenoid {
    // PCM
    private int pcm;

    // Solenoid ports
    private int forwardPort;
    private int reversePort;

    /**
     * Returns the pcm in use
     */
    public int getPCM() {
        return pcm;
    }

    /**
     * This method returns the forward port for the double solenoid
     * 
     * @return The port number
     */
    public int getForwardPort() {
        return forwardPort;
    }

    /**
     * This method returns the reverse port for the double solenoid
     * 
     * @return The port number
     */
    public int getReversePort() {
        return reversePort;
    }

    /**
     * This method returns a useful string describing the DoubleSolenoid
     */
    @Override
    public String toString() {
        return "[DoubleSolenoid:" + pcm + ":" + forwardPort + ":" + reversePort + "] value: " + this.get();
    }

    /**
     * Returns true if the double solenoid is off
     * 
     * @return
     */
    public boolean isDisabled() {
        return (this.get() == DoubleSolenoid.Value.kOff);
    }

    /**
     * Returns true if the double solenoid is set to forward
     * 
     * @return
     */
    public boolean isForward() {
        return (this.get() == DoubleSolenoid.Value.kForward);
    }

    /**
     * Returns true if the double solenoid is set to reverse
     * 
     * @return
     */
    public boolean isReverse() {
        return (this.get() == DoubleSolenoid.Value.kForward);
    }

    /**
     * This method sets the solenoid to off
     */
    public void disable() {
        this.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Sets the solenoid to forward
     */
    public void forward() {
        this.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets the solenoid to reverse
     */
    public void reverse() {
        this.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * This is the same constructor as the wpilib class.
     * 
     * @param forwardPort The forward solenoid port
     * @param reversePort The reverse solenoid port
     */
    public AluminatiDoubleSolenoid(int forwardPort, int reversePort) {
        // Call wpilib constructor
        super(forwardPort, reversePort);

        // Save ports for later
        this.forwardPort = forwardPort;
        this.reversePort = reversePort;
    }

    public AluminatiDoubleSolenoid(int pcm, int forwardPort, int reversePort) {
        super(pcm, forwardPort, reversePort);

        this.pcm = pcm;
        this.forwardPort = forwardPort;
        this.reversePort = reversePort;
    }
}
