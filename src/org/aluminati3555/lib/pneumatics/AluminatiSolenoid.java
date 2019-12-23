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

import edu.wpi.first.wpilibj.Solenoid;

public class AluminatiSolenoid extends Solenoid {
    // PCM
    private int pcm;

    // Port
    private int port;

    /**
     * Returns the pcm in use
     */
    public int getPCM() {
        return pcm;
    }

    /**
     * Gets the port of the solenoid
     * 
     * @return The port
     */
    public int getPort() {
        return port;
    }

    /**
     * This method returns a useful string about the solenoid
     */
    @Override
    public String toString() {
        return "[Solenoid:" + pcm + ":" + port + "] value: " + this.get();
    }

    /**
     * Returns true if the solenoid is on
     * 
     * @return
     */
    public boolean isEnabled() {
        return this.get();
    }

    /**
     * Returns true if the solenoid is off
     * 
     * @return
     */
    public boolean isDisabled() {
        return !this.get();
    }

    /**
     * Turns on the solenoid
     */
    public void enable() {
        this.set(true);
    }

    /**
     * Turns off the solenoid
     */
    public void disable() {
        this.set(false);
    }

    /**
     * This constructor calls the superclass's constructor and saves the port
     * 
     * @param port The solenoid port
     */
    public AluminatiSolenoid(int port) {
        super(port);

        this.port = port;
    }

    public AluminatiSolenoid(int pcm, int port) {
        super(pcm, port);

        this.pcm = pcm;
        this.port = port;
    }
}
