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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.aluminati3555.lib.drive;

import org.aluminati3555.lib.pneumatics.AluminatiDoubleSolenoid;

/**
 * This class allows for easily shifting between a high and low gear. Not
 * inverted is when forward is the high gear
 * 
 * @author Caleb Heydon
 */
public class AluminatiShifter {
    // Solenoids
    private AluminatiDoubleSolenoid solenoid;

    // Inverted
    boolean inverted = false;

    /**
     * Returns the solenoid
     */
    public AluminatiDoubleSolenoid getSolenoid() {
        return solenoid;
    }

    /**
     * Returns a useful string
     */
    @Override
    public String toString() {
        return "[Shifter:" + solenoid.getForwardPort() + ":" + solenoid.getReversePort() + "]";
    }

    /**
     * Returns true if inverted
     */
    public boolean isInverted() {
        return inverted;
    }

    /**
     * Sets inverted
     * 
     * @param inverted
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    /**
     * Disables the solenoids
     */
    public void disable() {
        solenoid.disable();
    }

    /**
     * Puts the drive in high gear
     */
    public void high() {
        if (inverted) {
            solenoid.reverse();
        } else {
            solenoid.forward();
        }
    }

    /**
     * Puts the drive in low gear
     */
    public void low() {
        if (inverted) {
            solenoid.forward();
        } else {
            solenoid.reverse();
        }
    }

    /**
     * Returns true if the drive is in high gear
     * 
     * @return
     */
    public boolean isHigh() {
        if (inverted) {
            return solenoid.isReverse();
        } else {
            return solenoid.isForward();
        }
    }

    /**
     * Returns true if the drive is in low gear
     * 
     * @return
     */
    public boolean isLow() {
        return !isHigh();
    }

    public AluminatiShifter(AluminatiDoubleSolenoid solenoid) {
        this.solenoid = solenoid;
    }
}
