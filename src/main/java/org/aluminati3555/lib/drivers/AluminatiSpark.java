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

package org.aluminati3555.lib.drivers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;

/**
 * This is a wrapper class for the wpilib spark controller
 * 
 * @author Caleb Heydon
 */

public class AluminatiSpark extends Spark implements AluminatiPoweredDevice {
    // PDP
    private PowerDistributionPanel pdp;
    private int pdpChannel;

    // Current warning
    private boolean currentWarning;
    private double currentWarningThreshold;

    /**
     * Returns the pdp channel
     */
    public int getPDPChannel() {
        return pdpChannel;
    }

    /**
     * Returns true if a current warning is enabled
     */
    public boolean isCurrentWarningEnabled() {
        return currentWarning;
    }

    /**
     * Returns the current warning threshold
     */
    public double getCurrentWarningThreshold() {
        return currentWarningThreshold;
    }

    /**
     * Use true to enable a current warning
     */
    public void setEnableCurrentWarning(boolean enabled) {
        currentWarning = enabled;
    }

    /**
     * Sets the current warning threshold
     */
    public void setCurrentWarningThreshold(double threshold) {
        currentWarningThreshold = threshold;
    }

    /**
     * Returns a useful string about the motor controller
     */
    @Override
    public String toString() {
        return "[Spark:" + this.getChannel() + "], currentWarning: " + currentWarning;
    }

    /**
     * Returns the output current
     */
    public double getOutputCurrent() {
        return pdp.getCurrent(pdpChannel);
    }

    /**
     * This method is overridden to allow for current warnings
     */
    @Override
    public void set(double power) {
        super.set(power);

        if (currentWarning) {
            // Check current
            if (pdp.getCurrent(pdpChannel) >= currentWarningThreshold) {
                // Warn drivers
                DriverStation.reportWarning(this.toString() + " exceeded its current warning threshold", false);
            }
        }
    }

    /**
     * Creates a default spark
     * 
     * @param port The pwm port
     */
    public AluminatiSpark(int port) {
        super(port);
    }

    /**
     * Creates a default spark with current monitoring but not current warning
     * 
     * @param port       The pwm port
     * @param pdpChannel The pdp port
     */
    public AluminatiSpark(int port, int pdpChannel) {
        this(port);
        this.pdpChannel = pdpChannel;

        pdp = new PowerDistributionPanel();
    }

    /**
     * Creates a new spark with current monitoring and warning
     * 
     * @param port                    The pwm port
     * @param pdpChannel              The pdp port
     * @param currentWarningThreshold Current warning threshold
     */
    public AluminatiSpark(int port, int pdpChannel, double currentWarningThreshold) {
        this(port, pdpChannel);

        this.currentWarningThreshold = currentWarningThreshold;
        this.currentWarning = true;
    }
}
