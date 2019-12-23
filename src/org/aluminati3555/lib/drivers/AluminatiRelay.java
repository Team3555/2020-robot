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
import edu.wpi.first.wpilibj.Relay;

/**
 * This is the wrapper class for the relay wpilib class
 */

public class AluminatiRelay extends Relay implements AluminatiPoweredDevice {
    // Relay port
    private int relayPort;

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
     * Returns the relay port
     */
    public int getRelayPort() {
        return relayPort;
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
     * Returns a helpful string about the relay
     */
    @Override
    public String toString() {
        return "[Relay:" + relayPort + ":" + pdpChannel + "] currentWarning: " + currentWarning;
    }

    /**
     * Returns the output current
     */
    public double getOutputCurrent() {
        return pdp.getCurrent(pdpChannel);
    }

    /**
     * Sets the relay state and checks the current warning
     */
    @Override
    public void set(Value value) {
        super.set(value);

        if (currentWarning) {
            // Check current
            if (pdp.getCurrent(pdpChannel) >= currentWarningThreshold) {
                // Warn drivers
                DriverStation.reportWarning(this.toString() + " exceeded its current warning threshold", false);
            }
        }
    }

    /**
     * Returns true if the relay is disabled
     */
    public boolean isDisabled() {
        return (this.get() == Value.kOff);
    }

    /**
     * Returns true if the relay is forwarded
     */
    public boolean isForward() {
        return (this.get() == Value.kForward) || (this.get() == Value.kOn);
    }

    /**
     * Returns true if the relay is reversed
     */
    public boolean isReverse() {
        return (this.get() == Value.kReverse);
    }

    /**
     * Disables the relay
     */
    public void disable() {
        this.set(Value.kOff);
    }

    /**
     * Sets the relay to forward
     */
    public void forward() {
        this.set(Value.kForward);
    }

    /**
     * Sets the relay to reverse
     */
    public void reverse() {
        this.set(Value.kReverse);
    }

    /**
     * Creates a new relay with now current warning or current reporting
     * 
     * @param port The relay port
     */
    public AluminatiRelay(int port) {
        super(port);

        relayPort = port;
    }

    /**
     * This constructor creates a new relay with no current warning but with current
     * monitoring
     * 
     * @param port       Relay port
     * @param pdpChannel Pdp channel
     */
    public AluminatiRelay(int port, int pdpChannel) {
        this(port);

        this.pdpChannel = pdpChannel;
        pdp = new PowerDistributionPanel();
    }

    /**
     * Creates a new relay with current warning and monitoring
     */
    public AluminatiRelay(int port, int pdpChannel, double currentWarningThreshold) {
        this(port, pdpChannel);

        this.currentWarningThreshold = currentWarningThreshold;
        currentWarning = true;
    }
}
