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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.aluminati3555.lib.data.AluminatiData;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * This is the wrapper class for the Victor SPX. It allows for an optional
 * current warning and provides an easy method to get the current being used.
 * 
 * @author Caleb Heydon
 */
public class AluminatiVictorSPX extends VictorSPX implements AluminatiPoweredDevice, AluminatiCriticalDevice {
    // Faults
    private Faults faults;

    private boolean versionOK;

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
     * Returns a usefule string about the motor controller
     */
    @Override
    public String toString() {
        return "[VictorSPX:" + this.getDeviceID() + ":" + pdpChannel + "] currentWarning: " + currentWarning;
    }

    /**
     * Returns the output current of the motor controller
     */
    @Override
    public double getOutputCurrent() {
        return pdp.getCurrent(pdpChannel);
    }

    /**
     * This method is overriden to allow for a current warning
     */
    @Override
    public void set(ControlMode mode, double power) {
        super.set(mode, power);

        if (currentWarning) {
            // Check current
            if (pdp.getCurrent(pdpChannel) >= currentWarningThreshold) {
                // Warn drivers
                DriverStation.reportWarning(this.toString() + " exceeded its current warning threshold", false);
            }
        }
    }

    /**
     * Returns true if the motor controller is ok
     */
    public boolean isOK() {
        this.getFaults(faults);

        return (!faults.hasAnyFault() && versionOK);
    }

    /**
     * Verifies that a firmware version greater than or equal to the minimum is
     * installed
     */
    private void checkFirmwareVersion() {
        if (this.getFirmwareVersion() < AluminatiData.minVictorSPXFirmwareVersion) {
            versionOK = false;
            DriverStation.reportWarning(this.toString() + " has too old of firmware (may not work)", false);
        } else {
            versionOK = true;
        }
    }

    /**
     * Initializes the motor controller
     */
    private void setupVictor() {
        // Check firmware version
        checkFirmwareVersion();

        // Restore factory defaults
        this.configFactoryDefault();

        // Clear faults
        this.clearStickyFaults();

        // Set deadband
        this.configNeutralDeadband(AluminatiData.deadband);

        // Disable
        this.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Basic constructor with no current warning and no ability to monitor current
     * 
     * @param canID The can id of the motor controller
     */
    public AluminatiVictorSPX(int canID) {
        super(canID);
        faults = new Faults();

        setupVictor();
    }

    /**
     * Basic constructor with no current warning, but it has the ability to enable a
     * current warning later. It is also possible to retrive the current with this
     * constructor
     * 
     * @param canID      The can id
     * @param pdpChannel The pdp channel
     */
    public AluminatiVictorSPX(int canID, int pdpChannel) {
        this(canID);

        this.pdpChannel = pdpChannel;
        pdp = new PowerDistributionPanel();
    }

    /**
     * This constructor allows for a current warning
     * 
     * @param canID                   The can id
     * @param pdpChannel              The pdp channel
     * @param currentWarningThreshold The current warning threshold
     */
    public AluminatiVictorSPX(int canID, int pdpChannel, double currentWarningThreshold) {
        this(canID, pdpChannel);

        this.currentWarningThreshold = currentWarningThreshold;
        currentWarning = true;
    }
}
