/**
 * Copyright (c) 2020 Team 3555
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
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import org.aluminati3555.lib.data.AluminatiData;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Wrapper class for the SparkMax
 * 
 * @author Caleb Heydon
 */
public class AluminatiSparkMax extends CANSparkMax
        implements AluminatiPoweredDevice, AluminatiCriticalDevice, AluminatiMotorController {
    private boolean firmwareOK;

    private CANPIDController pidController;

    @Override
    public String toString() {
        return "[SparkMax:" + this.getDeviceId() + "]";
    }

    /**
     * Configures the pid constants
     * 
     * @param p
     * @param i
     * @param d
     */
    public void configPID(double p, double i, double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    /**
     * Configures the integral zone
     */
    public void configIZone(double iZone) {
        pidController.setIZone(iZone);
    }

    /**
     * Configures the feedforward gain
     */
    public void configFF(double ff) {
        pidController.setFF(ff);
    }

    /**
     * Configures the output range
     */
    public void configOutputRange(double min, double max) {
        pidController.setOutputRange(min, max);
    }

    /**
     * Returns the can device id
     */
    public int getDeviceID() {
        return this.getDeviceId();
    }

    /**
     * Sets the motor controller's neutral mode
     */
    public void setNeutralMode(NeutralMode neutralMode) {
        if (neutralMode == NeutralMode.Brake) {
            this.setIdleMode(IdleMode.kBrake);
        } else if (neutralMode == NeutralMode.Coast) {
            this.setIdleMode(IdleMode.kCoast);
        }
    }

    /**
     * Empty method with driverstation error (don't follow ctre motor controllers)
     */
    public void follow(IMotorController motorController) {
        DriverStation.reportWarning(this.toString() + " is unable to follow CTRE motor controllers", false);
    }

    /**
     * Follows another spark max
     */
    public void follow(AluminatiSparkMax sparkMax) {
        this.follow(sparkMax);
    }

    /**
     * Sets the motor controller output
     */
    public void set(ControlMode controlMode, double x) {
        if (controlMode == ControlMode.PercentOutput) {
            this.set(x);
        } else if (controlMode == ControlMode.Velocity) {
            pidController.setReference(x, ControlType.kVelocity);
        } else {
            DriverStation.reportWarning(this.toString() + " is unable to use the specified control mode", false);
        }
    }

    /**
     * Returns true if the SparkMax is functioning properly
     */
    public boolean isOK() {
        return (this.getFaults() == 0 && firmwareOK);
    }

    /**
     * Verifies the firmware version
     */
    private void checkFirmwareVersion() {
        if (this.getFirmwareVersion() < AluminatiData.minSparkMaxFirmwareVersion) {
            firmwareOK = false;
            DriverStation.reportWarning(this.toString() + " has too old of firmware (may not work)", false);
        } else {
            firmwareOK = true;
        }
    }

    /**
     * Initializes the SparkMax
     */
    private void setupSparkMax() {
        // Verify the firmware version
        checkFirmwareVersion();

        // Restore factory settings
        this.restoreFactoryDefaults(true);

        // Clear faults
        this.clearFaults();

        // Disable
        this.set(0);
    }

    public AluminatiSparkMax(int canID) {
        super(canID, MotorType.kBrushless);

        setupSparkMax();
    }

    public AluminatiSparkMax(int canID, MotorType motorType) {
        super(canID, motorType);

        setupSparkMax();
        this.pidController = this.getPIDController();
    }
}
