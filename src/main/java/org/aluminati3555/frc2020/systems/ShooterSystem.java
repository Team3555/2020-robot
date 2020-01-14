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

package org.aluminati3555.frc2020.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.aluminati3555.lib.drivers.AluminatiMotorGroup;
import org.aluminati3555.lib.net.AluminatiTunable;
import org.aluminati3555.lib.pneumatics.AluminatiDoubleSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class controls the shooter flywheel and retractable hood
 * 
 * @author Caleb Heydon
 */
public class ShooterSystem implements AluminatiSystem {
    // Constants
    private static final int SHOOTER_CURRENT_LIMIT = 30;
    private static final double RATIO = 0.5;

    private AluminatiMotorGroup flywheelMotorGroup;
    private AluminatiDoubleSolenoid hoodSolenoid;

    private boolean shooterEnabled;
    private double setpoint;

    /**
     * Sets the target rpm
     */
    public void set(double rpm) {
        this.setpoint = rpm;
        this.shooterEnabled = true;
    }

    /**
     * Returns the velocity of the flywheel
     */
    public double getVelocity() {
        return flywheelMotorGroup.getMasterSparkMax().getEncoder().getVelocity();
    }

    /**
     * Stops the motor
     */
    public void stop() {
        this.shooterEnabled = false;
        this.setpoint = 0;
    }

    /**
     * Extends the hood
     */
    public void extendHood() {
        hoodSolenoid.forward();
    }

    /**
     * Retracts the hood
     */
    public void retractHood() {
        hoodSolenoid.reverse();
    }

    public void update(double timestamp, boolean enabled) {
        if (!flywheelMotorGroup.getMasterSparkMax().isOK()) {
            DriverStation.reportError("Fault detected in shooter", false);
        }

        if (enabled) {
            // Add manual controls here
        }

        if (shooterEnabled) {
            flywheelMotorGroup.getMasterSparkMax().set(ControlMode.Velocity, setpoint);
        } else {
            flywheelMotorGroup.getMasterSparkMax().set(ControlMode.PercentOutput, 0);
        }
    }

    public ShooterSystem(AluminatiMotorGroup flywheelMotorGroup, AluminatiDoubleSolenoid hoodSolenoid) {
        this.flywheelMotorGroup = flywheelMotorGroup;
        this.hoodSolenoid = hoodSolenoid;

        // Configure velocity for gear ratio
        this.flywheelMotorGroup.getMasterSparkMax().getEncoder().setVelocityConversionFactor(RATIO);

        // Configure PID
        this.flywheelMotorGroup.getMasterSparkMax().configPID(0.1, 0, 0);
        this.flywheelMotorGroup.getMasterSparkMax().configIZone(400);

        // Setup tuning listener
        new AluminatiTunable(5807) {
            protected void update(TuningData data) {
                flywheelMotorGroup.getMasterSparkMax().configPID(data.kP, data.kI, data.kD);
            }
        };

        // Configure current limit
        this.flywheelMotorGroup.getMasterSparkMax().setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
    }
}
