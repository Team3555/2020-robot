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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.aluminati3555.lib.drivers.AluminatiMotorGroup;
import org.aluminati3555.lib.net.AluminatiTunable;
import org.aluminati3555.lib.pneumatics.AluminatiSolenoid;
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
    private static final int ENCODER_TICKS_PER_ROTATION = 4096;

    /**
     * Converts rpm to native units
     */
    public static int convertRPMToNativeUnits(double rpm) {
        return (int) (rpm * ENCODER_TICKS_PER_ROTATION / 600.0);
    }

    /**
     * Converts native units to rpm
     */
    public static int convertNativeUnitsToRPM(double nativeUnits) {
        return (int) (nativeUnits / ENCODER_TICKS_PER_ROTATION * 600.0);
    }

    private AluminatiMotorGroup motorGroup;
    private AluminatiSolenoid hoodSolenoid;

    private boolean shooterEnabled;
    private double setpoint;

    /**
     * Sets the target rpm
     */
    public void set(double rpm) {
        this.setpoint = convertRPMToNativeUnits(rpm);
        this.shooterEnabled = true;
    }

    /**
     * Returns the velocity of the flywheel
     */
    public double getVelocity() {
        return convertNativeUnitsToRPM(motorGroup.getMasterTalon().getSelectedSensorVelocity());
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
        hoodSolenoid.disable();
    }

    /**
     * Retracts the hood
     */
    public void retractHood() {
        hoodSolenoid.enable();
    }

    public void update(double timestamp, boolean enabled) {
        if (!motorGroup.getMasterSparkMax().isOK()) {
            DriverStation.reportError("Fault detected in shooter", false);
        }

        if (enabled) {
            // Add manual controls here
        }

        if (shooterEnabled) {
            motorGroup.getMasterSparkMax().set(ControlMode.Velocity, setpoint);
        } else {
            motorGroup.getMasterSparkMax().set(ControlMode.PercentOutput, 0);
        }
    }

    public ShooterSystem(AluminatiMotorGroup motorGroup, AluminatiSolenoid hoodSolenoid) {
        this.motorGroup = motorGroup;
        this.hoodSolenoid = hoodSolenoid;

        // Configure encoder
        motorGroup.getMasterTalon().configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure PID
        motorGroup.getMasterTalon().config_kP(0, 0.1);
        motorGroup.getMasterTalon().config_kI(0, 0);
        motorGroup.getMasterTalon().config_kD(0, 0);
        motorGroup.getMasterTalon().config_IntegralZone(0, 400);

        // Setup tuning listener
        new AluminatiTunable(5805) {
            protected void update(TuningData data) {
                motorGroup.getMasterTalon().config_kP(0, data.kP);
                motorGroup.getMasterTalon().config_kI(0, data.kI);
                motorGroup.getMasterTalon().config_kD(0, data.kD);
            }
        };

        // Configure current limit
        motorGroup.getMasterTalon().configPeakCurrentDuration(500);
        motorGroup.getMasterTalon().configPeakCurrentLimit(SHOOTER_CURRENT_LIMIT);
        motorGroup.getMasterTalon().configContinuousCurrentLimit(SHOOTER_CURRENT_LIMIT);
        motorGroup.getMasterTalon().enableCurrentLimit(true);
    }
}
