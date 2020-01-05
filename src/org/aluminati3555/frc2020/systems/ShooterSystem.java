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

import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
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
    private static final int ENCODER_TICKS_PER_ROTATION = 4096;

    /**
     * Converts rpm to encoder ticks per revolution
     */
    private static int rpmToEncoderTicks(double rpm) {
        return (int) (rpm * ENCODER_TICKS_PER_ROTATION / 600.0);
    }

    /**
     * Converts encoder ticks to rpm
     */
    private static double encoderTicksToRPM(int ticks) {
        return (int) (ticks / ENCODER_TICKS_PER_ROTATION * 600.0);
    }

    private AluminatiTalonSRX flywheelMotor;
    private AluminatiDoubleSolenoid hoodSolenoid;

    private boolean shooterEnabled;
    private int setpoint;

    /**
     * Sets the target rpm
     */
    public void set(double rpm) {
        this.setpoint = rpmToEncoderTicks(rpm);
        this.shooterEnabled = true;
    }

    /**
     * Returns the velocity of the flywheel
     */
    public double getVelocity() {
        return encoderTicksToRPM(flywheelMotor.getSelectedSensorVelocity());
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
        if (!flywheelMotor.isOK()) {
            DriverStation.reportError("Fault detected in shooter", false);
        }

        if (!flywheelMotor.isEncoderOK()) {
            DriverStation.reportError("Encoder failure detected in shooter", false);
        }

        if (enabled) {
            // Add manual controls here
        }

        if (shooterEnabled) {
            flywheelMotor.set(ControlMode.Velocity, setpoint);
        } else {
            flywheelMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public ShooterSystem(AluminatiTalonSRX flywheelMotor, AluminatiDoubleSolenoid hoodSolenoid) {
        this.flywheelMotor = flywheelMotor;
        this.hoodSolenoid = hoodSolenoid;

        // Configure encoder
        this.flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure PID
        this.flywheelMotor.config_kP(0, 0.1);
        this.flywheelMotor.config_kI(0, 0);
        this.flywheelMotor.config_kD(0, 0);
        this.flywheelMotor.config_IntegralZone(0, 400);

        // Setup tuning listener
        new AluminatiTunable(5806) {
            protected void update(TuningData data) {
                flywheelMotor.config_kP(0, data.kP);
                flywheelMotor.config_kI(0, data.kI);
                flywheelMotor.config_kD(0, data.kD);
            }
        };

        // Configure current limit
        this.flywheelMotor.configPeakCurrentDuration(500);
        this.flywheelMotor.configPeakCurrentLimit(SHOOTER_CURRENT_LIMIT);
        this.flywheelMotor.configContinuousCurrentLimit(SHOOTER_CURRENT_LIMIT);
        this.flywheelMotor.enableCurrentLimit(true);
    }
}
