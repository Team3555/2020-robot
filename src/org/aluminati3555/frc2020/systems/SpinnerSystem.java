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
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.net.AluminatiTunable;
import org.aluminati3555.lib.pneumatics.AluminatiDoubleSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class controls the spinner for the control panel
 * 
 * @author Caleb Heydon
 */
public class SpinnerSystem implements AluminatiSystem {
    // Constants
    private static final int SPINNER_CURRENT_LIMIT = 30;
    private static final double RADIUS = 2;
    private static final double CIRCUMFERENCE = 2 * Math.PI * RADIUS;
    private static final int ENCODER_TICKS_PER_ROTATION = 4096;

    /**
     * Returns the number of encoder ticks from inches
     * 
     * @param inches
     * @return
     */
    private static int inchesToEncoderTicks(double inches) {
        return (int) (ENCODER_TICKS_PER_ROTATION * (inches / CIRCUMFERENCE));
    }

    /**
     * Converts encoder ticks to inches
     */
    private static double encoderTicksToInches(int ticks) {
        return (ticks / ENCODER_TICKS_PER_ROTATION) * CIRCUMFERENCE;
    }

    private AluminatiTalonSRX spinnerMotor;
    private AluminatiDoubleSolenoid extenderSolenoid;
    private State state;

    private int setpoint;

    /**
     * Sets the wheel to turn a certain number of inches
     * 
     * @param inches
     */
    public void set(double inches) {
        setpoint = inchesToEncoderTicks(inches);
    }

    /**
     * Returns the desired setpoint in inches
     */
    public double getDistance() {
        return encoderTicksToInches(setpoint);
    }

    /**
     * This method zeros the encoder
     */
    public void reset() {
        this.state = State.OPEN_LOOP;
        spinnerMotor.setSelectedSensorPosition(0);
    }

    /**
     * Extends the spinner mechanism
     */
    public void extend() {
        extenderSolenoid.forward();
    }

    /**
     * Retracts the spinner mechanism
     */
    public void retract() {
        extenderSolenoid.reverse();
    }

    public void update(double timestamp, boolean enabled) {
        // Report failures to driver
        if (!this.spinnerMotor.isOK()) {
            DriverStation.reportError("Fault detected in spinner", false);
        }

        if (!this.spinnerMotor.isEncoderOK()) {
            DriverStation.reportError("Encoder failure detected in spinner", false);
        }

        if (enabled) {
            // Add joystick binding to exit closed loop control

            if (state == State.OPEN_LOOP) {
                // Add joystick bindings for manual control here
            }
        }

        if (this.state == State.CLOSED_LOOP) {
            // Update motor for closed loop control
            this.spinnerMotor.set(ControlMode.Position, setpoint);
        }
    }

    public SpinnerSystem(AluminatiTalonSRX spinnerMotor, AluminatiDoubleSolenoid extenderSolenoid) {
        this.spinnerMotor = spinnerMotor;
        this.extenderSolenoid = extenderSolenoid;

        this.state = State.OPEN_LOOP;

        // Configure encoder
        this.spinnerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure PID
        this.spinnerMotor.config_kP(0, 0.1);
        this.spinnerMotor.config_kI(0, 0);
        this.spinnerMotor.config_kD(0, 0);
        this.spinnerMotor.config_IntegralZone(0, 400);

        // Setup tuning listener
        new AluminatiTunable(5806) {
            protected void update(TuningData data) {
                spinnerMotor.config_kP(0, data.kP);
                spinnerMotor.config_kI(0, data.kI);
                spinnerMotor.config_kD(0, data.kD);
            }
        };

        // Configure current limit
        this.spinnerMotor.configPeakCurrentDuration(500);
        this.spinnerMotor.configPeakCurrentLimit(SPINNER_CURRENT_LIMIT);
        this.spinnerMotor.configContinuousCurrentLimit(SPINNER_CURRENT_LIMIT);
        this.spinnerMotor.enableCurrentLimit(true);

        // Configure brake mode
        this.spinnerMotor.setNeutralMode(NeutralMode.Brake);
    }

    private enum State {
        OPEN_LOOP, CLOSED_LOOP
    }
}
