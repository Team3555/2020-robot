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
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import org.aluminati3555.frc2020.RobotFaults;
import org.aluminati3555.frc2020.Robot.ControlPanelColor;
import org.aluminati3555.lib.drivers.AluminatiColorSensor;
import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.net.AluminatiTunable;
import org.aluminati3555.lib.pneumatics.AluminatiSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

import edu.wpi.first.wpilibj.util.Color;

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

    private static final Color BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private static final Color GREEN = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private static final Color RED = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private static final Color YELLOW = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private static final double MIN_ACTUAL_PROXIMITY = 0.5;

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
    private AluminatiSolenoid extenderSolenoid;
    private AluminatiColorSensor colorSensor;

    private RobotFaults robotFaults;

    private ColorMatch colorMatch;
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
        extenderSolenoid.enable();
    }

    /**
     * Retracts the spinner mechanism
     */
    public void retract() {
        extenderSolenoid.disable();
    }

    /**
     * Returns the control panel color seen by the color sensor
     */
    public ControlPanelColor getColor() {
        if (colorSensor.getActualProximityPercent() < MIN_ACTUAL_PROXIMITY) {
            return ControlPanelColor.UNKOWN;
        }

        ColorMatchResult result = colorMatch.matchClosestColor(colorSensor.getColor());
        if (result.color == BLUE) {
            return ControlPanelColor.BLUE;
        } else if (result.color == RED) {
            return ControlPanelColor.RED;
        } else if (result.color == GREEN) {
            return ControlPanelColor.GREEN;
        } else if (result.color == YELLOW) {
            return ControlPanelColor.YELLOW;
        }

        return ControlPanelColor.UNKOWN;
    }

    public void update(double timestamp, boolean enabled) {
        // Report failures to driver
        if (!this.spinnerMotor.isOK()) {
            robotFaults.setSpinnerFault(true);
        }

        if (!this.spinnerMotor.isEncoderOK()) {
            robotFaults.setSpinnerFault(true);
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

    public SpinnerSystem(AluminatiTalonSRX spinnerMotor, AluminatiSolenoid extenderSolenoid,
            AluminatiColorSensor colorSensor, RobotFaults robotFaults) {
        this.spinnerMotor = spinnerMotor;
        this.extenderSolenoid = extenderSolenoid;
        this.colorSensor = colorSensor;

        this.robotFaults = robotFaults;

        this.colorMatch = new ColorMatch();
        this.colorMatch.addColorMatch(BLUE);
        this.colorMatch.addColorMatch(GREEN);
        this.colorMatch.addColorMatch(RED);
        this.colorMatch.addColorMatch(YELLOW);

        this.state = State.OPEN_LOOP;

        // Configure encoder
        this.spinnerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure PID
        this.spinnerMotor.config_kP(0, 0.1);
        this.spinnerMotor.config_kI(0, 0);
        this.spinnerMotor.config_kD(0, 0);
        this.spinnerMotor.config_IntegralZone(0, 400);

        // Setup tuning listener
        new AluminatiTunable(5807) {
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
