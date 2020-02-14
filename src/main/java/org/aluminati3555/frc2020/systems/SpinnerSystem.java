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

import org.aluminati3555.frc2020.RobotFaults;
import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.drivers.AluminatiXboxController;
import org.aluminati3555.lib.pneumatics.AluminatiSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class controls the spinner for the control panel
 * 
 * @author Caleb Heydon
 */
public class SpinnerSystem implements AluminatiSystem {
    private static final int SPINNER_CURRENT_LIMIT = 30;
    private static final double SPINNER_DEADBAND = 0.1;

    private AluminatiTalonSRX spinnerMotor;
    private AluminatiSolenoid extenderSolenoid;

    private AluminatiXboxController operatorController;

    private RobotFaults robotFaults;

    /**
     * Returns true if the mechanism is up
     */
    public boolean isUp() {
        return extenderSolenoid.get();
    }

    /**
     * Returns true if the mechanism is down
     */
    public boolean isDown() {
        return !extenderSolenoid.get();
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

    public void update(double timestamp, boolean enabled) {
        // Report failures to driver
        if (!this.spinnerMotor.isOK()) {
            robotFaults.setSpinnerFault(true);
        }

        if (!this.spinnerMotor.isEncoderOK()) {
            robotFaults.setSpinnerFault(true);
        }

        if (enabled) {
            if (operatorController.getTriggerAxis(Hand.kLeft) >= 0.5) {
                extend();
            } else {
                retract();
            }

            double rightJoystick = operatorController.getX(Hand.kRight);
            if (Math.abs(rightJoystick) > SPINNER_DEADBAND && isUp()) {
                spinnerMotor.set(ControlMode.PercentOutput, rightJoystick);
            } else {
                spinnerMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            // The control panel manipulator is never used in auto
            spinnerMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public SpinnerSystem(AluminatiTalonSRX spinnerMotor, AluminatiSolenoid extenderSolenoid,
            AluminatiXboxController operatorController, RobotFaults robotFaults) {
        this.spinnerMotor = spinnerMotor;
        this.extenderSolenoid = extenderSolenoid;

        this.operatorController = operatorController;

        this.robotFaults = robotFaults;

        // Configure encoder
        this.spinnerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure current limit
        this.spinnerMotor.configPeakCurrentDuration(500);
        this.spinnerMotor.configPeakCurrentLimit(SPINNER_CURRENT_LIMIT);
        this.spinnerMotor.configContinuousCurrentLimit(SPINNER_CURRENT_LIMIT);
        this.spinnerMotor.enableCurrentLimit(true);

        // Configure brake mode
        this.spinnerMotor.setNeutralMode(NeutralMode.Brake);
    }
}
