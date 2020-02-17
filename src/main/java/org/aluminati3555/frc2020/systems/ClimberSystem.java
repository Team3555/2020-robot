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
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.aluminati3555.frc2020.RobotFaults;
import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.drivers.AluminatiXboxController;
import org.aluminati3555.lib.pneumatics.AluminatiSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class controls climber
 * 
 * @author Caleb Heydon
 */
public class ClimberSystem implements AluminatiSystem {
    private static final int ARM_CURRENT_LIMIT = 40;
    private static final int SPOOL_CURRENT_LIMIT = 40;
    private static final double ARM_DEADBAND = 0.1;
    private static final double LOWERING_DELAY = 0.1;

    private AluminatiTalonSRX armMotor;
    private AluminatiTalonSRX spoolMotor;
    private AluminatiSolenoid ratchetSolenoid;

    private IntakeSystem intakeSystem;

    private AluminatiXboxController operatorController;

    private RobotFaults robotFaults;

    private boolean isLowering;
    private double startLoweringTime;

    /**
     * Locks the ratchet
     */
    public void lockRatchet() {
        ratchetSolenoid.disable();
    }

    /**
     * Unlocks the ratchet
     */
    public void unlockRatchet() {
        ratchetSolenoid.enable();
    }

    /**
     * Returns true if the ratchet release is activated
     */
    private boolean getRatchetRelease() {
        return (operatorController.getRawButton(9) && operatorController.getRawButton(10)
                && operatorController.getX(Hand.kLeft) >= 0.5 && operatorController.getX(Hand.kRight) <= 0.5);
    }

    public void update(double timestamp, boolean enabled) {
        // Check for faults
        if (!armMotor.isOK()) {
            robotFaults.setClimberFault(true);
        }

        if (!spoolMotor.isOK()) {
            robotFaults.setClimberFault(true);
        }

        if (enabled) {
            // Manual control

            double leftJoystick = operatorController.getY(Hand.kLeft);
            int pov = operatorController.getPOV();

            if (((Math.abs(leftJoystick) > ARM_DEADBAND && !operatorController.getRawButton(9)) || pov == 0 || pov == 45
                    || pov == 315 || pov == 180 || pov == 135 || pov == 225) && intakeSystem.isUp()) {
                // Extend intake to prevent interference
                intakeSystem.extend();
            }

            if (leftJoystick < ARM_DEADBAND && !operatorController.getRawButton(9)) {
                // Lift arms
                armMotor.set(ControlMode.PercentOutput, leftJoystick);
            } else if (leftJoystick > ARM_DEADBAND && !operatorController.getRawButton(9)) {
                // Lower arms
                armMotor.set(ControlMode.PercentOutput, leftJoystick);
            } else {
                armMotor.set(ControlMode.PercentOutput, 0);
            }

            if (pov == 0 || pov == 45 || pov == 315) {
                if (!isLowering) {
                    // Remember time started lowering
                    isLowering = true;
                    startLoweringTime = timestamp;

                    unlockRatchet();
                }

                if (timestamp >= startLoweringTime + LOWERING_DELAY) {
                    spoolMotor.set(ControlMode.PercentOutput, -1);
                } else {
                    spoolMotor.set(ControlMode.PercentOutput, 0);
                }
            } else if (pov == 180 || pov == 135 || pov == 225) {
                isLowering = false;
                lockRatchet();

                spoolMotor.set(ControlMode.PercentOutput, 1);
            } else if (getRatchetRelease()) {
                isLowering = false;
                unlockRatchet();

                spoolMotor.set(ControlMode.PercentOutput, 0);
            } else {
                isLowering = false;
                lockRatchet();

                spoolMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            // We never need to use the climber in auto
            armMotor.set(ControlMode.PercentOutput, 0);
            spoolMotor.set(ControlMode.PercentOutput, 0);

            lockRatchet();
        }
    }

    public ClimberSystem(AluminatiTalonSRX armMotor, AluminatiTalonSRX spoolMotor, AluminatiSolenoid ratchetSolenoid,
            IntakeSystem intakeSystem, AluminatiXboxController operatorController, RobotFaults robotFaults) {
        this.armMotor = armMotor;
        this.spoolMotor = spoolMotor;
        this.ratchetSolenoid = ratchetSolenoid;
        this.intakeSystem = intakeSystem;

        this.operatorController = operatorController;

        this.robotFaults = robotFaults;

        this.isLowering = false;

        // Configure current limit
        this.armMotor.configPeakCurrentDuration(500);
        this.armMotor.configPeakCurrentLimit(ARM_CURRENT_LIMIT);
        this.armMotor.configContinuousCurrentLimit(ARM_CURRENT_LIMIT);
        this.armMotor.enableCurrentLimit(true);

        // Configure brake mode
        this.armMotor.setNeutralMode(NeutralMode.Brake);

        // Configure current limit for spool motor
        this.spoolMotor.configPeakCurrentDuration(500);
        this.spoolMotor.configPeakCurrentLimit(SPOOL_CURRENT_LIMIT);
        this.spoolMotor.configContinuousCurrentLimit(SPOOL_CURRENT_LIMIT);
        this.spoolMotor.enableCurrentLimit(true);

        // Configure brake mode
        this.spoolMotor.setNeutralMode(NeutralMode.Brake);
    }
}
