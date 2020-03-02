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

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.aluminati3555.frc2020.RobotFaults;
import org.aluminati3555.frc2020.util.ShooterUtil;
import org.aluminati3555.lib.drivers.AluminatiMotorGroup;
import org.aluminati3555.lib.drivers.AluminatiSpark;
import org.aluminati3555.lib.drivers.AluminatiXboxController;
import org.aluminati3555.lib.net.AluminatiTuneable;
import org.aluminati3555.lib.pid.AluminatiTuneablePIDController;
import org.aluminati3555.lib.system.AluminatiSystem;
import org.aluminati3555.lib.vision.AluminatiLimelight;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class controls the shooter flywheel and retractable hood
 * 
 * @author Caleb Heydon
 */
public class ShooterSystem implements AluminatiSystem {
    // Constants
    private static final int SHOOTER_CURRENT_LIMIT = 20;
    private static final int ENCODER_TICKS_PER_ROTATION = 4096;
    private static final double ALLOWED_ERROR = 2000;
    private static final int SHORT_SHOT_RPM = 4000;
    private static final double HOOD_STOP_CURRENT = 16;
    private static final double HOOD_UP_TIME = 0.4;
    private static final double HOOD_MID_TIME = 0.24;

    /**
     * Converts rpm to native units
     */
    private static int convertRPMToNativeUnits(double rpm) {
        return (int) (rpm * ENCODER_TICKS_PER_ROTATION / 600.0);
    }

    /**
     * Converts native units to rpm
     */
    private static int convertNativeUnitsToRPM(double nativeUnits) {
        return (int) (nativeUnits / ENCODER_TICKS_PER_ROTATION * 600.0);
    }

    private AluminatiMotorGroup motorGroup;
    private AluminatiSpark hoodMotor;
    private AluminatiXboxController driverController;
    private AluminatiXboxController operatorController;
    private AluminatiLimelight limelight;
    private DriveSystem driveSystem;
    private MagazineSystem magazineSystem;

    private RobotFaults robotFaults;

    private double hoodStartTime;
    private HoodPosition hoodPosition;
    private HoodAction hoodAction;
    private ArrayList<HoodAction> hoodActionList;

    private boolean wasTracking;

    private boolean shooterEnabled;
    private double setpoint;

    private AluminatiTuneablePIDController turnController;

    /**
     * Sets the target rpm
     */
    public synchronized void set(double rpm) {
        this.setpoint = convertRPMToNativeUnits(rpm);
        this.shooterEnabled = true;
    }

    /**
     * Returns the target rpm
     */
    public synchronized double get() {
        if (!shooterEnabled) {
            return 0;
        }

        return convertNativeUnitsToRPM(setpoint);
    }

    /**
     * Returns the motor output as a percentage between 0 and 1
     */
    public double getOutputPercent() {
        return motorGroup.getMasterTalon().getMotorOutputPercent();
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
     * Returns the hood position
     */
    public HoodPosition getHoodPosition() {
        return hoodPosition;
    }

    /**
     * Extends the hood
     */
    public void extendHood() {
        if (hoodPosition == HoodPosition.UP || hoodPosition == HoodPosition.MID || (hoodPosition == HoodPosition.UNKNOWN
                && (hoodAction == HoodAction.EXTEND || hoodAction == HoodAction.EXTEND_MID))) {
            // The hood is either up on its way up
            return;
        }

        // Prevent double action
        if (hoodActionList.size() > 0 && hoodActionList.get(hoodActionList.size() - 1) == HoodAction.EXTEND) {
            return;
        }

        if (hoodActionList.size() < 2) {
            hoodActionList.add(HoodAction.EXTEND);
        }
    }

    /**
     * Extends the hood for the close shot
     */
    public void extendHoodMid() {
        if (hoodPosition == HoodPosition.UP || hoodPosition == HoodPosition.MID || (hoodPosition == HoodPosition.UNKNOWN
                && (hoodAction == HoodAction.EXTEND || hoodAction == HoodAction.EXTEND_MID))) {
            // The hood is either up on its way up
            return;
        }

        // Prevent double action
        if (hoodActionList.size() > 0 && hoodActionList.get(hoodActionList.size() - 1) == HoodAction.EXTEND_MID) {
            return;
        }

        if (hoodActionList.size() < 2) {
            hoodActionList.add(HoodAction.EXTEND_MID);
        }
    }

    /**
     * Retracts the hood
     */
    public void retractHood() {
        if (hoodPosition == HoodPosition.DOWN
                || (hoodPosition == HoodPosition.UNKNOWN && hoodAction == HoodAction.RETRACT)) {
            // The hood is either down or on its way down
            return;
        }

        // Prevent double action
        if (hoodActionList.size() > 0 && hoodActionList.get(hoodActionList.size() - 1) == HoodAction.RETRACT) {
            return;
        }

        if (hoodActionList.size() < 2) {
            hoodActionList.add(HoodAction.RETRACT);
        }
    }

    public void update(double timestamp, SystemMode mode) {
        if (!motorGroup.getMasterTalon().isOK()) {
            robotFaults.setShooterFault(true);
        }

        if (!motorGroup.isEncoderOK()) {
            robotFaults.setShooterFault(true);
        }

        if (mode == SystemMode.OPERATOR_CONTROLLED) {
            // Do hood control
            if (operatorController.getY(Hand.kLeft) >= 0.5) {
                retractHood();
            } else {
                extendHood();
            }

            if (driverController.getTriggerAxis(Hand.kLeft) >= 0.5) {
                // Driver wants the robot to align with vision target and flywheel to get up to
                // speed

                // Configure limelight for vision tracking
                limelight.setPipeline(1);

                // Only run if there is a target
                if (limelight.hasTarget()) {
                    // Reset the pid controller if it is being reused
                    if (!wasTracking) {
                        turnController.reset(timestamp);
                    }

                    // Tell the drivetrain not to use controller for driving
                    driveSystem.setVisionTracking(true);

                    double output = -turnController.update(0, limelight.getX(), timestamp);
                    driveSystem.manualArcadeDrive(output, -driverController.getSquaredLeftY());

                    // Mark pid controller as used
                    wasTracking = true;
                } else {
                    driveSystem.setVisionTracking(false);
                }

                // Set flywheel speed
                double targetHeight = limelight.getVertical();
                // double rpm = limelight.hasTarget() ? ShooterUtil.calculateRPM(targetHeight) :
                // SHORT_SHOT_RPM;
                double rpm = 4800;
                set(rpm);

                if ((driverController.getTriggerAxis(Hand.kRight) >= 0.5
                        || operatorController.getTriggerAxis(Hand.kRight) >= 0.5)
                        && (true || Math.abs(rpm - getVelocity()) <= ALLOWED_ERROR && limelight.hasTarget())) {
                    // Fire power cells
                    magazineSystem.startFeedingPowerCells();
                }
            } else if (driverController.getRawButton(5)) {
                // Driver wants the robot to align with vision target and flywheel to get up to
                // speed

                // Configure limelight for driver vision
                limelight.setPipeline(0);

                wasTracking = false;

                // Set flywheel speed
                set(SHORT_SHOT_RPM);

                if ((driverController.getRawButton(6)) && Math.abs(SHORT_SHOT_RPM - getVelocity()) <= ALLOWED_ERROR) {
                    // Fire power cells
                    magazineSystem.startFeedingPowerCells();
                }
            } else {
                // Configure limelight for driver vision
                limelight.setPipeline(0);

                // Tell the drivetrain to use controller for driving
                driveSystem.setVisionTracking(false);

                wasTracking = false;

                // Stop feeding power cells
                magazineSystem.stopFeedingPowerCells();

                // Disable flywheel
                stop();
            }
        }

        if (mode != SystemMode.DISABLED) {
            // Set flywheel rpm
            if (shooterEnabled) {
                motorGroup.getMasterTalon().set(ControlMode.Velocity, setpoint);
            } else {
                motorGroup.getMasterTalon().set(ControlMode.PercentOutput, 0);
            }

            // Update hood action
            if ((hoodAction == HoodAction.NONE
                    || ((hoodAction == HoodAction.EXTEND || hoodAction == HoodAction.EXTEND_MID)
                            && hoodActionList.size() > 0 && hoodActionList.get(0) == HoodAction.RETRACT))
                    && hoodActionList.size() > 0) {
                hoodAction = hoodActionList.get(0);
                hoodActionList.remove(0);

                hoodPosition = HoodPosition.UNKNOWN;

                if (hoodAction == HoodAction.EXTEND || hoodAction == HoodAction.EXTEND_MID) {
                    hoodStartTime = timestamp;
                }
            }

            // Update extend action
            if (hoodAction == HoodAction.EXTEND) {
                // Check for completion
                if (timestamp >= hoodStartTime + HOOD_UP_TIME) {
                    hoodAction = HoodAction.NONE;
                    hoodPosition = HoodPosition.UP;
                }
            }

            if (hoodAction == HoodAction.EXTEND_MID) {
                // Check for completion
                if (timestamp >= hoodStartTime + HOOD_MID_TIME) {
                    hoodAction = HoodAction.NONE;
                    hoodPosition = HoodPosition.MID;
                }
            }

            // Update retract action
            if (hoodAction == HoodAction.RETRACT) {
                // Check for completion
                if (hoodMotor.getOutputCurrent() >= HOOD_STOP_CURRENT) {
                    hoodAction = HoodAction.NONE;
                    hoodPosition = HoodPosition.DOWN;
                }
            }

            // Update output to motor
            switch (hoodAction) {
                case EXTEND:
                case EXTEND_MID:
                    hoodMotor.set(-1);
                    break;
                case RETRACT:
                    hoodMotor.set(1);
                    break;
                case NONE:
                    if (hoodPosition == HoodPosition.DOWN) {
                        hoodMotor.set(0.05);
                    } else {
                        hoodMotor.set(0);
                    }
                    break;
                default:
                    hoodMotor.set(0);
                    break;
            }
        }
    }

    public ShooterSystem(AluminatiMotorGroup motorGroup, AluminatiSpark hoodMotor,
            AluminatiXboxController driverController, AluminatiXboxController operatorController,
            AluminatiLimelight limelight, DriveSystem driveSystem, MagazineSystem magazineSystem,
            RobotFaults robotFaults) {
        this.motorGroup = motorGroup;
        this.hoodMotor = hoodMotor;
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.limelight = limelight;
        this.driveSystem = driveSystem;
        this.magazineSystem = magazineSystem;

        this.robotFaults = robotFaults;

        this.hoodPosition = HoodPosition.UP;
        this.hoodAction = HoodAction.NONE;
        this.hoodActionList = new ArrayList<HoodAction>();

        this.wasTracking = false;

        // Invert master
        this.motorGroup.getMaster().setInverted(true);

        // Configure encoder
        this.motorGroup.getMasterTalon().configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure PID
        this.motorGroup.getMasterTalon().config_kP(0, 0.2);
        this.motorGroup.getMasterTalon().config_kI(0, 0);
        this.motorGroup.getMasterTalon().config_kD(0, 0);
        this.motorGroup.getMasterTalon().config_kF(0, 0.027);
        this.motorGroup.getMasterTalon().config_IntegralZone(0, 400);

        this.motorGroup.getMasterTalon().configClosedloopRamp(1);

        this.motorGroup.coast();

        // Setup tuning listener
        new AluminatiTuneable(5806) {
            protected void update(TuningData data) {
                motorGroup.getMasterTalon().config_kP(0, data.kP);
                motorGroup.getMasterTalon().config_kI(0, data.kI);
                motorGroup.getMasterTalon().config_kD(0, data.kD);
            }
        };

        // Configure current limit
        this.motorGroup.getMasterTalon().configPeakCurrentDuration(500);
        this.motorGroup.getMasterTalon().configPeakCurrentLimit(SHOOTER_CURRENT_LIMIT);
        this.motorGroup.getMasterTalon().configContinuousCurrentLimit(SHOOTER_CURRENT_LIMIT);
        this.motorGroup.getMasterTalon().enableCurrentLimit(true);

        this.turnController = new AluminatiTuneablePIDController(5808, 0.05, 0, 0, 400, 1, 0.6, 0);
    }

    public enum HoodPosition {
        UP, DOWN, UNKNOWN, MID
    }

    private enum HoodAction {
        EXTEND, RETRACT, NONE, EXTEND_MID
    }
}
