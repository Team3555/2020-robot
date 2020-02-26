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
    private static final int SHOOTER_CURRENT_LIMIT = 30;
    private static final int ENCODER_TICKS_PER_ROTATION = 4096;
    private static final double ALLOWED_ERROR = 5;
    private static final int SHORT_SHOT_RPM = 1000;

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

    private HoodState hoodState;

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
     * Extends the hood
     */
    public void extendHood() {
        hoodState = HoodState.UP;
    }

    /**
     * Retracts the hood
     */
    public void retractHood() {
        hoodState = HoodState.DOWN;
    }

    /**
     * Returns the hood's state
     */
    public HoodState getHoodState() {
        return hoodState;
    }

    public void update(double timestamp, SystemMode mode) {
        if (!motorGroup.getMasterTalon().isOK()) {
            robotFaults.setShooterFault(true);
        }

        if (!motorGroup.isEncoderOK()) {
            robotFaults.setShooterFault(true);
        }

        if (mode == SystemMode.OPERATOR_CONTROLLED) {
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

                    double output = turnController.update(0, limelight.getX(), timestamp);
                    driveSystem.manualArcadeDrive(output, -driverController.getSquaredLeftY());

                    // Mark pid controller as used
                    wasTracking = true;
                } else {
                    driveSystem.setVisionTracking(false);
                }

                // Extend hood
                extendHood();

                // Set flywheel speed
                double targetHeight = limelight.getVertical();
                double rpm = limelight.hasTarget() ? ShooterUtil.calculateRPM(targetHeight) : SHORT_SHOT_RPM;
                set(rpm);

                if ((driverController.getTriggerAxis(Hand.kRight) >= 0.5
                        || operatorController.getTriggerAxis(Hand.kRight) >= 0.5)
                        && Math.abs(rpm - getVelocity()) <= ALLOWED_ERROR && limelight.hasTarget()) {
                    // Fire power cells
                    magazineSystem.startFeedingPowerCells();
                } else {
                    magazineSystem.stopFeedingPowerCells();
                }
            } else if (driverController.getRawButton(5)) {
                // Driver wants the robot to align with vision target and flywheel to get up to
                // speed

                // Configure limelight for driver vision
                limelight.setPipeline(0);

                wasTracking = false;

                // Retract hood
                retractHood();

                // Set flywheel speed
                set(SHORT_SHOT_RPM);

                if ((driverController.getRawButton(6)) && Math.abs(SHORT_SHOT_RPM - getVelocity()) <= ALLOWED_ERROR) {
                    // Fire power cells
                    magazineSystem.startFeedingPowerCells();
                } else {
                    magazineSystem.stopFeedingPowerCells();
                }
            } else {
                // Configure limelight for driver vision
                limelight.setPipeline(0);

                // Tell the drivetrain to use controller for driving
                driveSystem.setVisionTracking(false);

                wasTracking = false;

                // Stop feeding power cells
                magazineSystem.stopFeedingPowerCells();

                // Retract hood
                retractHood();

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

            // Update hood
            if (hoodState == HoodState.DOWN) {
                hoodMotor.set(1);
            } else {
                hoodMotor.set(-1);
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

        this.hoodState = HoodState.DOWN;

        this.wasTracking = false;

        // Invert master
        this.motorGroup.getMaster().setInverted(true);

        // Configure encoder
        this.motorGroup.getMasterTalon().configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure PID
        this.motorGroup.getMasterTalon().config_kP(0, 0.01);
        this.motorGroup.getMasterTalon().config_kI(0, 0);
        this.motorGroup.getMasterTalon().config_kD(0, 0);
        this.motorGroup.getMasterTalon().config_kF(0, 0.0278);
        this.motorGroup.getMasterTalon().config_IntegralZone(0, 400);

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

        this.turnController = new AluminatiTuneablePIDController(5808, 0.1, 0, 0.1, 400, 1, 1, 0);
    }

    private enum HoodState {
        UP, DOWN
    }
}
