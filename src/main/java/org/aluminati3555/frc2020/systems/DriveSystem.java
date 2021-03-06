/**
 * Copyright (c) 2019 Team 3555
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

import org.aluminati3555.frc2020.RobotFaults;
import org.aluminati3555.lib.drive.AluminatiDrive;
import org.aluminati3555.lib.drivers.AluminatiMotorGroup;
import org.aluminati3555.lib.drivers.AluminatiXboxController;
import org.aluminati3555.lib.drivers.AluminatiGyro;
import org.aluminati3555.lib.loops.Looper;
import org.aluminati3555.lib.system.AluminatiSystem;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;

/**
 * This is the drivetrain class for the robot
 * 
 * @author Caleb Heydon
 */
public class DriveSystem extends AluminatiDrive implements AluminatiSystem {
    // Constants
    private static final int DRIVE_CURRENT_LIMIT_1 = 40;

    private AluminatiXboxController driverController;

    private RobotFaults robotFaults;

    private boolean visionTracking;

    /**
     * Sets this to true to disable normal driving while using the limelight
     */
    public void setVisionTracking(boolean visionTracking) {
        this.visionTracking = visionTracking;
    }

    public void update(double timestamp, SystemMode mode) {
        // Report failures to driver
        if (!this.getLeftGroup().isOK()) {
            robotFaults.setDriveFault(true);
        }

        if (!this.getLeftGroup().isEncoderOK()) {
            robotFaults.setDriveFault(true);
        }

        if (!this.getRightGroup().isOK()) {
            robotFaults.setDriveFault(true);
        }

        if (!this.getRightGroup().isEncoderOK()) {
            robotFaults.setDriveFault(true);
        }

        if (!this.getGyro().isOK()) {
            robotFaults.setDriveFault(true);
        }

        if (mode == SystemMode.OPERATOR_CONTROLLED) {
            if (!visionTracking && this.getDriveState() == DriveState.OPEN_LOOP) {
                this.arcadeDrive(driverController);
            }
        }
    }

    public DriveSystem(Looper looper, RobotState robotState, AluminatiMotorGroup left, AluminatiMotorGroup right,
            AluminatiGyro gyro, AluminatiXboxController driverController, RobotFaults robotFaults) {
        super(looper, robotState, left, right, gyro);

        this.driverController = driverController;
        this.robotFaults = robotFaults;

        this.getLeftGroup().getMasterTalon().configPeakCurrentDuration(500);
        this.getLeftGroup().getMasterTalon().configPeakCurrentLimit(DRIVE_CURRENT_LIMIT_1);
        this.getLeftGroup().getMasterTalon().configContinuousCurrentLimit(DRIVE_CURRENT_LIMIT_1);
        this.getLeftGroup().getMasterTalon().enableCurrentLimit(true);

        this.getRightGroup().getMasterTalon().configPeakCurrentDuration(500);
        this.getRightGroup().getMasterTalon().configPeakCurrentLimit(DRIVE_CURRENT_LIMIT_1);
        this.getRightGroup().getMasterTalon().configContinuousCurrentLimit(DRIVE_CURRENT_LIMIT_1);
        this.getRightGroup().getMasterTalon().enableCurrentLimit(true);

        this.coast();
    }
}
