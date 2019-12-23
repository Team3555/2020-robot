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

package org.aluminati3555.lib.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.control.PathFollower.Parameters;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;

import org.aluminati3555.lib.data.AluminatiData;
import org.aluminati3555.lib.drivers.AluminatiMotorGroup;
import org.aluminati3555.lib.drivers.AluminatiJoystick;
import org.aluminati3555.lib.drivers.AluminatiXboxController;
import org.aluminati3555.lib.loops.Loop;
import org.aluminati3555.lib.loops.Looper;
import org.aluminati3555.lib.util.AluminatiUtil;

import org.aluminati3555.lib.drivers.AluminatiCriticalDevice;
import org.aluminati3555.lib.drivers.AluminatiGyro;
import org.aluminati3555.lib.trajectoryfollowingmotion.Kinematics;
import org.aluminati3555.lib.trajectoryfollowingmotion.PathContainer;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;

/**
 * This system controls the drivetrain
 * 
 * @author Caleb Heydon
 */
public class AluminatiDrive implements AluminatiCriticalDevice {
    // Class members
    private DriveState driveState;
    private RobotState robotState;

    private AluminatiMotorGroup left;
    private AluminatiMotorGroup right;
    private AluminatiGyro gyro;

    private AluminatiDriveHelper driveHelper;
    private AluminatiShifter shifter;

    private PathFollower pathFollower;

    private double controlCoefficient;
    private boolean inverted;

    @Override
    public String toString() {
        return "[Drive]";
    }

    /**
     * Returns the state of the drive
     * 
     * @return
     */
    public DriveState getDriveState() {
        return driveState;
    }

    /**
     * Returns the left motor group
     */
    public AluminatiMotorGroup getLeftGroup() {
        return left;
    }

    /**
     * Returns the right motor group
     */
    public AluminatiMotorGroup getRightGroup() {
        return right;
    }

    /**
     * Returns the gyro
     */
    public AluminatiGyro getGyro() {
        return gyro;
    }

    /**
     * Returns true if the system is ok
     */
    public boolean isOK() {
        return (left.isOK() && right.isOK() && gyro.isOK() && left.isEncoderOK() && right.isEncoderOK());
    }

    /**
     * Returns the shifter
     */
    public AluminatiShifter getShifter() {
        return shifter;
    }

    /**
     * Returns the joystick coefficient
     */
    public double getJoystickCoefficient() {
        return controlCoefficient;
    }

    /**
     * Sets the joystick coefficient
     */
    public void setJoystickCoefficient(double controlCoefficient) {
        this.controlCoefficient = controlCoefficient;
    }

    /**
     * Returns true if the drive is inverted
     */
    public boolean isInverted() {
        return inverted;
    }

    /**
     * Sets the drive inverted
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    /**
     * Sets the control mode to percent output and to 0
     */
    private synchronized void resetMasters() {
        left.getMaster().set(ControlMode.PercentOutput, 0);
        right.getMaster().set(ControlMode.PercentOutput, 0);
    }

    /**
     * Puts the drive in coast mode
     */
    public void coast() {
        left.coast();
        right.coast();
    }

    /**
     * Puts the drive in brake mode
     */
    public void brake() {
        left.brake();
        right.brake();
    }

    /**
     * Returns the distance the left side of the drive has travelled
     * 
     * @return
     */
    public double getLeftDistanceInches() {
        return AluminatiUtil.rotationsToInches(
                left.getMasterTalon().getSelectedSensorPosition() / (double) (AluminatiData.encoderUnitsPerRotation));
    }

    /**
     * Returns the distance the right side of the drive has travelled
     * 
     * @return
     */
    public double getRightDistanceInches() {
        return AluminatiUtil.rotationsToInches(
                right.getMasterTalon().getSelectedSensorPosition() / (double) (AluminatiData.encoderUnitsPerRotation));
    }

    /**
     * Returns the left wheel velocity in inches per second
     * 
     * @return
     */
    public double getLeftVelocityInchesPerSecond() {
        return AluminatiUtil.rpmToInchesPerSecond(
                AluminatiUtil.convertNativeUnitsToRPM(left.getMasterTalon().getSelectedSensorVelocity()));
    }

    /**
     * Returns the right wheel velocity in inches per second
     * 
     * @return
     */
    public double getRightVelocityInchesPerSecond() {
        return AluminatiUtil.rpmToInchesPerSecond(
                AluminatiUtil.convertNativeUnitsToRPM(right.getMasterTalon().getSelectedSensorVelocity()));
    }

    /**
     * Sets the left velocity in inches per second
     */
    public void setLeftVelocityInchesPerSecond(double velocity) {
        double output = AluminatiUtil.convertRPMToNativeUnits(AluminatiUtil.inchesPerSecondToRPM(velocity));
        left.getMaster().set(ControlMode.Velocity, output);
    }

    /**
     * Sets the right velocity in inches per second
     */
    public void setRightVelocityInchesPerSecond(double velocity) {
        double output = AluminatiUtil.convertRPMToNativeUnits(AluminatiUtil.inchesPerSecondToRPM(velocity));
        right.getMaster().set(ControlMode.Velocity, output);
    }

    /**
     * Starts a path
     */
    public void startPath(PathContainer pathContainer, double timestamp) {
        // Stop the current path if the drive is in path following mode
        if (driveState == DriveState.PATH_FOLLOWING) {
            stopPath();
        }

        // Stop a path if there is one
        stopPath();

        // Generate the path and get starting position
        Path path = pathContainer.buildPath();
        boolean reversed = pathContainer.isReversed();

        // Create the path follower
        pathFollower = new PathFollower(path, reversed,
                new Parameters(
                        new Lookahead(AluminatiData.minLookAhead, AluminatiData.maxLookAhead,
                                AluminatiData.minLookAheadSpeed, AluminatiData.maxLookAheadSpeed),
                        AluminatiData.inertiaSteeringGain, AluminatiData.pathFollowingProfileKP,
                        AluminatiData.pathFollowingProfileKI, AluminatiData.pathFollowingProfileKV,
                        AluminatiData.pathFollowingProfileKFFV, AluminatiData.pathFollowingProfileKFFA,
                        AluminatiData.pathFollowingProfileKS, AluminatiData.pathFollowingMaxVel,
                        AluminatiData.pathFollowingMaxAccel, AluminatiData.pathFollowingGoalPosTolerance,
                        AluminatiData.pathFollowingGoalVelTolerance, AluminatiData.pathStopSteeringDistance));

        driveState = DriveState.PATH_FOLLOWING;
    }

    /**
     * Updates the path follower
     * 
     * @param timestamp
     */
    private synchronized void updatePathFollower(double timestamp) {
        if (pathFollower != null && !pathFollower.isFinished()) {
            Pose2d robotPose = robotState.getLatestFieldToVehicle().getValue();
            Twist2d command = pathFollower.update(timestamp, robotPose, robotState.getDistanceDriven(),
                    robotState.getPredictedVelocity().dx);

            DriveSignal setpoint = Kinematics.inverseKinematics(command);
            setLeftVelocityInchesPerSecond(setpoint.getLeft());
            setRightVelocityInchesPerSecond(setpoint.getRight());
        }
    }

    /**
     * Returns true if the path is complete
     * 
     * @return
     */
    public boolean isPathDone() {
        if (pathFollower == null) {
            return true;
        }

        return pathFollower.isFinished();
    }

    /**
     * Returns true if the marker has been passed
     */
    public boolean hasPassedPathMarker(String marker) {
        if (pathFollower == null) {
            return false;
        }

        return pathFollower.hasPassedMarker(marker);
    }

    /**
     * Stops the current path
     */
    public void stopPath() {
        if (pathFollower != null) {
            pathFollower.forceFinish();
        }

        resetMasters();
        driveState = DriveState.OPEN_LOOP;
    }

    /**
     * Drives the robot using arcade drive
     */
    public void arcadeDrive(AluminatiJoystick joystick) {
        driveHelper.aluminatiDrive(-joystick.getSquaredY() * controlCoefficient,
                joystick.getSquaredX() * controlCoefficient, true, (shifter == null) ? true : shifter.isHigh());

        if (!inverted) {
            left.getMaster().set(ControlMode.PercentOutput, driveHelper.getLeftPower());
            right.getMaster().set(ControlMode.PercentOutput, driveHelper.getRightPower());
        } else {
            left.getMaster().set(ControlMode.PercentOutput, -driveHelper.getRightPower());
            right.getMaster().set(ControlMode.PercentOutput, -driveHelper.getLeftPower());
        }
    }

    /**
     * Drives the robot using arcade drive
     */
    public void arcadeDrive(AluminatiXboxController controller) {
        driveHelper.aluminatiDrive(-controller.getSquaredY() * controlCoefficient,
                controller.getSquaredX() * controlCoefficient, true, (shifter == null) ? true : shifter.isHigh());

        if (!inverted) {
            left.getMaster().set(ControlMode.PercentOutput, driveHelper.getLeftPower());
            right.getMaster().set(ControlMode.PercentOutput, driveHelper.getRightPower());
        } else {
            left.getMaster().set(ControlMode.PercentOutput, -driveHelper.getRightPower());
            right.getMaster().set(ControlMode.PercentOutput, -driveHelper.getLeftPower());
        }
    }

    /**
     * Drives the robot using cheesy drive
     */
    public void cheesyDrive(AluminatiJoystick joystick, int cheesyDriveButton) {
        driveHelper.aluminatiDrive(-joystick.getSquaredY() * controlCoefficient,
                joystick.getSquaredX() * controlCoefficient, joystick.getRawButton(cheesyDriveButton),
                (shifter == null) ? true : shifter.isHigh());

        if (!inverted) {
            left.getMaster().set(ControlMode.PercentOutput, driveHelper.getLeftPower());
            right.getMaster().set(ControlMode.PercentOutput, driveHelper.getRightPower());
        } else {
            left.getMaster().set(ControlMode.PercentOutput, -driveHelper.getRightPower());
            right.getMaster().set(ControlMode.PercentOutput, -driveHelper.getLeftPower());
        }
    }

    /**
     * Drives the robot using cheesy drive
     */
    public void cheesyDrive(AluminatiXboxController controller, int cheesyDriveButton) {
        driveHelper.aluminatiDrive(-controller.getSquaredY() * controlCoefficient,
                controller.getSquaredX() * controlCoefficient, controller.getRawButton(cheesyDriveButton),
                (shifter == null) ? true : shifter.isHigh());

        if (!inverted) {
            left.getMaster().set(ControlMode.PercentOutput, driveHelper.getLeftPower());
            right.getMaster().set(ControlMode.PercentOutput, driveHelper.getRightPower());
        } else {
            left.getMaster().set(ControlMode.PercentOutput, -driveHelper.getRightPower());
            right.getMaster().set(ControlMode.PercentOutput, -driveHelper.getLeftPower());
        }
    }

    /**
     * Manuall uses arcade drive (no helper) with no squared output, control
     * coefficient, or drive inversion
     */
    public void manualArcadeDrive(double x, double y) {
        left.getMaster().set(ControlMode.PercentOutput, y + x);
        right.getMaster().set(ControlMode.PercentOutput, y - x);
    }

    /**
     * Zeros the encoders
     */
    public void zeroEncoders() {
        left.getMasterTalon().setSelectedSensorPosition(0);
        right.getMasterTalon().setSelectedSensorPosition(0);
    }

    public AluminatiDrive(Looper looper, RobotState robotState, AluminatiMotorGroup left, AluminatiMotorGroup right,
            AluminatiGyro gyro) {
        this.robotState = robotState;
        this.left = left;
        this.right = right;
        this.gyro = gyro;

        driveHelper = new AluminatiDriveHelper();
        driveHelper.aluminatiDrive(0, 0, true, true);

        controlCoefficient = 1;

        // Configure for path following
        AluminatiUtil.configTalonsPathFollowing(left.getMasterTalon(), right.getMasterTalon());

        left.getMaster().set(ControlMode.PercentOutput, driveHelper.getLeftPower());
        right.getMaster().set(ControlMode.PercentOutput, driveHelper.getRightPower());

        driveState = DriveState.OPEN_LOOP;

        // Register drive loop
        looper.register(new DriveLoop());
    }

    public AluminatiDrive(Looper looper, RobotState robotState, AluminatiMotorGroup left, AluminatiMotorGroup right,
            AluminatiGyro gyro, AluminatiShifter shifter) {
        this(looper, robotState, left, right, gyro);
        this.shifter = shifter;
    }

    private class DriveLoop implements Loop {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {
            // Set drive state
            if (driveState == DriveState.PATH_FOLLOWING && pathFollower == null) {
                resetMasters();
                driveState = DriveState.OPEN_LOOP;
            } else if (driveState == DriveState.PATH_FOLLOWING && isPathDone()) {
                // Reset masters
                resetMasters();
                driveState = DriveState.OPEN_LOOP;
            }

            // We do not need to do anything if the state is OPEN_LOOP or MOTION_PROFILING
            if (driveState == DriveState.PATH_FOLLOWING) {
                // Update the path follower
                updatePathFollower(timestamp);
            }
        }

        public void onStop(double timestamp) {

        }

        public String getName() {
            return "[DriveLoop]";
        }
    }

    public enum DriveState {
        OPEN_LOOP, PATH_FOLLOWING
    }
}
