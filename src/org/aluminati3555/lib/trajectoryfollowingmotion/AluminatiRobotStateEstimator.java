package org.aluminati3555.lib.trajectoryfollowingmotion;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

import org.aluminati3555.lib.drive.AluminatiDrive;
import org.aluminati3555.lib.loops.Loop;

/**
 * This class keeps track of where the robot is on the field. It is based off of
 * the robot state estimator from team 195.
 * 
 * @author Caleb Heydon
 */
public class AluminatiRobotStateEstimator implements Loop {
    private RobotState robotState;
    private AluminatiDrive drive;

    private double lastLeftDistance;
    private double lastRightDistance;

    @Override
    public String toString() {
        return "[RobotStateEstimator]";
    }

    public void onStart(double timestamp) {
        lastLeftDistance = drive.getLeftDistanceInches();
        lastRightDistance = drive.getRightDistanceInches();
    }

    public void onLoop(double timestamp) {
        double leftDistance = drive.getLeftDistanceInches();
        double rightDistance = drive.getRightDistanceInches();
        Rotation2d heading = drive.getGyro().getHeading();

        Twist2d odometryVelocity = robotState.generateOdometryFromSensors(leftDistance - lastLeftDistance,
                rightDistance - lastRightDistance, heading);
        Twist2d predictedVelocity = Kinematics.forwardKinematics(drive.getLeftVelocityInchesPerSecond(),
                drive.getRightVelocityInchesPerSecond());
        robotState.addObservations(timestamp, odometryVelocity, predictedVelocity);

        lastLeftDistance = leftDistance;
        lastRightDistance = rightDistance;
    }

    public void onStop(double timestamp) {

    }

    public String getName() {
        return toString();
    }

    public AluminatiRobotStateEstimator(RobotState robotState, AluminatiDrive drive) {
        this.robotState = robotState;
        this.drive = drive;
    }
}
