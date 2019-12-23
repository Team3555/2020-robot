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

package org.aluminati3555.frc2020.auto;

import com.team254.lib.geometry.Pose2d;

import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;

/**
 * This class resets the robot's position for running a path
 * 
 * @author Caleb Heydon
 */
public class ActionResetRobotPose implements AluminatiAutoTask {
    private RobotState robotState;
    private DriveSystem driveSystem;
    private Pose2d robotPose;

    public void start(double timestamp) {
        robotState.reset(timestamp, robotPose, driveSystem);
    }

    public void update(double timestamp) {

    }

    public void stop() {

    }

    public void advanceState() {

    }

    public boolean isComplete() {
        return true;
    }

    public ActionResetRobotPose(RobotState robotState, DriveSystem driveSystem, Pose2d robotPose) {
        this.robotState = robotState;
        this.driveSystem = driveSystem;
        this.robotPose = robotPose;
    }
}
