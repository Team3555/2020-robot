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

package org.aluminati3555.frc2020.auto.modes;

import org.aluminati3555.frc2020.auto.actions.ActionResetRobotPose;
import org.aluminati3555.frc2020.auto.actions.ActionRunPath;
import org.aluminati3555.frc2020.paths.PathGoForward;
import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.auto.AluminatiAutoTaskList;
import org.aluminati3555.lib.trajectoryfollowingmotion.PathContainer;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;

/**
 * This mode goes forward into the sector
 * 
 * @author Caleb Heydon
 */
public class ModeGoForward implements AluminatiAutoTask {
    private AluminatiAutoTaskList taskList;

    @Override
    public String toString() {
        return "GoForward";
    }

    public void start(double timestamp) {
        taskList.start(timestamp);
    }

    public void update(double timestamp) {
        taskList.update(timestamp);
    }

    public void stop() {
        taskList.stop();
    }

    public boolean isComplete() {
        return taskList.isComplete();
    }

    public ModeGoForward(RobotState robotState, DriveSystem driveSystem) {
        taskList = new AluminatiAutoTaskList();
        PathContainer pathContainer = new PathGoForward();

        taskList.add(new ActionResetRobotPose(robotState, driveSystem, pathContainer.getStartPose()));
        taskList.add(new ActionRunPath(driveSystem, pathContainer));
    }
}