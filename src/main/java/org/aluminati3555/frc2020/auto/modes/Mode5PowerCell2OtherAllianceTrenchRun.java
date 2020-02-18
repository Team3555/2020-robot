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

import org.aluminati3555.frc2020.auto.actions.ActionAlignWithVision;
import org.aluminati3555.frc2020.auto.actions.ActionExtendHood;
import org.aluminati3555.frc2020.auto.actions.ActionExtendIntake;
import org.aluminati3555.frc2020.auto.actions.ActionResetRobotPose;
import org.aluminati3555.frc2020.auto.actions.ActionRetractHood;
import org.aluminati3555.frc2020.auto.actions.ActionRetractIntake;
import org.aluminati3555.frc2020.auto.actions.ActionRunPath;
import org.aluminati3555.frc2020.auto.actions.ActionSetIntakeSpeed;
import org.aluminati3555.frc2020.auto.actions.ActionSetLimelightLEDMode;
import org.aluminati3555.frc2020.auto.actions.ActionSetLimelightPipeline;
import org.aluminati3555.frc2020.auto.actions.ActionShootPowerCell;
import org.aluminati3555.frc2020.paths.Path5PowerCell2OtherAllianceTrenchRun1;
import org.aluminati3555.frc2020.paths.Path5PowerCell2OtherAllianceTrenchRun2;
import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.frc2020.systems.MagazineSystem;
import org.aluminati3555.frc2020.systems.IntakeSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.auto.AluminatiAutoTaskList;
import org.aluminati3555.lib.trajectoryfollowingmotion.PathContainer;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;
import org.aluminati3555.lib.vision.AluminatiLimelight;
import org.aluminati3555.lib.vision.AluminatiLimelight.LEDMode;

/**
 * This auto mode steals two power cells from the opposing alliance and shoots
 * five into the high goal
 * 
 * @author Caleb Heydon
 */
public class Mode5PowerCell2OtherAllianceTrenchRun implements AluminatiAutoTask {
    private AluminatiAutoTaskList taskList;

    @Override
    public String toString() {
        return "5PowerCell2OtherAllianceTrenchRun";
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

    public Mode5PowerCell2OtherAllianceTrenchRun(RobotState robotState, AluminatiLimelight limelight,
            DriveSystem driveSystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem,
            MagazineSystem magazineSystem) {
        taskList = new AluminatiAutoTaskList();

        PathContainer path1 = new Path5PowerCell2OtherAllianceTrenchRun1();
        PathContainer path2 = new Path5PowerCell2OtherAllianceTrenchRun2();

        // Set robot position
        taskList.add(new ActionResetRobotPose(robotState, driveSystem, path1.getStartPose()));
        
        // Turn on the limelight leds
        taskList.add(new ActionSetLimelightLEDMode(limelight, LEDMode.ON));
        
        // Retract hood if it is extended
        taskList.add(new ActionRetractHood(shooterSystem));
        
        // Actuate intake and spin motors
        taskList.add(new ActionExtendIntake(intakeSystem));
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 0.5));
        
        // Run path
        taskList.add(new ActionRunPath(driveSystem, path1));

        // Stop intake and retract it
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 0));
        taskList.add(new ActionRetractIntake(intakeSystem));

        // Run path two
        taskList.add(new ActionRunPath(driveSystem, path2));

        // Set the limelight to the tracking pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 1));

        // Shoot five power cells
        taskList.add(new ActionExtendHood(shooterSystem));
        taskList.add(new ActionAlignWithVision(driveSystem, limelight));
        taskList.add(new ActionShootPowerCell(limelight, shooterSystem, magazineSystem, 5));
        taskList.add(new ActionRetractHood(shooterSystem));

        // Set the limelight to the driver pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 0));

        // Reset the limelight leds
        taskList.add(new ActionSetLimelightLEDMode(limelight, LEDMode.CURRENT_PIPELINE));
    }
}
