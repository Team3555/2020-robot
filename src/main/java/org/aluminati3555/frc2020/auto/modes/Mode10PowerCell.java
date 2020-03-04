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
import org.aluminati3555.frc2020.auto.actions.ActionExtendIntake;
import org.aluminati3555.frc2020.auto.actions.ActionHomeHood;
import org.aluminati3555.frc2020.auto.actions.ActionOnPathMarkerPassed;
import org.aluminati3555.frc2020.auto.actions.ActionResetRobotPose;
import org.aluminati3555.frc2020.auto.actions.ActionRetractIntake;
import org.aluminati3555.frc2020.auto.actions.ActionRunPath;
import org.aluminati3555.frc2020.auto.actions.ActionSetHoodPosition;
import org.aluminati3555.frc2020.auto.actions.ActionSetIntakeSpeed;
import org.aluminati3555.frc2020.auto.actions.ActionSetLimelightLEDMode;
import org.aluminati3555.frc2020.auto.actions.ActionSetLimelightPipeline;
import org.aluminati3555.frc2020.auto.actions.ActionShootPowerCell;
import org.aluminati3555.frc2020.paths.Path10PowerCell1;
import org.aluminati3555.frc2020.paths.Path10PowerCell2;
import org.aluminati3555.frc2020.paths.Path10PowerCell3;
import org.aluminati3555.frc2020.paths.Path10PowerCell4;
import org.aluminati3555.frc2020.paths.Path10PowerCell5;
import org.aluminati3555.frc2020.paths.Path10PowerCell6;
import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.frc2020.systems.MagazineSystem;
import org.aluminati3555.frc2020.systems.IntakeSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem.HoodPosition;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.auto.AluminatiAutoTaskList;
import org.aluminati3555.lib.auto.AluminatiParallelAutoTask;
import org.aluminati3555.lib.trajectoryfollowingmotion.PathContainer;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;
import org.aluminati3555.lib.vision.AluminatiLimelight;
import org.aluminati3555.lib.vision.AluminatiLimelight.LEDMode;

/**
 * This auto mode scores 10 power cells
 * 
 * @author Caleb Heydon
 */
public class Mode10PowerCell implements AluminatiAutoTask {
    private AluminatiAutoTaskList taskList;

    @Override
    public String toString() {
        return "10PowerCell";
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

    public Mode10PowerCell(RobotState robotState, AluminatiLimelight limelight,
            DriveSystem driveSystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem,
            MagazineSystem magazineSystem) {
        taskList = new AluminatiAutoTaskList();

        PathContainer path1 = new Path10PowerCell1();
        PathContainer path2 = new Path10PowerCell2();
        PathContainer path3 = new Path10PowerCell3();
        PathContainer path4 = new Path10PowerCell4();
        PathContainer path5 = new Path10PowerCell5();
        PathContainer path6 = new Path10PowerCell6();

        // Set robot position
        taskList.add(new ActionResetRobotPose(robotState, driveSystem, path1.getStartPose()));
        
        // Home hood and lower it
        taskList.add(new ActionHomeHood(shooterSystem));
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.DOWN));
        
        // Actuate intake and spin motors
        taskList.add(new ActionExtendIntake(intakeSystem));
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 1));
        
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
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.UP));
        taskList.add(new ActionAlignWithVision(driveSystem, limelight));
        taskList.add(new ActionShootPowerCell(limelight, shooterSystem, magazineSystem, 5));
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.DOWN));

        // Set the limelight to the driver pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 0));

        // Reset the limelight leds
        taskList.add(new ActionSetLimelightLEDMode(limelight, LEDMode.CURRENT_PIPELINE));

        // Go get two more power cells from generator switch
        taskList.add(
                new AluminatiParallelAutoTask(new ActionRunPath(driveSystem, path3),
                        new ActionOnPathMarkerPassed(driveSystem, "Path10PowerCell3A",
                                new AluminatiParallelAutoTask(new ActionExtendIntake(intakeSystem),
                                        new ActionSetIntakeSpeed(intakeSystem, 1)))));
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 0));
        taskList.add(new ActionRetractIntake(intakeSystem));

        // Go get three more power cells from trench run
        taskList.add(new ActionRunPath(driveSystem, path4));

        taskList.add(new ActionExtendIntake(intakeSystem));
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 1));

        taskList.add(new ActionRunPath(driveSystem, path5));

        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 0));
        taskList.add(new ActionRetractIntake(intakeSystem));

        // Go back and shoot five power cells
        taskList.add(new ActionRunPath(driveSystem, path6));

        // Set the limelight to the tracking pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 1));

        // Shoot five power cells
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.UP));
        taskList.add(new ActionAlignWithVision(driveSystem, limelight));
        taskList.add(new ActionShootPowerCell(limelight, shooterSystem, magazineSystem, 5));
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.DOWN));

        // Set the limelight to the driver pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 0));
    }
}
