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

import com.team254.lib.geometry.Rotation2d;

import org.aluminati3555.frc2020.auto.actions.ActionAlignWithVision;
import org.aluminati3555.frc2020.auto.actions.ActionExtendHood;
import org.aluminati3555.frc2020.auto.actions.ActionExtendIntake;
import org.aluminati3555.frc2020.auto.actions.ActionOnPathMarkerPassed;
import org.aluminati3555.frc2020.auto.actions.ActionResetRobotPose;
import org.aluminati3555.frc2020.auto.actions.ActionRetractHood;
import org.aluminati3555.frc2020.auto.actions.ActionRetractIntake;
import org.aluminati3555.frc2020.auto.actions.ActionRunPath;
import org.aluminati3555.frc2020.auto.actions.ActionSetIntakeSpeed;
import org.aluminati3555.frc2020.auto.actions.ActionSetLimelightLEDMode;
import org.aluminati3555.frc2020.auto.actions.ActionSetLimelightPipeline;
import org.aluminati3555.frc2020.auto.actions.ActionShootPowerCell;
import org.aluminati3555.frc2020.auto.actions.ActionTurnToHeading;
import org.aluminati3555.frc2020.paths.Path8PowerCell5TrenchRun1;
import org.aluminati3555.frc2020.paths.Path8PowerCell5TrenchRun2;
import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.frc2020.systems.MagazineSystem;
import org.aluminati3555.frc2020.systems.IntakeSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.auto.AluminatiAutoTaskList;
import org.aluminati3555.lib.auto.AluminatiParallelAutoTask;
import org.aluminati3555.lib.trajectoryfollowingmotion.PathContainer;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;
import org.aluminati3555.lib.vision.AluminatiLimelight;
import org.aluminati3555.lib.vision.AluminatiLimelight.LEDMode;

/**
 * This mode shoots three power cells into the high goal, gets five more from
 * the trench run, and then shoots them
 * 
 * @author Caleb Heydon
 */
public class Mode8PowerCell5TrenchRun implements AluminatiAutoTask {
    private static final Rotation2d TARGET_ZONE_HEADING = Rotation2d.fromDegrees(30);

    private AluminatiAutoTaskList taskList;

    @Override
    public String toString() {
        return "8PowerCell5TrenchRun";
    }

    public void start(final double timestamp) {
        taskList.start(timestamp);
    }

    public void update(final double timestamp) {
        taskList.update(timestamp);
    }

    public void stop() {
        taskList.stop();
    }

    public boolean isComplete() {
        return taskList.isComplete();
    }

    public Mode8PowerCell5TrenchRun(final RobotState robotState, final AluminatiLimelight limelight, final DriveSystem driveSystem,
            final IntakeSystem intakeSystem, final ShooterSystem shooterSystem, final MagazineSystem magazineSystem) {
        taskList = new AluminatiAutoTaskList();

        final PathContainer path1 = new Path8PowerCell5TrenchRun1();
        final PathContainer path2 = new Path8PowerCell5TrenchRun2();

        // Set robot position
        taskList.add(new ActionResetRobotPose(robotState, driveSystem, path1.getStartPose()));

        // Turn on the limelight leds
        taskList.add(new ActionSetLimelightLEDMode(limelight, LEDMode.ON));

        // Set the limelight to the tracking pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 1));

        // Shoot three power cells
        taskList.add(new ActionExtendHood(shooterSystem));
        taskList.add(new ActionTurnToHeading(driveSystem, TARGET_ZONE_HEADING));
        taskList.add(new ActionAlignWithVision(driveSystem, limelight));
        taskList.add(new ActionShootPowerCell(limelight, shooterSystem, magazineSystem, 3));
        taskList.add(new ActionRetractHood(shooterSystem));

        // Set the limelight to the driver pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 0));

        // Go get five more power cells
        taskList.add(new ActionTurnToHeading(driveSystem, Rotation2d.fromDegrees(0)));
        taskList.add(
                new AluminatiParallelAutoTask(new ActionRunPath(driveSystem, path1),
                        new ActionOnPathMarkerPassed(driveSystem, "Path8PowerCell5TrenchRun1A",
                                new AluminatiParallelAutoTask(new ActionExtendIntake(intakeSystem),
                                        new ActionSetIntakeSpeed(intakeSystem, 0.5)))));
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 0));
        taskList.add(new ActionRetractIntake(intakeSystem));

        // Go back to the initiation line
        taskList.add(new ActionRunPath(driveSystem, path2));

        // Set the limelight to the tracking pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 1));

        // Shoot five power cells
        taskList.add(new ActionExtendHood(shooterSystem));
        taskList.add(new ActionTurnToHeading(driveSystem, TARGET_ZONE_HEADING));
        taskList.add(new ActionAlignWithVision(driveSystem, limelight));
        taskList.add(new ActionShootPowerCell(limelight, shooterSystem, magazineSystem, 5));
        taskList.add(new ActionRetractHood(shooterSystem));

        // Set the limelight to the driver pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 0));

        // Reset the limelight leds
        taskList.add(new ActionSetLimelightLEDMode(limelight, LEDMode.CURRENT_PIPELINE));
    }
}
