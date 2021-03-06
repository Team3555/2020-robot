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
import org.aluminati3555.frc2020.auto.actions.ActionExtendIntake;
import org.aluminati3555.frc2020.auto.actions.ActionOnPathMarkerPassed;
import org.aluminati3555.frc2020.auto.actions.ActionResetRobotPose;
import org.aluminati3555.frc2020.auto.actions.ActionRetractIntake;
import org.aluminati3555.frc2020.auto.actions.ActionRunPath;
import org.aluminati3555.frc2020.auto.actions.ActionSetHoodPosition;
import org.aluminati3555.frc2020.auto.actions.ActionSetIntakeSpeed;
import org.aluminati3555.frc2020.auto.actions.ActionSetLimelightPipeline;
import org.aluminati3555.frc2020.auto.actions.ActionShootPowerCell;
import org.aluminati3555.frc2020.auto.actions.ActionTurnToHeading;
import org.aluminati3555.frc2020.paths.Path5PowerCell2ShieldGenerator1;
import org.aluminati3555.frc2020.paths.Path5PowerCell2ShieldGenerator2;
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

/**
 * This mode gets two more power cells from the shield generator and then shoots
 * five
 * 
 * @author Caleb Heydon
 */
public class Mode5PowerCell2ShieldGenerator implements AluminatiAutoTask {
    private static final Rotation2d TARGET_ZONE_HEADING = Rotation2d.fromDegrees(30);

    private AluminatiAutoTaskList taskList;

    @Override
    public String toString() {
        return "5PowerCell2ShieldGenerator";
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

    public Mode5PowerCell2ShieldGenerator(final RobotState robotState, final AluminatiLimelight limelight, final DriveSystem driveSystem,
            final IntakeSystem intakeSystem, final ShooterSystem shooterSystem, final MagazineSystem magazineSystem) {
        taskList = new AluminatiAutoTaskList();

        final PathContainer path1 = new Path5PowerCell2ShieldGenerator1();
        final PathContainer path2 = new Path5PowerCell2ShieldGenerator2();

        // Set robot position
        taskList.add(new ActionResetRobotPose(robotState, driveSystem, path1.getStartPose()));

        // Lower hood
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.DOWN));

        // Go get two more power cells
        taskList.add(new ActionTurnToHeading(driveSystem, Rotation2d.fromDegrees(0)));
        taskList.add(
                new AluminatiParallelAutoTask(new ActionRunPath(driveSystem, path1),
                        new ActionOnPathMarkerPassed(driveSystem, "Path8PowerCell2ShieldGeneratorA",
                                new AluminatiParallelAutoTask(new ActionExtendIntake(intakeSystem),
                                        new ActionSetIntakeSpeed(intakeSystem, 1)))));
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 0));
        taskList.add(new ActionRetractIntake(intakeSystem));

        // Go back to the initiation line
        taskList.add(new ActionRunPath(driveSystem, path2));

        // Set the limelight to the tracking pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 1));

        // Shoot five power cells
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.UP));
        taskList.add(new ActionTurnToHeading(driveSystem, TARGET_ZONE_HEADING));
        taskList.add(new ActionAlignWithVision(driveSystem, limelight));
        taskList.add(new ActionShootPowerCell(limelight, shooterSystem, magazineSystem, 5));
        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.DOWN));

        // Set the limelight to the driver pipeline
        taskList.add(new ActionSetLimelightPipeline(limelight, 0));
    }
}
