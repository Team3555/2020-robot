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

import org.aluminati3555.frc2020.auto.actions.ActionExtendIntake;
import org.aluminati3555.frc2020.auto.actions.ActionRunCode;
import org.aluminati3555.frc2020.auto.actions.ActionSetHoodPosition;
import org.aluminati3555.frc2020.auto.actions.ActionSetIntakeSpeed;
import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.frc2020.systems.MagazineSystem;
import org.aluminati3555.frc2020.systems.IntakeSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem.HoodPosition;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.auto.AluminatiAutoTaskList;
import org.aluminati3555.lib.net.AluminatiTuneable;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;
import org.aluminati3555.lib.vision.AluminatiLimelight;

/**
 * This auto mode helps with tuning the long shot
 * 
 * @author Caleb Heydon
 */
public class ModeTuneLongShot implements AluminatiAutoTask {
    private static final Object LOCK = new Object();
    private static final double DEFAULT_SETPOINT = 4800;

    private AluminatiAutoTaskList taskList;

    private ShooterSystem shooterSystem;
    private MagazineSystem magazineSystem;

    private double setpoint;
    private boolean runShooter;

    @Override
    public String toString() {
        return "TuneLongShot";
    }

    public void start(double timestamp) {
        taskList.start(timestamp);

        synchronized (LOCK) {
            setpoint = DEFAULT_SETPOINT;
            runShooter = false;
        }
    }

    public void update(double timestamp) {
        taskList.update(timestamp);

        synchronized (LOCK) {
            if (runShooter) {
                shooterSystem.set(setpoint);
                magazineSystem.startFeedingPowerCells();
            }
        }
    }

    public void stop() {
        taskList.stop();
        shooterSystem.stop();
        magazineSystem.stopFeedingPowerCells();
    }

    public boolean isComplete() {
        return taskList.isComplete();
    }

    public ModeTuneLongShot(RobotState robotState, AluminatiLimelight limelight, DriveSystem driveSystem,
            IntakeSystem intakeSystem, ShooterSystem shooterSystem, MagazineSystem magazineSystem) {
        taskList = new AluminatiAutoTaskList();

        this.shooterSystem = shooterSystem;
        this.magazineSystem = magazineSystem;
        setpoint = DEFAULT_SETPOINT;
        runShooter = false;

        new AluminatiTuneable(5805) {
            protected void update(TuningData data) {
                synchronized (LOCK) {
                    setpoint = data.kP;
                }
            }
        };

        taskList.add(new ActionSetHoodPosition(shooterSystem, HoodPosition.UP));

        // Actuate intake and spin motors
        taskList.add(new ActionExtendIntake(intakeSystem));
        taskList.add(new ActionSetIntakeSpeed(intakeSystem, 1));

        // Signal to start flywheel
        taskList.add(new ActionRunCode(() -> runShooter = true));

        // Never exit
        taskList.add(new ModeDoNothing());
    }
}
