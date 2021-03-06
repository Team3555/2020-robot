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

package org.aluminati3555.frc2020.auto.actions;

import com.team254.lib.geometry.Rotation2d;

import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.pid.AluminatiPIDController;

/**
 * This action turns the robot to a heading
 * 
 * @author Caleb Heydon
 */
public class ActionTurnToHeading implements AluminatiAutoTask {
    private static AluminatiPIDController controller;

    /**
     * Initializes the PID controller
     */
    public static final void initialize() {
        controller = new AluminatiPIDController(0.05, 0, 0, 400, 1, 0.6, 0);
    }

    private DriveSystem driveSystem;
    private Rotation2d setpoint;

    public void start(double timestamp) {
        controller.reset(timestamp);
    }

    public void update(double timestamp) {
        double output = -controller.update(setpoint.getDegrees(), driveSystem.getGyro().getHeading().getDegrees(),
                timestamp);

        driveSystem.manualArcadeDrive(output, 0);
    }

    public void stop() {
        driveSystem.manualArcadeDrive(0, 0);
    }

    public boolean isComplete() {
        return controller.isComplete();
    }

    public ActionTurnToHeading(DriveSystem driveSystem, Rotation2d setpoint) {
        this.driveSystem = driveSystem;
        this.setpoint = setpoint;
    }
}
