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

package org.aluminati3555.frc2020.auto.modes;

import java.util.ArrayList;

import com.team254.lib.physics.DriveCharacterization;
import com.team254.lib.physics.DriveCharacterization.CharacterizationConstants;
import com.team254.lib.physics.DriveCharacterization.DataPoint;

import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This auto mode characterizes the drivetrain
 * 
 * @author Caleb Heydon
 */
public class ModeCharacterizeDrive implements AluminatiAutoTask {
    private DriveSystem driveSystem;
    private ArrayList<DataPoint> points;

    private boolean done;
    private double startTime;
    private double power;

    @Override
    public String toString() {
        return "CharacterizeDrive";
    }

    public void start(double timestamp) {
        done = false;
        startTime = timestamp;
        power = 0;
    }

    public void update(double timestamp) {
        if (power < 1) {
            power += 0.005;
        }
        driveSystem.manualArcadeDrive(0, power);

        points.add(new DataPoint(
                (driveSystem.getLeftVelocityInchesPerSecond() + driveSystem.getRightVelocityInchesPerSecond()) / 2,
                power * 12, timestamp - startTime));

        if (power >= 1) {
            done = true;
        }
    }

    public void stop() {
        driveSystem.manualArcadeDrive(0, 0);

        CharacterizationConstants output = DriveCharacterization.characterizeDrive(points, points);
        
        SmartDashboard.putNumber("kV", output.kv);
        SmartDashboard.putNumber("kS", output.ks);

        System.out.println("kV: " + output.kv);
        System.out.println("kS: " + output.ks);
    }

    public boolean isComplete() {
        return done;
    }

    public ModeCharacterizeDrive(DriveSystem driveSystem) {
        this.driveSystem = driveSystem;

        this.points = new ArrayList<DataPoint>();
    }
}
