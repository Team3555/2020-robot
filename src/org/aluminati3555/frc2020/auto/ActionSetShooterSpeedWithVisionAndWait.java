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

package org.aluminati3555.frc2020.auto;

import org.aluminati3555.frc2020.ShooterUtil;
import org.aluminati3555.frc2020.systems.ShooterSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.vision.AluminatiLimelight;

/**
 * This action sets the shooter rpm using the limelight
 * 
 * @author Caleb Heydon
 */
public class ActionSetShooterSpeedWithVisionAndWait implements AluminatiAutoTask {
    private ShooterSystem shooterSystem;
    private AluminatiLimelight limelight;

    private ActionWaitForShooterSpeed waitAction;

    public void start(double timestamp) {
        double targetHeight = limelight.getVertical();
        double rpm = ShooterUtil.calculateRPM(targetHeight);

        shooterSystem.set(rpm);
        waitAction = new ActionWaitForShooterSpeed(shooterSystem, rpm);
    }

    public void update(double timestamp) {

    }

    public void stop() {

    }

    public boolean isComplete() {
        if (waitAction == null) {
            return false;
        }

        return waitAction.isComplete();
    }

    public ActionSetShooterSpeedWithVisionAndWait(ShooterSystem shooterSystem, AluminatiLimelight limelight) {
        this.shooterSystem = shooterSystem;
        this.limelight = limelight;
    }
}
