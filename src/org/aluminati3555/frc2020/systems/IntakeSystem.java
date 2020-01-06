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

package org.aluminati3555.frc2020.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.aluminati3555.lib.drivers.AluminatiVictorSPX;
import org.aluminati3555.lib.pneumatics.AluminatiDoubleSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class controls the intake for the power cells
 * 
 * @author Caleb Heydon
 */
public class IntakeSystem implements AluminatiSystem {
    private AluminatiVictorSPX intakeMotor;
    private AluminatiDoubleSolenoid extenderSolenoid;

    /**
     * Extends the intake
     */
    public void extend() {
        extenderSolenoid.forward();
    }

    /**
     * Retracts the intake
     */
    public void retract() {
        extenderSolenoid.reverse();
    }

    /**
     * Sets the speed of the intake
     */
    public void setSpeed(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the motor to 0 and retracts the intake
     */
    public void reset() {
        setSpeed(0);
        retract();
    }

    public void update(double timestamp, boolean enabled) {
        if (!intakeMotor.isOK()) {
            DriverStation.reportError("Fault detected in intake", false);
        }

        if (enabled) {
            // Add manual controls here
        }
    }

    public IntakeSystem(AluminatiVictorSPX intakeMotor, AluminatiDoubleSolenoid extenderSolenoid) {
        this.intakeMotor = intakeMotor;
        this.extenderSolenoid = extenderSolenoid;
    }
}
