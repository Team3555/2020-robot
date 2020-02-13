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

import org.aluminati3555.frc2020.RobotFaults;
import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.drivers.AluminatiXboxController;
import org.aluminati3555.lib.pneumatics.AluminatiSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

/**
 * This class controls the intake for the power cells
 * 
 * @author Caleb Heydon
 */
public class IntakeSystem implements AluminatiSystem {
    private AluminatiTalonSRX intakeMotor;
    private AluminatiSolenoid extenderSolenoid;

    private AluminatiXboxController operatorController;

    private MagazineSystem magazineSystem;

    private RobotFaults robotFaults;

    private double speed;

    /**
     * Returns true if the intake is down
     */
    public boolean isDown() {
        return extenderSolenoid.get();
    }

    /**
     * Returns true if the intake is up
     */
    public boolean isUp() {
        return !extenderSolenoid.get();
    }

    /**
     * Extends the intake
     */
    public void extend() {
        extenderSolenoid.enable();
    }

    /**
     * Retracts the intake
     */
    public void retract() {
        extenderSolenoid.disable();
    }

    /**
     * Sets the speed of the intake
     */
    public void setSpeed(double speed) {
        this.speed = speed;
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
            robotFaults.setIntakeFault(true);
        }

        if (enabled) {
            // Control intake position
            if (operatorController.getRawButtonPressed(5)) {
                extend();
            }
            
            if (operatorController.getRawButtonReleased(5)) {
                retract();
            }

            // Control intake speed
            if (operatorController.getRawButtonPressed(6)) {
                setSpeed(0.5);

                // Run the magazine as well
                magazineSystem.startFeedingPowerCells();
            }
            
            if (operatorController.getRawButtonReleased(6)) {
                setSpeed(0);
                magazineSystem.stopFeedingPowerCells();
            }

            // Check for button to eject power cells
            if (operatorController.getRawButtonPressed(3)) {
                extend();
                setSpeed(-0.5);
                magazineSystem.startEjectingPowerCells();
            }

            if (operatorController.getRawButtonReleased(3)) {
                magazineSystem.stopEjectingPowerCells();
                setSpeed(0);
                retract();
            }
        }

        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public IntakeSystem(AluminatiTalonSRX intakeMotor, AluminatiSolenoid extenderSolenoid,
            AluminatiXboxController operatorController, MagazineSystem magazineSystem, RobotFaults robotFaults) {
        this.intakeMotor = intakeMotor;
        this.extenderSolenoid = extenderSolenoid;

        this.operatorController = operatorController;

        this.magazineSystem = magazineSystem;

        this.robotFaults = robotFaults;
    }
}
