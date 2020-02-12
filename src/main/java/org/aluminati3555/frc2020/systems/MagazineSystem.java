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
import org.aluminati3555.lib.drivers.AluminatiVictorSPX;
import org.aluminati3555.lib.system.AluminatiSystem;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class controls the mechanism that feeds the power cells to the shooter
 * 
 * @author Caleb Heydon
 */
public class MagazineSystem implements AluminatiSystem {
    private static final int ENCODER_TICKS_PER_ROTATION = 4096;

    private AluminatiVictorSPX motor;
    private AluminatiTalonSRX feederMotor;
    private DigitalInput photoelectricSensor;

    private RobotFaults robotFaults;

    /**
     * Feeds a specified number of power cells to the shooter
     */
    public void feedPowerCell(int numberOfPowerCells) {

    }

    /**
     * Starts continuously feeding power cells to the shooter
     */
    public void startFeedingPowerCells() {

    }

    /**
     * Stops continously feeding power cells to the shooter
     */
    public void stopFeedingPowerCells() {

    }

    public void update(double timestamp, boolean enabled) {
        if (photoelectricSensor.get()) {
            motor.set(ControlMode.PercentOutput, 0.5);
        }
    }

    public MagazineSystem(AluminatiVictorSPX motor, AluminatiTalonSRX feederMotor, DigitalInput photoelectricSensor,
            RobotFaults robotFaults) {
        this.motor = motor;
        this.feederMotor = feederMotor;
        this.photoelectricSensor = photoelectricSensor;

        this.robotFaults = robotFaults;
    }
}
