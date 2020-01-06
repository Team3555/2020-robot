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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.aluminati3555.lib.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team254.lib.geometry.Rotation2d;

import org.aluminati3555.lib.data.AluminatiData;

/**
 * This is a simple wrapper class for the pigeon gyro
 * 
 * @author Caleb Heydon
 */
public class AluminatiPigeon extends PigeonIMU implements AluminatiGyro {
    /**
     * Returns a helpful string
     */
    @Override
    public String toString() {
        return "[Pigeon:" + super.getDeviceID() + "]";
    }

    /**
     * Returns the pitch
     */
    public double getPitch() {
        double[] gyroData = new double[3];
        this.getYawPitchRoll(gyroData);
        return gyroData[1];
    }

    /**
     * Returns the roll
     */
    public double getRoll() {
        double[] gyroData = new double[3];
        this.getYawPitchRoll(gyroData);
        return gyroData[0];
    }

    /**
     * Returns true if the gyro is ok
     */
    public boolean isOK() {
        boolean ok = (this.getState() == PigeonState.Ready);
        return ok;
    }

    /**
     * Returns true if the robot is tipping over
     */
    public boolean isTipping() {
        double pitch = Math.abs(getPitch());
        double roll = Math.abs(getRoll());

        if (pitch >= AluminatiData.minTippingAngle || roll >= AluminatiData.minTippingAngle) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns the heading
     * 
     * @return
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(this.getFusedHeading());
    }

    /**
     * Sets the heading
     */
    public void setHeading(Rotation2d heading) {
        this.setFusedHeading(heading.getDegrees());
    }

    /**
     * Does some basic initialization
     */
    private void setupPigeon() {
        // Reset
        this.configFactoryDefault();

        setHeading(Rotation2d.fromDegrees(0));
    }

    /**
     * Use this for when it is attached to a TalonSRX
     * 
     * @param talon
     */
    public AluminatiPigeon(AluminatiTalonSRX talon) {
        super(talon);
        setupPigeon();
    }

    /**
     * For use with only a device number
     * 
     * @param canID
     */
    public AluminatiPigeon(int canID) {
        super(canID);
        setupPigeon();
    }
}
