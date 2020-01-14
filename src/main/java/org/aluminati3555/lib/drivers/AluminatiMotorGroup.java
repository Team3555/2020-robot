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

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * This class automatically makes all other motors follow the master
 * 
 * @author Caleb Heydon
 */
public class AluminatiMotorGroup implements AluminatiCriticalDevice {
    // Motors
    private AluminatiMotorController master;
    private AluminatiMotorController[] motors;

    // Inverted
    private boolean inverted;

    /**
     * Returns the master talon srx
     */
    public AluminatiTalonSRX getMasterTalon() {
        if (!(master instanceof AluminatiTalonSRX)) {
            return null;
        }

        return (AluminatiTalonSRX) master;
    }

    /**
     * Returns the master spark max
     */
    public AluminatiSparkMax getMasterSparkMax() {
        if (!(master instanceof AluminatiSparkMax)) {
            return null;
        }

        return (AluminatiSparkMax) master;
    }

    /**
     * Returns the master motor controller
     */
    public AluminatiMotorController getMaster() {
        return master;
    }

    /**
     * Returns all motors
     * 
     * @return
     */
    public AluminatiMotorController[] getMotors() {
        return motors;
    }

    /**
     * Returns true if the drive is inverted
     * 
     * @return
     */
    public boolean getInverted() {
        return inverted;
    }

    /**
     * Sets the drive as inverted
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;

        if (inverted) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setInverted(true);
            }
        } else {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setInverted(false);
            }
        }
    }

    /**
     * Returns a useful string
     */
    @Override
    public String toString() {
        String output = "[MotorGroup";
        for (int i = 0; i < motors.length; i++) {
            output += ":" + motors[i].getDeviceID();
        }
        output += "]";

        return output;
    }

    /**
     * Returns true if the encoder on the master talon is ok
     */
    public boolean isEncoderOK() {
        if (master instanceof AluminatiTalonSRX) {
            return ((AluminatiTalonSRX) master).isEncoderOK();
        }

        return true;
    }

    /**
     * Returns true if this side of the drive is ok
     */
    public boolean isOK() {
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] instanceof AluminatiTalonSRX && !((AluminatiTalonSRX) motors[i]).isOK()) {
                return false;
            }
        }

        return true;
    }

    /**
     * Puts the group into coast mode
     */
    public void coast() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setNeutralMode(NeutralMode.Coast);
        }
    }

    /**
     * Puts the group into brake mode
     */
    public void brake() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setNeutralMode(NeutralMode.Brake);
        }
    }

    /**
     * Makes the follower motors follow the master
     * 
     * @param master
     * @param followers
     */
    public AluminatiMotorGroup(AluminatiMotorController master, AluminatiMotorController... followers) {
        this.master = master;

        motors = new AluminatiMotorController[followers.length + 1];
        motors[0] = master;

        for (int i = 1; i < motors.length; i++) {
            motors[i] = followers[i - 1];

            if (master instanceof AluminatiSparkMax) {
                // Use different method for REV
                followers[i - 1].follow((AluminatiSparkMax) master);
                continue;
            } else if (!(master instanceof IMotorController)) {
                // Skip over unknown motor controllers
                continue;
            }

            followers[i - 1].follow((IMotorController) master);
        }
    }

    /**
     * Allows all of the drive motors to be inverted
     */
    public AluminatiMotorGroup(boolean inverted, AluminatiMotorController master, AluminatiMotorController... followers) {
        this(master, followers);

        setInverted(inverted);
    }
}
