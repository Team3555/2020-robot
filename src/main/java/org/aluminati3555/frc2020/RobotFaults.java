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

package org.aluminati3555.frc2020;

/**
 * This class keeps track of the robot faults in each system
 * 
 * @author Caleb Heydon
 */
public class RobotFaults {
    private boolean climberFault = false;
    private boolean driveFault = false;
    private boolean intakeFault = false;
    private boolean magazineFault = false;
    private boolean shooterFault = false;
    private boolean spinnerFault = false;

    /**
     * Returns true if there is a fault in the climber
     */
    public synchronized boolean getClimberFault() {
        return climberFault;
    }

    /**
     * Returns true if there is a fault in the drive
     */
    public synchronized boolean getDriveFault() {
        return driveFault;
    }

    /**
     * Returns true if there is a fault in the intake
     */
    public synchronized boolean getIntakeFault() {
        return intakeFault;
    }

    /**
     * Returns true if there is a fault in the magazine
     */
    public synchronized boolean getMagazineFault() {
        return magazineFault;
    }

    /**
     * Returns true if there is a fault in the shooter
     */
    public synchronized boolean getShooterFault() {
        return shooterFault;
    }

    /**
     * Returns true if there is a fault in the spinner
     */
    public synchronized boolean getSpinnerFault() {
        return spinnerFault;
    }

    /**
     * Sets the climber fault
     */
    public synchronized void setClimberFault(boolean fault) {
        climberFault = fault;
    }

    /**
     * Sets the drive fault
     */
    public synchronized void setDriveFault(boolean fault) {
        driveFault = fault;
    }

    /**
     * Sets the intake fault
     */
    public synchronized void setIntakeFault(boolean fault) {
        intakeFault = fault;
    }

    /**
     * Sets the magazine fault
     */
    public synchronized void setMagazineFault(boolean fault) {
        magazineFault = fault;
    }

    /**
     * Sets the shooter fault
     */
    public synchronized void setShooterFault(boolean fault) {
        shooterFault = fault;
    }

    /**
     * Sets the spinner fault
     */
    public synchronized void setSpinnerFault(boolean fault) {
        spinnerFault = fault;
    }
}
