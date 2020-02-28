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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.aluminati3555.frc2020.RobotFaults;
import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.drivers.AluminatiVictorSPX;
import org.aluminati3555.lib.net.AluminatiTuneable;
import org.aluminati3555.lib.system.AluminatiSystem;

/**
 * This class controls the mechanism that feeds the power cells to the shooter
 * 
 * @author Caleb Heydon
 */
public class MagazineSystem implements AluminatiSystem {
    private static final int ENCODER_TICKS_PER_ROTATION = 4096;
    private static final double FEED_BASE_TIME = 1;
    private static final double FEED_TIME = 0.5;
    private static final double FEEDER_RPM = 2000;

    private static final int FEEDER_CURRENT_LIMIT = 30;

    /**
     * Converts rpm to native units
     */
    private static int convertRPMToNativeUnits(double rpm) {
        return (int) (rpm * ENCODER_TICKS_PER_ROTATION / 600.0);
    }

    /**
     * Converts native units to rpm
     */
    private static int convertNativeUnitsToRPM(double nativeUnits) {
        return (int) (nativeUnits / ENCODER_TICKS_PER_ROTATION * 600.0);
    }

    private AluminatiVictorSPX motor;
    private AluminatiTalonSRX feederMotor;

    private RobotFaults robotFaults;

    private State state;

    private double stopTime;

    /**
     * Returns the output of the feeder motor as a percent between 0 and 1
     */
    public double getFeederOutputPercent() {
        return feederMotor.getMotorOutputPercent();
    }

    /**
     * Feeds a specified number of power cells to the shooter
     */
    public void feedPowerCell(int numberOfPowerCells, double timestamp) {
        stopTime = timestamp + FEED_BASE_TIME + numberOfPowerCells * FEED_TIME;
        state = State.TIMING;
    }

    /**
     * Starts intaking power cells
     */
    public void startIntakingPowerCells() {
        state = State.INTAKE;
    }

    /**
     * Stops intaking power cells
     */
    public void stopIntakingPowerCells() {
        if (state == State.INTAKE) {
            state = State.OFF;
        }
    }

    /**
     * Starts continuously feeding power cells to the shooter
     */
    public void startFeedingPowerCells() {
        state = State.CONTINUOUS_FORWARD;
    }

    /**
     * Stops continuously feeding power cells to the shooter
     */
    public void stopFeedingPowerCells() {
        if (state == State.CONTINUOUS_FORWARD) {
            state = State.OFF;
        }
    }

    /**
     * Starts continuously ejecting power cells
     */
    public void startEjectingPowerCells() {
        state = State.CONTINUOUS_REVERSE;
    }

    /**
     * Stops continuously ejecting power cells
     */
    public void stopEjectingPowerCells() {
        if (state == State.CONTINUOUS_REVERSE) {
            state = State.OFF;
        }
    }

    /**
     * Returns the velocity of the feeder wheel
     */
    public double getFeederVelocity() {
        return convertNativeUnitsToRPM(feederMotor.getSelectedSensorVelocity());
    }

    public void update(double timestamp, SystemMode mode) {
        // Check for errors
        if (!motor.isOK()) {
            robotFaults.setMagazineFault(true);
        }

        if (!feederMotor.isOK()) {
            robotFaults.setMagazineFault(true);
        }

        // Update states
        if (state == State.TIMING && timestamp >= stopTime) {
            state = State.OFF;
        }

        if (mode == SystemMode.OPERATOR_CONTROLLED || mode == SystemMode.AUTONOMOUS) {
            switch (state) {
                case INTAKE:
                    motor.set(ControlMode.PercentOutput, 1);
                    feederMotor.set(ControlMode.PercentOutput, 0);
                    break;
                case CONTINUOUS_FORWARD:
                    motor.set(ControlMode.PercentOutput, 1);
                    feederMotor.set(ControlMode.Velocity, convertRPMToNativeUnits(FEEDER_RPM));
                    break;
                case CONTINUOUS_REVERSE:
                    motor.set(ControlMode.PercentOutput, -1);
                    feederMotor.set(ControlMode.PercentOutput, -1);
                    break;
                case OFF:
                    motor.set(ControlMode.PercentOutput, 0);
                    feederMotor.set(ControlMode.PercentOutput, 0);
                    break;
                case TIMING:
                    motor.set(ControlMode.PercentOutput, 0.5);
                    feederMotor.set(ControlMode.Velocity, convertRPMToNativeUnits(FEEDER_RPM));
                    break;
                default:
                    motor.set(ControlMode.PercentOutput, 0);
                    feederMotor.set(ControlMode.PercentOutput, 0);
                    break;
            }
        }
    }

    public MagazineSystem(AluminatiVictorSPX motor, AluminatiTalonSRX feederMotor, RobotFaults robotFaults) {
        this.motor = motor;
        this.feederMotor = feederMotor;

        this.robotFaults = robotFaults;

        // Set brake mode
        this.motor.setNeutralMode(NeutralMode.Brake);

        // Configure encoder
        this.feederMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        this.feederMotor.setInverted(true);

        this.feederMotor.setNeutralMode(NeutralMode.Coast);

        // Configure PID
        this.feederMotor.config_kP(0, 0.05);
        this.feederMotor.config_kI(0, 0);
        this.feederMotor.config_kD(0, 0);
        this.feederMotor.config_kF(0, 0.08);
        this.feederMotor.config_IntegralZone(0, 400);

        // Setup tuning listener
        new AluminatiTuneable(5804) {
            protected void update(TuningData data) {
                feederMotor.config_kP(0, data.kP);
                feederMotor.config_kI(0, data.kI);
                feederMotor.config_kD(0, data.kD);
            }
        };

        // Configure current limit
        this.feederMotor.configPeakCurrentDuration(500);
        this.feederMotor.configPeakCurrentLimit(FEEDER_CURRENT_LIMIT);
        this.feederMotor.configContinuousCurrentLimit(FEEDER_CURRENT_LIMIT);
        this.feederMotor.enableCurrentLimit(true);

        // Set brake mode
        this.feederMotor.setNeutralMode(NeutralMode.Brake);

        state = State.OFF;
    }

    private enum State {
        INTAKE, CONTINUOUS_FORWARD, CONTINUOUS_REVERSE, OFF, TIMING
    }
}
