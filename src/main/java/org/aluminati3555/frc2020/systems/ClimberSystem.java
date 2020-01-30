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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.net.AluminatiTunable;
import org.aluminati3555.lib.pneumatics.AluminatiSolenoid;
import org.aluminati3555.lib.system.AluminatiSystem;

/**
 * This class controls climber
 * 
 * @author Caleb Heydon
 */
public class ClimberSystem implements AluminatiSystem {
    private static final int ARM_CURRENT_LIMIT = 30;
    private static final int SPOOL_CURRENT_LIMIT = 30;

    private AluminatiTalonSRX armMotor;
    private AluminatiTalonSRX spoolMotor;
    private AluminatiSolenoid ratchetSolenoid;

    /**
     * Locks the ratchet
     */
    public void lockRatchet() {
        ratchetSolenoid.enable();
    }

    /**
     * Unlocks the ratchet
     */
    public void unlockRatchet() {
        ratchetSolenoid.disable();
    }

    public void update(double timestamp, boolean enabled) {

    }

    public ClimberSystem(AluminatiTalonSRX armMotor, AluminatiTalonSRX spoolMotor, AluminatiSolenoid ratchetSolenoid) {
        this.armMotor = armMotor;
        this.spoolMotor = spoolMotor;
        this.ratchetSolenoid = ratchetSolenoid;

        // Configure encoder
        this.armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Configure PID
        this.armMotor.config_kP(0, 0.1);
        this.armMotor.config_kI(0, 0);
        this.armMotor.config_kD(0, 0);
        this.armMotor.config_IntegralZone(0, 400);

        // Setup tuning listener
        new AluminatiTunable(5808) {
            protected void update(TuningData data) {
                armMotor.config_kP(0, data.kP);
                armMotor.config_kI(0, data.kI);
                armMotor.config_kD(0, data.kD);
            }
        };

        // Configure current limit
        this.armMotor.configPeakCurrentDuration(500);
        this.armMotor.configPeakCurrentLimit(ARM_CURRENT_LIMIT);
        this.armMotor.configContinuousCurrentLimit(ARM_CURRENT_LIMIT);
        this.armMotor.enableCurrentLimit(true);

        // Configure brake mode
        this.armMotor.setNeutralMode(NeutralMode.Brake);

        // Configure current limit for spool motor
        this.spoolMotor.configPeakCurrentDuration(500);
        this.spoolMotor.configPeakCurrentLimit(SPOOL_CURRENT_LIMIT);
        this.spoolMotor.configContinuousCurrentLimit(SPOOL_CURRENT_LIMIT);
        this.spoolMotor.enableCurrentLimit(true);

        // Configure brake mode
        this.spoolMotor.setNeutralMode(NeutralMode.Brake);
    }
}
