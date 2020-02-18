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

package org.aluminati3555.lib.pid;

/**
 * A general purpose PID controller
 */
public class AluminatiPIDController {
    private static final double DEFAULT_IZONE = 400;

    @Override
    public String toString() {
        return "[PIDController]";
    }

    // PID values
    private double kP;
    private double kI;
    private double kD;

    private double iZone;

    private boolean allowableErrorEnabled;
    private double allowableError;

    private double maxOutput;

    private double integral;
    private double lastError;
    private double currentError;
    private double lastTimestamp;

    /**
     * Sets the PID constants
     */
    protected void setPID(double kP, double kI, double kD) {
        synchronized (this) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    /**
     * Initializes the pid controller
     * 
     * @param timestamp
     */
    public void reset(double timestamp) {
        integral = 0;
        lastError = 0;
        currentError = Double.POSITIVE_INFINITY;
        lastTimestamp = timestamp;
    }

    /**
     * Returns true if the PID target has been reached
     * 
     * @return
     */
    public boolean isComplete() {
        if (allowableErrorEnabled) {
            return (Math.abs(currentError) <= allowableError);
        } else {
            return (currentError == 0);
        }
    }

    /**
     * Updates the PID and returns the current output
     * 
     * @param setpoint
     * @param currentPoint
     * @param timestamp
     * @return
     */
    public double update(double setpoint, double currentPoint, double timestamp) {
        synchronized (this) {
            double error = setpoint - currentPoint;
            double dt = timestamp - lastTimestamp;
            lastTimestamp = timestamp;

            integral += error * dt;
            if (integral >= iZone) {
                integral = 0;
            }

            double p = kP * error;
            double i = kI * integral;
            double d = kD * (error - lastError) / dt;
            lastError = error;
            currentError = error;

            double output = p + i + d;
            if (output < -maxOutput) {
                output = -maxOutput;
            } else if (output > maxOutput) {
                output = maxOutput;
            }

            return output;
        }
    }

    public AluminatiPIDController(double kP, double kI, double kD, double iZone, double allowableError,
            double maxOutput, double timestamp) {
        setPID(kP, kI, kD);
        this.iZone = iZone;

        if (allowableError != 0) {
            this.allowableErrorEnabled = true;
            this.allowableError = allowableError;
        }

        this.maxOutput = maxOutput;

        reset(timestamp);
    }

    public AluminatiPIDController(double kP, double kI, double kD, double timestamp) {
        this(kP, kI, kD, DEFAULT_IZONE, 0, Double.POSITIVE_INFINITY, timestamp);
    }
}
