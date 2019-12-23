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

/**
 * License for original code:
 * 
 * Copyright (c) 2018 Team 254
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

package org.aluminati3555.lib.drive;

/**
 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that
 * the "turning" stick controls the curvature of the robot's path rather than
 * its rate of heading change. This helps make the robot more controllable at
 * high speeds. Also handles the robot's quick turn functionality - "quick turn"
 * overrides constant-curvature turning for turn-in-place maneuvers.
 */
public class AluminatiDriveHelper {
    // Constants
    private static final double THROTTLE_DEADBAND = 0.02;
    private static final double WHEEL_DEADBAND = 0.02;

    private static final double HIGH_NEG_INERTIA_SCALAR = 4.0;

    private static final double LOW_NEG_INERTIA_THRESHOLD = 0.65;
    private static final double LOW_NEG_INERTIA_TURN_SCALAR = 3.5;
    private static final double LOW_NEG_INERTIA_CLOSE_SCALAR = 4.0;
    private static final double LOW_NEG_INERTIA_FAR_SCALAR = 5.0;

    private static final double HIGH_SENSITIVITY = 0.65;
    private static final double LOW_SENSITIVITY = 0.65;

    private static final double quickStopDeadband = 0.5;
    private static final double quickStopWeight = 0.1;
    private static final double quickStopScalar = 5.0;

    // Data
    private double mOldWheel;
    private double mQuickStopAccumlator;
    private double mNegInertiaAccumlator;

    // Output
    private double left;
    private double right;

    /**
     * Returns the drivetrain's left power
     * 
     * @return Left power
     */
    public double getLeftPower() {
        return left;
    }

    /**
     * Returns the drivetrain's right power
     * 
     * @return Right power
     */
    public double getRightPower() {
        return right;
    }

    /**
     * Returns a useful string
     */
    @Override
    public String toString() {
        return "[DriveHelper]";
    }

    /**
     * Call this method to update the drive outputs
     * 
     * @param throttle    The forward and backward power
     * @param wheel       The turn
     * @param isQuickTurn Use this for turning in place and for arcade drive
     * @param isHighGear  Set this to true for single speed
     */
    public void aluminatiDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {
        wheel = handleDeadband(wheel, WHEEL_DEADBAND);
        throttle = handleDeadband(throttle, THROTTLE_DEADBAND);

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = HIGH_NEG_INERTIA_SCALAR;
            sensitivity = HIGH_SENSITIVITY;
        } else {
            if (wheel * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = LOW_NEG_INERTIA_TURN_SCALAR;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(wheel) > LOW_NEG_INERTIA_THRESHOLD) {
                    negInertiaScalar = LOW_NEG_INERTIA_FAR_SCALAR;
                } else {
                    negInertiaScalar = LOW_NEG_INERTIA_CLOSE_SCALAR;
                }
            }
            sensitivity = LOW_SENSITIVITY;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle;

        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < quickStopDeadband) {
                double alpha = quickStopWeight;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator + alpha * limit(wheel, 1.0) * quickStopScalar;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }

        this.right = rightPwm;
        this.left = leftPwm;
    }

    private double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    private double limit(double v, double limit) {
        return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
    }
}
