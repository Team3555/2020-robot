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

package org.aluminati3555.lib.drivers;

import com.team254.lib.geometry.Rotation2d;

import org.aluminati3555.lib.util.AluminatiUtil;

/**
 * This class allows for the precision of two gyros to be combined
 * 
 * @author Caleb Heydon
 */
public class AluminatiDualGyro implements AluminatiGyro {
    // Gyros
    private AluminatiGyro gyro1;
    private AluminatiGyro gyro2;

    @Override
    public String toString() {
        return "[DualGyro]";
    }

    /**
     * Returns true if both gyros are ok
     */
    public boolean isOK() {
        return (gyro1.isOK() && gyro2.isOK());
    }

    /**
     * Returns the best heading or an average
     */
    public Rotation2d getHeading() {
        if ((gyro1.isOK() && gyro2.isOK()) || (!gyro1.isOK() && !gyro2.isOK())) {
            // Return an average if both gyros are ok or if neither are ok

            return AluminatiUtil.averageHeadings(gyro1.getHeading(), gyro2.getHeading());
        } else if (gyro1.isOK()) {
            return gyro1.getHeading();
        } else {
            return gyro2.getHeading();
        }
    }

    /**
     * Sets the current heading
     * 
     * @param heading
     */
    public void setHeading(Rotation2d heading) {
        gyro1.setHeading(heading);
        gyro2.setHeading(heading);
    }

    public AluminatiDualGyro(AluminatiGyro gyro1, AluminatiGyro gyro2) {
        this.gyro1 = gyro1;
        this.gyro2 = gyro2;
    }
}
