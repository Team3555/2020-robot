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

import edu.wpi.first.wpilibj.AnalogGyro;

public class AluminatiAnalogGyro extends AnalogGyro implements AluminatiGyro {
    // See
    // https://github.com/Team217/FRC-217-Libraries/blob/master/org/team217/wpi/AnalogGyro.java
    // for offset

    private int channel;
    private double offset;

    @Override
    public String toString() {
        return "[AnalogGyro:" + channel + "]";
    }

    /**
     * Returns true if the gyro is ok
     */
    public boolean isOK() {
        // There is no way to determine this
        return true;
    }

    /**
     * Returns the gyro heading
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(this.getAngle() + offset);
    }

    /**
     * Sets the gyro heading
     */
    public void setHeading(Rotation2d heading) {
        this.reset();
        this.offset = heading.getDegrees();
    }

    public AluminatiAnalogGyro(int channel) {
        super(channel);
        this.channel = channel;
    }
}
