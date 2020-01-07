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

package org.aluminati3555.lib.drivers;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This wrapper class interfaces with the REV Robotics ColorSensorV3
 * 
 * @author Caleb Heydon
 */
public class AluminatiColorSensor extends ColorSensorV3 {
    @Override
    public String toString() {
        return "[ColorSensor]";
    }

    /**
     * Returns the RGB color
     */
    public RGB getRGB() {
        Color color = this.getColor();
        return new RGB(color.red, color.green, color.blue);
    }

    /**
     * Returns the proximety as a value between 0 and 1. 0 is close and 1 is far.
     */
    public double getActualProximityPercent() {
        return 1 - this.getProximity() / 2047.0;
    }

    public AluminatiColorSensor() {
        // Use the robotRIO's designated I2C port
        super(I2C.Port.kOnboard);
    }

    public static class RGB {
        private double r;
        private double g;
        private double b;

        /**
         * Returns the R value
         */
        public double getR() {
            return r;
        }

        /**
         * Returns the G value
         */
        public double getG() {
            return g;
        }

        /**
         * Returns the B value
         */
        public double getB() {
            return b;
        }

        public RGB(double r, double b, double g) {
            this.r = r;
            this.b = b;
            this.g = g;
        }
    }
}
