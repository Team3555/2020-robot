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

import edu.wpi.first.wpilibj.Spark;

/**
 * This class is for controlling the REV Robotics LED driver
 * 
 * @author Caleb Heydon
 */
public class AluminatiLEDDriver {
    private Spark ledDriver;
    private Mode mode;

    @Override
    public String toString() {
        return "[LEDDriver]";
    }

    /**
     * Returns the current mode of the led driver
     */
    public Mode getMode() {
        return mode;
    }

    /**
     * Sets the led mode
     */
    public void setMode(Mode mode) {
        this.mode = mode;
        double output = 0;

        switch (mode) {
        default:
        case OFF:
            output = 0;
            break;
        case RAINBOW:
            output = -0.99;
            break;
        case PARTY:
            output = -0.97;
            break;
        case OCEAN:
            output = -0.95;
            break;
        case SCANNER:
            output = -0.35;
            break;
        case HEARTBEAT_SLOW:
            output = 0.03;
            break;
        case HEARTBEAT_MEDIUM:
            output = 0.05;
            break;
        case HEARTBEAT_FAST:
            output = 0.07;
            break;
        case WAVES:
            output = 0.53;
            break;
        case BLUE:
            output = 0.87;
            break;
        case BLUE_VIOLET:
            output = 0.89;
            break;
        case VIOLET:
            output = 0.91;
            break;
        }

        ledDriver.set(output);
    }

    public AluminatiLEDDriver(int pwmPort) {
        ledDriver = new Spark(pwmPort);
    }

    public enum Mode {
        OFF, RAINBOW, PARTY, OCEAN, SCANNER, HEARTBEAT_SLOW, HEARTBEAT_MEDIUM, HEARTBEAT_FAST, WAVES, BLUE, BLUE_VIOLET,
        VIOLET
    }
}
