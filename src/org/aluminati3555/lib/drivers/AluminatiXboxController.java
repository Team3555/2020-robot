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

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is the wrapper class for the joystick. It squares the output of the
 * joystick. The z axis is not squared. There is an option to square or not
 * square the joystick.
 * 
 * @author Caleb Heydon
 */

public class AluminatiXboxController extends XboxController {
    // Joystick port
    private int port;

    /**
     * Returns the joystick's port
     */
    public int getPort() {
        return port;
    }

    /**
     * Returns a useful string
     */
    @Override
    public String toString() {
        return "[XboxController:" + port + "]";
    }

    /**
     * Squares the output of one of axises
     * 
     * @param input The original
     * @return Squared output
     */
    private double squareOutput(double input) {
        boolean sign = (input < 0);
        double output = input * input;

        if (sign) {
            output *= -1;
        }

        return output;
    }

    /**
     * Returns the squared x
     * 
     * @return The squared x
     */
    public double getSquaredX() {
        return squareOutput(this.getX());
    }

    /**
     * Returns the squared y
     * 
     * @return The squared y
     */
    public double getSquaredY() {
        return squareOutput(this.getY());
    }

    /**
     * Saves the port number
     */
    public AluminatiXboxController(int port) {
        super(port);

        this.port = port;
    }
}
