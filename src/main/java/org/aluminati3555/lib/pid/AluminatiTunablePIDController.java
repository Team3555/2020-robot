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

import org.aluminati3555.lib.net.AluminatiTunable;

public class AluminatiTunablePIDController extends AluminatiPIDController {
    @Override
    public String toString() {
        return "[TunablePIDController]";
    }

    private void startListener(int port) {
        new AluminatiTunable(port) {
            protected void update(TuningData data) {
                setPID(data.kP, data.kI, data.kD);
            }
        };
    }

    public AluminatiTunablePIDController(int port, double kP, double kI, double kD, double iZone, double allowableError,
            double maxOutput, double timestamp) {
        super(kP, kI, kD, iZone, allowableError, maxOutput, timestamp);

        startListener(port);
    }

    public AluminatiTunablePIDController(int port, double kP, double kI, double kD, double timestamp) {
        super(kP, kI, kD, timestamp);

        startListener(port);
    }
}
