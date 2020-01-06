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

package org.aluminati3555.frc2020.auto;

import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.lib.auto.AluminatiAutoTask;

/**
 * This action waits for a path marker to be passed before starting the action
 * (use with parallel action)
 * 
 * @author Caleb Heydon
 */
public class ActionOnPathMarkerPassed implements AluminatiAutoTask {
    private DriveSystem driveSystem;
    private String marker;

    private AluminatiAutoTask task;

    private State state;

    public void start(double timestamp) {
        state = State.WAITING;
    }

    public void update(double timestamp) {
        if (state == State.WAITING && driveSystem.hasPassedPathMarker(marker)) {
            state = State.RUNNING;
            task.start(timestamp);
        } else if (state == State.RUNNING) {
            if (task.isComplete()) {
                state = State.COMPLETE;
                return;
            } else {
                task.update(timestamp);
            }
        }
    }

    public void stop() {
        task.stop();
        state = State.COMPLETE;
    }

    public boolean isComplete() {
        return (state == State.COMPLETE);
    }

    public ActionOnPathMarkerPassed(DriveSystem driveSystem, String marker, AluminatiAutoTask task) {
        this.driveSystem = driveSystem;
        this.marker = marker;
        this.task = task;
    }

    private enum State {
        WAITING, RUNNING, COMPLETE
    }
}
