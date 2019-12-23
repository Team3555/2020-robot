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

package org.aluminati3555.lib.auto;

/**
 * This allows for multiple auto tasks to run at the same time
 * 
 * @author Caleb Heydon
 */
public class AluminatiParallelAutoTask implements AluminatiAutoTask {
    // The tasks
    private AluminatiAutoTask[] tasks;

    @Override
    public String toString() {
        return "[ParallelAutoTask]";
    }

    /**
     * Returns the array of tasks
     * 
     * @return
     */
    public AluminatiAutoTask[] getTasks() {
        return tasks;
    }

    public void start(double timestamp) {
        for (int i = 0; i < tasks.length; i++) {
            tasks[i].start(timestamp);
        }
    }

    public void update(double timestamp) {
        for (int i = 0; i < tasks.length; i++) {
            tasks[i].update(timestamp);
        }
    }

    public boolean isComplete() {
        for (int i = 0; i < tasks.length; i++) {
            if (!tasks[i].isComplete()) {
                return false;
            }
        }

        return true;
    }

    public void stop() {
        for (int i = 0; i < tasks.length; i++) {
            tasks[i].stop();
        }
    }

    public AluminatiParallelAutoTask(AluminatiAutoTask... tasks) {
        this.tasks = tasks;
    }
}
