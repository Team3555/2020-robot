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

import java.util.ArrayList;

/**
 * This class allows for auto modes to be easily created. Do not add tasks once
 * started
 * 
 * @author Caleb Heydon
 */
public class AluminatiAutoTaskList implements AluminatiAutoTask {
    private ArrayList<AluminatiAutoTask> taskList;
    private int taskIndex;

    @Override
    public String toString() {
        return "[AutoTaskList]";
    }

    public void start(double timestamp) {
        taskList.get(taskIndex).start(timestamp);
    }

    public void update(double timestamp) {
        // Completely done?
        if (isComplete()) {
            return;
        }

        // Current task done?
        if (taskList.get(taskIndex).isComplete()) {
            taskIndex++;

            // Start new task if there is one
            if (!isComplete()) {
                taskList.get(taskIndex).start(timestamp);
            }

            // Wait till next loop to update new task
            return;
        }

        taskList.get(taskIndex).update(timestamp);
    }

    public boolean isComplete() {
        return (taskIndex >= taskList.size());
    }

    public void stop() {
        if (!isComplete()) {
            taskList.get(taskIndex).stop();
            taskIndex = taskList.size();
        }
    }

    /**
     * Clears the task list
     */
    public void clear() {
        taskList.clear();
    }

    /**
     * Adds a task to the list
     */
    public void add(AluminatiAutoTask task) {
        taskList.add(task);
    }

    /**
     * Removes a task
     */
    public void remove(AluminatiAutoTask task) {
        taskList.remove(task);
    }

    /**
     * Removes a task by index
     */
    public void remove(int index) {
        taskList.remove(index);
    }

    /**
     * Returns the index of the current task
     */
    public int getTaskIndex() {
        return taskIndex;
    }

    /**
     * Returns the current task
     */
    public AluminatiAutoTask getCurrentTask() {
        return taskList.get(taskIndex);
    }

    public AluminatiAutoTaskList() {
        taskList = new ArrayList<AluminatiAutoTask>();
        taskIndex = 0;
    }
}
