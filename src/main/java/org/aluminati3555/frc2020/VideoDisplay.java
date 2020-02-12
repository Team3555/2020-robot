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

package org.aluminati3555.frc2020;

import org.aluminati3555.frc2020.Robot.ControlPanelColor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Uses a camera stream to give info to the drivers
 * 
 * @author Caleb Heydon
 */
public class VideoDisplay {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;

    private static final Scalar BLACK = new Scalar(0, 0, 0);
    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar RED = new Scalar(255, 0, 0);
    private static final Scalar YELLOW = new Scalar(255, 255, 0);

    private Mat frame;
    private CvSource outputStream;

    private double targetTime;
    private double lastTime;

    /**
     * Release all resources
     */
    public void release() {
        frame.release();
    }

    /**
     * Sends a new frame
     */
    public void update(ControlPanelColor controlPanelColor, double averageDT, double x, double y, double heading,
            String auto) {
        double currentTime = Timer.getFPGATimestamp();
        double delta = currentTime - lastTime;
        if (delta < targetTime) {
            // Skip to keep fps down
            return;
        }

        // Draw on frame
        frame.setTo(BLACK);

        // Set background color for control panel color
        if (controlPanelColor == ControlPanelColor.BLUE) {
            frame.setTo(BLUE);
        } else if (controlPanelColor == ControlPanelColor.GREEN) {
            frame.setTo(GREEN);
        } else if (controlPanelColor == ControlPanelColor.RED) {
            frame.setTo(RED);
        } else if (controlPanelColor == ControlPanelColor.YELLOW) {
            frame.setTo(YELLOW);
        }

        // Fill rectangle for header
        Imgproc.rectangle(frame, new Point(0, 0), new Point(WIDTH - 1, 100), BLACK, -1);

        // Write header
        Imgproc.putText(frame, "averageDT: " + averageDT + " seconds", new Point(5, 15), Core.FONT_HERSHEY_PLAIN, 1,
                GREEN);
        Imgproc.putText(frame, "x: " + x + " inches", new Point(5, 30), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "y: " + y + " inches", new Point(5, 45), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "heading: " + heading + " degrees", new Point(5, 60), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "auto: " + auto, new Point(5, 75), Core.FONT_HERSHEY_PLAIN, 1, GREEN);

        outputStream.putFrame(frame);

        lastTime = currentTime;
    }

    public VideoDisplay(String name, double targetFPS) {
        frame = new Mat(HEIGHT, WIDTH, CvType.CV_8UC3, BLACK);
        outputStream = CameraServer.getInstance().putVideo(name, WIDTH, HEIGHT);

        targetTime = 1 / targetFPS;
        lastTime = Timer.getFPGATimestamp();
    }
}
