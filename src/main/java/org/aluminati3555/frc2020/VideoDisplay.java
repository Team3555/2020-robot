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

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

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
    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        double delta = currentTime - lastTime;
        if (delta < targetTime) {
            // Skip to keep fps down
            return;
        }

        outputStream.putFrame(frame);

        lastTime = currentTime;
    }

    public VideoDisplay(String name, double targetFPS) {
        frame = new Mat(HEIGHT, WIDTH, CvType.CV_8UC3, new Scalar(0, 255, 0));
        outputStream = CameraServer.getInstance().putVideo(name, WIDTH, HEIGHT);

        targetTime = 1 / targetFPS;
        lastTime = Timer.getFPGATimestamp();
    }
}
