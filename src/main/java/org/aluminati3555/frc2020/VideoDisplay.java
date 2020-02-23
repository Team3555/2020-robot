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
import org.aluminati3555.frc2020.Robot.RobotMode;
import org.aluminati3555.lib.vision.AluminatiLimelight;
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
    private static final int HEADER_HEIGHT = 400;

    private static final Scalar BLACK = new Scalar(0, 0, 0);
    private static final Scalar BLUE = new Scalar(255, 0, 0);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar RED = new Scalar(0, 0, 255);
    private static final Scalar YELLOW = new Scalar(0, 255, 255);

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
            String auto, RobotFaults robotFaults, AluminatiLimelight limelight, double shooterSetpoint,
            double shooterVelocity, double feederVelocity, RobotMode robotMode) {
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
        Imgproc.rectangle(frame, new Point(0, 0), new Point(WIDTH - 1, HEADER_HEIGHT), BLACK, -1);

        // Write header
        Imgproc.putText(frame, "averageDT: " + averageDT + " seconds", new Point(5, 15), Core.FONT_HERSHEY_PLAIN, 1,
                GREEN);
        Imgproc.putText(frame, "x: " + x + " inches", new Point(5, 30), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "y: " + y + " inches", new Point(5, 45), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "heading: " + heading + " degrees", new Point(5, 60), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "auto: " + auto, new Point(5, 75), Core.FONT_HERSHEY_PLAIN, 1, GREEN);

        if (!robotFaults.getClimberFault()) {
            Imgproc.putText(frame, "ClimberSystem: OK", new Point(5, 105), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        } else {
            Imgproc.putText(frame, "ClimberSystem: FAULT", new Point(5, 105), Core.FONT_HERSHEY_PLAIN, 1, RED);
        }

        if (!robotFaults.getDriveFault()) {
            Imgproc.putText(frame, "DriveSystem: OK", new Point(5, 120), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        } else {
            Imgproc.putText(frame, "DriveSystem: FAULT", new Point(5, 120), Core.FONT_HERSHEY_PLAIN, 1, RED);
        }

        if (!robotFaults.getIntakeFault()) {
            Imgproc.putText(frame, "IntakeSystem: OK", new Point(5, 135), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        } else {
            Imgproc.putText(frame, "IntakeSystem: FAULT", new Point(5, 135), Core.FONT_HERSHEY_PLAIN, 1, RED);
        }

        if (!robotFaults.getMagazineFault()) {
            Imgproc.putText(frame, "MagazineSystem: OK", new Point(5, 150), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        } else {
            Imgproc.putText(frame, "MagazineSystem: FAULT", new Point(5, 150), Core.FONT_HERSHEY_PLAIN, 1, RED);
        }

        if (!robotFaults.getShooterFault()) {
            Imgproc.putText(frame, "ShooterSystem: OK", new Point(5, 165), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        } else {
            Imgproc.putText(frame, "ShooterSystem: FAULT", new Point(5, 165), Core.FONT_HERSHEY_PLAIN, 1, RED);
        }

        if (!robotFaults.getSpinnerFault()) {
            Imgproc.putText(frame, "SpinnerSystem: OK", new Point(5, 180), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        } else {
            Imgproc.putText(frame, "SpinnerSystem: FAULT", new Point(5, 180), Core.FONT_HERSHEY_PLAIN, 1, RED);
        }

        Imgproc.putText(frame, "hasTarget: " + limelight.hasTarget(), new Point(5, 210), Core.FONT_HERSHEY_PLAIN, 1,
                GREEN);
        Imgproc.putText(frame, "targetX: " + limelight.getX() + " degrees", new Point(5, 225), Core.FONT_HERSHEY_PLAIN,
                1, GREEN);
        Imgproc.putText(frame, "targetY: " + limelight.getX() + " degrees", new Point(5, 240), Core.FONT_HERSHEY_PLAIN,
                1, GREEN);
        Imgproc.putText(frame, "targetWidth: " + limelight.getHorizontal() + " degrees", new Point(5, 255),
                Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "targetHeight: " + limelight.getVertical() + " degrees", new Point(5, 270),
                Core.FONT_HERSHEY_PLAIN, 1, GREEN);

        Imgproc.putText(frame, "shooterSetpoint: " + shooterSetpoint + " RPM", new Point(5, 300),
                Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        Imgproc.putText(frame, "shooterVelocity: " + shooterVelocity + " RPM", new Point(5, 315),
                Core.FONT_HERSHEY_PLAIN, 1, GREEN);

        Imgproc.putText(frame, "feederVelocity: " + feederVelocity + " RPM", new Point(5, 345), Core.FONT_HERSHEY_PLAIN,
                1, GREEN);

        if (robotMode == RobotMode.AUTONOMOUS) {
            Imgproc.putText(frame, "robotMode: AUTONOMOUS", new Point(5, 375), Core.FONT_HERSHEY_PLAIN, 1, GREEN);
        } else {
            Imgproc.putText(frame, "robotMode: OPERATOR_CONTROLLED", new Point(5, 375), Core.FONT_HERSHEY_PLAIN, 1,
                    GREEN);
        }

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
