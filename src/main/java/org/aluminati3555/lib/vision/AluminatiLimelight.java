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

package org.aluminati3555.lib.vision;

import org.aluminati3555.lib.drivers.AluminatiPoweredDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * This class provides an interface to the limelight
 * 
 * @author Caleb Heydon
 */
public class AluminatiLimelight implements AluminatiPoweredDevice {
    // Constants
    public static final String DEFAULT_TABLE_NAME = "limelight";

    // Limelight
    private String tableName;
    private NetworkTable limelightTable;

    private NetworkTableEntry tvEntry;
    private NetworkTableEntry txEntry;
    private NetworkTableEntry tyEntry;
    private NetworkTableEntry taEntry;
    private NetworkTableEntry tsEntry;
    private NetworkTableEntry tlEntry;
    private NetworkTableEntry tshortEntry;
    private NetworkTableEntry tlongEntry;
    private NetworkTableEntry thorEntry;
    private NetworkTableEntry tvertEntry;
    private NetworkTableEntry getpipeEntry;
    private NetworkTableEntry camtranEntry;

    private NetworkTableEntry ledModeEntry;
    private NetworkTableEntry camModeEntry;
    private NetworkTableEntry pipelineEntry;
    private NetworkTableEntry streamEntry;
    private NetworkTableEntry snapshotEntry;

    // PDP
    private PowerDistributionPanel pdp;
    private int pdpChannel;

    // Current warning
    private boolean currentWarning;
    private double currentWarningThreshold;

    /**
     * Returns the pdp channel
     */
    public int getPDPChannel() {
        return pdpChannel;
    }

    /**
     * Returns true if a current warning is enabled
     */
    public boolean isCurrentWarningEnabled() {
        return currentWarning;
    }

    /**
     * Returns the current warning threshold
     */
    public double getCurrentWarningThreshold() {
        return currentWarningThreshold;
    }

    /**
     * Returns the limelight table
     */
    public NetworkTable getLimelightTable() {
        return limelightTable;
    }

    /**
     * Use true to enable a current warning
     */
    public void setEnableCurrentWarning(boolean enabled) {
        currentWarning = enabled;
    }

    /**
     * Sets the current warning threshold
     */
    public void setCurrentWarningThreshold(double threshold) {
        currentWarningThreshold = threshold;
    }

    /**
     * Provides a useful string
     */
    @Override
    public String toString() {
        return "[Limelight:\"" + tableName + "\"]";
    }

    /**
     * Returns the output current
     */
    public double getOutputCurrent() {
        return pdp.getCurrent(pdpChannel);
    }

    /**
     * Checks the current and reports it if the warning is enabled
     */
    private void checkCurrent() {
        if (currentWarning) {
            // Check current
            if (pdp.getCurrent(pdpChannel) >= currentWarningThreshold) {
                // Warn drivers
                DriverStation.reportWarning(this.toString() + " exceeded its current warning threshold", false);
            }
        }
    }

    /**
     * Returns true if the limelight has detected a target
     * 
     * @return
     */
    public boolean hasTarget() {
        checkCurrent();

        if (tvEntry.getDouble(0) == 1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns the x location of the target
     * 
     * @return
     */
    public double getX() {
        checkCurrent();

        return txEntry.getDouble(0);
    }

    /**
     * Returns the y location of the target
     * 
     * @return
     */
    public double getY() {
        checkCurrent();

        return tyEntry.getDouble(0);
    }

    /**
     * Returns the percentage of the screen the target takes up (0-1)
     * 
     * @return
     */
    public double getArea() {
        checkCurrent();

        return taEntry.getDouble(0);
    }

    /**
     * Returns the rotation of the target in degrees
     * 
     * @return
     */
    public double getRotation() {
        checkCurrent();

        return tsEntry.getDouble(0);
    }

    /**
     * Returns the latency in ms
     */
    public int getLatency() {
        checkCurrent();

        return (int) tlEntry.getDouble(0);
    }

    /**
     * Returns the length of the side with the shortest side of the target's
     * bounding box
     * 
     * @return
     */
    public double getShort() {
        checkCurrent();

        return tshortEntry.getDouble(0);
    }

    /**
     * Returns the length of the side with the longest side of the target's bounding
     * box
     * 
     * @return
     */
    public double getLong() {
        checkCurrent();

        return tlongEntry.getDouble(0);
    }

    /**
     * Gets the length of the bounding box
     */
    public double getHorizontal() {
        checkCurrent();

        return thorEntry.getDouble(0);
    }

    /**
     * Gets the height of the bounding box
     */
    public double getVertical() {
        checkCurrent();

        return tvertEntry.getDouble(0);
    }

    /**
     * Returns the piple currently in use
     */
    public int getPipeline() {
        checkCurrent();

        return (int) getpipeEntry.getDouble(0);
    }

    /**
     * Returns the camtran data. Refer to limelight docs
     * 
     * @return
     */
    public double[] getCamtranData() {
        checkCurrent();

        return camtranEntry.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
    }

    /**
     * Sets the led mode
     */
    public void setLEDMode(LEDMode ledMode) {
        checkCurrent();

        int value = 0;

        switch (ledMode) {
        case CURRENT_PIPELINE:
            value = 0;
            break;
        case OFF:
            value = 1;
            break;
        case BLINK:
            value = 2;
            break;
        case ON:
            value = 3;
            break;
        default:
            value = 0;
        }

        ledModeEntry.setNumber(value);
    }

    /**
     * Sets the camera mode
     * 
     * @param cameraMode
     */
    public void setCameraMode(CameraMode cameraMode) {
        checkCurrent();

        int value = 0;

        switch (cameraMode) {
        case VISION:
            value = 0;
            break;
        case DRIVER_CAMERA:
            value = 1;
            break;
        default:
            value = 0;
        }

        camModeEntry.setNumber(value);
    }

    /**
     * Sets the pipeline
     */
    public void setPipeline(int pipeline) {
        checkCurrent();

        pipelineEntry.setNumber(pipeline);
    }

    /**
     * Sets the streaming mode
     * 
     * @param streamMode The streaming mode
     */
    public void setStreamMode(StreamMode streamMode) {
        checkCurrent();

        int value = 0;

        switch (streamMode) {
        case STANDARD:
            value = 0;
            break;
        case PIP_MAIN:
            value = 1;
            break;
        case PIP_SECONDARY:
            value = 2;
            break;
        default:
            value = 0;
        }

        streamEntry.setNumber(value);
    }

    /**
     * Sets the snapshot mode
     */
    public void setSnapshotMode(SnapshotMode snapshotMode) {
        checkCurrent();

        int value = 0;

        switch (snapshotMode) {
        case NONE:
            value = 0;
            break;
        case TWO_PER_SECOND:
            value = 1;
            break;
        default:
            value = 0;
        }

        snapshotEntry.setNumber(value);
    }

    /**
     * Creates a limelight with a custom table name
     * 
     * @param tableName The limelight table name
     */
    public AluminatiLimelight(String tableName) {
        this.tableName = tableName;

        // Create new table
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);

        // Create entries
        tvEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tsEntry = limelightTable.getEntry("ts");
        tlEntry = limelightTable.getEntry("tl");
        tshortEntry = limelightTable.getEntry("tshort");
        tlongEntry = limelightTable.getEntry("tlong");
        thorEntry = limelightTable.getEntry("thor");
        tvertEntry = limelightTable.getEntry("tvert");
        getpipeEntry = limelightTable.getEntry("getpipe");
        camtranEntry = limelightTable.getEntry("camtran");

        ledModeEntry = limelightTable.getEntry("ledMode");
        camModeEntry = limelightTable.getEntry("camMode");
        pipelineEntry = limelightTable.getEntry("pipeline");
        streamEntry = limelightTable.getEntry("stream");
        snapshotEntry = limelightTable.getEntry("snapshot");
    }

    /**
     * Creates a new limelight with the default table name
     */
    public AluminatiLimelight() {
        this(DEFAULT_TABLE_NAME);
    }

    /**
     * Default limelight table name with current monitoring but without current
     * warning
     * 
     * @param pdpChannel PDP channel
     */
    public AluminatiLimelight(int pdpChannel) {
        this();

        pdp = new PowerDistributionPanel();
    }

    /**
     * Default limelight table with current monitoring and warning
     */
    public AluminatiLimelight(int pdpChannel, double currentWarningThreshold) {
        this(pdpChannel);

        this.currentWarningThreshold = currentWarningThreshold;
        this.currentWarning = true;
    }

    public AluminatiLimelight(String tableName, int pdpChannel) {
        this(tableName);

        pdp = new PowerDistributionPanel();
    }

    public AluminatiLimelight(String tableName, int pdpChannel, double currentWarningThreshold) {
        this(tableName, pdpChannel);

        this.currentWarningThreshold = currentWarningThreshold;
        this.currentWarning = true;
    }

    // Modes
    public enum LEDMode {
        CURRENT_PIPELINE, OFF, BLINK, ON
    }

    public enum CameraMode {
        VISION, DRIVER_CAMERA
    }

    public enum StreamMode {
        STANDARD, PIP_MAIN, PIP_SECONDARY
    }

    public enum SnapshotMode {
        NONE, TWO_PER_SECOND
    }
}
