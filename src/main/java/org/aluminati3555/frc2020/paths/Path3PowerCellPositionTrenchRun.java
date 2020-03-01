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

package org.aluminati3555.frc2020.paths;

import java.util.ArrayList;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import org.aluminati3555.lib.trajectoryfollowingmotion.PathBuilder;
import org.aluminati3555.lib.trajectoryfollowingmotion.PathContainer;
import org.aluminati3555.lib.trajectoryfollowingmotion.PathBuilder.Waypoint;

public class Path3PowerCellPositionTrenchRun implements PathContainer {
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(510, 95, 0, 0));
        waypoints.add(new Waypoint(486, 95, 15, 120));
        waypoints.add(new Waypoint(457, 79, 15, 120));
        waypoints.add(new Waypoint(430, 27, 15, 120));
        waypoints.add(new Waypoint(410, 27, 0, 120));

        return PathBuilder.buildPathFromWaypoints(waypoints);
    }
    
    public Pose2d getStartPose() {
        return new Pose2d(510, 95, Rotation2d.fromDegrees(0)); 
    }

    public boolean isReversed() {
        return true; 
    }
}
