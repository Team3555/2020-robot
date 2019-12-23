package org.aluminati3555.lib.trajectoryfollowingmotion;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

/**
 * Interface containing all information necessary for a path including the Path
 * itself, the Path's starting pose, and whether or not the robot should drive
 * in reverse along the path.
 */
public interface PathContainer {
    Path buildPath();

    Pose2d getStartPose();

    boolean isReversed();
}
