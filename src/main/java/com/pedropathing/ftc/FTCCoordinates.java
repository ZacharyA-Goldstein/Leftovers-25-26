package com.pedropathing.ftc;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;

/**
 * An enum that contains the FTC standard coordinate system.
 * This enum implements the {@link CoordinateSystem} interface, which specifies a way to convert to and from FTC standard coordinates.
 *
 * @author BeepBot99
 */
public enum FTCCoordinates implements CoordinateSystem {
    INSTANCE;

    /**
     * Converts a {@link Pose} from FTC standard coordinates to FTC standard coordinates
     * Since they are the same coordinate system, no conversion is necessary.
     *
     * @param pose The {@link Pose} to convert, in FTC standard coordinates
     * @return The converted {@link Pose}, in FTC standard coordinates
     */
    @Override
    public Pose convertToFtcStandard(Pose pose) {
        return pose;
    }

    /**
     * Converts a {@link Pose} from FTC standard coordinates to FTC standard coordinates.
     * Since they are the same coordinate system, no conversion is necessary.
     *
     * @param pose The {@link Pose} to convert, in FTC standard coordinates
     * @return The converted {@link Pose}, in FTC standard coordinates.
     */
    @Override
    public Pose convertFromFtcStandard(Pose pose) {
        return pose;
    }
}