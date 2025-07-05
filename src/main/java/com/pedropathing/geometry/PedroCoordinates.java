package com.pedropathing.geometry;

/**
 * <p>A coordinate system, such as FTC standard coordinates or Pedro coordinates</p>
 * <br>
 * Pedro coordinates are a coordinate system used by the Pedro Pathing library.
 * It is a 144x144 coordinate system with the origin at (72, 72),
 * with a heading of 0 degrees facing the red alliance wall from the center of the field.
 *
 * @author BeepBot99
 * @author Baron Henderson - 20077 The Indubitables
 */
public enum PedroCoordinates implements CoordinateSystem {
    INSTANCE;

    /**
     * Converts a {@link Pose} from this coordinate system to FTC standard coordinates
     *
     * @param pose The {@link Pose} to convert, in this coordinate system
     * @return The converted {@link Pose}, in FTC standard coordinates
     */
    @Override
    public Pose convertToFtcStandard(Pose pose) {
        Pose normalizedPose = pose.minus(new Pose(72, 72));
        return normalizedPose.rotate(-Math.PI / 2, true);
    }

    /**
     * Converts a {@link Pose} from FTC standard coordinates to this coordinate system
     *
     * @param pose The {@link Pose} to convert, in FTC standard coordinates
     * @return The converted {@link Pose}, in this coordinate system
     */
    @Override
    public Pose convertFromFtcStandard(Pose pose) {
        Pose rotatedPose = pose.rotate(-Math.PI / 2, true);
        return rotatedPose.plus(new Pose(72, 72));
    }
}
