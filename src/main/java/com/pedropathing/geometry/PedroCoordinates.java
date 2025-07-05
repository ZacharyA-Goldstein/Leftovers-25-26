package com.pedropathing.geometry;

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
