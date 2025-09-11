package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.decode;

/**
 * Motif states on the Obelisk.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public enum Motif {
    /**
     * Green-Purple-Purple
     */
    GPP(21),
    /**
     * Purple-Green-Purple
     */
    PGP(22),
    /**
     * Purple-Purple-Green
     */
    PPG(23);

    /**
     * AprilTag ID of this Motif.
     */
    public final int aprilTagId;

    Motif(int id) {
        aprilTagId = id;
    }
}
