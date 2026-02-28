package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

    public static record CameraParams(
            String name,
            Transform3d transforms) {
    };

    public static final String[] cameraNames = {
            "FL",
            "FR",
            "BL",
            "BR"
    };

    // All positions in meters, angles in radians
   public static final Transform3d[] vehicleToCameras = {
    // Front Left (FL): Facing 45° Left, Tilted 15° UP
   new Transform3d(new Translation3d(0.298, 0.286, 0.175),
        new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(45))),

    // Front Right (FR): Facing 45° Right, Tilted 15° UP
    new Transform3d(new Translation3d(0.298, -0.286, 0.175),
        new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-45))),

    // Back Left (BL): Facing 135° (Back-Left), Tilted 15° UP
    new Transform3d(new Translation3d(-0.298, 0.286, 0.175),
        new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(135))),

    // Back Right (BR): Facing -135° (Back-Right), Tilted 15° UP
    new Transform3d(new Translation3d(-0.298, -0.286, 0.175),
        new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-135)))
};


    
public static final List<CameraParams> cameras = List.of(
    new CameraParams("FR", vehicleToCameras[1]),
    new CameraParams("BL", vehicleToCameras[2]),
    new CameraParams("BR", vehicleToCameras[3])
);

    // Field Layout for visual localization and map generation
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);
}
