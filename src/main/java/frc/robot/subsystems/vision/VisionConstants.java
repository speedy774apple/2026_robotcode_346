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
            "FR",
            "BL",
            "BR"
    };

    // All positions in meters, angles in radians
    public static final Transform3d[] vehicleToCameras = {

            // Front (Top Camera) - FR
            new Transform3d(
                     new Translation3d(-0.26284, 0.08263, 0.67376),
                    new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(0))),

            // Back Left - BL
            new Transform3d(
                    new Translation3d(-0.2813, 0.2747, 0.1616),
                    new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(135))),

            // Back Right - BR
            new Transform3d(
                    new Translation3d(-0.2811, -0.2749, 0.1616),
                    new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-135)))
    };



    
    public static final List<CameraParams> cameras = List.of(
            new CameraParams(cameraNames[0], vehicleToCameras[0]),
            new CameraParams(cameraNames[1], vehicleToCameras[1]),
            new CameraParams(cameraNames[2], vehicleToCameras[2])
    );

    // Field Layout for visual localization and map generation
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);
}
