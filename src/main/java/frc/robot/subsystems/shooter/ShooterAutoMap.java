package frc.robot.subsystems.shooter;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterAutoMap {
	private ShooterAutoMap() {
	}

	private record Marker(double distanceFeet, double threeInchPercent) {
	}

	// need to add more markers
	private static final Marker[] THREE_INCH_MARKERS = new Marker[] {
			new Marker(8.0, 68.0),
			new Marker(10.0, 68.0)
	};

	static {
		Arrays.sort(THREE_INCH_MARKERS, Comparator.comparingDouble(Marker::distanceFeet));
	}

	public static double getTwoInchRpm(double distanceFeet) {
		return ShooterConstants.TALON_2_INCH_TARGET_RPM;
	}

	public static double getThreeInchPercent(double distanceFeet) {
		if (THREE_INCH_MARKERS.length == 0) {
			return 0.0;
		}
		if (THREE_INCH_MARKERS.length == 1) {
			return THREE_INCH_MARKERS[0].threeInchPercent();
		}

		if (distanceFeet <= THREE_INCH_MARKERS[0].distanceFeet()) {
			return interpolate(distanceFeet, THREE_INCH_MARKERS[0], THREE_INCH_MARKERS[1]);
		}

		for (int i = 0; i < THREE_INCH_MARKERS.length - 1; i++) {
			Marker a = THREE_INCH_MARKERS[i];
			Marker b = THREE_INCH_MARKERS[i + 1];
			if (distanceFeet <= b.distanceFeet()) {
				return interpolate(distanceFeet, a, b);
			}
		}

		int last = THREE_INCH_MARKERS.length - 1;
		return interpolate(distanceFeet, THREE_INCH_MARKERS[last - 1], THREE_INCH_MARKERS[last]);
	}

	public static double getThreeInchRpm(double distanceFeet) {
		return ShooterConstants.TALON_3_INCH_TARGET_RPM * (getThreeInchPercent(distanceFeet) / 100.0);
	}

	public static double getDistanceFeet(Pose2d robotPose, Translation2d targetTranslation) {
		return Units.metersToFeet(robotPose.getTranslation().getDistance(targetTranslation));
	}

	private static double interpolate(double x, Marker a, Marker b) {
		double dx = b.distanceFeet() - a.distanceFeet();
		if (Math.abs(dx) < 1e-9) {
			return a.threeInchPercent();
		}
		double t = (x - a.distanceFeet()) / dx;
		return a.threeInchPercent() + t * (b.threeInchPercent() - a.threeInchPercent());
		
	}
}
