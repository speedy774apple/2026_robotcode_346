package frc.robot.subsystems.shooter;

import java.util.Arrays;
import java.util.Comparator;
import java.util.function.ToDoubleFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterAutoMap {
	private ShooterAutoMap() {
	}

	private record Marker(double distanceFeet, double threeInchPercent, double twoInchPercent) {
	}

	
	private static final Marker[] MARKERS = new Marker[] {
			new Marker(5.0, 3120.0, 4260.0),
			new Marker(6.0, 3360.0, 4560 ),
            new Marker(7.0, 3566.0, 4848.0),
            new Marker(8.0, 3720, 5160),
            new Marker(9.0, 4020, 5520),
            new Marker(10.0, 4140, 5700),
            new Marker(11.0, 4560, 6000.0),
           
	};

	static {
		Arrays.sort(MARKERS, Comparator.comparingDouble(Marker::distanceFeet));
	}

	public static double getTwoInchRpm(double distanceFeet) {
		return ShooterConstants.TALON_2_INCH_TARGET_RPM * (getTwoInchPercent(distanceFeet) / 100.0);
	}

	public static double getThreeInchRpm(double distanceFeet) {
		return ShooterConstants.TALON_3_INCH_TARGET_RPM * (getThreeInchPercent(distanceFeet) / 100.0);
	}

	public static double getTwoInchPercent(double distanceFeet) {
		return getPercent(distanceFeet, Marker::twoInchPercent);
	}

	public static double getThreeInchPercent(double distanceFeet) {
		return getPercent(distanceFeet, Marker::threeInchPercent);
	}

	public static double getDistanceFeet(Pose2d robotPose, Translation2d targetTranslation) {
		return Units.metersToFeet(robotPose.getTranslation().getDistance(targetTranslation));
	}

	private static double getPercent(double distanceFeet, ToDoubleFunction<Marker> selector) {
		if (MARKERS.length == 0) {
			return 0.0;
		}
		if (MARKERS.length == 1) {
			return selector.applyAsDouble(MARKERS[0]);
		}

		if (distanceFeet <= MARKERS[0].distanceFeet()) {
			return interpolate(distanceFeet, MARKERS[0], MARKERS[1], selector);
		}

		for (int i = 0; i < MARKERS.length - 1; i++) {
			Marker a = MARKERS[i];
			Marker b = MARKERS[i + 1];
			if (distanceFeet <= b.distanceFeet()) {
				return interpolate(distanceFeet, a, b, selector);
			}
		}

		int last = MARKERS.length - 1;
		return interpolate(distanceFeet, MARKERS[last - 1], MARKERS[last], selector);
	}

	private static double interpolate(double x, Marker a, Marker b, ToDoubleFunction<Marker> selector) {
		double dx = b.distanceFeet() - a.distanceFeet();
		if (Math.abs(dx) < 1e-9) {
			return selector.applyAsDouble(a);
		}
		double t = (x - a.distanceFeet()) / dx;
		return selector.applyAsDouble(a) + t * (selector.applyAsDouble(b) - selector.applyAsDouble(a));
	}
}
