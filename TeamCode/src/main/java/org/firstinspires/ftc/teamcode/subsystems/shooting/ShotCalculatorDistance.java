package org.firstinspires.ftc.teamcode.subsystems.shooting;

import static org.firstinspires.ftc.teamcode.Utility.lerp;
import static org.firstinspires.ftc.teamcode.Utility.polarTo;

import com.opencsv.CSVReaderHeaderAware;
import com.opencsv.exceptions.CsvValidationException;

import com.pedropathing.geometry.Pose;

import java.io.FileReader;
import java.io.IOException;
import java.util.Map;
import java.util.TreeMap;

public class ShotCalculatorDistance implements ShotCalculator {
	private final TreeMap<Double, Double> distanceToRPM = new TreeMap<>();

	private final double minDistance = 0;
	private final double maxDistance = 100;

	private Pose robotPose;
	private Pose goalPose;

	public ShotCalculatorDistance() {
	}


	@Override
	public void init() {
		reset();
		clearShotPoints();
		loadFromCSV("shotTable.csv", "distance_inches", "rpm");
	}

	@Override
	public void reset() {
		robotPose = null;
		goalPose = null;
	}

	@Override
	public void updateRobotPose(Pose robotPose) {
		this.robotPose = robotPose;
	}

	@Override
	public void updateGoalPose(Pose goalPose) {
		this.goalPose = goalPose;
	}

	@Override
	public ShotSolution run() {
		if (robotPose == null || goalPose == null || distanceToRPM.isEmpty()) {
			return new ShotSolution(0.0, 0.0, 0.0, false);
		}

		double[] polar = polarTo(robotPose, goalPose);
		double distance = polar[0];
		double heading = polar[1];

		if (distance < minDistance || distance > maxDistance) {
			return new ShotSolution(distance, 0.0, heading, false);
		}

		double rpm = lookupRPM(distance);

		return new ShotSolution(distance, rpm, heading, true);
	}

	private double lookupRPM(double distance) {
		Map.Entry<Double, Double> lower = distanceToRPM.floorEntry(distance);
		Map.Entry<Double, Double> upper = distanceToRPM.ceilingEntry(distance);

		if (lower == null) {
			return distanceToRPM.firstEntry().getValue();
		}
		if (upper == null) {
			return distanceToRPM.lastEntry().getValue();
		}
		if (lower.getKey().equals(upper.getKey())) {
			return lower.getValue();
		}

		double t = (distance - lower.getKey()) / (upper.getKey() - lower.getKey());
		return lerp(lower.getValue(), upper.getValue(), t);
	}

	// testing
	public void addShotPoint(double distance, double rpm) {
		distanceToRPM.put(distance, rpm);
	}

	public void clearShotPoints() {
		distanceToRPM.clear();
	}

	public int loadFromCSV(String fileName, String keyHeader, String valueHeader) {
		int linesLoaded = 0;
		try (CSVReaderHeaderAware reader = new CSVReaderHeaderAware(new FileReader(fileName))) {
			Map<String, String> row;
			String k, v;
			while ((row = reader.readMap()) != null) { // 2 in 1
				k = row.get(keyHeader);
				v = row.get(valueHeader);
				if (k == null || v == null || k.startsWith("#")) {
					continue; // if missing column or comment, skip
				}
				k = k.trim();
				v = v.trim();
				if (k.isEmpty() || v.isEmpty()) {
					continue; // if blank row, skip
				}
				try {
					addShotPoint(Double.parseDouble(k), Double.parseDouble(v));
					linesLoaded++;
				} catch (NumberFormatException e) {
					// if messed up row, skip
				}
			}
		} catch (IOException | CsvValidationException e) {
			return -1;
		}

		return linesLoaded;
	}
}