package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Poses {
	public static final PoseFactory staticPoses = PoseFactory.degrees();
	public static final PoseFactory variablePoses = PoseFactory.degrees();
	public static double mirrorLineY = 70.75;
	public static final PoseFactory variablePosesMirrored = variablePoses.mirrorY(mirrorLineY);

	public static final Pose FLOWER_AUDIENCE = staticPoses.of(12.75, 46.75, 180);
	public static final Pose FLOWER_PARK = staticPoses.of(46.75, 128.75, 90);

	private static final Pose BASE_HIVE = variablePoses.of(57.75, 50.75, 90);
	private static final Pose BASE_SHOOT_TELEOP = variablePoses.of(57.75, 11.75, 90);
	private static final Pose MIRRORED_HIVE = variablePosesMirrored.of(57.75, 50.75, 90);
	private static final Pose MIRRORED_SHOOT_TELEOP = variablePosesMirrored.of(57.75, 11.75, 90);

	public static Pose HIVE = BASE_HIVE;
	public static Pose SHOOT_TELEOP = BASE_SHOOT_TELEOP;

	public static void updateForRobotPose(Pose robotPose) {
		boolean farSide = robotPose.y() > mirrorLineY;
		HIVE = farSide ? MIRRORED_HIVE : BASE_HIVE;
		SHOOT_TELEOP = farSide ? MIRRORED_SHOOT_TELEOP : BASE_SHOOT_TELEOP;
	}
}
