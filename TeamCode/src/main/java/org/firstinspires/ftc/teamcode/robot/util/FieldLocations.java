package org.firstinspires.ftc.teamcode.robot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder;

public enum FieldLocations {
    RED_ALLIANCE_SHIPPING_HUB_STEP_1(new Pose2d(12, -53, Math.toRadians(-30))),
    RED_ALLIANCE_SHIPPING_HUB_STEP_2(new Pose2d(-5, -47)); //, Math.toRadians(0)));

    public Pose2d pose;

    FieldLocations(Pose2d pose) {
        this.pose = pose;
    }
}
