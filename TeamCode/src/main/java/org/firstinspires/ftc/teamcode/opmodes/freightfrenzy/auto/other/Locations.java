package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.other;/*
package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;

public class Locations {


    public enum Motors {
        // Drivetrain
        RIGHT_FRONT (
                robot.drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,-53, Math.toRadians(-30)))
                .lineToConstantHeadtg(new Vector2d(-5,-47))
                .build()
        ),
        RIGHT_REAR ("br", false),
        LEFT_FRONT ("fl", true),
        LEFT_REAR ("bl", true),

        // Carousel mechanism
        CAROUSEL ("carousel", false),

        // Lift mechanism
        LIFT ("lift", false);
        

        Motors(TrajectorySequence ts, CheckmateRobot robot, Pose2d startpose){
            this.ts = ts;
            this.robot = ts;
            this.ts = ts;
        }
    }

}
*/
