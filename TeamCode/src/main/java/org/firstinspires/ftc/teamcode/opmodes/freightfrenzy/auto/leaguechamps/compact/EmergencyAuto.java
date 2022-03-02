package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.leaguechamps.compact;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.robot.util.Encoder;

@Disabled
@Autonomous
public class EmergencyAuto extends LinearOpMode {
    FreightFrenzyRobot robot;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.update();
        timer = new ElapsedTime();
        waitForStart();
        encoderDrive(0.5,2000,1,2000);
        encoderDrive(0.5,2000,-1,2000);

    }

    public void encoderDrive(double power, double distance, int direction, double timeout) {
        timer.reset();
        robot.drivetrain.resetEncoderValues(2);
        double speed = power * direction;
        while (Math.abs(robot.drivetrain.getEncoderValues(0)) < distance && Math.abs(robot.drivetrain.getEncoderValues(1)) < distance  && timer.milliseconds() <= timeout) {
            robot.drivetrain.setMotorPowers(speed, speed, speed, speed);
            telemetry.addData("LeftEnc: ", robot.drivetrain.getEncoderValues(0));
            telemetry.addData("RightEnc: ", robot.drivetrain.getEncoderValues(1));
            telemetry.update();
            robot.update();

        }
        robot.drivetrain.setMotorPowers(0,0,0,0);
    }
}
