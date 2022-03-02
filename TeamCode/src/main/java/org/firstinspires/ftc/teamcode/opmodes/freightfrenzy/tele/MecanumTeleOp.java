package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;//package org.firstinspires.ftc.teamcode.mecanum;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp
//public class MecanumTeleOp extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Declare our motors
//        // Make sure your ID's match your configuration
//        BreakoutMotor fl = hardwareMap.dcMotor.get("fl");
//        BreakoutMotor bl = hardwareMap.dcMotor.get("bl");
//        BreakoutMotor br = hardwareMap.dcMotor.get("br");
//        BreakoutMotor fr = hardwareMap.dcMotor.get("fr");
//
//        BreakoutMotor lift1 = hardwareMap.dcMotor.get("lift1");
//        BreakoutMotor lift2 = hardwareMap.dcMotor.get("lift2");
//
//        BreakoutMotor spinner = hardwareMap.dcMotor.get("spinner");
//        BreakoutMotor intake = hardwareMap.dcMotor.get("intake");
//
//        BreakoutServo boxLifter = new BreakoutServo();
//
//        // Reverse the right side motors
//        // Reverse left motors if you are using NeveRests
//        //fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        //bl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio, but only when
//            // at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontRightPower = (y - x + rx) / denominator;
//            double backRightPower = (y + x + rx) / denominator;
//            double frontLeftPower = (-y + x + rx) / denominator;
//            double backLeftPower = (-y - x + rx) / denominator;
//
//            fl.setPower(frontLeftPower);
//            bl.setPower(backLeftPower);
//            br.setPower(frontRightPower);
//            fr.setPower(backRightPower);
//
//
//
//        }
//    }
//}