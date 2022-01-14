package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;//package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.breakout.BreakoutMotor;
//import org.firstinspires.ftc.teamcode.breakout.BreakoutServo;
//import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;
//
//@TeleOp
//public class TeleOpRed extends LinearOpMode {
//    //Gamepad 1
//    float leftStick1x;
//    float leftStick1y;
//    float rightStick1x;
//    float rightStick1y;
//    float leftTrigger1;
//    float rightTrigger1;
//    boolean dpadLeft1;
//    boolean dpadRight1;
//
//    boolean dpadDown2;
//    boolean dpadUp2;
//    boolean dpadRight2;
//
//    //Gamepad 2
//    float leftStick2x;
//    float leftStick2y;
//    float rightStick2x;
//    float rightStick2y;
//    float leftTrigger2;
//    float rightTrigger2;
//    boolean aButton;
//    boolean bButton;
//    boolean xButton;
//    boolean yButton;
//
//    int boxPos = 0;
//    private boolean boxLifter = true;
//
//    private double boxTimer = 0;
//
//    private double duckTimer = 0;
//    private ElapsedTime timer = new ElapsedTime();
//
//
//    DcMotor fl;
//    DcMotor bl;
//    DcMotor br;
//    DcMotor fr;
//
//    DcMotor lift1;
//    DcMotor lift2;
//
//    DcMotor spinner;
//    DcMotor intake;
//    Servo box;
//
//    CheckmateRobot robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Declare our motors
//        // Make sure your ID's match your configuration
//
//        fl = hardwareMap.dcMotor.get("fl");
//        bl = hardwareMap.dcMotor.get("bl");
//        br = hardwareMap.dcMotor.get("br");
//        fr = hardwareMap.dcMotor.get("fr");
//
//        lift1 = hardwareMap.dcMotor.get("lift1");
//        lift2 = hardwareMap.dcMotor.get("lift2");
//
//        spinner = hardwareMap.dcMotor.get("spinner");
//        intake = hardwareMap.dcMotor.get("intake");
//
//        box = hardwareMap.servo.get("box");
//
//        // Reverse the right side motors
//        // Reverse left motors if you are using NeveRests
//        //fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        //bl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        box.setPosition(.7);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//
//            leftStick2x = gamepad2.left_stick_x;
//            leftStick2y = -gamepad2.left_stick_y;
//            rightStick2x = gamepad2.right_stick_x;
//            rightStick2y = -gamepad2.right_stick_y;
//            leftTrigger2 = gamepad2.left_trigger;
//            rightTrigger2 = gamepad2.right_trigger;
//            aButton = gamepad2.a;
//            bButton = gamepad2.b;
//            xButton = gamepad2.x;
//            yButton = gamepad2.y;
//            dpadLeft1 = gamepad1.dpad_left;
//            dpadRight1 = gamepad1.dpad_right;
//
//            leftStick1x = gamepad1.left_stick_x;
//            leftStick1y = gamepad1.left_stick_y;
//            rightStick1x = gamepad1.right_stick_x;
//            rightStick1y = gamepad1.right_stick_y;
//            leftTrigger1 = gamepad1.left_trigger;
//            rightTrigger1 = gamepad1.right_trigger;
//            dpadDown2 = gamepad2.dpad_down;
//            dpadUp2 = gamepad2.dpad_up;
//            dpadRight2 = gamepad2.dpad_right;
//
//            mecanumDrive();
//            manipulators();
//
//
//
//        }
//    }
//
//    public void mecanumDrive() {
//        double y = leftStick1x; // Remember, this is reversed!
//        double x =  leftStick1y * 1.1; // Counteract imperfect strafing
//        double rx = -rightStick1x;
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio, but only when
//        // at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontRightPower = (y - x + rx) / denominator;
//        double backRightPower = (y + x + rx) / denominator;
//        double frontLeftPower = (-y + x + rx) / denominator;
//        double backLeftPower = (-y - x + rx) / denominator;
//
//        robot.drivetrain.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
//    }
//
//    public void manipulators() {
//        //Manipulators
//
//        robot.carousel.spinner(duckTimer, timer.milliseconds(), gp2);
//
//        if (Math.abs(leftTrigger2) <= .1 && Math.abs(rightTrigger2) >= .1) {
//            intake.setPower((float) (rightTrigger2));
//        }
//        else if (Math.abs(rightTrigger2) <= .1 && Math.abs(leftTrigger2) >= .1) {
//            intake.setPower(-leftTrigger2);
//        }
//        else {
//            intake.setPower(0);
//        }
//
//
//
//        if (xButton) {
//            spinner.setPower(-0.5F);
//        }
//        else {
//            spinner.setPower((float) (-rightTrigger2*.78));
//        }
//
//        //Intake Servo
//        if (bButton && timer.milliseconds() - boxTimer > 250) {
//
//            if (boxPos == 0) {
//                box.setPosition(1);
//                boxPos = 1;
//                boxTimer = timer.milliseconds();
//            }
//            else if (boxPos == 1) {
//                boxLifter = !boxLifter;
//                box.setPosition(0);
//                boxPos = 2;
//                boxTimer = timer.milliseconds();
//
//            }
//            else {
//                boxLifter = !boxLifter;
//                box.setPosition(0.3);
//                boxPos = 0;
//                boxTimer = timer.milliseconds();
//
//            }
//        }
//
//
//        lift1.setPower(leftStick2y);
//        lift2.setPower(-leftStick2y);
//
//        //Duck Spinner Automatic
//        if (yButton) {
//            duckTimer = timer.milliseconds();
//            while (timer.milliseconds() - duckTimer < 1100 && !dpadDown2) {
//                float speed = (float) -(.0009*(timer.milliseconds() - duckTimer)+.1);
//                spinner.setPower(-speed);
//            }
//            while (timer.milliseconds() - duckTimer < 1600 && timer.milliseconds() - duckTimer >= 1100 && !dpadDown2) {
//                spinner.setPower(1);
//            }
//            while (timer.milliseconds() - duckTimer < 1650 && timer.milliseconds() - duckTimer >= 1600 && !dpadDown2) {
//                spinner.setPower((float)-0.1);
//            }
//        }
//    }
//}