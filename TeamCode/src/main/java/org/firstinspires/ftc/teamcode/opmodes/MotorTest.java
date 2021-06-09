package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.PIDController;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Motor Test", group = "TeleOp")
public class MotorTest extends OpMode {
    Robot robot;
    public final double DEADBAND_MAG_NORMAL = 0.1;
    public final double DEADBAND_MAG_SLOW_MODE = 0.03;
    boolean slowModeDrive;
    public boolean willResetIMU = true;

    double imuTarget = 0;
    double error = 0;
    double lastError = 0;
    double errorSum = 0;
    double errorChange = 0;
    boolean usePIDforMovement = true;

    PIDController pidDrive = new PIDController(15, 2, 3);

    boolean absHeadingMode = false;

    double loopStartTime = 0;
    double loopEndTime = 0;

    private boolean slow = false;
    private boolean claw = false;
    private boolean pusherThing = true;
    private boolean boxLifter = true;
    private double boxTimer = 0;
    private double pusherTimer = 0;
    private ElapsedTime timer = new ElapsedTime();

    //hungry hippo (1 servo, 2 positions)
    //foundation grabber - latch (2 servos, 2 positions)
    //lift (2 motors, continuous)
    //intake (2 motors, continuous)
    //arm (2 servos, continuous)
    //grabber (1 servo, 2 positions)

    double lastTime;

    Vector2d joystick1, joystick2;

    public void init() {
        robot = new Robot(this, false, false);
    }

    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
        lastTime = getRuntime();
    }


    public void start () {
        if (willResetIMU) robot.initIMU();

        pidDrive.setSetpoint(imuTarget);
        pidDrive.setOutputRange(-1, 1);
        pidDrive.setInputRange(-180, 180);
        pidDrive.enable();
        imuTarget = robot.getRobotHeading().getAngle();
    }

    public void loop() {
        loopStartTime = System.currentTimeMillis();

        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        joystick2 = new Vector2d(gamepad1.right_stick_x+((gamepad1.right_stick_x/Math.abs(gamepad1.right_stick_x))*0.1), -gamepad1.right_stick_y); //RIGHT joystick
        slowModeDrive = false;


        //Gamepad 2
        float leftStick2x = gamepad2.left_stick_x;
        float leftStick2y = -gamepad2.left_stick_y;
        float rightStick2x = gamepad2.right_stick_x;
        float rightStick2y = -gamepad2.right_stick_y;
        float leftTrigger2 = gamepad2.left_trigger;
        float rightTrigger2 = gamepad2.right_trigger;
        boolean aButton = gamepad2.a;
        boolean bButton = gamepad2.b;
        boolean xButton = gamepad2.x;
        boolean yButton = gamepad2.y;


        if (xButton) {
            robot.driveController.moduleRight.motor1.setPower(1);
        }

        if (yButton) {
            robot.driveController.moduleRight.motor2.setPower(1);
        }



        if (aButton) {
            robot.driveController.moduleLeft.motor1.setPower(1);
        }
        if (bButton) {
            robot.driveController.moduleLeft.motor2.setPower(-1);
        }

    }

    public void stop() {
        robot.driveController.updateUsingJoysticks(new Vector2d(0, 0), new Vector2d(0, 0), false);
        pidDrive.disable();
        super.stop();
    }

    public Vector2d checkDeadband(Vector2d joystick, boolean slowMode) {
        if (joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL)) {
            imuTarget = robot.getRobotHeading().getAngle();
        }
        else {
        }
        return  joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL) ?
                joystick : new Vector2d(0, 0);
    }

    public double pidController(double target, double current, double Kp, double Ki, double Kd) {
        error = target-current;
        errorChange = error-lastError;
        errorSum += error;
        double correction = Kp*error + Ki*errorSum + Kd*errorChange;
        lastError = error;
        return correction;
    }
}