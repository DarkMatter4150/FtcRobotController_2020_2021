package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.coyote.path.PathPoint;
import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.PIDController;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.opmodes.TestCameraT265.slamra;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "State Machine TeleOp", group = "TeleOp")
public class StateMachineTeleOp extends OpMode {

    public enum RobotState {
        MAIN,
        DRIVE_TO_POS_ONE,
        SHOOT,
        DRIVE_TO_POS_TWO
    };
    RobotState robotState = RobotState.MAIN;
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

    public static double SPEED = .5;
    public static double PVALUE = .05;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    double startingX = 0;
    double startingY = 0;
    double startingTheta = 0;

    double currentX = 0;
    double currentY = 0;
    double currentTheta = 0;

    double setPoint = 0;
    Pose currentPose = new Pose(startingX, startingY, startingTheta);

    private boolean slow = false;
    private boolean claw = false;
    private boolean pusherThing = true;
    private boolean boxLifter = true;
    private double boxTimer = 0;
    private double pusherTimer = 0;
    private ElapsedTime timer = new ElapsedTime();
    public Path current_path;

    //hungry hippo (1 servo, 2 positions)
    //foundation grabber - latch (2 servos, 2 positions)
    //lift (2 motors, continuous)
    //intake (2 motors, continuous)
    //arm (2 servos, continuous)
    //grabber (1 servo, 2 positions)

    double lastTime;



    public void init() {
        robot = new Robot(this, false, false);
        updateSLAMNav();
    }


    Vector2d joystick1, joystick2;

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
        boolean dPad1right = gamepad1.dpad_right;
        boolean dPad1left = gamepad1.dpad_left;
        boolean dPad2right = gamepad2.dpad_right;


        loopStartTime = System.currentTimeMillis();

        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        joystick2 = new Vector2d(gamepad1.right_stick_x+((gamepad1.right_stick_x/Math.abs(gamepad1.right_stick_x))*0.1), -gamepad1.right_stick_y); //RIGHT joystick
        slowModeDrive = false;

        switch (robotState) {
            case MAIN:
                if (dPad1right) {
                    robotState = RobotState.DRIVE_TO_POS_ONE;
                    break;
                } else if (dPad1left){
                    robotState = RobotState.DRIVE_TO_POS_TWO;
                    break;
                }
                else if (dPad2right) {
                    robotState = RobotState.SHOOT;
                    break;
                } else {
                        updateSLAMNav();
                        robot.setIntakePower(leftTrigger2);
                        robot.setConveyorPower(-leftTrigger2);
                        robot.setFlywheelPower((float) (-rightTrigger2*.8));

                        if (xButton) {
                            robot.setFlywheelPower(-1);
                        }
                        else {
                            robot.setFlywheelPower((float) (-rightTrigger2*.78));
                        }

                        if (yButton) {
                            robot.setIntakePower(-1);
                            robot.setConveyorPower(1);
                        }


                        if (aButton && timer.milliseconds() - pusherTimer > 250 && (boxLifter || pusherThing)) {
                            pusherThing = !pusherThing;
                            robot.setPusherThing(pusherThing);
                            pusherTimer = timer.milliseconds();
                        }
                        if (bButton && timer.milliseconds() - boxTimer > 250) {
                            boxLifter = !boxLifter;
                            robot.setBoxLifter(boxLifter);
                            boxTimer = timer.milliseconds();
                        }


                        telemetry.addData("Robot Heading: ", robot.getRobotHeading());

                        telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
                        telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));

                        telemetry.addData("Left Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation());
                        telemetry.addData("Right Orientation: ", robot.driveController.moduleRight.getCurrentOrientation());



                        //slow mode/range stuffs
                        if (gamepad1.left_trigger > 0.1) {
                            // joystick1 = joystick1.scale(0.3);
                            // joystick2 = joystick2.scale(0.4); //was 0.3
                            joystick1 = joystick1.scale((1-Math.abs(gamepad1.left_trigger))*.75);
                            joystick2 = joystick2.scale(1-Math.abs(gamepad1.left_trigger));
                            slowModeDrive = true;
                        }


                        if (usePIDforMovement) {
                            if (joystick1.getMagnitude() >= .1 && joystick2.getMagnitude() <= .1) {
                                pidDrive.setSetpoint(imuTarget);
                                //double pidTurn = pidController(imuTarget, robot.getRobotHeading().getAngle(), 2,1,1);
                                double pidTurn = pidDrive.performPID(robot.getRobotHeading().getAngle());
                                joystick2 = new Vector2d(joystick2.getX()-pidTurn, joystick2.getY());
                            }

                        }

                        robot.driveController.updateUsingJoysticks(
                                checkDeadband(joystick1, slowModeDrive).scale(Math.sqrt(2)),
                                checkDeadband(joystick2, slowModeDrive).scale(Math.sqrt(2)),
                                absHeadingMode
                        );


                        if (gamepad1.dpad_left) {
                            robot.driveController.setDrivingStyle(true);
                        } else if (gamepad1.dpad_right) {
                            robot.driveController.setDrivingStyle(false);
                        }

                        //todo: remove after done tuning
                        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


                        telemetry.addData("joystick 1", joystick1);
                        telemetry.addData("joystick 2", joystick2);

                        loopEndTime = System.currentTimeMillis();
                        telemetry.addData("Our loop time: ", loopEndTime - loopStartTime);
                        telemetry.addData("imuTarget", imuTarget);

                        telemetry.update();
                    break;
                }
            case SHOOT:
                robot.setBoxLifter(false);
                long setTime = System.currentTimeMillis();
                while (System.currentTimeMillis()- setTime < 1500) {
                    robot.setFlywheelPower(1);
                }
                robot.setPusherThing(true);
                robot.setPusherThing(false);
                robot.setPusherThing(true);
                robot.setPusherThing(false);
                robot.setPusherThing(true);
                robot.setPusherThing(false);
                robotState = RobotState.MAIN;
                break;
            case DRIVE_TO_POS_ONE:
                Path path = new Path().addPoint(new PathPoint(20, 53)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
                followCurvePath(path, .8*Math.sqrt(2), 0.08);
                robotState = RobotState.MAIN;
                break;
            case DRIVE_TO_POS_TWO:
                path = new Path().addPoint(new PathPoint(20, 0)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
                followCurvePath(path, .8*Math.sqrt(2), 0.08);
                robotState = RobotState.MAIN;
                break;
                default:
                    robotState = RobotState.MAIN;
                    break;

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

    public void updateSLAMNav() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);

        currentX = startingX + -translation.getY();
        currentY = startingY + translation.getX();
        currentTheta = startingTheta - rotation.getDegrees();
        currentPose = new Pose(currentX, currentY, currentTheta);

        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        telemetry.addData("Rotation", currentTheta);
        telemetry.update();
    }


    public void followCurvePath(Path path, double speed, double pvalue) {
        this.current_path = path;

        while (true) {

            if (!path.isComplete()) {
                updateSLAMNav();
                robot.updateBulkData();

                // Get our lookahead point
                path.update(currentPose);
                Pose lookahead_pose = path.getFollowPose();

                // Get the distance to our lookahead point
                double distance = Math.hypot(lookahead_pose.x-currentPose.x, lookahead_pose.y-currentPose.y);

                double speedFinal;
                // Find our drive speed based on distance
                if (distance < current_path.getFollowCircle().radius) {
                    speedFinal = Range.clip((distance / 8) + 0.1, 0, 1)*speed;
                } else {
                    speedFinal = speed;
                }

                // Find our turn speed based on angle difference
                double headingg = path.getHeadingGoal(Path.HeadingMethod.CONSTANT_ANGLE,currentPose);
                double turn_speed = Range.clip(Math.abs(currentPose.angle -lookahead_pose.angle) / (Math.PI/4) + 0.1, 0, 1);

                // Drive towards the lookahead point
                driveUsingPurePursuit(lookahead_pose, path, speedFinal, turn_speed, pvalue);
            }
            else {
                robot.driveController.update(Vector2d.ZERO, 0);//mvmt_a);
                break;
            }

        }
        robot.driveController.update(Vector2d.ZERO, 0);//mvmt_a);

    }

    public void driveUsingPurePursuit(Pose pose, Path path, double drive_speed, double turn_speed, double pvalue) {

        // Find the angle to the pose
        Vector2d directionPP = new Vector2d((pose.x - currentPose.x), (pose.y-currentPose.y)).normalize(drive_speed);

        // Find movement vector to drive towards that point
        double mvmt_a = -Math.signum(Range.clip((pose.angle - currentPose.angle - (Math.PI/2)), -1, 1)) * turn_speed;
        mvmt_a = mvmt_a *pvalue;//- pidController(path.getHeadingGoal(path.heading_method,pose), currentPose.angle,.0000000000004,0,0);



        // Update actual motor powers with our movement vector

        robot.updateBulkData();
        updateSLAMNav();
        robot.driveController.updateTracking();
        telemetry.addData("Driving robot", "");
        telemetry.addData("Current X: ", currentX);
        telemetry.addData("Current Y: ", currentY);
        telemetry.addData("Current Theta: ", currentTheta);
        telemetry.update();

        robot.driveController.updatePositionTracking(telemetry); //update position tracking
        robot.driveController.update(directionPP, mvmt_a);

    }

}