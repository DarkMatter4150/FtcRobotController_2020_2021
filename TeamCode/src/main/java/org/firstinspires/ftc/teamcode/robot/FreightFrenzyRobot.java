package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.robot.subsystems.Capper;
import org.firstinspires.ftc.teamcode.robot.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.OdometryDeploy;
import org.firstinspires.ftc.teamcode.robot.subsystems.RealsenseLoader;
import org.firstinspires.ftc.teamcode.robot.util.LynxModuleUtil;

/**
 * The 2021-2022 robot class
 */
public class FreightFrenzyRobot extends AbstractRobot {
    public final Drivetrain drivetrain;
    public final Carousel carousel;
    public final Lift lift;
    public final Intake intake;
    public final Bucket bucket;
    public final RealsenseLoader odometry;
    public final Capper capper;
    public final OdometryDeploy deployer;


    /**
     * Set up the robot and initialize the subsystems you want to use
     * @param hardwareMap Passed in from the op mode
     */
    public FreightFrenzyRobot(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        odometry = new RealsenseLoader(hardwareMap);
        addSubsystem(odometry);

        // Set up the drivetrain
        drivetrain = new Drivetrain(hardwareMap);
        addSubsystem(drivetrain);

        // Set up the carousel motor
        carousel = new Carousel(hardwareMap);
        addSubsystem(carousel);

        // Set up the intake
        intake = new Intake(hardwareMap);
        addSubsystem(intake);

        // Set up the intake
        bucket = new Bucket(hardwareMap);
        addSubsystem(bucket);

        capper = new Capper(hardwareMap);
        addSubsystem(capper);

        // Set up the lift (it needs access to the bucket)
        lift = new Lift(hardwareMap,bucket,intake,capper);
        addSubsystem(lift);

        //Add the odometry deployer subsystem
        deployer = new OdometryDeploy(hardwareMap);
        addSubsystem(deployer);

    }
}
