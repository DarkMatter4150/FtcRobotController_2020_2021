package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.realsenseloader.RealsenseManager;

public class RealsenseLoader implements AbstractSubsystem {



    public RealsenseLoader(HardwareMap hardwareMap) {
        RealsenseManager.init(hardwareMap);
    }



    @Override
    public void update() { }



    @Override
    public void cleanup() {
        assert RealsenseManager.slamera != null;
        //RealsenseManager.slamera.exportRelocalizationMap(Environment.DIRECTORY_DOWNLOADS);
        RealsenseManager.slamera.stop();
    }
}
