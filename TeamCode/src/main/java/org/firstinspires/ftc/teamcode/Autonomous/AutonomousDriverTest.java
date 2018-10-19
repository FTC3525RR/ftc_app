package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;
@Autonomous (name = "Auto Driver test")

public class AutonomousDriverTest extends LinearOpMode {

    public void runOpMode(){

        AutonomousLibrary al = new AutonomousLibrary();
        RobotHardware rh = new RobotHardware();
        CommonLibrary cl = new CommonLibrary();
        rh.init(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()){

            //al.encoderTicksToInchesTest(telemetry, this);
            al.turnOnWheels();

        }

    }
}
