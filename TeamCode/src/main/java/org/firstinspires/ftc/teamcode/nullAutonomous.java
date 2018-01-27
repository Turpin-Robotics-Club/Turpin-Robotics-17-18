package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.move;

@Autonomous(name="AB AB 0", group="Autonomous Finals")
//@Disabled
public class nullAutonomous extends LinearOpMode {
    @Override
    public void runOpMode(){

        new move(this);
        Sensors.vuMark();
        while (opModeIsActive())
        {
            telemetry.addData("reddish",move.getReddish());
            telemetry.addData("VuMark", RobotConstants.vuMark/*== RelicRecoveryVuMark.RIGHT*/);
            telemetry.addData("Heading", Sensors.readGyro());
            telemetry.addData("Gyro Heading", Sensors.angles.thirdAngle);
            telemetry.addData("Offset rate", Sensors.gyrochange);
            telemetry.update();
        }
    }
}
