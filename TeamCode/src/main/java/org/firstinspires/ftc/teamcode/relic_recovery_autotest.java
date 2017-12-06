package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.move;
import org.firstinspires.ftc.teamcode.utils.Sensors;
/**
 * Created by Patrick on 11/20/2017.
 */
@Autonomous (name="relic_autotest", group="Autonomous Finals")
public class relic_recovery_autotest extends LinearOpMode{
    public void runOpMode() throws InterruptedException{
        //dont use move class-use sensors and we always do it as red
        move robot = new move(this,true);
        waitForStart();
        /*Sensors.readGyro();
        while (opModeIsActive())
        {
            telemetry.addData("Heading", Sensors.readGyro());
            telemetry.addData("Gyro Heading", Sensors.angles.thirdAngle);
            telemetry.addData("Offset rate", Sensors.gyrochange);
            telemetry.update();
        }
        */
        robot.left(-1,.65);
        robot.forward(1.5,.7);
        robot.turnLeft(90);
        robot.forward(1.25,.7);
        sleep(100000);
    }
}
