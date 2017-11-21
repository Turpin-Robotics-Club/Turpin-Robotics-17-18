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
        move obj = new move(this,true);
        Sensors.readGyro();
    }
}
