package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.newMove;
/**
 * Created by Patrick on 9/21/2017.
 */

public class tester extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        newMove drive = new newMove(this);

        waitForStart();
        Sensors.gyroDriftRead();

        drive.left(20, 1);
    }
}
