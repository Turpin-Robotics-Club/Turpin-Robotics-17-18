package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.move;

@Autonomous(name="A C 0", group="Autonomous Finals")
//@Disabled
public class driveForward extends LinearOpMode{



    public void runOpMode()
    {


        move drive = new move(this);
        drive.turnRight(5, 0.2);
        drive.lowerRaisin();
        sleep(1000);
        drive.raisein();
        drive.turnRight(-5, 0.2);
        drive.turnRight(-25,0.2);
        sleep(200);
        drive.turnRight(20,0.2);
        drive.forward(13,0.75);
        drive.right(-15, -0.75);
        drive.toColumn();
        drive.release();
        drive.forward(-6,-0.5);

    }
}
