package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.move;

@Autonomous(name="A C 0", group="Autonomous Finals")
//@Disabled
public class driveForward extends LinearOpMode{



    public void runOpMode()
    {


        move drive = new move(this);
        int count = 0;
        while (RobotConstants.vuMark==RelicRecoveryVuMark.UNKNOWN&&opModeIsActive()&&count<20)
        {
            drive.forward(3,0.23);
            count++;
        }
        drive.forward(26-(2.5*count),.2);
        drive.right(-9, -0.5);
        drive.toColumn();
        drive.forward(-2 ,-0.3);
        drive.liftZero();
        while (opModeIsActive());
    }
}
