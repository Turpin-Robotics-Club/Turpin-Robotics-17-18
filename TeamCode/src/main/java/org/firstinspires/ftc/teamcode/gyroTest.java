package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Gyro Test", group = "Sensors")
public class gyroTest extends OpMode {
    private BNO055IMU gyro;
    private Orientation angles;
    private double initAngle;

    public void init()
    {
        BNO055IMU.Parameters IMUparams = new BNO055IMU.Parameters();
        IMUparams.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparams.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparams.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUparams.loggingEnabled      = true;
        IMUparams.loggingTag          = "IMU";
        IMUparams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(IMUparams);
        telemetry.addData("IMUparams", IMUparams);
        sleep(500);
        angles  = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("angle", angles);

        initAngle = angles.thirdAngle;
    }
    public void loop()
    {
        telemetry.addData("offset",initAngle);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX , AngleUnit.DEGREES);
        telemetry.addData("all angles: ", angles);
        if(angles.thirdAngle-initAngle<-360)
            telemetry.addData("rotation", (angles.thirdAngle+720-initAngle));
        else if (angles.thirdAngle-initAngle<0)
            telemetry.addData("rotation", (angles.thirdAngle+360-initAngle));
        else
            telemetry.addData("rotation", (angles.thirdAngle-initAngle));
        telemetry.update();

    }

}
