package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.text.DecimalFormat;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class AutoV2 extends OpMode{

    // CONFIGURATION

    // Expansion Hub 1:
    // Motors:
    // Port 0: manArm
    // Servos
    // Port 0: leHand <-- Still needed
    // Sensors:
    // I2C Bus 0: Expansion Hub IMU
    // I2C Bus 1: frontDistance

    // Expansion Hub 2:
    // Motors:
    // Port 0: frontLeft
    // Port 1: rearLeft
    // Port 2: frontRight
    // Port 3: rearRight

    public DecimalFormat format = new DecimalFormat("##.00");
    private BNO055IMU imu;
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private DcMotor manArm;

    private boolean haveBlock = false;
    private boolean atAngle = false;

    private Servo elbow;
    private Servo hand;

    private DistanceSensor frontDistance;
    private DistanceSensor rearDistance;

    private String driveState = "driveForward"; // <-- Using string comparators to better explain...
                                        // ...what is happening with the autoState

    private String armState = "0block"; // <-- Using string comparators to better explain...
                                       // ...what is happening with the armState

    private int gyroCalibratedCount = 0;

    public Orientation angles;
    public Acceleration gravity;
    public double scalingFactor = 0.85;

    public double targetRotation;
    public double targetY;
    public double targetX;
    public float currHeading;


    @Override
    public void init(){
        motorInit();
        gyroInit();
        sensorInit();
        hand.setPosition(0.3);
    }



    @Override
    public void init_loop(){
        gyroLoop();
    }

    @Override
    public void loop(){
        gyroLoop();
        driveLoop();
        measureDistance();
        moveArm();
        //setDrivePower();

    }

    private void motorInit(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        manArm = hardwareMap.get(DcMotor.class, "manArm");

        hand = hardwareMap.get(Servo.class, "leHand");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        manArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        manArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //telemetry.addData("MOTORS", "Initialized");
    }

    private void gyroInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }

    private void sensorInit() {
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
    }

    private void gyroLoop() {
        //telemetry.addData("IMU", imu.isGyroCalibrated() ? "Initialized" : "Initializing...");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("ANGLE", angles);
        //telemetry.addData("HEADING", angles.firstAngle);
    }

    private void driveLoop(){
      if(Math.round(currHeading) >= 90){
        atAngle = true;
        targetRotation = 0;
      }
      if(!atAngle){
        targetRotation = 0.8;
      }
      setDrivePower();
      telemetry.addData("currHeading", currHeading);
    }

    private void moveClaw() {
      haveBlock = false;
      hand.setPosition(0.3);
    }

    private void setDrivePower() {
        currHeading = Math.round(angles.firstAngle);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final double rotation = -Math.pow(targetRotation, 3.0) * -0.5;
        final double y = Math.pow(targetY, 3.0) * -0.5;
        final double x = -Math.pow(targetX, 3.0) * -0.5;

        final double direction = Math.atan2(x, y) + 0.0;
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final float brake = 1.0f;

        final double lf = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
        final double rf = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;
        final double lr = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
        final double rr = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;

        frontLeft.setPower(-lf);
        frontRight.setPower(-rf);
        rearLeft.setPower(lr);
        rearRight.setPower(rr);
    }

    private void moveArm() {
        //telemetry.addData("ManArm", manArm.getCurrentPosition());
        switch(armState){
          case "0block":
          manArm.setTargetPosition(0);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0 && !haveBlock) {
            haveBlock = true;
            hand.setPosition(1);
            if(hand.getPosition() == 1) {
              armState = "1block";
            }
          }
          break;

          case "1block":
          manArm.setTargetPosition(420);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(1);
          }

          break;

          case "2block":
          manArm.setTargetPosition(600);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(1);
          }

          break;

          case "3block":
          manArm.setTargetPosition(880);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(1);
          }

          break;
        }
        manArm.setPower(0.3);
        manArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void measureDistance() {
        telemetry.addData("frontDistance", String.format("%.01f cm", frontDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("rearDistance", String.format("%.01f cm", rearDistance.getDistance(DistanceUnit.CM)));
        // telemetry.addData("FL", frontLeft.getCurrentPosition());
        // telemetry.addData("BL", rearLeft.getCurrentPosition());
        // telemetry.addData("FR", frontRight.getCurrentPosition());
        // telemetry.addData("BR", rearRight.getCurrentPosition());
        // telemetry.addData("driveState:", driveState);
  }


}
