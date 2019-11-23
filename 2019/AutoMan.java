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

public class AutoMan extends OpMode{

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

    private Servo elbow;
    private Servo hand;

    private DistanceSensor frontDistance;

    private String driveState = "step1"; // <-- Using string comparators to better explain...
                                        // ...what is happening with the autoState

    private String armState = "0block"; // <-- Using string comparators to better explain...
                                       // ...what is happening with the armState

    private int gyroCalibratedCount = 0;

    public Orientation angles;
    public Acceleration gravity;

    @Override
    public void init(){
        motorInit();
        gyroInit();
        sensorInit();
        hand.setPosition(0.5);
    }

    @Override
    public void init_loop(){
        gyroLoop();
    }

    @Override
    public void loop(){
        gyroLoop();
        measureDistance();
        driveLoop();
        moveArm();
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

        hand.setPosition(0.5);
        telemetry.addData("MOTORS", "Initialized");
    }

    private void gyroInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }

    private void sensorInit() {
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
    }

    private void gyroLoop() {
        telemetry.addData("IMU", imu.isGyroCalibrated() ? "Initialized" : "Initializing...");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("ANGLE", angles);
        telemetry.addData("HEADING", angles.firstAngle);
    }

    private void driveLoop(){

        switch (driveState){
            case "step1":
                frontLeft.setTargetPosition(-2150);
                frontRight.setTargetPosition(-2150);
                rearLeft.setTargetPosition(2150);
                rearRight.setTargetPosition(2150);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.5);
                frontRight.setPower(0.5);
                rearLeft.setPower(0.5);
                rearRight.setPower(0.5);

                if (frontLeft.getCurrentPosition()<= -2100){
                    driveState = "spinCycle";
                }

            break;

            case "spinCycle":
                frontLeft.setTargetPosition(-2700);
                frontRight.setTargetPosition(-1500);
                rearLeft.setTargetPosition(2700);
                rearRight.setTargetPosition(1500);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                rearLeft.setPower(0.5);
                rearRight.setPower(-0.5);

                if (frontLeft.getCurrentPosition()<= -2700){
                    driveState = "chargeBackwards";
                }
            break;

            case "chargeBackwards":
                frontLeft.setTargetPosition(-900);
                frontRight.setTargetPosition(300);
                rearLeft.setTargetPosition(800);
                rearRight.setTargetPosition(-400);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(-0.5);
                frontRight.setPower(-0.5);
                rearLeft.setPower(-0.5);
                rearRight.setPower(-0.5);

                if (frontLeft.getCurrentPosition() >= -900){
                  driveState = "180oogabooga";
                }
            break;

            case "180oogabooga":
              frontLeft.setTargetPosition(400);
              frontRight.setTargetPosition(-850);
              rearLeft.setTargetPosition(-400);
              rearRight.setTargetPosition(1000);

              frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              frontLeft.setPower(-0.5);
              frontRight.setPower(0.5);
              rearLeft.setPower(-0.5);
              rearRight.setPower(0.5);

              if (frontLeft.getCurrentPosition() >= 300){
                driveState = "stackBlock";
              }
            break;

            case "stackBlock":
              moveClaw();
              if (hand.getPosition() == 0.5){
                driveState = "backUp";
              }
              //stacking blocks will be in this case
              // setDrivePower();
            break;

            case "backUp":
            //   frontLeft.setTargetPosition(2500);
            //   frontRight.setTargetPosition(1250);
            //   rearLeft.setTargetPosition(-2500);
            //   rearRight.setTargetPosition(-200);

            //   frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //   frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //   rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //   rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //   frontLeft.setPower(-0.5);
            //   frontRight.setPower(0.5);
            //   rearLeft.setPower(0.5);
            //   rearRight.setPower(-0.5);

            // if (frontLeft.getCurrentPosition() >= 2600 ){
            //   armState = "0block";
            // }

            break;
        }
    }

    private void moveClaw() {
      hand.setPosition(0.5);
    }

    private void moveArm() {
        telemetry.addData("ManArm", manArm.getCurrentPosition());
        switch(armState){
          case "0block":
          manArm.setTargetPosition(110);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(0);
            if(hand.getPosition() == 0) {
              armState = "1block";
            }
          }
          break;

          case "1block":
          manArm.setTargetPosition(500);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(0);
          }

          break;

          case "2block":
          manArm.setTargetPosition(800);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(0);
          }

          break;

          case "3block":
          manArm.setTargetPosition(1100);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(0);
          }

          break;
        }
        manArm.setPower(0.75);
        manArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void measureDistance() {
        telemetry.addData("frontDistance", String.format("%.01f cm", frontDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("BL", rearLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.addData("BR", rearRight.getCurrentPosition());
        telemetry.addData("driveState:", driveState);
  }


}
