package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="AutoModeTEST (DONT USE)", group="Autonomous")
public class AutoModeTest extends LinearOpMode {





    //call center() to get the center from the camera
    @Override
    public void runOpMode() throws InterruptedException {
        //Intended to start at the pin

        waitForStart();
        Robot.init_autoOp(telemetry, hardwareMap,gamepad1,gamepad2);


        //need to somehow constantly update the motors
        //maybe need to calculate the encoder count before hand
        //or think this through a little more
        //maybe edit the target position every certain amount of time
        //So do something maybe that reads the current position, and then changes the target number based on teh new angle
        //maybe just write some code for continually changing the direction, and then reupdate the mtoor power and stuff
        //I think if we just keep updating we should get the right value ifs we just change hte motor speed
        //The only problem will come if we swap fro mpositive to negative, but otherwise if we just update the speed it should work fine
        //maybe do some check of making sure its not going negative
        //


        //Here is an overview

        moveOrient(0,-1,1,0,2000,2000);
        moveOrient(-1,0,0,1,2000,2000);






        /*
        Robot.init_autoOp(telemetry, hardwareMap,gamepad1,gamepad2);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setPower(1);
            Robot.motors[i].setTargetPosition( (int) Math.abs((1000)) );
        }

         */
        //coudl do somethign about
        /*
        int mode = -1;

        Robot.resetTime();
        initMode0();

        telemetry.addData("Mode","0");
        telemetry.update();
        while(!mode0());
        initMode1();

        telemetry.addData("Mode","1");
        telemetry.update();
        while(!mode1());
        Robot.resetTime();
        Robot.moveVx(0,0,.3,10000,true);
        while(!mode1());

         */
        //maybe make this equal to teh game pad stuff, so we just mimic what woudl happen with game pad

    }


    public void sleep(int maxtime) {
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < maxtime) {
            //nothing
        }

    }
    public void moveOrient(double vx, double vy, double rot,int rot2, int dist, int max_time) {


        Robot.resetTime();
        double[] powers = Robot.orientMode(vx, vy, rot,rot2);
        double[] powers_orig = Robot.orientMode(vx, vy, rot,rot2);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotors(powers, 1);
        while (!checkDone(max_time)) { //need to check if some are done
            powers = Robot.orientMode(vx, vy, rot,rot2);
            boolean can_change = true;
            for (int i = 0; powers.length > i; i++) {
                if ( (powers[i] == 0 || powers_orig[i] == 0) || (Math.abs(powers[i])/powers[i] != Math.abs(powers_orig[i])/powers_orig[i])  ) {
                  can_change = false;
                }
            }
            if(can_change) {
                Robot.setMotors(powers, 1);
            }
        }
        Robot.reset_pow();
        sleep(100);

    }
    public void moveDir(double vx, double vy, double rot,int dist, int max_time) {


        Robot.resetTime();
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotors(powers, 1);
        while (!checkDone(max_time)) { //need to check if some are done
            Robot.setMotors(powers, 1);
        }
        Robot.reset_pow();
        sleep(100);

    }
    public void moveDirMax(double vx, double vy, double rot,int dist, int max_time) {


        Robot.resetTime();
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotorsMax(powers, 1);
        while (!checkDone(max_time)) { //need to check if some are done
            Robot.setMotorsMax(powers, 1);
        }
        Robot.reset_pow();
        sleep(100);
    }
    public void initMode0() {

        //nothign for now
        Robot.resetTime();
        Robot.moveVx(.5,1,10000,true);


    }
    public void initMode1() {
        //nothign for now
        Robot.resetTime();
        Robot.moveVx(-.3,.7,10000,true);


    }

    public boolean checkDone(int maxTime) { //true if done
        //return true when done
        if (Robot.timeElapsed() > maxTime ) { //max amount of time alloted
            telemetry.addData("out of time","time ran out");
            return true;
        }
        if(isStopRequested()) {
            throw new RuntimeException();
            //should force it to stop
            //return true;
        }
        if(Robot.motorsFinished() >= 4 && Robot.timeElapsed() > 150) { //150 milli before it will end
            return true; //expiermental code here
        }
       /*
       if(Robot.motorsFinished() >= 2) {
           return true;
           //then done
       }

        */


        return false;
    }




}
