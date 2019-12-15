package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoModeLeftBlue", group="Autonomous")
public class AutoMode extends LinearOpMode {





    //call center() to get the center from the camera
    @Override
    public void runOpMode() throws InterruptedException {
        //Intended to start at the pin
        waitForStart();
        Robot.init_autoOp(telemetry, hardwareMap,gamepad1,gamepad2);


        //maybe move forward first?

        moveDirMax(0,-1,0,100,1000);

        moveDirMax(1,0,0,5500,14000);

        ElapsedTime outTime = new ElapsedTime();
        while(outTime.milliseconds() < 500) {
            Robot.outtake();
        }
        Robot.reset_pow();
        moveDirMax(-1,0,0,5500,14000);
        //will need to d
        moveDirMax(0,1,0,500,1000);


        moveDirMax(0,-1.3,0,2000,14000); //coudl maket his shorter?

        //moveDir(0,.5,0,2000,14000); //should go back to start?
        //need to sleep for short time after each of these motions

        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -.7, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*500)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!checkDone(2000)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake();
            }
             Robot.reset_pow();
        }
        moveDirMax(0,1.3,0,1400,14000);
        moveDirMax(1,0,0,5000,14000);

        {
            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 1000) {
                Robot.pantagraphUp(pantTime);
                //and move back maybe

            }
        }
        Robot.reset_pow();
        moveDirMax(0,1.3,0,1000,2000);//move back
        moveDirMax(-1,0,0,1000,14000);

        /*
        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -.1, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*500)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!checkDone(2000)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake();
            }
            Robot.reset_pow();
        }
        */


        moveDirMax(0,1,0,1000,2000);

        //extra block
        //DONT NEED THIS
        moveDirMax(0,-1,0,100,2000);

        moveDirMax(-1,0,0,9500,2000);

        moveDirMax(0,-1.3,0,2000,14000);



        //moveDir(0,.5,0,2000,14000); //should go back to start?
        //need to sleep for short time after each of these motions
        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -.7, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*500)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!checkDone(2000)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake();
            }
            Robot.reset_pow();
        }

        moveDirMax(0,1.3,0,2000,14000);
        moveDirMax(1,0,0,5500,14000);

        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(1, 0, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*1000)); //can amke this faster
            }
            Robot.setMotorsMax(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!checkDone(2000)) { //need to check if some are done
                Robot.setMotorsMax(powers, 1);
                Robot.pantagraphUp(pantTime);
                Robot.outtake();
            }
            Robot.reset_pow();
        }
        moveDirMax(-1,0,0,1900,14000);
        //maybe push out and move thing up while doing this
        //will I have enough time???
        //maybe need to drop off and move at the same time


        //moveDir(-2,0,0,4000,2000); //move back
        //moveDir(0,0,1,100,1500);




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
