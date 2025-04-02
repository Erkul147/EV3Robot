package carDrive;

import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.*;

public class Main {
    public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
    public static EV3IRSensor IRSensor = new EV3IRSensor(SensorPort.S4);
    public static RegulatedMotor leftMotor = new MindsensorsGlideWheelMRegulatedMotor(MotorPort.A);
    public static RegulatedMotor rightMotor = new MindsensorsGlideWheelMRegulatedMotor(MotorPort.B);

    private static long startTime;
    private static final long RUN_TIME = 30000;
    private static float startDistance;
    
    public static void driveForward() {
        leftMotor.setSpeed(500);
        rightMotor.setSpeed(500);
        leftMotor.forward();
        rightMotor.forward();
    }
    
    public static void stopMotors() {
        leftMotor.stop();
        rightMotor.stop();
    }

    public static void turn(int rotation) {
        leftMotor.setSpeed(1000);
        rightMotor.setSpeed(900);
        leftMotor.rotate(rotation, true);
        rightMotor.rotate(-rotation);
        stopMotors();
    }

    public static void borderDetection() {
        SampleProvider sp = colorSensor.getRedMode(); 
        float[] sample = new float[sp.sampleSize()];

        sp.fetchSample(sample, 0);
        float intensity = sample[0];

        System.out.println("Light Intensity: " + intensity);
        Delay.msDelay(100);

        if (intensity >= 0.01) { 
            stopMotors();
            leftMotor.backward();
            rightMotor.backward();
            Delay.msDelay(2000);
            stopMotors();
            turn(1650); 
        }
    }

    public static void avoidCollision() {
        SampleProvider sp = IRSensor.getDistanceMode();
        float[] sample = new float[sp.sampleSize()];
        
        sp.fetchSample(sample, 0);
        float distance = sample[0];
        
        if (distance < 15) { 
            stopMotors();
            leftMotor.backward();
            rightMotor.backward();
            Delay.msDelay(1000);
            stopMotors();
            turn(1650); 
        }
    }

    public static void returnToStart() {
        SampleProvider sp = IRSensor.getDistanceMode();
        float[] sample = new float[sp.sampleSize()];

        stopMotors();
        turn(1650);

        while (true) {
            sp.fetchSample(sample, 0);
            float distance = sample[0];

            if (distance <= startDistance) { 
                stopMotors();
                Sound.beepSequence();
                break;
            } else {
                driveForward();
            }
        }
    }

    public static void main(String[] args) {
        startTime = System.currentTimeMillis();
        SampleProvider sp = IRSensor.getDistanceMode();
        float[] sample = new float[sp.sampleSize()];
        sp.fetchSample(sample, 0);
        startDistance = sample[0];

        while ((System.currentTimeMillis() - startTime) < RUN_TIME) {
            driveForward(); 
            borderDetection();
            avoidCollision();
            if (Button.ENTER.isDown()) {
                break;
            }
        }
        
        Sound.beep();
        returnToStart();
        
        leftMotor.stop();
        rightMotor.stop();
        colorSensor.close();
        IRSensor.close();
    }
}
