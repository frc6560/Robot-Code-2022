// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotIds;

import java.awt.Color;

public class RGBLighting extends SubsystemBase {
  // a dictionary of all the colors
  private HashMap<String, boolean[]> colorsDict = new HashMap<>();
  private Solenoid[] ledLights = new Solenoid[3];

  private boolean climb = false;

  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

  Debouncer climbStopDebouncer = new Debouncer(6.0, DebounceType.kBoth);
  Debouncer shooterReadyDebouncer = new Debouncer(0.3, DebounceType.kRising);

  /** Creates a new RGBLighting. */
  public RGBLighting() {
    ledLights[0] = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.LED_LIGHT_RED);
    ledLights[1] = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.LED_LIGHT_GREEN);
    ledLights[2] = new Solenoid(PneumaticsModuleType.CTREPCM, RobotIds.LED_LIGHT_BLUE);

    colorsDict.put("red", new boolean[] {true, false, false});
    colorsDict.put("green", new boolean[] {false, true, false});
    colorsDict.put("blue", new boolean[] {false, false, true});
    colorsDict.put("yellow", new boolean[] {true, true, false});
    colorsDict.put("purple", new boolean[] {true, false, true});
    colorsDict.put("cyan", new boolean[] {false, true, true});
    colorsDict.put("white", new boolean[] {true, true, true});
    colorsDict.put("black", new boolean[] {false, false, false});

  }

  @Override
  public void periodic() {
    boolean shooterReady = ntInst.getTable("Shooter").getEntry("Shooter Ready").getBoolean(false);
    double climbExtension = ntInst.getTable("Climb").getEntry("Left Climb Pos").getDouble(0.0);
    double climbVelocity = ntInst.getTable("Climb").getEntry("Left Climb Vel").getDouble(0.0);

    


    if(climb && climbStopDebouncer.calculate(Math.abs(climbVelocity) < 2)) {
      // setColor("rainbow");
      setColor("cyan");
    } else if(climb){
      setColor("purple");
    } else if (shooterReadyDebouncer.calculate(shooterReady)) {
      setColor("green");
    } else {
      setColor("red");
    }

    if(climbExtension > 3.0){
      climb = true;
    }
    
  }

  public void setColor(String color) {
    // set the color of the lights
    if (color.toLowerCase().equals("rainbow")) {
      rainbow(60.0);
    } else {
      boolean[] colorArray = colorsDict.get(color);
      if (colorArray != null) {
        ledLights[0].set(colorArray[0]);
        ledLights[1].set(colorArray[1]);
        ledLights[2].set(colorArray[2]);
      }
    }
  }

  private double lastTime = Timer.getFPGATimestamp();
  public void rainbow(double length) {
    double initTime = Timer.getFPGATimestamp();
    if (Timer.getFPGATimestamp() - initTime < length) {
      double delta = Timer.getFPGATimestamp() - lastTime;
      if (delta > 0.5) {
        lastTime = Timer.getFPGATimestamp();
        double hue = (Timer.getFPGATimestamp() % 1.0) * 360;
        int[] rgb = convertRGB(Color.HSBtoRGB((float) hue, (float) 1.0, (float) 1.0));
        ledLights[0].set((rgb[0] > 0.5));
        ledLights[1].set((rgb[1] > 0.5));
        ledLights[2].set((rgb[2] > 0.5));
      }
    }
  }

  public int[] convertRGB(int in) {
    Color color = new Color(in);
    return new int[] {color.getRed(), color.getGreen(), color.getBlue()};
  }

  public void blinkColor(String color, double frequency) {
    // set the color of the lights
    boolean[] colorArray = colorsDict.get(color);
    Solenoid ledRed = ledLights[0];
    Solenoid ledGreen = ledLights[1];
    Solenoid ledBlue = ledLights[2];
    
    Debouncer debouncerRed = new Debouncer(frequency, DebounceType.kBoth);
    Debouncer debouncerGreen = new Debouncer(frequency, DebounceType.kBoth);
    Debouncer debouncerBlue = new Debouncer(frequency, DebounceType.kBoth);

    // blink the lights
    if(colorArray[0]) ledRed.set(debouncerRed.calculate(!ledRed.get()));
    if(colorArray[1]) ledGreen.set(debouncerGreen.calculate(!ledGreen.get()));
    if(colorArray[2]) ledBlue.set(debouncerBlue.calculate(!ledBlue.get()));
  }
}
