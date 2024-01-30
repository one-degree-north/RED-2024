// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.VirtualSubsystem;

public class LEDs extends VirtualSubsystem {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final Swerve m_swerve;
  private final Notifier loadingNotifier;

  private Alliance alliance = null;
  private Color teleopColor;
  private Color autoColor;

  // Constants
  private static final int length = 60;
  private static final double waveExponent = 0.4;
  private static final double waveCycleLength = 30;
  private static final double waveFastCycleDuration = 0.25;
  private static final double waveSlowCycleDuration = 3;
  private static final double breathFastDuration = 0.25;
  private static final double breathSlowDuration = 1;
  
  private Supplier<Pose2d> autoPose;

  // States
  private SubsystemState visionState = SubsystemState.NOTREADY;
  private SubsystemState autoAlignState = SubsystemState.NOTREADY;
  private SubsystemState[] subsystems = {visionState, autoAlignState};
  
  /** Creates a new LEDs. */
  public LEDs(int port, Swerve swerve, Supplier<Pose2d> autoPose) {
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_swerve = swerve;
    this.autoPose = autoPose;
    
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    Section.UNDERGLOW,
                    Color.kWhite,
                    Color.kBlack,
                    breathFastDuration,
                    System.currentTimeMillis() / 1000.0);
                m_led.setData(m_ledBuffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public synchronized void periodic() {
    loadingNotifier.stop();

    if (DriverStation.isFMSAttached() || DriverStation.isDSAttached()) {
      alliance = DriverStation.getAlliance().get();
    }
    if (alliance != null) {
      switch (alliance) {
        case Red:
          teleopColor = Color.kRed;
          autoColor = Color.kYellow;
          break;
        case Blue:
          teleopColor = Color.kBlue;
          autoColor = Color.kPurple;
          break;
        default:
          teleopColor = Color.kBlack;
          autoColor = Color.kBlack;
          break;
      }
    }
    

    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      
      /* Vision and AutoAlign states use the same LEDs, hence they share logic */
      if (m_swerve.getTagSeenSinceLastDisable()){
        breath(Section.UNDERGLOW, Color.kGreen, Color.kBlack, breathSlowDuration);
        visionState = SubsystemState.READY;
      }
      else if (m_swerve.allCamerasEnabled()){
        breath(Section.UNDERGLOW, Color.kWhite, Color.kBlack, breathSlowDuration);
        visionState = SubsystemState.NOTREADY;
      }
      else {
        solid(Section.UNDERGLOW, Color.kWhite);
        visionState = SubsystemState.NOTREADY;
      }
      
      if (visionState == SubsystemState.READY) 
        autoAlign();

      /* Logic underneath here checks if all subsystems are ready */
      boolean allSubsystemsReady = true;
      for (SubsystemState s: subsystems) {
        if (s == SubsystemState.NOTREADY) {
          allSubsystemsReady = false;
          break;
        }
      }

      if (allSubsystemsReady) {
        wave(Section.FULL, Color.kGreen, Color.kBlack, waveCycleLength, waveFastCycleDuration, false);
      }

    }
  
    else if (DriverStation.isEnabled()) {
      /* Main logic for drivetrain colors when enabled */
      if (DriverStation.isTeleop()) {
        solid(Section.UNDERGLOW, teleopColor);
      } else if (DriverStation.isAutonomous()) {
        solid(Section.UNDERGLOW, autoColor);
      } else if (DriverStation.isTest()) {
        breath(Section.UNDERGLOW, teleopColor, Color.kBlack, breathSlowDuration);
      }
    }
  

    else if (DriverStation.isEStopped()) {
      breath(Section.FULL, Color.kHotPink, Color.kDeepPink, breathFastDuration);
    }

    m_led.setData(m_ledBuffer);
  }

  private void autoAlign() {
    double allowableError = 0.05;
    if (autoPose.get() != null) {
      Pose2d targetPose = autoPose.get();
      Pose2d currentPose = m_swerve.getPose();
      boolean xPoseAligned = false;
      boolean yPoseAligned = false;

      Pose2d relativePose = targetPose.relativeTo(currentPose);
      if (Math.abs(relativePose.getX()) <= allowableError) {
        solid(Section.FRONTDRIVE, Color.kBlack);
        solid(Section.BACKDRIVE, Color.kBlack);
        xPoseAligned = true;
      } else if (relativePose.getX() > allowableError) {
        solid(Section.FRONTDRIVE, Color.kRed);
        solid(Section.BACKDRIVE, Color.kBlack);
        xPoseAligned = false;
        autoAlignState = SubsystemState.NOTREADY;
      } else if (relativePose.getX() < -allowableError) {
        solid(Section.FRONTDRIVE, Color.kBlack);
        solid(Section.BACKDRIVE, Color.kRed);
        xPoseAligned = false;
        autoAlignState = SubsystemState.NOTREADY;
      }

      if (Math.abs(relativePose.getY()) <= allowableError) {
        solid(Section.LEFTDRIVE, Color.kBlack);
        solid(Section.RIGHTDRIVE, Color.kBlack);
        yPoseAligned = true;
      } else if (relativePose.getY() > allowableError) {
        solid(Section.LEFTDRIVE, Color.kRed);
        solid(Section.RIGHTDRIVE, Color.kBlack);
        yPoseAligned = false;
        autoAlignState = SubsystemState.NOTREADY;
      } else if (relativePose.getY() < -allowableError) {
        solid(Section.LEFTDRIVE, Color.kBlack);
        solid(Section.RIGHTDRIVE, Color.kRed);
        yPoseAligned = false;
        autoAlignState = SubsystemState.NOTREADY;
      }
      if (xPoseAligned && yPoseAligned) {
        if (Math.abs(MathUtil.angleModulus(relativePose.getRotation().getRadians())) <= allowableError) {
          solid(Section.LEFTDRIVE, Color.kBlack);
          solid(Section.RIGHTDRIVE, Color.kBlack);
          solid(Section.FRONTDRIVE, Color.kBlack);
          solid(Section.BACKDRIVE, Color.kBlack);
          autoAlignState = SubsystemState.READY;
        } else if (MathUtil.angleModulus(relativePose.getRotation().getRadians()) > allowableError) {
          wave(Section.UNDERGLOW, Color.kRed, Color.kBlack, waveCycleLength, waveSlowCycleDuration, true);
          autoAlignState = SubsystemState.NOTREADY;
        } else if (MathUtil.angleModulus(relativePose.getRotation().getRadians()) < -allowableError) {
          wave(Section.UNDERGLOW, Color.kRed, Color.kBlack, waveCycleLength, waveSlowCycleDuration, false);
          autoAlignState = SubsystemState.NOTREADY;
        }
      }
    }


  }

  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        m_ledBuffer.setLED(i, color);
      }
    }
  }

  // TODO: Test reverse wave written by copilot

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration, boolean reverse) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    int start = reverse ? section.end() - 1 : section.start();
    int end = reverse ? section.start() - 1 : section.end();
    int step = reverse ? -1 : 1;
    for (int i = start; reverse ? i > end : i < end; i += step) {
      x += xDiffPerLed;
      if (reverse ? i <= section.end() && i >= section.start() : i >= section.start() && i < section.end()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        m_ledBuffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  private void breath(Section section, Color c1, Color c2, double duration) {
    breath(section, c1, c2, duration, Timer.getFPGATimestamp());
  }

  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  private static enum SubsystemState {
    NOTREADY, READY
  }

  private static enum Section {
    FULL, UNDERGLOW, LEFTDRIVE, RIGHTDRIVE, FRONTDRIVE, BACKDRIVE;

    private int start() {
      switch (this) {
        case FULL:
          return 0;
        case UNDERGLOW:
          return 0;
        case LEFTDRIVE:
          return 0;
        case BACKDRIVE:
          return 10;
        case RIGHTDRIVE:
          return 20;
        case FRONTDRIVE:
          return 30;
        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case FULL:
          return length;
        case UNDERGLOW:
          return 40;
        case LEFTDRIVE:
          return 10;
        case BACKDRIVE:
          return 20;
        case RIGHTDRIVE:
          return 30;
        case FRONTDRIVE:
          return 40;
        default:
          return length;
      }
    }

  }
}
