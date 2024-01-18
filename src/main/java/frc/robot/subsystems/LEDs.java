// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private static final double waveCycleLength = 25;
  private static final double waveFastCycleDuration = 0.25;
  private static final double waveSlowCycleDuration = 3;
  private static final double breathFastDuration = 0.25;
  private static final double breathSlowDuration = 1;

  
  /** Creates a new LEDs. */
  public LEDs(int port, Swerve swerve) {
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_swerve = swerve;
    
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    Section.FULL,
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
        teleopColor = Color.kWhite;
        autoColor = Color.kWhite;
        break;
    }
    

    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      if (!m_swerve.getTagSeenSinceLastDisable())
        solid(Section.FULL, Color.kWhite);
      else if (m_swerve.PoseEstimator.allCamerasEnabled())
        breath(Section.FULL, Color.kWhite, Color.kBlack, breathSlowDuration);
      else
        breath(Section.FULL, Color.kGreen, Color.kBlack, breathSlowDuration);
    } 
  
    else if (DriverStation.isTeleopEnabled()) {
      solid(Section.FULL, teleopColor);
    } else if (DriverStation.isAutonomousEnabled()) {
      solid(Section.FULL, autoColor);
    } else if (DriverStation.isTestEnabled()) {
      breath(Section.FULL, teleopColor, Color.kBlack, breathSlowDuration);
    }
    

    else if (DriverStation.isEStopped()) {
      breath(Section.FULL, Color.kHotPink, Color.kDeepPink, breathFastDuration);
    }

    m_led.setData(m_ledBuffer);
  }

  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        m_ledBuffer.setLED(i, color);
      }
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
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

  private static enum Section {
    FULL;

    private int start() {
      switch (this) {
        case FULL:
          return 0;

        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case FULL:
          return length;
        default:
          return length;
      }
    }

  }
}
