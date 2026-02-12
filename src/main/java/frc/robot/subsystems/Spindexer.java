// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ShooterConstants;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  private SparkFlex spindexer;
  private SparkFlexConfig spindexerConfig;
  
  private boolean isShooting;
  /** Creates a new Spindexer. */
  public Spindexer() {
    spindexer = new SparkFlex(SPINDEXER_ID, MotorType.kBrushless);
    spindexerConfig = new SparkFlexConfig();

    spindexerConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60);

      spindexer.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    isShooting = false;
  }

  /**
   * @param Shooting updates the isShooting instance variable to whether or not the shooter is running
   */
  public void setShooting(boolean shooting){
    isShooting = shooting;
  }

  /**
   * @return isShooting: whether or not the shooter is currently running
   */
  public boolean getShooting(){
    return isShooting;
  }

  /**
   * Spins the spindexer at a different speed depending on if the Shooter is running or not
   */
  public void spinSpindexer(){
    if(!isShooting)
      spindexer.set(0.4);
    else
      spindexer.set(0.8);
  }

  /**
   * @return Returns a command for running the spindexer
   */
  public Command runSpindexer(){
    return Commands.run(() -> spinSpindexer(), this);
  }

  /**
   * Puts data for the spindexer onto smartdashboard
   */
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addBooleanProperty("Is shooting", () -> getShooting(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
