// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.ClimberIO.ClimbPosition;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  public Climber(ClimberIO io) {
    this.io = io;
    this.inputs = new ClimberIOInputsAutoLogged();
  }

  public void setClimberPos(ClimbPosition position) {
    io.setClimbPosition(position);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs("Climber", inputs);
  }
}
