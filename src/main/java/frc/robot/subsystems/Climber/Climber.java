// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.ClimberIO.ClimbPosition;

public class Climber extends SubsystemBase {

  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs;
  BooleanSupplier movementFinished;


  public Climber(ClimberIO io) {
    this.io = io;
    this.inputs = new ClimberIOInputsAutoLogged();
    movementFinished = io::isMovementFinished;
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

  // Commands

  public Command goFirstRungPos() {
     return this.runOnce(() -> setClimberPos(ClimberIO.ClimbPosition.RUNG_ONE)).until(movementFinished).withName("Rung One Pos");
  }

  public Command goSecondRungPos() {
     return this.runOnce(() -> setClimberPos(ClimberIO.ClimbPosition.RUNG_TWO)).until(movementFinished).withName("Rung Two Pos");
  }

  public Command goThirdRungPos() {
     return this.runOnce(() -> setClimberPos(ClimberIO.ClimbPosition.RUNG_THREE)).until(movementFinished).withName("Rung Three Pos");
  }

  public Command goAutonomousPos() {
     return this.runOnce(() -> setClimberPos(ClimberIO.ClimbPosition.AUTO)).until(movementFinished).withName("Auto Climber Pos");
  }

  public Command goZeroPos() {
     return this.runOnce(() -> setClimberPos(ClimberIO.ClimbPosition.ZERO)).until(movementFinished).withName("Zero Climber Pos");
  }


}
