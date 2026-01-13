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

  public void setClimberPosLoad(ClimbPosition position) {
    io.setClimbPositionWithLoad(position);
  }

  public void setClimberPosNoLoad(ClimbPosition position) {
   io.setClimbPositionNoLoad(position);
  }
  

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs("Climber", inputs);
  }

  // Commands

   public Command climbElevatorCommand(ClimbPosition pos, int pidSlot) {
      if (pidSlot == 0) {
         return this.runOnce(() -> setClimberPosNoLoad(pos)).until(movementFinished).withName("Climber elevatorNo Load");
      } else if (pidSlot == 1) {
         return this.runOnce(() -> setClimberPosLoad(pos)).until(movementFinished).withName("Climber elevator Load");
      }
      return null;
   }
    
}
