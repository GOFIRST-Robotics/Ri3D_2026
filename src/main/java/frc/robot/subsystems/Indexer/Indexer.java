// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;

  public Indexer(IndexerIO io) {
    this.io = io;
    this.inputs = new IndexerIOInputsAutoLogged();
  }

  public void runIndexer(double RPM) {
    io.setIndexerRPM(RPM);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    io.periodic();
  }


  public Command runIndexerCommand() {
      return this.startEnd(
          () -> runIndexer(IndexerConstants.INDEXER_MOTOR_RPM), 
          () -> runIndexer(0)                                   
      ).withName("Run Indexer");
  }

  public Command runIndexerCommandDutyCycle() {
      return this.startEnd(
          () -> io.setIndexerKDutyCycle(0.25), 
          () -> io.setIndexerKDutyCycle(0.0)                                   
      ).withName("Run Indexer");
  }

}
