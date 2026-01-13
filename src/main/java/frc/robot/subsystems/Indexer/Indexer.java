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

  public Command startIndexer() {
    return this.run(() -> runIndexer(IndexerConstants.INDEXER_MOTOR_RPM)).withName("Run Indexer");
  }
  public Command startIndexerReverse() {
    return this.run(() -> runIndexer(-IndexerConstants.INDEXER_MOTOR_RPM)).withName("Run Indexer");
  }  
  public Command stopIndexer() {
    return this.run(() -> runIndexer(0)).withName("Stop Indexer");
  }

}
