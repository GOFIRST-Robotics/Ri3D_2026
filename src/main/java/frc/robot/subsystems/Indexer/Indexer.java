// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public void runElevator(double RPM) {
    io.setElevatorRPM(RPM);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    io.periodic();
  }
}
