/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pnuematics extends SubsystemBase {
  /**
   * Creates a new Pnuematics.
   */

  //public Compressor airPump;
  private DoubleSolenoid gearShifter;
  private DoubleSolenoid intakeMover;
  public Pnuematics() {
    //airPump = new Compressor(Constants.AIR_PUMP_CAN);
    gearShifter = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.DRIVE_SHIFTER_PCM_A, Constants.DRIVE_SHIFTER_PCM_B);
    intakeMover = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
    setShiftLow();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShifterHigh() {
    gearShifter.set(Value.kReverse);
  }

  public void setShiftLow() {
    gearShifter.set(Value.kForward);
  }
  public Value getShifter() {
    return gearShifter.get();
  }
  public void gearShifter() {
    if (this.getShifter() == Value.kReverse) {
      this.setShiftLow();
    } else {
      this.setShifterHigh();
    }
  }
  public void setIntakeIn() {
    intakeMover.set(Value.kReverse);
  }

  public void setIntakeOut() {
    intakeMover.set(Value.kForward);
  }
  public Value getIntakePCM() {
    return intakeMover.get();
  }
  public void changeIntakeMode() {
    if (this.getIntakePCM() == Value.kReverse) {
      this.setIntakeOut();
    } else {
      this.setIntakeIn();
    }
  }
}
