package frc.robot.subsystems.auto;

public enum ChoreoPath {
  bottomStartToneutralZ,
  BottomStartToShoot,
  DepotIntake,
  DepotToShootT,
  IntakeBtmToAlliance,
  IntakeToptoAlliance,
  MidToDepot,
  NeutralZBtmIntake,
  NeutralZTopIntake,
  ShootTtoDepot,
  topStartToneutralZ,
  ShootBtoNeutralZ;

  public final String filepath;

  ChoreoPath() {
    this.filepath = this.toString();
  }

  ChoreoPath(String filepath) {
    this.filepath = filepath;
  }
}
