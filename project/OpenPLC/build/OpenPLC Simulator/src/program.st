PROGRAM main
  VAR
    TemperatureC : REAL;
    AutoMode : BOOL := FALSE;
    ManualMotor : BOOL := FALSE;
    DryThreshold : UINT := 1800;
    WetThreshold : UINT := 2300;
    AutoMotorState : BOOL := FALSE;
  END_VAR
  VAR
    MoistureRaw AT %IW100 : UINT;
    LightRaw AT %IW101 : UINT;
    TemperatureRaw AT %IW102 : INT;
    TemperatureOK AT %IW103 : UINT;
    MotorCommand AT %QX100.0 : BOOL;
    MotorFeedback AT %IX100.0 : BOOL;
  END_VAR

  IF TemperatureOK = 1 THEN
      TemperatureC := INT_TO_REAL(TemperatureRaw) / 100.0;
  ELSE
      TemperatureC := 0.0;
  END_IF;

  IF MoistureRaw < DryThreshold THEN
      AutoMotorState := TRUE;
  ELSIF MoistureRaw > WetThreshold THEN
      AutoMotorState := FALSE;
  END_IF;

  IF AutoMode THEN
      MotorCommand := AutoMotorState;
  ELSE
      MotorCommand := ManualMotor;
  END_IF;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 1);
    PROGRAM instance0 WITH task0 : main;
  END_RESOURCE
END_CONFIGURATION
