PROGRAM main
  VAR
    TemperatureC : REAL;
    BatteryVoltageV : REAL;
    ShuntVoltageMv : REAL;
    WaterPresent : BOOL;
    DryDelayTimer : TON;
    PumpTimer : TON;
    AbsorbTimer : TON;
    State : INT := 0;
  END_VAR
  VAR
    MoistureRaw AT %IW0 : UINT;
    LightRaw AT %IW1 : UINT;
    TemperatureRaw AT %IW2 : INT;
    TemperatureOK AT %IW3 : UINT;
    BatteryVoltageMv AT %IW4 : UINT;
    MotorCurrentMa AT %IW5 : UINT;
    InaOK AT %IW6 : UINT;
    ShuntVoltageUv AT %IW7 : UINT;
    MotorFeedback AT %IX0.0 : BOOL;
    HallActive AT %IX0.1 : BOOL;
    MotorCommand AT %QX0.0 : BOOL;
  END_VAR

  IF TemperatureOK = 1 THEN
      TemperatureC := INT_TO_REAL(TemperatureRaw) / 100.0;
  ELSE
      TemperatureC := 0.0;
  END_IF;

  BatteryVoltageV :=
      UINT_TO_REAL(BatteryVoltageMv) / 1000.0;

  ShuntVoltageMv :=
      UINT_TO_REAL(ShuntVoltageUv) / 1000.0;

  WaterPresent := HallActive;

  CASE State OF

  0:
      MotorCommand := FALSE;

      DryDelayTimer(
          IN := WaterPresent AND (MoistureRaw > 3000),
          PT := T#6s
      );

      IF DryDelayTimer.Q THEN
          State := 1;
      END_IF;

  1:
      MotorCommand := WaterPresent;

      PumpTimer(
          IN := TRUE,
          PT := T#3s
      );

      IF PumpTimer.Q OR NOT WaterPresent THEN
          MotorCommand := FALSE;
          State := 2;
      END_IF;

  2:
      MotorCommand := FALSE;

      AbsorbTimer(
          IN := TRUE,
          PT := T#5m
      );

      IF AbsorbTimer.Q THEN
          IF WaterPresent AND (MoistureRaw > 2300) THEN
              State := 1;
          ELSE
              State := 0;
          END_IF;
      END_IF;

  ELSE
      MotorCommand := FALSE;
      State := 0;

  END_CASE;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 1);
    PROGRAM instance0 WITH task0 : main;
  END_RESOURCE
END_CONFIGURATION
