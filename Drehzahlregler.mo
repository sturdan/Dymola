within ;
model Drehzahlregler
  Praktikum3.DCMotor dCMotor
    annotation (Placement(transformation(extent={{-30,2},{-10,22}})));
  Modelica.Blocks.Continuous.LimPID PID(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1,
    Ti=0.5,
    Td=0,
    yMax=24) annotation (Placement(transformation(extent={{62,30},{82,50}})));
  DCMotorDriver dCMotorDriver
    annotation (Placement(transformation(extent={{-62,-58},{-34,-32}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=24)
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-87,-45})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-84,-96},{-64,-76}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{-4,4},{16,24}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=300,
    duration=0.2,
    startTime=0.2)
    annotation (Placement(transformation(extent={{-18,34},{2,54}})));
equation
  connect(ground.p, constantVoltage.n) annotation (Line(points={{-74,-76},{-86,
          -76},{-86,-52},{-87,-52}}, color={0,0,255}));
  connect(PID.y, dCMotorDriver.tarVol) annotation (Line(points={{83,40},{90,40},
          {90,96},{-48,96},{-48,-30.7}}, color={0,0,127}));
  connect(dCMotor.flange_a, speedSensor.flange) annotation (Line(points={{-11.3,
          13.7},{-11.65,13.7},{-11.65,14},{-4,14}}, color={0,0,0}));
  connect(dCMotorDriver.p_out, dCMotor.pin_n) annotation (Line(points={{-34,
          -39.8},{-34,-39.9},{-24.7,-39.9},{-24.7,4.3}}, color={0,0,255}));
  connect(dCMotorDriver.n_out, dCMotor.pin_p) annotation (Line(points={{-34,
          -50.2},{-34,-51.1},{-14.7,-51.1},{-14.7,4.3}}, color={0,0,255}));
  connect(constantVoltage.p, dCMotorDriver.p_in) annotation (Line(points={{-87,
          -38},{-74,-38},{-74,-39.8},{-62,-39.8}}, color={0,0,255}));
  connect(constantVoltage.n, dCMotorDriver.n_in) annotation (Line(points={{-87,
          -52},{-74,-52},{-74,-50.2},{-62,-50.2}}, color={0,0,255}));
  connect(speedSensor.w, PID.u_m) annotation (Line(points={{17,14},{56,14},{56,
          28},{72,28}}, color={0,0,127}));
  connect(ramp.y, PID.u_s) annotation (Line(points={{3,44},{36,44},{36,40},{60,
          40}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.3")));
end Drehzahlregler;
