within ;
model MotorTest
  Praktikum3.DCMotor dCMotor
    annotation (Placement(transformation(extent={{-44,-6},{-24,14}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=24)
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=0,
        origin={-31,-49})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-14,-88},{6,-68}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0)
    annotation (Placement(transformation(extent={{0,-4},{20,16}})));
equation
  connect(constantVoltage.p, dCMotor.pin_n) annotation (Line(points={{-38,-49},
          {-38,-3.7},{-38.7,-3.7}},            color={0,0,255}));
  connect(constantVoltage.n,ground. p) annotation (Line(points={{-24,-49},{-8,
          -49},{-8,-68},{-4,-68}}, color={0,0,255}));
  connect(dCMotor.pin_p,constantVoltage. n) annotation (Line(points={{-28.7,
          -3.7},{-28.7,-37.05},{-24,-37.05},{-24,-49}}, color={0,0,255}));
  connect(inertia.flange_a, dCMotor.flange_a) annotation (Line(points={{0,6},{
          -12,6},{-12,5.7},{-25.3,5.7}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.3")));
end MotorTest;
