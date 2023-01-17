within ;
model DCMotorDriver
  "Model of a DC-DC-Converter for driving a DC-Motor. The target voltage (tarVol) is set between p_out and n_out."

  parameter Modelica.SIunits.Voltage motorVoltage = 24 "nominal voltage of motor to drive";

  Modelica.Electrical.Analog.Interfaces.PositivePin p_in "positive supply voltage pin (in)" annotation (Placement(
        transformation(extent={{-110,30},{-90,50}}), iconTransformation(extent={{-110,30},
            {-90,50}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n_in "negative supply voltage pin (in)" annotation (Placement(
        transformation(extent={{-110,-50},{-90,-30}}), iconTransformation(
          extent={{-110,-50},{-90,-30}})));
  Modelica.Blocks.Interfaces.RealInput tarVol "target voltage to set between p_out and n_out" annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,110})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n_out "negative output voltage (grounded)" annotation (Placement(
        transformation(extent={{90,-50},{110,-30}}), iconTransformation(extent={{90,-50},
            {110,-30}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p_out "positive output voltage (max. motorVoltage)" annotation (Placement(
        transformation(extent={{90,30},{110,50}}), iconTransformation(extent={{90,30},
            {110,50}})));

  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={80,20})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-80,-10})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,-20})));
  Modelica.Blocks.Math.Product product
    annotation (Placement(transformation(extent={{52,-24},{32,-4}})));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={-44,20})));
  Modelica.Blocks.Math.Division division
    annotation (Placement(transformation(extent={{0,-30},{-20,-10}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={80,-70})));

        Modelica.SIunits.Power powerIn;
        Modelica.SIunits.Power powerOut;

  Modelica.Blocks.Continuous.FirstOrder firstOrder(k=1, T=0.01) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,50})));
equation

  powerIn = (p_in.v-n_in.v)*p_in.i;
  powerOut = (p_out.v-n_out.v)*p_out.i;

  connect(signalVoltage.p, p_out)
    annotation (Line(points={{80,30},{80,40},{100,40}}, color={0,0,255}));
  connect(p_in, signalCurrent.p)
    annotation (Line(points={{-100,40},{-80,40},{-80,0}}, color={0,0,255}));
  connect(n_in, signalCurrent.n) annotation (Line(points={{-100,-40},{-80,-40},{
          -80,-20}},  color={0,0,255}));
  connect(currentSensor.n, n_out)
    annotation (Line(points={{80,-30},{80,-40},{100,-40}}, color={0,0,255}));
  connect(signalVoltage.n, currentSensor.p)
    annotation (Line(points={{80,10},{80,-10}}, color={0,0,255}));
  connect(currentSensor.i, product.u2)
    annotation (Line(points={{69,-20},{54,-20}}, color={0,0,127}));
  connect(voltageSensor.p, signalCurrent.p) annotation (Line(points={{-44,30},{-44,
          40},{-80,40},{-80,0}},      color={0,0,255}));
  connect(voltageSensor.n, signalCurrent.n) annotation (Line(points={{-44,10},{-44,
          -40},{-80,-40},{-80,-20}},      color={0,0,255}));
  connect(signalCurrent.i, division.y) annotation (Line(points={{-68,-10},{-60,
          -10},{-60,-20},{-21,-20}},color={0,0,127}));
  connect(ground.p, n_out)
    annotation (Line(points={{80,-60},{80,-40},{100,-40}}, color={0,0,255}));
  connect(division.u1, product.y)
    annotation (Line(points={{2,-14},{31,-14}}, color={0,0,127}));
  connect(voltageSensor.v, division.u2) annotation (Line(points={{-33,20},{-30,
          20},{-30,-40},{20,-40},{20,-26},{2,-26}},
        color={0,0,127}));
  connect(product.u1, signalVoltage.v) annotation (Line(points={{54,-8},{60,-8},
          {60,20},{68,20}}, color={0,0,127}));
  connect(tarVol, firstOrder.u)
    annotation (Line(points={{0,110},{0,62}}, color={0,0,127}));
  connect(firstOrder.y, signalVoltage.v)
    annotation (Line(points={{0,39},{0,20},{68,20}}, color={0,0,127}));
       annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.None,
          textString="DC-Driver"),
        Text(
          extent={{-120,20},{-80,-20}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.None,
          textString="I"),
        Text(
          extent={{80,20},{120,-20}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.None,
          textString="O"),
        Line(points={{-90,40},{-84,40},{-80,40}}, color={0,0,0}),
        Line(points={{-90,-40},{-84,-40},{-80,-40}},
                                                  color={0,0,0}),
        Line(points={{80,-40},{86,-40},{90,-40}}, color={0,0,0}),
        Line(points={{80,40},{86,40},{90,40}},    color={0,0,0})}),Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.3")));
end DCMotorDriver;
