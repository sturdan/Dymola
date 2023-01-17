within ;
package Praktikum3
  model DCMotor
    import SI = Modelica.SIunits;

    parameter SI.Resistance R = 2.14      "motor resistance";
    parameter SI.Inertia J = 0.00000149    "motor rotor interita";
    parameter SI.DampingCoefficient d = 0.00000109     "motor damping";
    parameter SI.Inductance L = 0.000278  "motor inductance";
    parameter SI.ElectricalTorqueConstant k = 0.0256 "motor constant";

    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (
        Placement(transformation(extent={{-58,-88},{-38,-68}}),
          iconTransformation(extent={{-56,-86},{-38,-68}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
        Placement(transformation(extent={{42,-88},{62,-68}}),
          iconTransformation(extent={{44,-86},{62,-68}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
        Placement(transformation(extent={{76,6},{96,26}}), iconTransformation(
            extent={{78,8},{96,26}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)
      annotation (Placement(transformation(extent={{12,-26},{32,-6}})));
    Modelica.Mechanics.Rotational.Components.Damper damper(d=d)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={56,-36})));
    Modelica.Electrical.Analog.Basic.EMF emf(k=k)
      annotation (Placement(transformation(extent={{-6,-48},{14,-28}})));
    Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)      annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={14,20})));
    Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)        annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-46,4})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed
      annotation (Placement(transformation(extent={{46,-66},{66,-46}})));
  equation
    connect(pin_n, pin_n) annotation (Line(points={{-48,-78},{-48,-120},{-56,
            -120},{-56,-78},{-48,-78}}, color={0,0,255}));
    connect(emf.n, pin_p) annotation (Line(points={{4,-48},{4,-78},{52,-78}},
          color={0,0,255}));
    connect(emf.p, resistor.n)
      annotation (Line(points={{4,-28},{4,10},{14,10}},
                                                   color={0,0,255}));
    connect(inductor.n, resistor.p)
      annotation (Line(points={{-46,-6},{-46,30},{14,30}},
                                                   color={0,0,255}));
    connect(inductor.p, pin_n) annotation (Line(points={{-46,14},{-46,-78},{-48,-78}},
                   color={0,0,255}));
    connect(inertia.flange_a, emf.flange)
      annotation (Line(points={{12,-16},{-14,-16},{-14,-38},{14,-38}},
                                                   color={0,0,0}));
    connect(flange_a, inertia.flange_b) annotation (Line(points={{86,16},{44,16},
            {44,-16},{32,-16}}, color={0,0,0}));
    connect(damper.flange_b, fixed.flange)
      annotation (Line(points={{56,-46},{56,-56}}, color={0,0,0}));
    connect(damper.flange_a, inertia.flange_b)
      annotation (Line(points={{56,-26},{56,-16},{32,-16}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Bitmap(extent={{-74,-74},{98,72}}, fileName=
                "modelica://Praktikum3/Bilder/DCMotor.PNG")}),   Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DCMotor;

  model DCMotor_Angle
    DCMotor_Speed drehzahlregler
      annotation (Placement(transformation(extent={{-22,-50},{18,-22}})));
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=70,
      Ti=0.05,
      Td=0,
      yMax=9000)
               annotation (Placement(transformation(extent={{56,4},{76,24}})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
      annotation (Placement(transformation(extent={{32,-48},{52,-28}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=3.14)
      annotation (Placement(transformation(extent={{20,4},{40,24}})));
  equation
    connect(drehzahlregler.flange, angleSensor.flange) annotation (Line(points=
            {{18,-36},{26,-36},{26,-38},{32,-38}}, color={0,0,0}));
    connect(angleSensor.phi, PID.u_m)
      annotation (Line(points={{53,-38},{66,-38},{66,2}}, color={0,0,127}));
    connect(realExpression.y, PID.u_s)
      annotation (Line(points={{41,14},{54,14}}, color={0,0,127}));
    connect(PID.y, drehzahlregler.u) annotation (Line(points={{77,14},{80,14},{
            80,52},{-64,52},{-64,-30.68},{-22.4,-30.68}}, color={0,0,127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=10,
        __Dymola_NumberOfIntervals=50000,
        __Dymola_Algorithm="Dassl"));
  end DCMotor_Angle;

  model DCMotor_Speed
    Praktikum3.DCMotor dCMotor
      annotation (Placement(transformation(extent={{-30,2},{-10,22}})));
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      k=1.4,
      Ti=0.05,
      Td=0,
      yMax=24) annotation (Placement(transformation(extent={{64,28},{84,48}})));
    DCMotorDriver dCMotorDriver
      annotation (Placement(transformation(extent={{-62,-58},{-34,-32}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{4,14},{24,34}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange flange
      annotation (Placement(transformation(extent={{96,-4},{104,4}})));
    Modelica.Blocks.Interfaces.RealInput u
      annotation (Placement(transformation(extent={{-108,32},{-96,44}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{-76,-110},{-56,-90}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{62,-110},{82,-90}})));
  equation
    connect(PID.y, dCMotorDriver.tarVol) annotation (Line(points={{85,38},{90,
            38},{90,96},{-48,96},{-48,-30.7}},
                                           color={0,0,127}));
    connect(dCMotor.flange_a, speedSensor.flange) annotation (Line(points={{-11.3,
            13.7},{-11.65,13.7},{-11.65,24},{4,24}},  color={0,0,0}));
    connect(dCMotorDriver.p_out, dCMotor.pin_n) annotation (Line(points={{-34,
            -39.8},{-34,-39.9},{-24.7,-39.9},{-24.7,4.3}}, color={0,0,255}));
    connect(dCMotorDriver.n_out, dCMotor.pin_p) annotation (Line(points={{-34,
            -50.2},{-34,-51.1},{-14.7,-51.1},{-14.7,4.3}}, color={0,0,255}));
    connect(speedSensor.w, PID.u_m) annotation (Line(points={{25,24},{50,24},{
            50,26},{74,26}},
                          color={0,0,127}));
    connect(dCMotor.flange_a, flange) annotation (Line(points={{-11.3,13.7},{
            41.35,13.7},{41.35,0},{100,0}}, color={0,0,0}));
    connect(flange, flange)
      annotation (Line(points={{100,0},{100,0}}, color={0,0,0}));
    connect(u, PID.u_s)
      annotation (Line(points={{-102,38},{62,38}}, color={0,0,127}));
    connect(pin_n, dCMotorDriver.n_in) annotation (Line(points={{72,-100},{74,
            -100},{74,-70},{-62,-70},{-62,-50.2}}, color={0,0,255}));
    connect(dCMotorDriver.p_in, pin_p) annotation (Line(points={{-62,-39.8},{
            -66,-39.8},{-66,-100}}, color={0,0,255}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(__Dymola_Algorithm="Dassl"));
  end DCMotor_Speed;

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
          coordinateSystem(preserveAspectRatio=false)));
  end DCMotorDriver;

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
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
        tau_constant=-0.0263)
      annotation (Placement(transformation(extent={{-48,22},{-28,42}})));
  equation
    connect(constantVoltage.p, dCMotor.pin_n) annotation (Line(points={{-38,-49},
            {-38,-3.7},{-38.7,-3.7}},            color={0,0,255}));
    connect(constantVoltage.n,ground. p) annotation (Line(points={{-24,-49},{-8,
            -49},{-8,-68},{-4,-68}}, color={0,0,255}));
    connect(dCMotor.pin_p,constantVoltage. n) annotation (Line(points={{-28.7,
            -3.7},{-28.7,-37.05},{-24,-37.05},{-24,-49}}, color={0,0,255}));
    connect(constantTorque.flange, dCMotor.flange_a) annotation (Line(points={{
            -28,32},{-16,32},{-16,5.7},{-25.3,5.7}}, color={0,0,0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
  end MotorTest;

  model DCMotor_Speed_Test
    DCMotor            dCMotor
      annotation (Placement(transformation(extent={{-32,-14},{-12,6}})));
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      k=1.4,
      Ti=0.05,
      Td=0,
      yMax=24) annotation (Placement(transformation(extent={{62,12},{82,32}})));
    DCMotorDriver dCMotorDriver
      annotation (Placement(transformation(extent={{-64,-74},{-36,-48}})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=24)
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=270,
          origin={-89,-61})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-86,-112},{-66,-92}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{2,-2},{22,18}})));
    Modelica.Blocks.Sources.Ramp ramp(height=300, duration=0.2)
      annotation (Placement(transformation(extent={{4,30},{24,50}})));
  equation
    connect(ground.p,constantVoltage. n) annotation (Line(points={{-76,-92},{
            -88,-92},{-88,-68},{-89,-68}},
                                       color={0,0,255}));
    connect(PID.y,dCMotorDriver. tarVol) annotation (Line(points={{83,22},{88,
            22},{88,80},{-50,80},{-50,-46.7}},
                                           color={0,0,127}));
    connect(dCMotor.flange_a,speedSensor. flange) annotation (Line(points={{-13.3,
            -2.3},{-13.65,-2.3},{-13.65,8},{2,8}},    color={0,0,0}));
    connect(dCMotorDriver.p_out,dCMotor. pin_n) annotation (Line(points={{-36,
            -55.8},{-36,-55.9},{-26.7,-55.9},{-26.7,-11.7}},
                                                           color={0,0,255}));
    connect(dCMotorDriver.n_out,dCMotor. pin_p) annotation (Line(points={{-36,
            -66.2},{-36,-67.1},{-16.7,-67.1},{-16.7,-11.7}},
                                                           color={0,0,255}));
    connect(constantVoltage.p,dCMotorDriver. p_in) annotation (Line(points={{-89,-54},
            {-76,-54},{-76,-55.8},{-64,-55.8}},      color={0,0,255}));
    connect(constantVoltage.n,dCMotorDriver. n_in) annotation (Line(points={{-89,-68},
            {-76,-68},{-76,-66.2},{-64,-66.2}},      color={0,0,255}));
    connect(speedSensor.w,PID. u_m) annotation (Line(points={{23,8},{48,8},{48,
            10},{72,10}}, color={0,0,127}));
    connect(ramp.y, PID.u_s) annotation (Line(points={{25,40},{50,40},{50,22},{
            60,22}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DCMotor_Speed_Test;

  model DCMotor_Angle_Test
    DCMotor_Speed drehzahlregler
      annotation (Placement(transformation(extent={{-36,-54},{4,-26}})));
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=70,
      Ti=0.05,
      Td=0,
      yMax=9000)
               annotation (Placement(transformation(extent={{48,2},{68,22}})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
      annotation (Placement(transformation(extent={{26,-50},{46,-30}})));
    Modelica.Blocks.Sources.Ramp ramp(height=3.14, duration=0.2)
      annotation (Placement(transformation(extent={{-6,0},{14,20}})));
  equation
    connect(drehzahlregler.flange, angleSensor.flange)
      annotation (Line(points={{4,-40},{26,-40}}, color={0,0,0}));
    connect(angleSensor.phi, PID.u_m)
      annotation (Line(points={{47,-40},{58,-40},{58,0}}, color={0,0,127}));
    connect(PID.y, drehzahlregler.u) annotation (Line(points={{69,12},{74,12},{
            74,50},{-70,50},{-70,-34.68},{-36.4,-34.68}}, color={0,0,127}));
    connect(ramp.y, PID.u_s) annotation (Line(points={{15,10},{34,10},{34,12},{
            46,12}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DCMotor_Angle_Test;
  annotation (uses(Modelica(version="3.2.3")));
end Praktikum3;
