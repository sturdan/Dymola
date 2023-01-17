within ;
package Praktikum2
  model DCMotor
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (
        Placement(transformation(extent={{-58,-88},{-38,-68}}),
          iconTransformation(extent={{-56,-86},{-38,-68}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
        Placement(transformation(extent={{42,-88},{62,-68}}),
          iconTransformation(extent={{44,-86},{62,-68}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
        Placement(transformation(extent={{76,6},{96,26}}), iconTransformation(
            extent={{78,8},{96,26}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.0005100)
      annotation (Placement(transformation(extent={{12,-26},{32,-6}})));
    Modelica.Mechanics.Rotational.Components.Damper damper(d=0.000305)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={56,-36})));
    Modelica.Electrical.Analog.Basic.EMF emf(k=0.0816)
      annotation (Placement(transformation(extent={{-6,-48},{14,-28}})));
    Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.0983) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={14,20})));
    Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.000133) annotation (
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
    connect(inductor.p, pin_n) annotation (Line(points={{-46,14},{-46,-78},{-48,
            -78}}, color={0,0,255}));
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
                "modelica://Praktikum2/./Bilder/DCMotor.PNG")}), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DCMotor;

  model SimpleBattery
    Modelica.Electrical.Analog.Interfaces.Pin pin
      annotation (Placement(transformation(extent={{-26,72},{-6,92}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{-26,-94},{-6,-74}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
        Placement(transformation(
          extent={{-16,-16},{16,16}},
          rotation=90,
          origin={-16,-34})));
    Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5) annotation (
        Placement(transformation(
          extent={{-16,-16},{16,16}},
          rotation=90,
          origin={-16,14})));
    SOC2Voltage sOC2Voltage(
      Nominal_VDC=18,
      Min_VDC=14,
      Min_SOC=0.1)
      annotation (Placement(transformation(extent={{-82,-44},{-62,-24}})));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-16,50})));
    Modelica.Blocks.Continuous.Integrator integrator(k=1/36000,  y_start=1)
      annotation (Placement(transformation(extent={{26,40},{46,60}})));
  equation
    connect(sOC2Voltage.y, signalVoltage.v)
      annotation (Line(points={{-61,-34},{-35.2,-34}}, color={0,0,127}));
    connect(signalVoltage.p, pin_n)
      annotation (Line(points={{-16,-50},{-16,-84}}, color={0,0,255}));
    connect(signalVoltage.n, resistor.p)
      annotation (Line(points={{-16,-18},{-16,-2}}, color={0,0,255}));
    connect(resistor.n, currentSensor.p)
      annotation (Line(points={{-16,30},{-16,40}}, color={0,0,255}));
    connect(currentSensor.n, pin)
      annotation (Line(points={{-16,60},{-16,82},{-16,82}}, color={0,0,255}));
    connect(currentSensor.i, integrator.u)
      annotation (Line(points={{-5,50},{24,50}}, color={0,0,127}));
    connect(integrator.y, sOC2Voltage.u) annotation (Line(points={{47,50},{48,
            50},{48,-94},{-92,-94},{-92,-34},{-84,-34}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end SimpleBattery;

  block SOC2Voltage "<html>Voltage dependent SOC. 
  <br>Input: SOC<br>Output: corresponding voltage</html>"
    extends Modelica.Blocks.Interfaces.SISO;
    parameter Modelica.SIunits.Voltage Nominal_VDC "Nominal DC voltage";
    parameter Modelica.SIunits.Voltage Min_VDC "Minimal DC voltage at Min_SOC";
    parameter Real Min_SOC = 0.1 "Minimum state of charge";

  equation
    assert(u>Min_SOC, "Sie haben die Batterie unterladen und somit zerstört!");
    assert(u<1.01, "Sie haben die Batterie um mindestens 1% ueberladen und somit zerstört!");

    y = Nominal_VDC - (Nominal_VDC - Min_VDC)*(1 - u)/(1 - Min_SOC);
    annotation (Icon(graphics={Text(
            extent={{-100,20},{0,-20}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="SOC"), Text(
            extent={{0,20},{100,-20}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="V")}));
  end SOC2Voltage;

  model MotorBatteryTest
    DCMotor dCMotor
      annotation (Placement(transformation(extent={{-42,10},{-22,30}})));
    SimpleBattery simpleBattery annotation (Placement(transformation(
          extent={{-16,-6},{16,6}},
          rotation=90,
          origin={-28,-50})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  equation
    connect(simpleBattery.pin, dCMotor.pin_n) annotation (Line(points={{-32.92,
            -52.56},{-32.92,-1.8},{-36.7,-1.8},{-36.7,12.3}}, color={0,0,255}));
    connect(simpleBattery.pin_n, dCMotor.pin_p) annotation (Line(points={{-22.96,
            -52.56},{-22.96,-5.64},{-26.7,-5.64},{-26.7,12.3}},        color={0,
            0,255}));
    connect(ground.p, simpleBattery.pin_n) annotation (Line(points={{0,-40},{
            -14,-40},{-14,-52.56},{-22.96,-52.56}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end MotorBatteryTest;

  model MotorTest
    Praktikum2.DCMotor dCMotor
      annotation (Placement(transformation(extent={{-22,-12},{-6,8}})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=16)
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=0,
          origin={-17,-33})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-12,-72},{8,-52}})));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
        tau_constant=-1) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={36,0})));
  equation
    connect(constantVoltage.p, dCMotor.pin_n) annotation (Line(points={{-24,-33},
            {-36,-33},{-36,-9.7},{-17.76,-9.7}}, color={0,0,255}));
    connect(constantTorque.flange, dCMotor.flange_a) annotation (Line(points={{
            26,8.88178e-16},{20,8.88178e-16},{20,-0.3},{-7.04,-0.3}}, color={0,
            0,0}));
    connect(constantVoltage.n, ground.p) annotation (Line(points={{-10,-33},{-6,
            -33},{-6,-52},{-2,-52}}, color={0,0,255}));
    connect(dCMotor.pin_p, constantVoltage.n) annotation (Line(points={{-9.76,
            -9.7},{-9.76,-21.05},{-10,-21.05},{-10,-33}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end MotorTest;

  model EScooter
    SimpleBattery simpleBattery annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-50,-62})));
    DCMotor dCMotor
      annotation (Placement(transformation(extent={{-60,-6},{-40,14}})));
    Modelica.Mechanics.Rotational.Components.IdealRollingWheel
      idealRollingWheel(radius=0.075)
      annotation (Placement(transformation(extent={{-16,-4},{4,16}})));
    Modelica.Mechanics.Translational.Sources.SignForce signForce(f_nominal=-7.848,
        v0=1e-06)
      annotation (Placement(transformation(extent={{-10,56},{10,76}})));
    Modelica.Mechanics.Translational.Sources.QuadraticSpeedDependentForce
      quadraticSpeedDependentForce(f_nominal=-11.90856, v_nominal(displayUnit=
            "m/s") = 5.8)
      annotation (Placement(transformation(extent={{14,-78},{34,-58}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-38,-82},{-18,-62}})));
  equation
    connect(dCMotor.flange_a, idealRollingWheel.flangeR) annotation (Line(
          points={{-41.3,5.7},{-28.65,5.7},{-28.65,6},{-16,6}}, color={0,0,0}));
    connect(simpleBattery.pin, ground.p) annotation (Line(points={{-41.8,-60.4},
            {-34.9,-60.4},{-34.9,-62},{-28,-62}}, color={0,0,255}));
    connect(simpleBattery.pin, dCMotor.pin_p) annotation (Line(points={{-41.8,
            -60.4},{-41.8,-32.2},{-44.7,-32.2},{-44.7,-3.7}}, color={0,0,255}));
    connect(simpleBattery.pin_n, dCMotor.pin_n) annotation (Line(points={{-58.4,
            -60.4},{-58.4,-32.2},{-54.7,-32.2},{-54.7,-3.7}}, color={0,0,255}));
    connect(quadraticSpeedDependentForce.flange, idealRollingWheel.flangeT)
      annotation (Line(points={{34,-68},{50,-68},{50,6},{4,6}}, color={0,127,0}));
    connect(signForce.flange, idealRollingWheel.flangeT)
      annotation (Line(points={{10,66},{8,66},{8,6},{4,6}}, color={0,127,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end EScooter;
  annotation (uses(Modelica(version="3.2.3")));
end Praktikum2;
