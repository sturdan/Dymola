within ;
package Praktikum4
  partial model twoPin "abstract model with two pins"

    import SI = Modelica.SIunits;

    SI.Voltage u;
    SI.Current i;

    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));

  equation
      u = pin_p.v - pin_n.v;
      pin_n.i + pin_p.i = 0;
      i = pin_p.i;

  end twoPin;

  model capacitor
    "ideal capacitor"

    extends twoPin;

    import SI = Modelica.SIunits;

    parameter SI.Capacitance C;

  equation
    i = C * der(u);

    annotation (Icon(graphics={Bitmap(extent={{-94,-50},{98,52}}, fileName=
                "modelica://Praktikum4/./Bilder/Kapazität.png")}));
  end capacitor;

  model inductance
   "ideal inductance"

    extends twoPin;

    import SI = Modelica.SIunits;

    parameter SI.Inductance L;

  equation
    u = L * der(i);

    annotation (Icon(graphics={Bitmap(extent={{-114,-66},{126,62}}, fileName=
                "modelica://Praktikum4/./Bilder/Induktivität.png")}));
  end inductance;

  model SCARAKinematics

    parameter Modelica.SIunits.Inertia linkInerta = 1e-5;
    parameter Modelica.SIunits.Mass linkMass = 1;
    Modelica.Mechanics.MultiBody.Joints.Revolute theta1(useAxisFlange=true, n(
          displayUnit="1") = {0,0,1}) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-40,-60})));
    Modelica.Mechanics.MultiBody.Joints.Revolute theta2(useAxisFlange=true, n(
          displayUnit="1") = {0,0,1})
      annotation (Placement(transformation(extent={{14,10},{34,30}})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape link1(
      animateSphere=true,
      r={0,0,0.5},
      r_CM={0,0,0.25},
      m=linkMass,
      I_11=linkInerta,
      I_22=linkInerta,
      I_33=linkInerta,
      shapeType="cylinder",
      r_shape={0,0,0},
      lengthDirection(displayUnit="1") = {0,0,1},
      widthDirection(displayUnit="1") = {0,1,0},
      length=0.5,
      width=0.05,
      height=0.05) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-40,-20})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape link2(
      animateSphere=true,
      r={0.5,0,0},
      r_CM={0.25,0,0},
      m=linkMass,
      I_11=linkInerta,
      I_22=linkInerta,
      I_33=linkInerta,
      shapeType="cylinder",
      r_shape={0,0,0},
      lengthDirection(displayUnit="1") = {1,0,0},
      widthDirection(displayUnit="1") = {0,1,0},
      length=0.5,
      width=0.05,
      height=0.05) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-10,20})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape link3(
      animateSphere=true,
      r={0.5,0,0},
      r_CM={0.25,0,0},
      m=linkMass,
      I_11=linkInerta,
      I_22=linkInerta,
      I_33=linkInerta,
      shapeType="cylinder",
      r_shape={0,0,0},
      lengthDirection(displayUnit="1") = {1,0,0},
      widthDirection(displayUnit="1") = {0,1,0},
      length=0.5,
      width=0.05,
      height=0.05) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={60,20})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange1 annotation (
        Placement(transformation(extent={{-90,-80},{-70,-60}}),
          iconTransformation(extent={{-90,-80},{-70,-60}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange2 annotation (
        Placement(transformation(extent={{-10,34},{10,54}}), iconTransformation(
            extent={{-10,34},{10,54}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange=true, n(
          displayUnit="1") = {0,0,-1})
      annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={90,0})));
    Modelica.Mechanics.Translational.Interfaces.Flange_a flange_trans
      annotation (Placement(transformation(extent={{64,-10},{84,10}}),
          iconTransformation(extent={{64,-10},{84,10}})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape link4(
      animateSphere=false,
      r={0,0,0},
      r_CM={0,0,0},
      m=linkMass,
      I_11=linkInerta,
      I_22=linkInerta,
      I_33=linkInerta,
      shapeType="box",
      r_shape={0,0,0},
      lengthDirection(displayUnit="1") = {1,0,0},
      widthDirection(displayUnit="1") = {0,1,0},
      length=0.1,
      width=0.1,
      height=0.1)  annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={90,-30})));
    inner Modelica.Mechanics.MultiBody.World world(label2="z", n(displayUnit="1")=
           {0,0,-1})
      annotation (Placement(transformation(extent={{10,-92},{-10,-72}})));
  equation
    connect(theta1.frame_b, link1.frame_a) annotation (Line(
        points={{-40,-50},{-40,-30}},
        color={95,95,95},
        thickness=0.5));
    connect(link1.frame_b, link2.frame_a) annotation (Line(
        points={{-40,-10},{-40,20},{-20,20}},
        color={95,95,95},
        thickness=0.5));
    connect(link2.frame_b, theta2.frame_a) annotation (Line(
        points={{0,20},{14,20}},
        color={95,95,95},
        thickness=0.5));
    connect(theta2.frame_b, link3.frame_a) annotation (Line(
        points={{34,20},{50,20}},
        color={95,95,95},
        thickness=0.5));
    connect(theta2.axis, flange2)
      annotation (Line(points={{24,30},{24,44},{0,44}}, color={0,0,0}));
    connect(flange1, flange1)
      annotation (Line(points={{-80,-70},{-80,-70}}, color={0,0,0}));
    connect(theta1.axis, flange1) annotation (Line(points={{-50,-60},{-66,-60},
            {-66,-70},{-80,-70}}, color={0,0,0}));
    connect(link3.frame_b, prismatic.frame_a) annotation (Line(
        points={{70,20},{90,20},{90,10}},
        color={95,95,95},
        thickness=0.5));
    connect(flange_trans, prismatic.axis)
      annotation (Line(points={{74,0},{74,-8},{84,-8}}, color={0,127,0}));
    connect(prismatic.frame_b, link4.frame_a) annotation (Line(
        points={{90,-10},{90,-20}},
        color={95,95,95},
        thickness=0.5));
    connect(world.frame_b, theta1.frame_a) annotation (Line(
        points={{-10,-82},{-40,-82},{-40,-70}},
        color={95,95,95},
        thickness=0.5));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(
            points={{-30,0},{30,0}},
            color={0,0,0},
            pattern=LinePattern.None,
            thickness=0.5,
            origin={-80,-46},
            rotation=-90),
          Ellipse(
            extent={{-6,20},{6,-20}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            origin={-80,-86},
            rotation=-90),
          Rectangle(
            extent={{-20,20},{20,-20}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-80,-66},
            rotation=-90),
          Ellipse(
            extent={{-6,20},{6,-20}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            origin={-80,-46},
            rotation=-90),
          Line(
            points={{-20,-6.98423e-15},{20,3.67394e-15}},
            color={0,0,0},
            thickness=0.5,
            origin={-100,-66},
            rotation=-90),
          Line(
            points={{-20,-6.98423e-15},{20,3.67394e-15}},
            color={0,0,0},
            thickness=0.5,
            origin={-60,-66},
            rotation=-90),
          Line(
            points={{-30,0},{30,0}},
            color={0,0,0},
            pattern=LinePattern.None,
            thickness=0.5,
            origin={0,66},
            rotation=-90),
          Ellipse(
            extent={{-6,20},{6,-20}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            origin={0,26},
            rotation=-90),
          Rectangle(
            extent={{-20,20},{20,-20}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={0,46},
            rotation=-90),
          Ellipse(
            extent={{-6,20},{6,-20}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            origin={0,66},
            rotation=-90),
          Line(
            points={{-20,-6.98423e-15},{20,3.67394e-15}},
            color={0,0,0},
            thickness=0.5,
            origin={-20,46},
            rotation=-90),
          Line(
            points={{-20,-6.98423e-15},{20,3.67394e-15}},
            color={0,0,0},
            thickness=0.5,
            origin={20,46},
            rotation=-90),
          Line(
            points={{-80,-46},{-80,48},{-20,48}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{0,66},{0,76},{0,80},{30,80},{30,10},{0,10},{0,20}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{30,46},{36,46},{50,46}},
            color={0,0,0},
            thickness=0.5),
          Rectangle(
            extent={{50,60},{90,20}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),
          Polygon(
            points={{50,60},{60,70},{100,70},{90,60},{50,60}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{90,20},{100,30},{100,70},{90,60},{90,20}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{74,20},{74,10}},
            color={0,0,0},
            thickness=0.5)}),                                      Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end SCARAKinematics;

  model SCARA
    SCARAKinematics sCARAKinematics
      annotation (Placement(transformation(extent={{-52,36},{82,92}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{-20,-100},{-6,-86}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{12,-100},{26,-86}})));
    DCMotor_Angle dCMotor_Angle(
      k_Angle=10,
      i_Angle=0,
      d_Angle=0,
      yMax_Angle=9000)
      annotation (Placement(transformation(extent={{-70,-52},{-42,-28}})));
    DCMotor_Angle dCMotor_Angle1(
      k_Angle=10,
      i_Angle=0,
      d_Angle=0,
      yMax_Angle=9000)
      annotation (Placement(transformation(extent={{-14,-52},{14,-28}})));
    DCMotor_Angle dCMotor_Angle2(
      k_Angle=1,
      i_Angle=0,
      d_Angle=0,
      yMax_Angle=9000)
      annotation (Placement(transformation(extent={{44,-50},{72,-26}})));
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(useSupport=
          false, ratio=100)
                           annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-40,20})));
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear2(useSupport=
          false, ratio=100)
                           annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={16,20})));
    Modelica.Mechanics.Rotational.Components.IdealGearR2T idealGearR2T(ratio=100)
                annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={84,18})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=24)
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=0,
          origin={5,-93})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-100,-104},{-80,-84}})));
    Modelica.Blocks.Math.Gain GearRatio2(k=100) annotation (Placement(
          transformation(
          extent={{-5,-5},{5,5}},
          rotation=-90,
          origin={-23,-29})));
    Modelica.Blocks.Math.Gain GearRatio1(k=100) annotation (Placement(
          transformation(
          extent={{-5,-5},{5,5}},
          rotation=-90,
          origin={-77,-29})));
    Modelica.Blocks.Math.Gain GearR2TRatio(k=100) annotation (Placement(
          transformation(
          extent={{-5,-5},{5,5}},
          rotation=-90,
          origin={33,-29})));
    Modelica.Blocks.Sources.Sine sine(amplitude=1.57, freqHz=0.5) annotation (
        Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=270,
          origin={-77,-7})));
    Modelica.Blocks.Sources.Sine sine1(amplitude=1.57, freqHz=0.5) annotation (
        Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=270,
          origin={-23,-7})));
    Modelica.Blocks.Sources.Sine sine2(amplitude=0.2, freqHz=0.25) annotation (
        Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=270,
          origin={33,-3})));
  equation
    connect(dCMotor_Angle.pin_p, dCMotor_Angle2.pin_p) annotation (Line(points={{-65.24,
            -50.56},{-64,-50.56},{-64,-64},{50,-64},{50,-48.56},{48.76,-48.56}},
          color={0,0,255}));
    connect(dCMotor_Angle1.pin_p, dCMotor_Angle2.pin_p) annotation (Line(points={{-9.24,
            -50.56},{-8,-50.56},{-8,-64},{50,-64},{50,-48.56},{48.76,-48.56}},
                                                                        color={
            0,0,255}));
    connect(dCMotor_Angle2.pin_n, pin_n) annotation (Line(points={{67.52,-48.32},
            {70,-48.32},{70,-93},{19,-93}},
                                        color={0,0,255}));
    connect(dCMotor_Angle.pin_n, pin_n) annotation (Line(points={{-46.48,-50.32},
            {-48,-50.32},{-48,-80},{70,-80},{70,-93},{19,-93}},
                                                            color={0,0,255}));
    connect(dCMotor_Angle1.pin_n, pin_n) annotation (Line(points={{9.52,-50.32},
            {8,-50.32},{8,-80},{70,-80},{70,-93},{19,-93}},
                                                      color={0,0,255}));
    connect(dCMotor_Angle.flange_a, idealGear.flange_a) annotation (Line(points={{-42.28,
            -39.04},{-42.28,-40},{-40,-40},{-40,10}},           color={0,0,0}));
    connect(idealGear.flange_b, sCARAKinematics.flange1) annotation (Line(
          points={{-40,30},{-40,44.4},{-38.6,44.4}}, color={0,0,0}));
    connect(dCMotor_Angle1.flange_a,idealGear2. flange_a) annotation (Line(
          points={{13.72,-39.04},{13.72,-40},{16,-40},{16,10}},
                                                         color={0,0,0}));
    connect(idealGearR2T.flangeT, sCARAKinematics.flange_trans)
      annotation (Line(points={{84,28},{84,64},{64.58,64}}, color={0,127,0}));
    connect(dCMotor_Angle2.flange_a, idealGearR2T.flangeR) annotation (Line(
          points={{71.72,-37.04},{78,-37.04},{78,8},{84,8}},
                                                        color={0,0,0}));
    connect(ground.p, dCMotor_Angle2.pin_p) annotation (Line(points={{-90,-84},
            {-76,-84},{-76,-64},{50,-64},{50,-48.56},{48.76,-48.56}},
                                                               color={0,0,255}));
    connect(GearRatio2.y, dCMotor_Angle1.u) annotation (Line(points={{-23,-34.5},
            {-23,-39.04},{-14.28,-39.04}}, color={0,0,127}));
    connect(GearR2TRatio.y, dCMotor_Angle2.u) annotation (Line(points={{33,
            -34.5},{33,-37.04},{43.72,-37.04}}, color={0,0,127}));
    connect(GearRatio1.y, dCMotor_Angle.u) annotation (Line(points={{-77,-34.5},
            {-77,-39.04},{-70.28,-39.04}}, color={0,0,127}));
    connect(pin_p, dCMotor_Angle2.pin_p) annotation (Line(points={{-13,-93},{
            -64,-93},{-64,-64},{50,-64},{50,-48.56},{48.76,-48.56}}, color={0,0,
            255}));
    connect(pin_p, constantVoltage.p)
      annotation (Line(points={{-13,-93},{-2,-93}}, color={0,0,255}));
    connect(constantVoltage.n, pin_n) annotation (Line(points={{12,-93},{16,-93},
            {16,-93},{19,-93}}, color={0,0,255}));
    connect(idealGear2.flange_b, sCARAKinematics.flange2)
      annotation (Line(points={{16,30},{16,76.32},{15,76.32}}, color={0,0,0}));
    connect(sine1.y, GearRatio2.u)
      annotation (Line(points={{-23,-12.5},{-23,-23}}, color={0,0,127}));
    connect(sine.y, GearRatio1.u) annotation (Line(points={{-77,-12.5},{-77,
            -17.25},{-77,-17.25},{-77,-23}}, color={0,0,127}));
    connect(sine2.y, GearR2TRatio.u) annotation (Line(points={{33,-8.5},{33,-23}},
                                           color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
  end SCARA;

  model DCMotor_Angle
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=k_Angle,
      Ti=i_Angle,
      Td=d_Angle,
      yMax=yMax_Angle)
               annotation (Placement(transformation(extent={{52,4},{72,24}})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
      annotation (Placement(transformation(extent={{32,-30},{52,-10}})));

     parameter Real k_Angle;
     parameter Real i_Angle;
     parameter Real d_Angle;
     parameter Real yMax_Angle;




    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
      annotation (Placement(transformation(extent={{88,-2},{108,18}}),
          iconTransformation(extent={{88,-2},{108,18}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{-76,-98},{-56,-78}}),
          iconTransformation(extent={{-76,-98},{-56,-78}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{58,-96},{78,-76}}),
          iconTransformation(extent={{58,-96},{78,-76}})));
    Modelica.Blocks.Interfaces.RealInput u
      annotation (Placement(transformation(extent={{-122,-12},{-82,28}}),
          iconTransformation(extent={{-122,-12},{-82,28}})));
    DCMotor_Speed dCMotor_Speed(
      k_Speed=1.4,
      i_Speed=0.05,
      d_Speed=0,
      yMax_Speed=24)
      annotation (Placement(transformation(extent={{-34,-40},{0,-20}})));
  equation
    connect(angleSensor.phi, PID.u_m)
      annotation (Line(points={{53,-20},{62,-20},{62,2}}, color={0,0,127}));
    connect(flange_a, flange_a)
      annotation (Line(points={{98,8},{98,8},{96,8},{96,8},{98,8},{98,8}},
                                                     color={0,0,0}));
    connect(PID.u_s, u)
      annotation (Line(points={{50,14},{-26,14},{-26,8},{-102,8}},
                                                   color={0,0,127}));
    connect(angleSensor.flange, dCMotor_Speed.flange) annotation (Line(points={{32,-20},
            {8,-20},{8,-30},{-2.72,-30}},      color={0,0,0}));
    connect(flange_a, dCMotor_Speed.flange) annotation (Line(points={{98,8},{82,
            8},{82,-30},{-2.72,-30}},   color={0,0,0}));
    connect(PID.y, dCMotor_Speed.u) annotation (Line(points={{73,14},{90,14},{
            90,78},{-60,78},{-60,-30},{-29.58,-30}},     color={0,0,127}));
    connect(dCMotor_Speed.pin_p, pin_p) annotation (Line(points={{-28.56,-37.4},
            {-30,-37.4},{-30,-72},{-66,-72},{-66,-88}},
                                                      color={0,0,255}));
    connect(dCMotor_Speed.pin_n, pin_n) annotation (Line(points={{-7.48,-37.6},
            {-7.48,-70},{68,-70},{68,-86}}, color={0,0,255}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false), graphics={Bitmap(extent=
               {{-152,-92},{156,70}}, fileName=
                "modelica://Praktikum4/./Bilder/DCMotor.PNG"), Bitmap(extent={{
                -64,58},{46,96}}, fileName=
                "modelica://Praktikum4/./Bilder/Angle.png")}),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=10,
        __Dymola_NumberOfIntervals=50000,
        __Dymola_Algorithm="Dassl"));
  end DCMotor_Angle;

  model DCMotor_Speed
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      k= k_Speed,
      Ti= i_Speed,
      Td= d_Speed,
      yMax= yMax_Speed) annotation (Placement(transformation(extent={{62,26},{82,46}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{20,6},{40,26}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange flange
      annotation (Placement(transformation(extent={{80,-4},{88,4}}),
          iconTransformation(extent={{80,-4},{88,4}})));
    Modelica.Blocks.Interfaces.RealInput u
      annotation (Placement(transformation(extent={{-80,-6},{-68,6}}),
          iconTransformation(extent={{-80,-6},{-68,6}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{-78,-84},{-58,-64}}),
          iconTransformation(extent={{-78,-84},{-58,-64}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{46,-86},{66,-66}}),
          iconTransformation(extent={{46,-86},{66,-66}})));
    DCMotorDriver dCMotorDriver1
      annotation (Placement(transformation(extent={{-54,-54},{-30,-32}})));
    DCMotor dCMotor
      annotation (Placement(transformation(extent={{-26,-16},{-6,4}})));

     parameter Real k_Speed;
     parameter Real i_Speed;
     parameter Real d_Speed;
     parameter Real yMax_Speed;

  equation
    connect(speedSensor.w, PID.u_m) annotation (Line(points={{41,16},{72,16},{
            72,24}},      color={0,0,127}));
    connect(flange, flange)
      annotation (Line(points={{84,0},{84,0}},   color={0,0,0}));
    connect(u, PID.u_s)
      annotation (Line(points={{-74,0},{-70,0},{-70,36},{60,36}},
                                                   color={0,0,127}));
    connect(PID.y, dCMotorDriver1.tarVol) annotation (Line(points={{83,36},{90,
            36},{90,96},{-42,96},{-42,-30.9}}, color={0,0,127}));
    connect(pin_p, dCMotorDriver1.p_in) annotation (Line(points={{-68,-74},{-68,
            -38.6},{-54,-38.6}},     color={0,0,255}));
    connect(dCMotorDriver1.p_out, dCMotor.pin_n) annotation (Line(points={{-30,-38.6},
            {-20.7,-38.6},{-20.7,-13.7}},       color={0,0,255}));
    connect(dCMotorDriver1.n_out, dCMotor.pin_p) annotation (Line(points={{-30,-47.4},
            {-10.7,-47.4},{-10.7,-13.7}},       color={0,0,255}));
    connect(dCMotor.flange_a, flange) annotation (Line(points={{-7.3,-4.3},{45.35,
            -4.3},{45.35,0},{84,0}},        color={0,0,0}));
    connect(speedSensor.flange, dCMotor.flange_a) annotation (Line(points={{20,16},
            {6,16},{6,-4.3},{-7.3,-4.3}},     color={0,0,0}));
    connect(dCMotorDriver1.n_in, pin_n) annotation (Line(points={{-54,-47.4},{
            -56,-47.4},{-56,-64},{56,-64},{56,-76}}, color={0,0,255}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false), graphics={Bitmap(extent=
               {{-174,-88},{184,54}}, fileName=
                "modelica://Praktikum4/Bilder/DCMotor.PNG"), Bitmap(
            extent={{-72,-28},{72,28}},
            fileName="modelica://Praktikum4/Bilder/Speed.png",
            origin={0,70},
            rotation=0)}),
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
          origin={4,26})));
    Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)        annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-46,4})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed
      annotation (Placement(transformation(extent={{46,-66},{66,-46}})));
  equation
    connect(emf.n, pin_p) annotation (Line(points={{4,-48},{4,-78},{52,-78}},
          color={0,0,255}));
    connect(emf.p, resistor.n)
      annotation (Line(points={{4,-28},{4,16}},    color={0,0,255}));
    connect(inductor.n, resistor.p)
      annotation (Line(points={{-46,-6},{-46,36},{4,36}},
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
                "modelica://Praktikum4/./Bilder/DCMotor.PNG")}), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DCMotor;
  annotation (uses(Modelica(version="3.2.3")));
end Praktikum4;
