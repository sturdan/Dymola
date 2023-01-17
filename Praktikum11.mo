package Praktikum11
  model ElectricDrive
    parameter Real maxTorque = 150;
    Modelica.Blocks.Interfaces.RealInput u annotation (
      Placement(transformation(extent = {{-124, -20}, {-84, 20}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque annotation (
      Placement(transformation(extent = {{-6, -10}, {14, 10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
      Placement(transformation(extent = {{88, -10}, {108, 10}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 1) annotation (
      Placement(transformation(extent = {{42, -10}, {62, 10}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 0.1) annotation (
      Placement(transformation(extent = {{-72, -10}, {-52, 10}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax = maxTorque) annotation (
      Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
  equation
    connect(torque.flange, inertia.flange_a) annotation (
      Line(points = {{14, 0}, {42, 0}}, color = {0, 0, 0}));
    connect(inertia.flange_b, flange_a) annotation (
      Line(points = {{62, 0}, {98, 0}}, color = {0, 0, 0}));
    connect(u, firstOrder.u) annotation (
      Line(points = {{-104, 0}, {-74, 0}}, color = {0, 0, 127}));
    connect(firstOrder.y, limiter.u) annotation (
      Line(points = {{-51, 0}, {-42, 0}}, color = {0, 0, 127}));
    connect(limiter.y, torque.tau) annotation (
      Line(points = {{-19, 0}, {-8, 0}}, color = {0, 0, 127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end ElectricDrive;

  model DriveTrain
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = 6) annotation (
      Placement(transformation(extent = {{-8, -10}, {12, 10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
      Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation (
      Placement(transformation(extent = {{90, -10}, {110, 10}})));
  equation
    connect(idealGear.flange_b, flange_b) annotation (
      Line(points = {{12, 0}, {100, 0}}, color = {0, 0, 0}));
    connect(idealGear.flange_a, flange_a) annotation (
      Line(points = {{-8, 0}, {-100, 0}}, color = {0, 0, 0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end DriveTrain;

  model Chassis1D
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
      Placement(transformation(extent = {{-84, -12}, {-64, 8}})));
    Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a1 annotation (
      Placement(transformation(extent = {{76, -12}, {96, 8}})));
    Modelica.Mechanics.Rotational.Components.IdealRollingWheel idealRollingWheel(radius = 0.31) annotation (
      Placement(transformation(extent = {{-28, -14}, {-8, 6}})));
  equation
    connect(idealRollingWheel.flangeT, flange_a1) annotation (
      Line(points = {{-8, -4}, {38, -4}, {38, -2}, {86, -2}}, color = {0, 127, 0}));
    connect(idealRollingWheel.flangeR, flange_a) annotation (
      Line(points = {{-28, -4}, {-52, -4}, {-52, -2}, {-74, -2}}, color = {0, 0, 0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Chassis1D;

  model Body1D
    Modelica.Mechanics.Translational.Components.Mass mass(m = 1300) annotation (
      Placement(transformation(extent = {{-14, -6}, {6, 14}})));
    Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a annotation (
      Placement(transformation(extent = {{-96, -6}, {-76, 14}})));
    Modelica.Mechanics.Translational.Sources.QuadraticSpeedDependentForce quadraticSpeedDependentForce(f_nominal = -35.478, v_nominal(displayUnit = "m/s") = 10) annotation (
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 0, origin = {72, 4})));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(extent = {{22, -80}, {42, -60}})));
    Modelica.Blocks.Interfaces.RealOutput y annotation (
      Placement(transformation(extent = {{72, -80}, {92, -60}})));
  equation
    connect(quadraticSpeedDependentForce.flange, mass.flange_b) annotation (
      Line(points = {{62, 4}, {6, 4}}, color = {0, 127, 0}));
    connect(flange_a, flange_a) annotation (
      Line(points = {{-86, 4}, {-86, 4}}, color = {0, 127, 0}));
    connect(mass.flange_a, flange_a) annotation (
      Line(points = {{-14, 4}, {-86, 4}}, color = {0, 127, 0}));
    connect(mass.flange_a, speedSensor.flange) annotation (
      Line(points = {{-14, 4}, {-14, -70}, {22, -70}}, color = {0, 127, 0}));
    connect(speedSensor.v, y) annotation (
      Line(points = {{43, -70}, {82, -70}}, color = {0, 0, 127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Body1D;

  model Driver
    parameter String fileName = "NoName" "File where matrix is stored" annotation (
      Dialog(group = "Table data definition", loadSelector(filter = "Text files (*.txt);;MATLAB MAT-files (*.mat)", caption = "Open file in which table is present")));
    parameter Real k;
    Modelica.Blocks.Interfaces.RealInput car_speed annotation (
      Placement(transformation(extent = {{-98, -44}, {-70, -16}})));
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(columns = {2, 3}, extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
      fileName="C:/Users/sturmdan/Documents/Dymola/sort1_steering.txt",                                                                                                                                    tableName = "Cycle", tableOnFile = true) annotation (
      Placement(transformation(extent = {{-96, 48}, {-76, 68}})));
    Modelica.Blocks.Math.Feedback feedback annotation (
      Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
    Modelica.Blocks.Math.Gain KmhMs(k = 1 / 3.6) annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-72, 18})));
    Modelica.Blocks.Interfaces.RealOutput car_torque annotation (
      Placement(transformation(extent = {{54, -10}, {74, 10}})));
    Modelica.Blocks.Math.Gain gain(k = k) annotation (
      Placement(transformation(extent = {{-6, -10}, {14, 10}})));
    Modelica.Blocks.Math.Gain gain1(k=0.5)   annotation (
      Placement(visible = true, transformation(extent = {{38, 46}, {58, 66}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation (
      Placement(transformation(extent={{8,66},{28,46}})));
    Modelica.Blocks.Interfaces.RealInput driving_angle_deg
      annotation (Placement(transformation(extent={{-80,76},{-56,100}})));
    Modelica.Blocks.Math.Gain deg_to_rad(k=3.14/180)
      annotation (Placement(transformation(extent={{-40,50},{-26,64}})));
    Modelica.Blocks.Interfaces.RealOutput steering_angle annotation (
      Placement(transformation(extent = {{78, 46}, {98, 66}})));
  equation
    connect(car_speed, feedback.u2) annotation (
      Line(points = {{-84, -30}, {-62, -30}, {-62, -8}, {-50, -8}}, color = {0, 0, 127}));
    connect(KmhMs.y, feedback.u1) annotation (
      Line(points = {{-72, 7}, {-74, 7}, {-74, 0}, {-58, 0}}, color = {0, 0, 127}));
    connect(feedback.y, gain.u) annotation (
      Line(points = {{-41, 0}, {-8, 0}}, color = {0, 0, 127}));
    connect(gain.y, car_torque) annotation (
      Line(points = {{15, 0}, {64, 0}}, color = {0, 0, 127}));
    connect(combiTimeTable.y[1], KmhMs.u) annotation (
      Line(points = {{-75, 58}, {-76, 58}, {-76, 30}, {-72, 30}}, color = {0, 0, 127}));
    connect(combiTimeTable.y[2], deg_to_rad.u) annotation (Line(points={{-75,58},
            {-58,58},{-58,57},{-41.4,57}}, color={0,0,127}));
    connect(deg_to_rad.y, feedback1.u1) annotation (Line(points={{-25.3,57},{
            -2.5,57},{-2.5,56},{10,56}}, color={0,0,127}));
  connect(feedback1.y, gain1.u) annotation (
      Line(points={{27,56},{36,56}},      color = {0, 0, 127}));
  connect(gain1.y, steering_angle) annotation (
      Line(points = {{59, 56}, {88, 56}}, color = {0, 0, 127}));
    connect(driving_angle_deg, feedback1.u2)
      annotation (Line(points={{-68,88},{18,88},{18,64}}, color={0,0,127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Driver;

  model HybrdiCar1D
    Driver driver(fileName = "C:/Users/sturmdan/Documents/Dymola/sort1.txt", k = 50) annotation (
      Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
    Body1D body1D annotation (
      Placement(transformation(extent = {{56, -10}, {76, 10}})));
    Chassis1D chassis1D annotation (
      Placement(transformation(extent = {{26, -10}, {46, 10}})));
    DriveTrain driveTrain annotation (
      Placement(transformation(extent = {{-4, -10}, {16, 10}})));
    ElectricDrive electricDrive annotation (
      Placement(transformation(extent = {{-36, 4}, {-16, 24}})));
    CombustionEngineDrive combustionEngineDrive(omegaStart = 0) annotation (
      Placement(transformation(extent = {{-36, -28}, {-16, -8}})));
    Modelica.Blocks.Math.Gain gain(k = 0.5) annotation (
      Placement(transformation(extent = {{-52, -4}, {-44, 4}})));
  equation
    connect(chassis1D.flange_a1, body1D.flange_a) annotation (
      Line(points = {{44.6, -0.2}, {42.8, -0.2}, {42.8, 0.4}, {57.4, 0.4}}, color = {0, 127, 0}));
    connect(driveTrain.HL, chassis1D.flange_a) annotation (
      Line(points = {{16, 0}, {22, 0}, {22, -0.2}, {28.6, -0.2}}, color = {0, 0, 0}));
    connect(body1D.y, driver.car_speed) annotation (
      Line(points = {{74.2, -7}, {94, -7}, {94, -46}, {-88, -46}, {-88, -3}, {-78.4, -3}}, color = {0, 0, 127}));
    connect(gain.u, driver.car_torque) annotation (
      Line(points = {{-52.8, 0}, {-63.6, 0}}, color = {0, 0, 127}));
    connect(gain.y, combustionEngineDrive.u) annotation (
      Line(points = {{-43.6, 0}, {-42, 0}, {-42, -18}, {-36.2, -18}}, color = {0, 0, 127}));
    connect(electricDrive.u, gain.y) annotation (
      Line(points = {{-36.4, 14}, {-42, 14}, {-42, 0}, {-43.6, 0}}, color = {0, 0, 127}));
    connect(electricDrive.flange_a, driveTrain.flange_a) annotation (
      Line(points = {{-16.2, 14}, {-10, 14}, {-10, 0}, {-4, 0}}, color = {0, 0, 0}));
    connect(combustionEngineDrive.flange_a, driveTrain.flange_a) annotation (
      Line(points = {{-16, -18}, {-12, -18}, {-12, 0}, {-4, 0}}, color = {0, 0, 0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)),
      experiment(StopTime = 100, __Dymola_Algorithm = "Dassl"));
  end HybrdiCar1D;

  model CombustionEngineDrive
    Real powerOutput = inertia.w * torque.tau "power output of CombustionEngine";
    parameter Real omegaStart = 0;
    Modelica.Mechanics.Rotational.Sources.Torque torque annotation (
      Placement(transformation(extent = {{-4, -10}, {16, 10}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 1, w(start = omegaStart, fixed = true)) annotation (
      Placement(transformation(extent = {{44, -10}, {64, 10}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 0.1) annotation (
      Placement(transformation(extent = {{-70, -10}, {-50, 10}})));
    Modelica.Blocks.Interfaces.RealInput u annotation (
      Placement(transformation(extent = {{-122, -20}, {-82, 20}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
      Placement(transformation(extent = {{90, -10}, {110, 10}})));
    Modelica.Blocks.Nonlinear.VariableLimiter variableLimiter annotation (
      Placement(transformation(extent = {{-38, -10}, {-18, 10}})));
    Modelica.Blocks.Tables.CombiTable1D combiTable1D(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
      fileName="C:/Users/sturmdan/Documents/Dymola/ced.txt",                                                                                                                  tableName = "maxTorque", tableOnFile = true) annotation (
      Placement(transformation(extent = {{-66, 34}, {-46, 54}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {44, 56})));
    Modelica.Blocks.Sources.RealExpression realExpression annotation (
      Placement(transformation(extent = {{-72, -50}, {-52, -30}})));
    Modelica.Blocks.Math.Gain gain(k = 60 / (3.14 * 2)) annotation (
      Placement(transformation(extent = {{-88, 36}, {-74, 50}})));
  equation
    connect(torque.flange, inertia.flange_a) annotation (
      Line(points = {{16, 0}, {44, 0}}, color = {0, 0, 0}));
    connect(inertia.flange_b, flange_a) annotation (
      Line(points = {{64, 0}, {100, 0}}, color = {0, 0, 0}));
    connect(u, firstOrder.u) annotation (
      Line(points = {{-102, 0}, {-72, 0}}, color = {0, 0, 127}));
    connect(firstOrder.y, variableLimiter.u) annotation (
      Line(points = {{-49, 0}, {-40, 0}}, color = {0, 0, 127}));
    connect(variableLimiter.y, torque.tau) annotation (
      Line(points = {{-17, 0}, {-6, 0}}, color = {0, 0, 127}));
    connect(combiTable1D.y[1], variableLimiter.limit1) annotation (
      Line(points = {{-45, 44}, {-42, 44}, {-42, 8}, {-40, 8}}, color = {0, 0, 127}));
    connect(speedSensor.flange, flange_a) annotation (
      Line(points = {{44, 46}, {84, 46}, {84, 0}, {100, 0}}, color = {0, 0, 0}));
    connect(realExpression.y, variableLimiter.limit2) annotation (
      Line(points = {{-51, -40}, {-48, -40}, {-48, -8}, {-40, -8}}, color = {0, 0, 127}));
    connect(combiTable1D.u[1], gain.y) annotation (
      Line(points = {{-68, 44}, {-70, 44}, {-70, 43}, {-73.3, 43}}, color = {0, 0, 127}));
    connect(gain.u, speedSensor.w) annotation (
      Line(points = {{-89.4, 43}, {-92, 43}, {-92, 44}, {-94, 44}, {-94, 67}, {44, 67}}, color = {0, 0, 127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end CombustionEngineDrive;

  model Motortest
    CombustionEngineDrive combustionEngineDrive(omegaStart = 80) annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y = 1500) annotation (
      Placement(transformation(extent = {{-58, -10}, {-38, 10}})));
  equation
    connect(realExpression.y, combustionEngineDrive.u) annotation (
      Line(points = {{-37, 0}, {-10.2, 0}}, color = {0, 0, 127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Motortest;

  package PlanarMechanics "Library to model 2-dimensional, planar mechanical systems"
    extends Modelica.Icons.Package;
    import SI = Modelica.SIunits;
    import MB = Modelica.Mechanics.MultiBody;

    package Examples "Collection of introductory examples"
      extends Modelica.Icons.ExamplesPackage;

      model FreeBody "A simple free falling body"
        extends Modelica.Icons.Example;
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-30, -10}, {-10, 10}})));
        Parts.Body body(m = 1, I = 0.1, animate = true, a(each fixed = false), r(each fixed = true), v(each fixed = true), phi(fixed = true), w(fixed = true)) annotation (
          Placement(transformation(extent = {{10, -10}, {30, 10}})));
        annotation (
          experiment(StopTime = 3),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The gravity is defined in the planarWorld component</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/FreeBody_1.png\" alt=\"Diagram FreeBody_1\"></p>
<p>The DAE has 73&nbsp;scalar unknowns and 73&nbsp;scalar equations.</p>
</html>"));
      end FreeBody;

      model Pendulum "A free swinging pendulum"
        extends Modelica.Icons.Example;
        Parts.Body body(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{30, -10}, {50, 10}})));
        Joints.Revolute revolute(useFlange = false, phi(fixed = true, start = 0), w(fixed = true, start = 0)) annotation (
          Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
        Parts.FixedTranslation fixedTranslation(r = {1, 0}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Parts.Fixed fixed(phi = 0) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-60, 0})));
        inner PlanarWorld planarWorld(defaultWidthFraction = 10) annotation (
          Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
      equation
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-20, 0}, {-15, 0}, {-10, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, body.frame_a) annotation (
          Line(points = {{10, 0}, {10, 0}, {30, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_a, fixed.frame) annotation (
          Line(points = {{-40, 0}, {-46, 0}, {-46, -1.22125e-015}, {-50, -1.22125e-015}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 3),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/Pendulum_1.png\" alt=\"Diagram Pendulum_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/Pendulum_2.png\" alt=\"Diagram Pendulum_2\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/Pendulum_3.png\" alt=\"Diagram Pendulum_3\"></p>
<p>Selected continuous time states</p>
<ul>
<li>revolute.phi</li>
<li>revolute.w</li>
</ul>
</html>"));
      end Pendulum;

      model PendulumExcited "A swinging pendulum excited by a world force"
        extends Modelica.Icons.Example;
        Parts.Body body(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{60, -10}, {80, 10}})));
        Joints.Revolute revolute(useFlange = false, phi(fixed = true, start = 0), w(fixed = true, start = 0), stateSelect = StateSelect.always) annotation (
          Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
        Parts.FixedTranslation fixedTranslation(r = {1, 0}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Parts.Fixed fixed(phi = 0) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-60, 0})));
        inner PlanarWorld planarWorld(defaultWidthFraction = 10, defaultN_to_m = 10) annotation (
          Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
        Sensors.CutForce cutForce(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation (
          Placement(transformation(extent = {{20, -10}, {40, 10}})));
        Sources.WorldForce worldForce(color = {255, 0, 0}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation (
          Placement(transformation(extent = {{20, -50}, {40, -30}})));
        Modelica.Blocks.Sources.Sine signalVec3[3](each freqHz = 1, amplitude = {0, -5, 0}, each startTime = 1.8) "Vector of three excitation signals" annotation (
          Placement(transformation(extent = {{-20, -50}, {0, -30}})));
      equation
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-20, 0}, {-15, 0}, {-10, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_a, fixed.frame) annotation (
          Line(points = {{-40, 0}, {-46, 0}, {-46, -1.22125e-015}, {-50, -1.22125e-015}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, cutForce.frame_a) annotation (
          Line(points = {{10, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutForce.frame_b, body.frame_a) annotation (
          Line(points = {{40, 0}, {60, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(worldForce.frame_b, body.frame_a) annotation (
          Line(points = {{40, -40}, {50, -40}, {50, 0}, {60, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(signalVec3.y, worldForce.force) annotation (
          Line(points = {{1, -40}, {18, -40}}, color = {0, 0, 127}));
        annotation (
          experiment(StopTime = 3),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This example demonstrates simple pendulum excited by a world force.
The animation parameters of cut and excitation forces can be changed
at once using default parameters of planarWorld.
</p>
</html>"));
      end PendulumExcited;

      model DoublePendulum "Simple double pendulum with two revolute joints and two bodies"
        extends Modelica.Icons.Example;
        Parts.Body body(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{0, 10}, {20, 30}})));
        Parts.FixedTranslation fixedTranslation(r = {1, 0}) annotation (
          Placement(transformation(extent = {{-40, 10}, {-20, 30}})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-90, 20})));
        Parts.Body body1(m = 0.2, I = 0.01) annotation (
          Placement(transformation(extent = {{60, -30}, {80, -10}})));
        Parts.FixedTranslation fixedTranslation1(r = {0.4, 0}) annotation (
          Placement(transformation(extent = {{30, -30}, {50, -10}})));
        inner PlanarWorld planarWorld(enableAnimation = true, animateWorld = true, animateGravity = true) annotation (
          Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
        Joints.Revolute revolute(phi(fixed = true, start = 0), w(fixed = true, start = 0), stateSelect = StateSelect.always) annotation (
          Placement(transformation(extent = {{-70, 10}, {-50, 30}})));
        Joints.Revolute revolute1(phi(fixed = true, start = 0), w(fixed = true, start = 0), stateSelect = StateSelect.always) annotation (
          Placement(transformation(extent = {{0, -30}, {20, -10}})));
      equation
        connect(fixedTranslation.frame_b, body.frame_a) annotation (
          Line(points = {{-20, 20}, {0, 20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation1.frame_b, body1.frame_a) annotation (
          Line(points = {{50, -20}, {50, -20}, {60, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed.frame, revolute.frame_a) annotation (
          Line(points = {{-80, 20}, {-76, 20}, {-70, 20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-50, 20}, {-40, 20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute1.frame_b, fixedTranslation1.frame_a) annotation (
          Line(points = {{20, -20}, {30, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, revolute1.frame_a) annotation (
          Line(points = {{-20, 20}, {-10, 20}, {-10, -20}, {0, -20}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 10),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Beware this is a chaotic system. However, the chaotic part should start after 10s.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/DoublePendulum_1.png\" alt=\"Diagram DoublePendulum_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/DoublePendulum_2.png\" alt=\"Diagram DoublePendulum_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>revolute.phi</li>
<li>revolute.w</li>
<li>revolute1.phi</li>
<li>revolute1.w</li>
</ul>
</html>"));
      end DoublePendulum;

      model MeasureDemo "Measure demo"
        extends Modelica.Icons.Example;
        Parts.Body body(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{10, 10}, {30, 30}})));
        Parts.FixedTranslation fixedTranslation(r = {1, 0}) annotation (
          Placement(transformation(extent = {{-40, 10}, {-20, 30}})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-90, 20})));
        Parts.Body body1(m = 0.4, I = 0.02) annotation (
          Placement(transformation(extent = {{70, -30}, {90, -10}})));
        Parts.FixedTranslation fixedTranslation1(r = {0.4, 0}) annotation (
          Placement(transformation(extent = {{30, -30}, {50, -10}})));
        inner PlanarWorld planarWorld(enableAnimation = true, animateWorld = false, animateGravity = false) annotation (
          Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
        Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation (
          Placement(transformation(extent = {{10, -10}, {-10, 10}}, origin = {-10, -90})));
        Sensors.RelativePosition relativePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.world) annotation (
          Placement(transformation(extent = {{40, 82}, {60, 102}})));
        Joints.Revolute revolute1(phi(fixed = true), w(fixed = true)) annotation (
          Placement(transformation(extent = {{-70, 10}, {-50, 30}})));
        Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation (
          Placement(transformation(extent = {{20, -80}, {0, -60}})));
        Sensors.RelativeVelocity relativeVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b) annotation (
          Placement(transformation(extent = {{20, 62}, {40, 82}})));
        Sensors.AbsoluteAcceleration absoluteAcceleration(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation (
          Placement(transformation(extent = {{10, -10}, {-10, 10}}, origin = {30, -50})));
        Sensors.RelativeAcceleration relativeAcceleration(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b) annotation (
          Placement(transformation(extent = {{0, 42}, {20, 62}})));
        Joints.Revolute revolute2(phi(fixed = true), w(fixed = true)) annotation (
          Placement(transformation(extent = {{0, -30}, {20, -10}})));
      equation
        connect(fixedTranslation.frame_b, body.frame_a) annotation (
          Line(points = {{-20, 20}, {-2, 20}, {10, 20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation1.frame_b, body1.frame_a) annotation (
          Line(points = {{50, -20}, {56, -20}, {70, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed.frame, revolute1.frame_a) annotation (
          Line(points = {{-80, 20}, {-70, 20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute1.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-50, 20}, {-40, 20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(absoluteAcceleration.frame_resolve, absoluteAcceleration.frame_a) annotation (
          Line(points = {{30, -60}, {30, -50}, {40, -50}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(revolute2.frame_b, fixedTranslation1.frame_a) annotation (
          Line(points = {{20, -20}, {20, -20}, {30, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute2.frame_a, fixedTranslation.frame_b) annotation (
          Line(points = {{0, -20}, {-10, -20}, {-10, 20}, {-20, 20}, {-20, 20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, relativeAcceleration.frame_a) annotation (
          Line(points = {{-20, 20}, {-20, 20}, {-10, 20}, {-10, 52}, {0, 52}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, relativeVelocity.frame_a) annotation (
          Line(points = {{-20, 20}, {-20, 20}, {-10, 20}, {-10, 72}, {20, 72}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, relativePosition.frame_a) annotation (
          Line(points = {{-20, 20}, {-20, 20}, {-10, 20}, {-10, 92}, {40, 92}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativeAcceleration.frame_b, body1.frame_a) annotation (
          Line(points = {{20, 52}, {60, 52}, {60, -20}, {70, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativeVelocity.frame_b, body1.frame_a) annotation (
          Line(points = {{40, 72}, {60, 72}, {60, -20}, {70, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativePosition.frame_b, body1.frame_a) annotation (
          Line(points = {{60, 92}, {60, 36}, {60, -20}, {70, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body1.frame_a, absoluteAcceleration.frame_a) annotation (
          Line(points = {{70, -20}, {70, -20}, {60, -20}, {60, -50}, {40, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body1.frame_a, absoluteVelocity.frame_a) annotation (
          Line(points = {{70, -20}, {70, -20}, {60, -20}, {60, -70}, {20, -70}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body1.frame_a, absolutePosition.frame_a) annotation (
          Line(points = {{70, -20}, {70, -20}, {60, -20}, {60, -90}, {0, -90}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 10),
          Documentation(info = "<html>
<p>This example shows how to use absolute and relative sensors for position, velocity and acceleration. For demonstration purposes a double pendulum is used.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end MeasureDemo;

      model PowerDistanceDemo "Power and distance sensor demo"
        extends Modelica.Icons.Example;
        Parts.Body body(I = 0.1, m = 0.5) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -90})));
        Parts.FixedTranslation fixedTranslation(r = {0, -1}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -30})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-70, 30})));
        Parts.Body body1(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{0, 20}, {20, 40}})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        Parts.Damper damper(d = 1) annotation (
          Placement(transformation(extent = {{-40, 40}, {-20, 60}})));
        Joints.Revolute revolute(w(fixed = true), phi(fixed = true, start = 2.6179938779915)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, 0})));
        Joints.Prismatic prismatic(r = {1, 0}, v(fixed = true), s(fixed = true, start = 1.0)) annotation (
          Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
        Sensors.Power power annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -60})));
        Sensors.Distance distance annotation (
          Placement(transformation(extent = {{2, 62}, {22, 82}})));
      equation
        connect(damper.frame_a, fixed.frame) annotation (
          Line(points = {{-40, 50}, {-60, 50}, {-60, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_a, body1.frame_a) annotation (
          Line(points = {{-10, 10}, {-10, 30}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-10, -10}, {-10, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed.frame, prismatic.frame_a) annotation (
          Line(points = {{-60, 30}, {-40, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_b, body1.frame_a) annotation (
          Line(points = {{-20, 30}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(damper.frame_b, prismatic.frame_b) annotation (
          Line(points = {{-20, 50}, {-10, 50}, {-10, 30}, {-20, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body.frame_a, power.frame_b) annotation (
          Line(points = {{-10, -80}, {-10, -70}}, color = {95, 95, 95}, thickness = 0.5));
        connect(distance.frame_a, fixed.frame) annotation (
          Line(points = {{2, 72}, {-60, 72}, {-60, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(distance.frame_b, power.frame_b) annotation (
          Line(points = {{22, 72}, {58, 72}, {58, -70}, {-10, -70}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, power.frame_a) annotation (
          Line(points = {{-10, -40}, {-10, -50}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 6),
          Documentation(info = "<html>
<p>This example shows how to use sensors for power and distance. The crane crab is used as an example.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end PowerDistanceDemo;

      model CraneCrab "A damped crane crab"
        extends Modelica.Icons.Example;
        Parts.Body body(I = 0.1, m = 0.5) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -60})));
        Parts.FixedTranslation fixedTranslation(r = {0, -1}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -30})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-70, 30})));
        Parts.Body body1(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{0, 20}, {20, 40}})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        Joints.Revolute revolute(w(fixed = true), phi(fixed = true, start = 2.6179938779915)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, 0})));
        Joints.Prismatic prismatic(r = {1, 0}, v(fixed = true), useFlange = true, s(fixed = true, start = 0), animate = true) annotation (
          Placement(transformation(extent = {{-40, 40}, {-20, 20}})));
        Modelica.Mechanics.Translational.Components.Damper damper1D(d = 10) annotation (
          Placement(transformation(extent = {{-40, 50}, {-20, 70}})));
      equation
        connect(fixedTranslation.frame_b, body.frame_a) annotation (
          Line(points = {{-10, -40}, {-10, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_a, body1.frame_a) annotation (
          Line(points = {{-10, 10}, {-10, 30}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-10, -10}, {-10, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed.frame, prismatic.frame_a) annotation (
          Line(points = {{-60, 30}, {-40, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_b, body1.frame_a) annotation (
          Line(points = {{-20, 30}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(damper1D.flange_b, prismatic.flange_a) annotation (
          Line(points = {{-20, 60}, {-20, 40}, {-30, 40}}, color = {0, 127, 0}));
        connect(damper1D.flange_a, prismatic.support) annotation (
          Line(points = {{-40, 60}, {-40, 40}, {-36, 40}}, color = {0, 127, 0}));
        annotation (
          experiment(StopTime = 10),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/CraneCrab_1.png\" alt=\"Diagram CraneCrab_1\"></p>
<p>Selected continuous time states</p>
<ul>
<li>prismatic.s</li>
<li>prismatic.v</li>
<li>revolute.phi</li>
<li>revolute.w</li>
</ul>
</html>"));
      end CraneCrab;

      model ControlledCraneCrab "A controlled crane crab"
        extends Modelica.Icons.Example;
        Parts.Body body(m = 70, I = 0) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -60})));
        Parts.FixedTranslation fixedTranslation(r = {0, 2.5}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -30})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-70, 30})));
        Parts.Body body1(m = 250, I = 0, a(start = {1, 0})) annotation (
          Placement(transformation(extent = {{0, 20}, {20, 40}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
          Placement(transformation(extent = {{20, -10}, {40, 10}})));
        Modelica.Mechanics.Translational.Sources.Force force annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-10, 70})));
        Modelica.Blocks.Continuous.PID PID(Td = 0.2, k = -320 * 9.81 * 5, initType = Modelica.Blocks.Types.InitPID.InitialState, xi_start = 0, Ti = 1e9) annotation (
          Placement(transformation(extent = {{40, 60}, {20, 80}})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        Joints.Prismatic prismatic(r = {1, 0}, useFlange = true, s(fixed = true), v(fixed = true)) annotation (
          Placement(transformation(extent = {{-40, 40}, {-20, 20}})));
        Joints.Revolute revolute(useFlange = true, w(fixed = true), phi(fixed = true, start = -0.34906585039887)) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {-10, 0})));
      equation
        connect(fixedTranslation.frame_b, body.frame_a) annotation (
          Line(points = {{-10, -40}, {-10, -45}, {-10, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(force.f, PID.y) annotation (
          Line(points = {{2, 70}, {2, 70}, {19, 70}}, color = {0, 0, 127}));
        connect(angleSensor.phi, PID.u) annotation (
          Line(points = {{41, 0}, {70, 0}, {70, 70}, {42, 70}}, color = {0, 0, 127}));
        connect(fixed.frame, prismatic.frame_a) annotation (
          Line(points = {{-60, 30}, {-40, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_b, body1.frame_a) annotation (
          Line(points = {{-20, 30}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.flange_a, force.flange) annotation (
          Line(points = {{-30, 40}, {-30, 40}, {-30, 70}, {-20, 70}}, color = {0, 127, 0}));
        connect(prismatic.frame_b, revolute.frame_a) annotation (
          Line(points = {{-20, 30}, {-10, 30}, {-10, 10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-10, -10}, {-10, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.flange_a, angleSensor.flange) annotation (
          Line(points = {{0, 0}, {0, 0}, {20, 0}}));
        annotation (
          experiment(StopTime = 3),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>A simple PID (actually PD) controlles the pendulum into upright position.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/ControlledCraneCrab_1.png\" alt=\"Diagram ControlledCraneCrab_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/ControlledCraneCrab_2.png\" alt=\"Diagram ControlledCraneCrab_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>actuatedPrismatic.s</li>
<li>actuatedPrismatic.v</li>
<li>actuatedRevolute.phi</li>
<li>actuatedRevolute.w</li>
<li>PID.D.x</li>
<li>PID.I.y</li>
</ul>
</html>"));
      end ControlledCraneCrab;

      model InvertedCraneCrab "An inverted model of a pendulum"
        extends Modelica.Icons.Example;
        Parts.Body body(I = 0.1, m = 0.5) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -60})));
        Parts.FixedTranslation fixedTranslation(r = {0, 1}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, -30})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-70, 30})));
        Parts.Body body1(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{0, 20}, {20, 40}})));
        Modelica.Mechanics.Translational.Sources.Force force(useSupport = false) annotation (
          Placement(transformation(extent = {{0, 60}, {-20, 80}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
          Placement(transformation(extent = {{8, -10}, {28, 10}})));
        Modelica.Blocks.Math.InverseBlockConstraints inverseBlockConstraints annotation (
          Placement(transformation(extent = {{38, -20}, {88, 20}})));
        Modelica.Blocks.Sources.Ramp ramp(startTime = 0, duration = 0.5, height = 0.5, offset = -0.5) annotation (
          Placement(transformation(extent = {{-6, 6}, {6, -6}}, rotation = 180, origin = {74, 0})));
        Modelica.Blocks.Continuous.FirstOrder firstOrder(initType = Modelica.Blocks.Types.Init.SteadyState, T = 0.1) annotation (
          Placement(transformation(extent = {{62, -6}, {50, 6}})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        Joints.Revolute revolute(useFlange = true) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {-10, 0})));
        Joints.Prismatic prismatic(useFlange = true, r = {1, 0}, s(fixed = true), v(fixed = true)) annotation (
          Placement(transformation(extent = {{-40, 40}, {-20, 20}})));
      equation
        connect(inverseBlockConstraints.u1, angleSensor.phi) annotation (
          Line(points = {{35.5, 0}, {33.875, 0}, {33.875, 0}, {32.25, 0}, {32.25, 0}, {29, 0}}, color = {0, 0, 127}));
        connect(inverseBlockConstraints.y1, force.f) annotation (
          Line(points = {{89.25, 0}, {96, 0}, {96, 70}, {2, 70}}, color = {0, 0, 127}));
        connect(fixedTranslation.frame_b, body.frame_a) annotation (
          Line(points = {{-10, -40}, {-10, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(ramp.y, firstOrder.u) annotation (
          Line(points = {{67.4, 0}, {64.7, 0}, {64.7, 0}, {63.2, 0}}, color = {0, 0, 127}));
        connect(firstOrder.y, inverseBlockConstraints.u2) annotation (
          Line(points = {{49.4, 0}, {47.8, 0}, {47.8, 0}, {46.2, 0}, {46.2, 0}, {43, 0}}, color = {0, 0, 127}));
        connect(revolute.flange_a, angleSensor.flange) annotation (
          Line(points = {{0, 0}, {0, 0}, {8, 0}}));
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{-10, -10}, {-10, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_a, body1.frame_a) annotation (
          Line(points = {{-10, 10}, {-10, 30}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed.frame, prismatic.frame_a) annotation (
          Line(points = {{-60, 30}, {-40, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_b, body1.frame_a) annotation (
          Line(points = {{-20, 30}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(force.flange, prismatic.flange_a) annotation (
          Line(points = {{-20, 70}, {-30, 70}, {-30, 40}}, color = {0, 127, 0}));
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The trajectory is stipulated, the force is being measured.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/InvertedCraneCrab_1.png\" alt=\"Diagram InvertedCraneCrab_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/InvertedCraneCrab_2.png\" alt=\"Diagram InvertedCraneCrab_2\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/InvertedCraneCrab_3.png\" alt=\"Diagram InvertedCraneCrab_3\"></p>
<p>Selected continuous time states</p>
<ul>
<li>prismatic.s</li>
<li>prismatic.v</li>
<li>revolute.phi</li>
</ul>
</html>"),experiment(StopTime = 3));
      end InvertedCraneCrab;

      model SpringDemo "Spring demo"
        extends Modelica.Icons.Example;
        Parts.Body body(I = 0.1, m = 0.5) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -40})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-70, 40})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, -40}, {-60, -20}})));
        Parts.FixedTranslation fixedTranslation annotation (
          Placement(transformation(extent = {{-40, 30}, {-20, 50}})));
        Parts.Spring spring1(c_y = 10, s_rely0 = -0.5, c_x = 1, c_phi = 1e5, enableAssert = false) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270)));
        Parts.Damper damper(d = 1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-30, 0})));
        Joints.Prismatic prismatic(animate = false, r = {0, 1}, s(start = 0.2, fixed = true), v(fixed = true)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {30, 0})));
      equation
        connect(fixed.frame, fixedTranslation.frame_a) annotation (
          Line(points = {{-60, 40}, {-40, 40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(spring1.frame_a, fixedTranslation.frame_b) annotation (
          Line(points = {{0, 10}, {0, 10}, {0, 40}, {-20, 40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(spring1.frame_b, body.frame_a) annotation (
          Line(points = {{0, -10}, {0, -10}, {0, -30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(damper.frame_a, spring1.frame_a) annotation (
          Line(points = {{-30, 10}, {0, 10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(damper.frame_b, spring1.frame_b) annotation (
          Line(points = {{-30, -10}, {0, -10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(spring1.frame_a, prismatic.frame_a) annotation (
          Line(points = {{0, 10}, {30, 10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_b, spring1.frame_b) annotation (
          Line(points = {{30, -10}, {0, -10}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 5),
          Documentation(info = "<html>
<p>This example shows how to use a spring and a damper separately. The motion is constrained by a prismatic joint. The spring passes a point of zero length.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end SpringDemo;

      model SpringDamperDemo "Spring damper demo"
        extends Modelica.Icons.Example;
        Parts.SpringDamper springDamper(s_relx0 = 0, d_y = 1, s_rely0 = 0, d_phi = 0, c_y = 5, c_x = 5, d_x = 1, c_phi = 0) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270)));
        Parts.Body body(I = 0.1, m = 0.5, v(each fixed = true), phi(fixed = true), w(fixed = true), r(each fixed = true, start = {1, 1})) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -40})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-70, 40})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, -40}, {-60, -20}})));
        Parts.FixedTranslation fixedTranslation(r = {-1, 0}) annotation (
          Placement(transformation(extent = {{-40, 30}, {-20, 50}})));
      equation
        connect(fixed.frame, fixedTranslation.frame_a) annotation (
          Line(points = {{-60, 40}, {-40, 40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_b, springDamper.frame_a) annotation (
          Line(points = {{-20, 40}, {0, 40}, {0, 10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(springDamper.frame_b, body.frame_a) annotation (
          Line(points = {{0, -10}, {0, -18}, {0, -30}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 5),
          Documentation(info = "<html>
<p>This example shows how to use a spring and a damper in combination. The motion of the body is not constrained.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>
          "));
      end SpringDamperDemo;

      model KinematicLoop "An example of a kinematic loop (manual state selection)"
        extends Modelica.Icons.Example;
        Joints.Revolute revolute1(extraLine = true, phi(start = Modelica.Math.asin(0.4 / 0.5 / 2)), stateSelect = StateSelect.always) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, 40})));
        Joints.Revolute revolute3(phi(start = -Modelica.Math.asin(0.4 / 0.5 / 2))) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {80, 40})));
        Joints.Revolute revolute2 annotation (
          Placement(transformation(extent = {{20, -20}, {40, 0}})));
        Joints.Revolute revolute4(w(fixed = true, start = 0), cylinderLength = planarWorld.defaultJointLength * 1.5, cylinderDiameter = planarWorld.defaultJointWidth / 2, cylinderColor = {155, 0, 0}, extraLine = true, stateSelect = StateSelect.always, phi(fixed = true, start = -0.69813170079773)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, -40})));
        Joints.Prismatic prismatic1(r = {1, 0}, useFlange = true, s(start = 0.4, fixed = true), v(start = 0, fixed = true)) annotation (
          Placement(transformation(extent = {{20, 70}, {40, 50}})));
        Modelica.Mechanics.Translational.Components.SpringDamper springDamper1D(c = 20, d = 4, s_rel0 = 0.4) annotation (
          Placement(transformation(extent = {{0, 80}, {20, 100}})));
        Parts.Body body(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {50, -60})));
        Parts.FixedTranslation fixedTranslation1(r = {0, -0.5}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, 10})));
        Parts.FixedTranslation fixedTranslation2(r = {0, -0.5}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {80, 10})));
        Parts.FixedTranslation fixedTranslation3(r = {0, -0.6}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {10, -60})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-50, 60})));
        Modelica.Mechanics.Translational.Components.Fixed fixed1D annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-46, 90})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, 0}, {-60, 20}})));
      equation
        connect(fixedTranslation1.frame_a, revolute1.frame_b) annotation (
          Line(points = {{-20, 20}, {-20, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation2.frame_a, revolute3.frame_b) annotation (
          Line(points = {{80, 20}, {80, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute2.frame_a, fixedTranslation1.frame_b) annotation (
          Line(points = {{20, -10}, {-20, -10}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute2.frame_b, fixedTranslation2.frame_b) annotation (
          Line(points = {{40, -10}, {80, -10}, {80, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation3.frame_a, revolute4.frame_b) annotation (
          Line(points = {{0, -60}, {-20, -60}, {-20, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute1.frame_a, fixed.frame) annotation (
          Line(points = {{-20, 50}, {-20, 60}, {-40, 60}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed1D.flange, springDamper1D.flange_a) annotation (
          Line(points = {{-46, 90}, {0, 90}}, color = {0, 127, 0}));
        connect(revolute4.frame_a, fixedTranslation1.frame_b) annotation (
          Line(points = {{-20, -30}, {-20, -16}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body.frame_a, fixedTranslation3.frame_b) annotation (
          Line(points = {{40, -60}, {30, -60}, {20, -60}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic1.frame_a, fixed.frame) annotation (
          Line(points = {{20, 60}, {-40, 60}}, color = {95, 95, 95}, thickness = 0.5));
        connect(springDamper1D.flange_b, prismatic1.flange_a) annotation (
          Line(points = {{20, 90}, {30, 90}, {30, 70}}, color = {0, 127, 0}));
        connect(prismatic1.frame_b, revolute3.frame_a) annotation (
          Line(points = {{40, 60}, {80, 60}, {80, 50}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 6),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>In this version, the states are manually selected.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/KinematicLoop_1.png\" alt=\"Diagram KinematicLoop_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/KinematicLoop_2.png\" alt=\"Diagram KinematicLoop_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>revolute1.phi</li>
<li>revolute1.w</li>
<li>revolute4.phi</li>
<li>revolute4.w</li>
</ul>
</html>"));
      end KinematicLoop;

      model KinematicLoop_DynamicStateSelection "An example of a kinematic loop"
        extends Modelica.Icons.Example;
        Joints.Revolute revolute1(phi(start = Modelica.Math.asin(0.4 / 0.5 / 2))) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, 40})));
        Joints.Revolute revolute3(phi(start = -Modelica.Math.asin(0.4 / 0.5 / 2))) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {80, 40})));
        Joints.Revolute revolute2 annotation (
          Placement(transformation(extent = {{20, -20}, {40, 0}})));
        Joints.Revolute revolute4(w(fixed = true, start = 0), phi(fixed = true, start = -0.69813170079773)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, -40})));
        Joints.Prismatic prismatic1(r = {1, 0}, useFlange = true, s(fixed = true, start = 0.4), v(fixed = true, start = 0)) annotation (
          Placement(transformation(extent = {{20, 70}, {40, 50}})));
        Modelica.Mechanics.Translational.Components.SpringDamper springDamper1D(c = 20, s_rel0 = 0.4, d = 4) annotation (
          Placement(transformation(extent = {{0, 80}, {20, 100}})));
        Parts.Body body(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {50, -60})));
        Parts.FixedTranslation fixedTranslation1(r = {0, -0.5}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, 10})));
        Parts.FixedTranslation fixedTranslation2(r = {0, -0.5}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {80, 10})));
        Parts.FixedTranslation fixedTranslation3(r = {0, -0.6}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {10, -60})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-50, 60})));
        Modelica.Mechanics.Translational.Components.Fixed fixed1D annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-46, 90})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-80, 0}, {-60, 20}})));
      equation
        connect(fixedTranslation1.frame_a, revolute1.frame_b) annotation (
          Line(points = {{-20, 20}, {-20, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation2.frame_a, revolute3.frame_b) annotation (
          Line(points = {{80, 20}, {80, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute2.frame_a, fixedTranslation1.frame_b) annotation (
          Line(points = {{20, -10}, {-20, -10}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute2.frame_b, fixedTranslation2.frame_b) annotation (
          Line(points = {{40, -10}, {80, -10}, {80, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation3.frame_a, revolute4.frame_b) annotation (
          Line(points = {{0, -60}, {-20, -60}, {-20, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute1.frame_a, fixed.frame) annotation (
          Line(points = {{-20, 50}, {-20, 60}, {-40, 60}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed1D.flange, springDamper1D.flange_a) annotation (
          Line(points = {{-46, 90}, {0, 90}}, color = {0, 127, 0}));
        connect(revolute4.frame_a, fixedTranslation1.frame_b) annotation (
          Line(points = {{-20, -30}, {-20, -15}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body.frame_a, fixedTranslation3.frame_b) annotation (
          Line(points = {{40, -60}, {20, -60}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed.frame, prismatic1.frame_a) annotation (
          Line(points = {{-40, 60}, {20, 60}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic1.frame_b, revolute3.frame_a) annotation (
          Line(points = {{40, 60}, {80, 60}, {80, 50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(springDamper1D.flange_b, prismatic1.flange_a) annotation (
          Line(points = {{20, 90}, {30, 90}, {30, 70}}, color = {0, 127, 0}));
        annotation (
          experiment(StopTime = 6),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>In this version, the states are not manually set but might be dynamically selected by the simulation environment.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/KinematicLoop_1.png\" alt=\"Diagram KinematicLoop_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/KinematicLoop_2.png\" alt=\"Diagram KinematicLoop_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>There are 2&nbsp;sets of dynamic state selection.</li>
<ul>
<li>From set&nbsp;1 there there are 2&nbsp;states to be selected from:</li>
<ul>
<li>body.frame_a.phi</li>
<li>revolute2.phi</li>
<li>revolute4.phi</li>
<li>springDamper.s_rel</li>
</ul>
<li>From set&nbsp;2 there there are 2&nbsp;states to be selected from:</li>
<ul>
<li>body.w</li>
<li>revolute2.w</li>
<li>revolute4.w</li>
<li>springDamper.v_rel</li>
</ul>
</ul>
</ul>
</html>"));
      end KinematicLoop_DynamicStateSelection;

      model PistonEngine "A piston engine (manual state selection)"
        extends Modelica.Icons.Example;
        Parts.Body bodyDrive(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-62, 20})));
        Joints.Revolute revoluteDrive(phi(fixed = true, start = 0), w(fixed = true, start = -2.5), stateSelect = StateSelect.always) annotation (
          Placement(transformation(extent = {{-70, 40}, {-50, 60}})));
        Parts.FixedTranslation fixedTranslationDisc(r = {0.3, 0}) annotation (
          Placement(transformation(extent = {{-30, 40}, {-10, 60}})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-90, 50})));
        Joints.Prismatic prismatic(r = {1, 0}, s(fixed = false, start = 0)) annotation (
          Placement(transformation(extent = {{30, -60}, {50, -40}})));
        Parts.Fixed fixed1 annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {70, -50})));
        Joints.Revolute revoluteDisc(phi(fixed = false, start = 0)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, 30})));
        Parts.FixedTranslation pistonRod(r = {0.8, 0}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270)));
        Parts.Body bodyPiston(I = 0.1, m = 3) annotation (
          Placement(transformation(extent = {{30, -30}, {50, -10}})));
        Joints.Revolute revolutePiston annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -30})));
        inner PlanarWorld planarWorld(defaultWidthFraction = 10) annotation (
          Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
      equation
        connect(fixed.frame, revoluteDrive.frame_a) annotation (
          Line(points = {{-80, 50}, {-70, 50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revoluteDrive.frame_b, fixedTranslationDisc.frame_a) annotation (
          Line(points = {{-50, 50}, {-30, 50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed1.frame, prismatic.frame_b) annotation (
          Line(points = {{60, -50}, {50, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslationDisc.frame_b, revoluteDisc.frame_a) annotation (
          Line(points = {{-10, 50}, {0, 50}, {0, 40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(bodyDrive.frame_a, revoluteDrive.frame_b) annotation (
          Line(points = {{-52, 20}, {-40, 20}, {-40, 50}, {-50, 50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revoluteDisc.frame_b, pistonRod.frame_a) annotation (
          Line(points = {{0, 20}, {0, 20}, {0, 10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolutePiston.frame_b, pistonRod.frame_b) annotation (
          Line(points = {{0, -20}, {0, -12}, {0, -10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_a, revolutePiston.frame_a) annotation (
          Line(points = {{30, -50}, {0, -50}, {0, -40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_a, bodyPiston.frame_a) annotation (
          Line(points = {{30, -50}, {20, -50}, {20, -20}, {30, -20}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This example contains an algebraic loop. A non-linear system must be solved for initialization and at simulation.</p>
<p>In this version, the state are manually selected.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/PistonEngine_1.png\" alt=\"Diagram PistonEngine_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/PistonEngine_2.png\" alt=\"Diagram PistonEngine_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>revoluteDrive.phi</li>
<li>revoluteDrive.w</li>
</ul>
</html>"),experiment(StopTime = 10));
      end PistonEngine;

      model PistonEngine_DynamicStateSelection "A piston engine"
        extends Modelica.Icons.Example;
        Parts.Body bodyDrive(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-62, 20})));
        Joints.Revolute revoluteDrive(phi(fixed = true, start = 0), w(fixed = true, start = -2.5)) annotation (
          Placement(transformation(extent = {{-70, 40}, {-50, 60}})));
        Parts.FixedTranslation fixedTranslationDisc(r = {0.3, 0}) annotation (
          Placement(transformation(extent = {{-30, 40}, {-10, 60}})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-90, 50})));
        Joints.Prismatic prismatic(r = {1, 0}, s(start = 0, fixed = false)) annotation (
          Placement(transformation(extent = {{30, -60}, {50, -40}})));
        Parts.Fixed fixed1 annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {70, -50})));
        Joints.Revolute revoluteDisc(phi(fixed = false, start = 0)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, 30})));
        Parts.FixedTranslation pistonRod(r = {0.8, 0}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270)));
        Parts.Body bodyPiston(I = 0.1, m = 3) annotation (
          Placement(transformation(extent = {{30, -30}, {50, -10}})));
        Joints.Revolute revolutePiston annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -30})));
        inner PlanarWorld planarWorld(defaultWidthFraction = 10) annotation (
          Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
      equation
        connect(fixed.frame, revoluteDrive.frame_a) annotation (
          Line(points = {{-80, 50}, {-70, 50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revoluteDrive.frame_b, fixedTranslationDisc.frame_a) annotation (
          Line(points = {{-50, 50}, {-42, 50}, {-30, 50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed1.frame, prismatic.frame_b) annotation (
          Line(points = {{60, -50}, {50, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslationDisc.frame_b, revoluteDisc.frame_a) annotation (
          Line(points = {{-10, 50}, {0, 50}, {0, 40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(bodyDrive.frame_a, revoluteDrive.frame_b) annotation (
          Line(points = {{-52, 20}, {-40, 20}, {-40, 50}, {-50, 50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revoluteDisc.frame_b, pistonRod.frame_a) annotation (
          Line(points = {{0, 20}, {0, 20}, {0, 10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolutePiston.frame_b, pistonRod.frame_b) annotation (
          Line(points = {{0, -20}, {0, -12}, {0, -12}, {0, -10}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_a, revolutePiston.frame_a) annotation (
          Line(points = {{30, -50}, {0, -50}, {0, -40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_a, bodyPiston.frame_a) annotation (
          Line(points = {{30, -50}, {20, -50}, {20, -20}, {30, -20}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This example contains an algebraic loop. A non-linear system must be solved for initialization and at simulation.</p>
<p>This version does not stipulate the state selection</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/PistonEngine_1.png\" alt=\"Diagram PistonEngine_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/PistonEngine_2.png\" alt=\"Diagram PistonEngine_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>There are 2&nbsp;sets of dynamic state selection.</li>
<ul>
<li>From set&nbsp;1 there is 1&nbsp;state to be selected from:</li>
<ul>
<li>prismatic.s</li>
<li>revoluteDisc.phi</li>
<li>revolutePiston.phi</li>
</ul>
<li>From set&nbsp;2 there is 1&nbsp;state to be selected from:</li>
<ul>
<li>prismatic.v</li>
<li>revoluteDisc.w</li>
<li>revoluteDrive.w</li>
</ul>
</ul>
</ul>
</html>"),experiment(StopTime = 10));
      end PistonEngine_DynamicStateSelection;

      model CounterSpin "Wheel with counter-spin and dry-friction law"
        extends Modelica.Icons.Example;
        Parts.Body body(m = 0.01, I = 0.0005, animate = false, r(each fixed = false), v(each fixed = false)) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-10, 0})));
        inner PlanarWorld planarWorld annotation (
          Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
        Joints.DryFrictionBasedRolling slipBasedRolling(R = 0.1, vAdhesion = 0.01, mu_S = 0.15, phi(fixed = true), w(fixed = true, start = 15), vx(fixed = true, start = 2), x(fixed = true), vSlide = 0.03, mu_A = 0.5) annotation (
          Placement(transformation(extent = {{20, -10}, {40, 10}})));
      equation
        connect(body.frame_a, slipBasedRolling.frame_a) annotation (
          Line(points = {{0, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 3),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/CounterSpin_1.png\" alt=\"Diagram CounterSpin_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/CounterSpin_2.png\" alt=\"Diagram CounterSpin_2\"></p>
<p>The model contains a large local stiffness before 2&nbsp;s</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/CounterSpin_3.png\" alt=\"Diagram CounterSpin_3\"></p>
<p>Selected continuous time states</p>
<ul>
<li>body.r[1]</li>
<li>body.v[1]</li>
<li>slipBasedRolling.phi</li>
<li>slipBasedRolling.w</li>
</ul>
</html>"));
      end CounterSpin;

      model WheelBasedCranCrab "A pendulum mounted on an ideal rolling wheel"
        extends Modelica.Icons.Example;
        Joints.IdealRolling idealRolling(R = 0.3, phi(fixed = true), w(fixed = true), x(fixed = true, start = 0)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-20, 50})));
        Parts.Body body(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{10, 20}, {30, 40}})));
        Joints.Revolute revolute(phi(fixed = true, start = 1.3962634015955), w(fixed = true)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, 10})));
        Parts.FixedTranslation fixedTranslation(r = {1, 0}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, -20})));
        Parts.Body body1(m = 2, I = 0.2) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-20, -50})));
        inner PlanarWorld planarWorld(defaultWidthFraction = 10, defaultZPosition = 0) annotation (
          Placement(transformation(extent = {{-80, 0}, {-60, 20}})));
      equation
        connect(revolute.frame_a, idealRolling.frame_a) annotation (
          Line(points = {{-20, 20}, {-20, 40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body.frame_a, idealRolling.frame_a) annotation (
          Line(points = {{10, 30}, {-20, 30}, {-20, 40}}, color = {95, 95, 95}, thickness = 0.5));
        connect(body1.frame_a, fixedTranslation.frame_b) annotation (
          Line(points = {{-20, -40}, {-20, -35}, {-20, -30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedTranslation.frame_a, revolute.frame_b) annotation (
          Line(points = {{-20, -10}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          experiment(StopTime = 4.5),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This model contains non-holonomic constraints.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/WheelBasedCranCrab_1.png\" alt=\"Diagram WheelBasedCranCrab_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/WheelBasedCranCrab_2.png\" alt=\"Diagram WheelBasedCranCrab_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>body1.frame_a.phi</li>
<li>body1.r[1]</li>
<li>body1.w</li>
<li>revolute.phi</li>
<li>revolute.w</li>
</ul>
</html>"));
      end WheelBasedCranCrab;

      model CraneCrabTo3D "A damped crane crab"
        extends Modelica.Icons.Example;
        Parts.Body body(I = 0.1, m = 0.5) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {50, -60})));
        Parts.Body body1(m = 1, I = 0.1) annotation (
          Placement(transformation(extent = {{60, 20}, {80, 40}})));
        Parts.FixedTranslation fixedTranslation(r = {0, -1}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {50, -30})));
        Parts.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-10, 30})));
        inner PlanarWorldIn3D planarWorld(inheritGravityFromMultiBody = true, constantGravity = {0, 0}, animateGravity = false, enableAnimation = true, connectToMultiBody = true) annotation (
          Placement(transformation(extent = {{0, -60}, {20, -40}})));
        Joints.Revolute revolute(w(fixed = true), phi(fixed = true, start = 2.6179938779915)) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {50, 0})));
        Joints.Prismatic prismatic(r = {1, 0}, v(fixed = true), useFlange = true, s(fixed = true, start = 0), animate = true) annotation (
          Placement(transformation(extent = {{20, 40}, {40, 20}})));
        Modelica.Mechanics.Translational.Components.Damper damper1D(d = 10) annotation (
          Placement(transformation(extent = {{20, 50}, {40, 70}})));
        inner MB.World world(n = {0, -1, 0}) annotation (
          Placement(transformation(extent = {{-100, -60}, {-80, -40}})));
        MB.Joints.Prismatic prismatic3D(s(fixed = true, start = -0.2), useAxisFlange = false, v(fixed = true, start = 0.2), n = {1, 0, 0}) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-30, -50})));
        MB.Parts.Body body3D(r_CM = zeros(3), m = 1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-10, -20})));
        MB.Parts.FixedRotation fixedRotation3D(n = {0, 1, 0}, angle = 45) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-60, -50})));
      equation
        connect(fixedTranslation.frame_b, body.frame_a) annotation (
          Line(points = {{50, -40}, {50, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_a, body1.frame_a) annotation (
          Line(points = {{50, 10}, {50, 30}, {60, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(revolute.frame_b, fixedTranslation.frame_a) annotation (
          Line(points = {{50, -10}, {50, -20}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixed.frame, prismatic.frame_a) annotation (
          Line(points = {{0, 30}, {20, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(prismatic.frame_b, body1.frame_a) annotation (
          Line(points = {{40, 30}, {60, 30}}, color = {95, 95, 95}, thickness = 0.5));
        connect(damper1D.flange_b, prismatic.flange_a) annotation (
          Line(points = {{40, 60}, {40, 40}, {30, 40}}, color = {0, 127, 0}));
        connect(body3D.frame_a, prismatic3D.frame_b) annotation (
          Line(points = {{-10, -30}, {-10, -50}, {-20, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(planarWorld.MBFrame_a, prismatic3D.frame_b) annotation (
          Line(points = {{0, -50}, {-20, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedRotation3D.frame_a, world.frame_b) annotation (
          Line(points = {{-70, -50}, {-80, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(fixedRotation3D.frame_b, prismatic3D.frame_a) annotation (
          Line(points = {{-50, -50}, {-40, -50}}, color = {95, 95, 95}, thickness = 0.5));
        connect(damper1D.flange_a, prismatic.support) annotation (
          Line(points = {{20, 60}, {20, 60}, {20, 40}, {24, 40}}, color = {0, 127, 0}));
        annotation (
          experiment(StopTime = 10),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Examples/CraneCrab_1.png\" alt=\"Diagram CraneCrab_1\"></p>
<p>Selected continuous time states</p>
<ul>
<li>prismatic.s</li>
<li>prismatic.v</li>
<li>revolute.phi</li>
<li>revolute.w</li>
</ul>
</html>"));
      end CraneCrabTo3D;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This package is a collection of simulatable examples which show the use of the library models and elements.
</p>
</html>"));
    end Examples;

    package GearComponents "Gear connection models"
      extends PlanarMechanics.Utilities.Icons.GearConnections;

      package Examples "Collection of simulatable models involving gear components"
        extends Modelica.Icons.ExamplesPackage;

        model PlanetaryGear "Rigid planetary gearbox"
          extends Modelica.Icons.Example;
          Utilities.RigidNoLossPlanetary planetary(useHeatPort = true, r_s = 1, r_p = 1, r_r = 3, J_s = 1e-3, J_p = 1e-3, J_c = 1e-3, J_r = 1e-3, connectToMultiBody = true) annotation (
            Placement(transformation(extent = {{0, 0}, {40, 40}})));
          Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C = 1) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-20, -30})));
          Modelica.Mechanics.Rotational.Sources.ConstantSpeed sunSpeed(w_fixed = 1) annotation (
            Placement(transformation(extent = {{-50, 0}, {-30, 20}})));
          Modelica.Mechanics.Rotational.Sources.ConstantTorque ringTorque(tau_constant = 1) annotation (
            Placement(transformation(extent = {{-50, 30}, {-30, 50}})));
          MB.Parts.FixedRotation fixedRotation(n_x = {0, -1, 0}, n_y = {0, 0, 1}, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence, angles = {90, 0, 90}) annotation (
            Placement(transformation(extent = {{-10, -70}, {10, -50}})));
          MB.Parts.Fixed fixed annotation (
            Placement(transformation(extent = {{-50, -70}, {-30, -50}})));
          inner MB.World world annotation (
            Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
        equation
          connect(heatCapacitor.port, planetary.heatPort) annotation (
            Line(points = {{-10, -30}, {0, -30}, {0, 0}}, color = {191, 0, 0}));
          connect(planetary.flange_Sun, sunSpeed.flange) annotation (
            Line(points = {{0, 20}, {-10, 20}, {-10, 10}, {-30, 10}}));
          connect(ringTorque.flange, planetary.flange_Ring) annotation (
            Line(points = {{-30, 40}, {-10, 40}, {-10, 32}, {0, 32}}));
          connect(fixed.frame_b, fixedRotation.frame_a) annotation (
            Line(points = {{-30, -60}, {-20, -60}, {-10, -60}}, color = {95, 95, 95}, thickness = 0.5));
          connect(fixedRotation.frame_b, planetary.frameVisualisation) annotation (
            Line(points = {{10, -60}, {20, -60}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5));
          annotation (
            Documentation(info = "<html>
<p>The model shows the possibilities of the gear connection models.
In this example only one of 3 planets is modelled. This reduction can be done because of the symmetry of the gears. For more advanced topics like load sharing between gears, more advanced models should be used.</p>
<p>
The ring gear is driven using a 1&nbsp;Nm load, the velocity of the sun is fixed to 1&nbsp;rad/s.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"),  experiment(StopTime = 10));
        end PlanetaryGear;

        model SpurGear "Rigid spur gear"
          extends Modelica.Icons.Example;
          PlanarMechanics.Joints.Revolute gearA_Bearing(useFlange = true, w(fixed = false), phi(fixed = true)) annotation (
            Placement(transformation(extent = {{-60, 10}, {-40, -10}})));
          Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed = 1) annotation (
            Placement(transformation(extent = {{-100, 10}, {-80, 30}})));
          PlanarMechanics.Parts.Body gearA(m = 1, I = 1e-3) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-20, 30})));
          PlanarMechanics.Parts.Fixed fixed_A annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-70, -52})));
          PlanarMechanics.Parts.Body gearB(m = 1, I = 1e-3) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {20, 30})));
          PlanarMechanics.Joints.Revolute gearB_Bearing(useFlange = true, phi(fixed = false)) annotation (
            Placement(transformation(extent = {{40, 10}, {60, -10}})));
          Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(tau_constant = 10) annotation (
            Placement(transformation(extent = {{-100, 38}, {-80, 58}})));
          inner PlanarMechanics.PlanarWorld planarWorld annotation (
            Placement(transformation(extent = {{60, 60}, {80, 80}})));
          RigidNoLossExternal gearwheelExternal annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}})));
          Parts.Fixed fixed_B(r = {2, 0}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {68, -50})));
          inner MB.World world annotation (
            Placement(transformation(extent = {{20, 60}, {40, 80}})));
        equation
          connect(gearA_Bearing.frame_b, gearA.frame_a) annotation (
            Line(points = {{-40, 0}, {-20, 0}, {-20, 20}}, color = {95, 95, 95}, thickness = 0.5));
          connect(constantSpeed.flange, gearA_Bearing.flange_a) annotation (
            Line(points = {{-80, 20}, {-50, 20}, {-50, 10}}));
          connect(fixed_A.frame, gearA_Bearing.frame_a) annotation (
            Line(points = {{-70, -42}, {-70, 0}, {-60, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(constantTorque.flange, gearB_Bearing.flange_a) annotation (
            Line(points = {{-80, 48}, {50, 48}, {50, 10}}));
          connect(gearwheelExternal.frame_a, gearA_Bearing.frame_b) annotation (
            Line(points = {{-10, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(gearwheelExternal.frame_b, gearB_Bearing.frame_a) annotation (
            Line(points = {{10, 0}, {40, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(gearwheelExternal.frame_b, gearB.frame_a) annotation (
            Line(points = {{10, 0}, {20, 0}, {20, 20}}, color = {95, 95, 95}, thickness = 0.5));
          connect(fixed_B.frame, gearB_Bearing.frame_b) annotation (
            Line(points = {{68, -40}, {68, 0}, {60, 0}}, color = {95, 95, 95}, thickness = 0.5));
          annotation (
            experiment(StopTime = 10),
            Documentation(info = "<html>
<p>Simple example of a spur gear in a planar environment.
The gear A is driven using a constant velocity of 1&nbsp;rad/s, the gear B is loaded by constant torque of 10&nbsp;Nm.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end SpurGear;

        package Utilities "Utility elements used for gear example models"
          extends Modelica.Icons.UtilitiesPackage;

          model RigidNoLossPlanetary "Planetary gearbox"
            extends
              PlanarMechanics.GearComponents.Examples.Utilities.Interfaces.PlanetaryGearInterface;
            parameter SI.Distance r_s(start = 1) "Radius of sun gear";
            parameter SI.Distance r_p(start = 1) "Radius of planet gear";
            parameter SI.Distance r_r(start = 3) "Radius of ring gear";
            parameter SI.Inertia J_s(start = 1e-3) "Inertia of the sun gear";
            parameter SI.Inertia J_p(start = 1e-3) "Inertia of the planet gear";
            parameter SI.Inertia J_c(start = 1e-3) "Inertia of the carrier";
            parameter SI.Inertia J_r(start = 1e-3) "Inertia of the ring gear";
            parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
              Evaluate = true,
              HideResult = true,
              choices(checkBox = true));
            parameter Integer Tooth_a(min = 1) = 20 "Number of teeth" annotation (
              Dialog(tab = "Animation", group = "if animation = true", enable = animate));
            parameter Real RGB_s[3] = {195, 0, 0} "Color (RGB values)" annotation (
              Dialog(tab = "Animation", group = "if animation = true", enable = animate));
            parameter Real RGB_p[3] = {0, 195, 195} "Color (RGB values)" annotation (
              Dialog(tab = "Animation", group = "if animation = true", enable = animate));
            parameter Real RGB_r[3] = {0, 0, 195} "Color (RGB values)" annotation (
              Dialog(tab = "Animation", group = "if animation = true", enable = animate));
            parameter SI.Distance z_offset = 0 "Offset of z-distance for simulation" annotation (
              Dialog(tab = "Animation", group = "if animation = true", enable = animate));
            parameter Boolean connectToMultiBody = false annotation (
              Evaluate = true,
              HideResult = true,
              choices(checkBox = true));
            PlanarMechanics.Parts.Body planet(m = 1, I = 1e-3, phi(fixed = false)) annotation (
              Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {82, 40})));
            PlanarMechanics.Parts.FixedTranslation carrierPart(r = {r_s + r_p, 0}) annotation (
              Placement(transformation(extent = {{-8, -50}, {12, -30}})));
            PlanarMechanics.Parts.Fixed fixed annotation (
              Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-90, 30})));
            RigidNoLossExternal sunPlanet(useHeatPort = true, Tooth_a = 40, StartAngle_b = 0, r_a = r_s, r_b = r_p, RGB_a = RGB_s, RGB_b = RGB_p, animate = animate, StartAngle_a = 0) annotation (
              Placement(transformation(extent = {{0, 0}, {20, 20}})));
            PlanarMechanics.Parts.FixedRotation carrierAngle(alpha = 0) annotation (
              Placement(transformation(extent = {{-40, -50}, {-20, -30}})));
            PlanarMechanics.Joints.Revolute bearing_Planet annotation (
              Placement(transformation(extent = {{20, -50}, {40, -30}})));
            RigidNoLossInternal planetRing(useHeatPort = true, Tooth_a = 40, z_offset = 0.15, r_a = r_p, r_b = r_r, animate = animate, RGB_a = RGB_p, RGB_b = RGB_r) annotation (
              Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {10, 40})));
            PlanarMechanics.Joints.Revolute bearing_Sun(useFlange = true) annotation (
              Placement(transformation(extent = {{-70, 0}, {-50, 20}})));
            PlanarMechanics.Joints.Revolute bearing_Carrier(useFlange = true) annotation (
              Placement(transformation(extent = {{-70, -30}, {-50, -50}})));
            PlanarMechanics.Joints.Revolute bearing_Ring(useFlange = true) annotation (
              Placement(transformation(extent = {{-70, 50}, {-50, 30}})));
            Modelica.Mechanics.Rotational.Components.Inertia sun(J = J_s) annotation (
              Placement(transformation(extent = {{-40, -20}, {-20, 0}})));
            Modelica.Mechanics.Rotational.Components.Inertia carrier(J = J_c, phi(start = 0)) annotation (
              Placement(transformation(extent = {{70, -36}, {90, -16}})));
            Modelica.Mechanics.Rotational.Components.Inertia ring(J = J_r) annotation (
              Placement(transformation(extent = {{-40, 50}, {-20, 70}})));
          public
            inner PlanarWorldIn3D planarWorld(nominalLength = 0.1, animateGravity = false, connectToMultiBody = connectToMultiBody) annotation (
              Placement(transformation(extent = {{60, -90}, {80, -70}})));
            MB.Parts.Body body3D(r_CM = zeros(3), m = 1e-5, animation = false) if connectToMultiBody annotation (
              Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-30, -80})));
            MB.Interfaces.Frame_a frameVisualisation if connectToMultiBody annotation (
              Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 90, origin = {0, -100})));
          equation
            connect(carrierAngle.frame_b, carrierPart.frame_a) annotation (
              Line(points = {{-20, -40}, {-8, -40}}, color = {95, 95, 95}, thickness = 0.5));
            connect(bearing_Planet.frame_a, carrierPart.frame_b) annotation (
              Line(points = {{20, -40}, {12, -40}}, color = {95, 95, 95}, thickness = 0.5));
            connect(planetRing.frame_a, bearing_Planet.frame_b) annotation (
              Line(points = {{20, 40}, {48, 40}, {48, -40}, {40, -40}}, color = {95, 95, 95}, thickness = 0.5));
            connect(sunPlanet.frame_b, planetRing.frame_a) annotation (
              Line(points = {{20, 10}, {48, 10}, {48, 40}, {20, 40}}, color = {95, 95, 95}, thickness = 0.5));
            connect(fixed.frame, bearing_Sun.frame_a) annotation (
              Line(points = {{-80, 30}, {-80, 10}, {-70, 10}}, color = {95, 95, 95}, thickness = 0.5));
            connect(bearing_Sun.flange_a, flange_Sun) annotation (
              Line(points = {{-60, 0}, {-60, 0}, {-60, -10}, {-100, -10}, {-100, 0}}));
            connect(bearing_Carrier.frame_b, carrierAngle.frame_a) annotation (
              Line(points = {{-50, -40}, {-40, -40}}, color = {95, 95, 95}, thickness = 0.5));
            connect(bearing_Carrier.frame_a, fixed.frame) annotation (
              Line(points = {{-70, -40}, {-80, -40}, {-80, 30}}, color = {95, 95, 95}, thickness = 0.5));
            connect(bearing_Ring.frame_a, fixed.frame) annotation (
              Line(points = {{-70, 40}, {-80, 40}, {-80, 30}}, color = {95, 95, 95}, thickness = 0.5));
            connect(bearing_Ring.flange_a, flange_Ring) annotation (
              Line(points = {{-60, 50}, {-60, 60}, {-100, 60}}));
            connect(planet.frame_a, bearing_Planet.frame_b) annotation (
              Line(points = {{72, 40}, {48, 40}, {48, -40}, {40, -40}}, color = {95, 95, 95}, thickness = 0.5));
            connect(sun.flange_a, bearing_Sun.flange_a) annotation (
              Line(points = {{-40, -10}, {-60, -10}, {-60, 0}}));
            connect(bearing_Sun.frame_b, sunPlanet.frame_a) annotation (
              Line(points = {{-50, 10}, {0, 10}}, color = {95, 95, 95}, thickness = 0.5));
            connect(carrier.flange_b, flange_Carrier) annotation (
              Line(points = {{90, -26}, {100, -26}, {100, 0}}));
            connect(bearing_Ring.frame_b, planetRing.frame_b) annotation (
              Line(points = {{-50, 40}, {0, 40}}, color = {95, 95, 95}, thickness = 0.5));
            connect(ring.flange_a, bearing_Ring.flange_a) annotation (
              Line(points = {{-40, 60}, {-60, 60}, {-60, 50}}));
            connect(sunPlanet.heatPort, internalHeatPort) annotation (
              Line(points = {{0, 0}, {60, 0}, {60, -60}, {-90, -60}, {-90, -80}, {-100, -80}}, color = {191, 0, 0}));
            connect(planetRing.heatPort, internalHeatPort) annotation (
              Line(points = {{20, 30}, {60, 30}, {60, -60}, {-90, -60}, {-90, -80}, {-100, -80}}, color = {191, 0, 0}));
            connect(bearing_Carrier.flange_a, carrier.flange_a) annotation (
              Line(points = {{-60, -30}, {-60, -26}, {70, -26}}));
            connect(planarWorld.MBFrame_a, frameVisualisation) annotation (
              Line(points = {{59.8, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}, thickness = 0.5));
            connect(body3D.frame_a, planarWorld.MBFrame_a) annotation (
              Line(points = {{-20, -80}, {59.8, -80}}, color = {95, 95, 95}, thickness = 0.5));
            annotation (
              Documentation(info = "<html>
<p>This model is a model of a standard planetary gearbox. The inertia of all gear models, as well as the mass of the planetary gear can be entered to get the behaviour of a complete planetary gear. In this example only one planet is used as the gearbox models are rigid.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
          end RigidNoLossPlanetary;

          package Interfaces "Connectors and partial models for gear utilities"
            extends Modelica.Icons.InterfacesPackage;

            partial model PlanetaryGearInterface "Planetary gear interface"
              extends PlanarMechanics.Utilities.Icons.PlanetaryGear;
              extends
                Modelica.Thermal.HeatTransfer.Interfaces.PartialConditionalHeatPort;
              Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_Ring "Flange of shaft" annotation (
                Placement(transformation(extent = {{-110, 50}, {-90, 70}}), iconTransformation(extent = {{-110, 50}, {-90, 70}})));
              Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_Sun "Flange of shaft" annotation (
                Placement(transformation(extent = {{-110, -10}, {-90, 10}}), iconTransformation(extent = {{-110, -10}, {-90, 10}})));
              Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_Carrier annotation (
                Placement(transformation(extent = {{90, -10}, {110, 10}}), iconTransformation(extent = {{90, -10}, {110, 10}})));
              annotation (
                Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This partial class contains common interfaces for a planetary gear model.</p>
</html>"));
            end PlanetaryGearInterface;
            annotation (
              Documentation(info = "<html>
<p>
This package contains connectors and partial models to be used in context with the examples of gear components.
</p>
</html>"));
          end Interfaces;
          annotation (
            Documentation(info = "<html>
<p>
This package contains auxiliary packages and elements to be only used in context with the examples of gear components.
</p>
</html>"));
        end Utilities;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This package is a collection of simulatable models which show the usage of gear components.
</p>
</html>"));
      end Examples;

      model RigidNoLossInternal "Internal rigid gear gonnection model"
        extends PlanarMechanics.Utilities.Icons.PlanarGearContactInternalL1;
        extends PlanarMechanics.Interfaces.PartialTwoFramesAndHeat;
        parameter SI.Distance r_a = 1 "Radius of gear A";
        parameter SI.Distance r_b = 1 "Radius of gear B";
        parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
          Evaluate = true,
          HideResult = true);
        parameter Integer Tooth_a(min = 1) = 20 "Number of Tooth" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        final parameter Integer Tooth_b(min = 1) = integer(PlanarMechanics.Utilities.Functions.round(Tooth_a / r_a * r_b)) "Number of Tooth" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        parameter Real RGB_a[3] = {195, 0, 0} "Color A (RGB values)" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "If animation = true", enable = animate));
        parameter Real RGB_b[3] = {0, 0, 195} "Color B (RGB values)" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "If animation = true", enable = animate));
        parameter SI.Distance z_offset = 0 "Offset of z-distance for simulation" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        SI.AngularVelocity w_a "Angular speed of gear A";
        SI.AngularVelocity w_b "Angular speed of gear B";
        SI.AngularVelocity w_gear "Angular speed of gear the overall gear contact";
        SI.Force F_n "Mesh normal force";
        SI.Velocity v_mesh "Mesh speed";
        SI.Acceleration a_mesh "Mesh acceleration";
        SI.Length xmesh_a "Mesh position of gear A";
        SI.Length xmesh_b "Mesh position of gear B";
        SI.Angle phi_gear "Gear angle";
      protected
        SI.Angle phi_gear2 = atan2(frame_a.y - frame_b.y, frame_a.x - frame_b.x) "Temporary Gear angle";
        Modelica.SIunits.AngularVelocity dphi_gear2 "Temporary Gear angle";
        //Visualization
        MB.Visualizers.Advanced.Shape pointA(shapeType = "cylinder", specularCoefficient = 0.5, length = 0.15, width = r_a / 10, height = r_a / 10, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -0.03}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, z_offset}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape pointB(shapeType = "cylinder", specularCoefficient = 0.5, length = 0.15, width = r_a / 10, height = r_a / 10, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -0.03}, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, z_offset}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape Gearwheel_a(shapeType = "gearwheel", color = RGB_a, specularCoefficient = 0, length = 0.1, width = r_a * 2, height = r_a * 2, lengthDirection = {0, 0, 1}, widthDirection = {0, 0, 1}, r_shape = {0, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, z_offset}) + planarWorld.r_0, extra = -Tooth_a * sign(r_a - r_b), R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, frame_a.phi, 0))) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape Gearwheel_b(shapeType = "gearwheel", color = RGB_b, specularCoefficient = 0, length = 0.1, width = r_b * 2, height = r_b * 2, lengthDirection = {0, 0, 1}, widthDirection = {0, 0, 1}, r_shape = {0, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, z_offset}) + planarWorld.r_0, extra = Tooth_b * sign(r_a - r_b), R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, frame_b.phi, 0))) if planarWorld.enableAnimation and animate;
        constant SI.Acceleration unitAcceleration = 1;
        constant SI.Force unitForce = 1;
      initial equation
        phi_gear = atan2(frame_a.y - frame_b.y, frame_a.x - frame_b.x);
      equation
        lossPower = 0;
        w_gear = der(phi_gear);
        dphi_gear2 = der(phi_gear2);
        der(phi_gear) = dphi_gear2;
        // Derivatives
        w_a = der(frame_a.phi);
        w_b = der(frame_b.phi);
        a_mesh = der(v_mesh);
        //  ********** Mesh position & speed ***************
        xmesh_a = frame_a.phi * r_a - phi_gear * r_a;
        xmesh_b = frame_b.phi * r_b - phi_gear * r_b;
        xmesh_a - xmesh_b = 0;
        v_mesh = der(xmesh_a);
        // ***********  EOM *******************
        // Torques on the axis
        frame_a.t = F_n * r_a;
        frame_b.t = -F_n * r_b;
        // Forces on the axis
        frame_a.fx = -sin(phi_gear) * F_n;
        frame_a.fy = cos(phi_gear) * F_n;
        // Force balace
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        annotation (
          defaultComponentName = "gear",
          Diagram(graphics = {Line(points = {{38, 0}, {98, 0}}, thickness = 1), Polygon(points = {{8.6901, 40.9645}, {9.3284, 42.8041}, {11.9096, 46.8087}, {14.5357, 46.0609}, {14.6201, 41.2972}, {14.1935, 39.3973}, {17.0172, 38.2626}, {18.024, 39.9292}, {21.3814, 43.3096}, {23.7946, 42.0322}, {22.8868, 37.3551}, {22.0745, 35.5854}, {24.6006, 33.8884}, {25.9319, 35.3093}, {29.9188, 37.9178}, {32.0136, 36.1665}, {30.1532, 31.7803}, {28.9908, 30.2182}, {31.1088, 28.0331}, {32.7064, 29.1461}, {37.1485, 30.8687}, {38.8335, 28.7202}, {36.1018, 24.8167}, {34.64, 23.5304}, {36.2574, 20.9526}, {38.0515, 21.7092}, {42.7547, 22.4705}, {43.9562, 20.0187}, {40.4725, 16.7684}, {38.7753, 15.8141}, {39.8213, 12.9564}, {41.7336, 13.3234}, {46.4923, 13.0903}, {47.1577, 10.4422}, {43.0745, 7.9872}, {41.2159, 7.4067}, {41.6449, 4.394}, {43.5917, 4.3554}, {48.198, 3.1379}, {48.2983, 0.4094}, {43.7938, -1.143}, {41.8552, -1.3244}, {41.6485, -4.3605}, {43.5446, -4.803}, {47.7971, -6.9516}, {47.3279, -9.6414}, {42.5992, -10.2233}, {40.6652, -9.9976}, {39.8317, -12.9244}, {41.5945, -13.7515}, {45.3073, -16.7372}, {44.2892, -19.2707}, {39.5428, -18.8567}, {37.6979, -18.2339}, {36.2742, -20.9235}, {37.8265, -22.099}, {40.8374, -25.7914}, {39.3147, -28.0578}, {34.7581, -26.6661}, {33.0831, -25.6733}, {31.1313, -28.0081}, {32.4052, -29.4806}, {34.5827, -33.7184}, {32.6221, -35.6187}, {28.4544, -33.31}, {27.0223, -31.9906}, {24.6278, -33.8686}, {25.5677, -35.5738}, {26.8165, -40.1717}, {24.5037, -41.6228}, {20.9071, -38.4981}, {19.7806, -36.9098}, {17.0479, -38.2489}, {17.6128, -40.1123}, {17.8783, -44.8693}, {15.3143, -45.8079}, {12.446, -42.0036}, {11.6744, -40.2159}, {8.723, -40.9575}, {8.8881, -42.8976}, {8.1588, -47.6059}, {5.4557, -47.9909}, {3.441, -43.6734}, {3.0579, -41.7643}, {0.0168, -41.8761}, {-0.225, -43.8082}, {-1.9173, -48.2619}, {-4.6414, -48.0765}, {-5.7144, -43.4344}, {-5.6922, -41.4874}, {-8.6901, -40.9645}, {-9.3284, -42.8041}, {-11.9096, -46.8087}, {-14.5357, -46.0609}, {-14.6201, -41.2972}, {-14.1935, -39.3973}, {-17.0172, -38.2626}, {-18.024, -39.9292}, {-21.3814, -43.3096}, {-23.7946, -42.0322}, {-22.8868, -37.3551}, {-22.0745, -35.5854}, {-24.6006, -33.8884}, {-25.9319, -35.3093}, {-29.9188, -37.9178}, {-32.0136, -36.1665}, {-30.1532, -31.7803}, {-28.9908, -30.2182}, {-31.1088, -28.0331}, {-32.7064, -29.1461}, {-37.1485, -30.8687}, {-38.8335, -28.7202}, {-36.1018, -24.8167}, {-34.64, -23.5304}, {-36.2574, -20.9526}, {-38.0515, -21.7092}, {-42.7547, -22.4705}, {-43.9562, -20.0187}, {-40.4725, -16.7684}, {-38.7753, -15.8141}, {-39.8213, -12.9564}, {-41.7336, -13.3234}, {-46.4923, -13.0903}, {-47.1577, -10.4422}, {-43.0745, -7.9872}, {-41.2159, -7.4067}, {-41.6449, -4.394}, {-43.5917, -4.3554}, {-48.198, -3.1379}, {-48.2983, -0.4094}, {-43.7938, 1.143}, {-41.8552, 1.3244}, {-41.6485, 4.3605}, {-43.5446, 4.803}, {-47.7971, 6.9516}, {-47.3279, 9.6414}, {-42.5992, 10.2233}, {-40.6652, 9.9976}, {-39.8317, 12.9244}, {-41.5945, 13.7515}, {-45.3073, 16.7372}, {-44.2892, 19.2707}, {-39.5428, 18.8567}, {-37.6979, 18.2339}, {-36.2742, 20.9235}, {-37.8265, 22.099}, {-40.8374, 25.7914}, {-39.3147, 28.0578}, {-34.7581, 26.6661}, {-33.0831, 25.6733}, {-31.1313, 28.0081}, {-32.4052, 29.4806}, {-34.5827, 33.7184}, {-32.6221, 35.6187}, {-28.4544, 33.31}, {-27.0223, 31.9906}, {-24.6278, 33.8686}, {-25.5677, 35.5738}, {-26.8165, 40.1717}, {-24.5037, 41.6228}, {-20.9071, 38.4981}, {-19.7806, 36.9098}, {-17.0479, 38.2489}, {-17.6128, 40.1123}, {-17.8783, 44.8693}, {-15.3143, 45.8079}, {-12.446, 42.0036}, {-11.6744, 40.2159}, {-8.723, 40.9575}, {-8.8881, 42.8976}, {-8.1588, 47.6059}, {-5.4557, 47.9909}, {-3.441, 43.6734}, {-3.0579, 41.7643}, {-0.0168, 41.8761}, {0.2251, 43.8082}, {1.9173, 48.2619}, {4.6414, 48.0765}, {5.7144, 43.4344}, {5.6922, 41.4874}}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, pattern = LinePattern.None), Polygon(points = {{-9.977, 22.4878}, {-8.8831, 24.6535}, {-6.9407, 26.8978}, {-5.8083, 27.739}, {-3.7455, 26.7376}, {-3.706, 25.3275}, {-4.2679, 22.413}, {-5.2931, 20.2139}, {-2.7696, 18.3875}, {-1.001, 20.0486}, {1.5919, 21.4932}, {2.9438, 21.8963}, {4.5396, 20.2498}, {4.0945, 18.9112}, {2.5696, 16.3647}, {0.8542, 14.6489}, {2.6008, 12.0695}, {4.8309, 13.0255}, {7.7614, 13.4962}, {9.1697, 13.4126}, {10.1062, 11.3196}, {9.23, 10.214}, {6.9262, 8.3426}, {4.7273, 7.3169}, {5.4865, 4.2957}, {7.909, 4.4313}, {10.8238, 3.8713}, {12.1185, 3.3112}, {12.2827, 1.0241}, {11.0812, 0.2848}, {8.2763, -0.6858}, {5.8592, -0.8975}, {5.5393, -3.9962}, {7.8621, -4.6973}, {10.4096, -6.2205}, {11.4347, -7.1897}, {10.8067, -9.395}, {9.4248, -9.6788}, {6.4571, -9.6315}, {4.1134, -9.0037}, {2.7529, -11.8061}, {4.6959, -13.2594}, {6.5687, -15.562}, {7.2005, -16.8233}, {5.8561, -18.6809}, {4.4606, -18.4749}, {1.688, -17.4154}, {-0.2997, -16.024}, {-2.5365, -18.192}, {-1.2078, -20.2222}, {-0.2355, -23.0265}, {-0.0732, -24.4278}, {-1.9718, -25.7135}, {-3.2128, -25.0427}, {-5.4558, -23.0988}, {-6.8477, -21.1115}, {-9.6911, -22.3837}, {-9.1369, -24.7459}, {-9.1823, -27.7137}, {-9.5091, -29.086}, {-11.733, -29.6448}, {-12.6697, -28.59}, {-14.1125, -25.9962}, {-14.7408, -23.6526}, {-17.8479, -23.8756}, {-18.135, -26.2849}, {-19.1927, -29.0581}, {-19.9692, -30.2359}, {-22.25, -30.0005}, {-22.7695, -28.6889}, {-23.2382, -25.758}, {-23.027, -23.3409}, {-26.023, -22.4878}, {-27.1169, -24.6535}, {-29.0593, -26.8978}, {-30.1917, -27.739}, {-32.2545, -26.7376}, {-32.294, -25.3275}, {-31.7321, -22.413}, {-30.7069, -20.2139}, {-33.2304, -18.3875}, {-34.999, -20.0486}, {-37.5919, -21.4932}, {-38.9438, -21.8963}, {-40.5396, -20.2498}, {-40.0945, -18.9112}, {-38.5696, -16.3647}, {-36.8542, -14.6489}, {-38.6008, -12.0695}, {-40.8309, -13.0255}, {-43.7614, -13.4962}, {-45.1697, -13.4126}, {-46.1062, -11.3196}, {-45.23, -10.214}, {-42.9262, -8.3426}, {-40.7273, -7.3169}, {-41.4865, -4.2957}, {-43.909, -4.4313}, {-46.8238, -3.8713}, {-48.1185, -3.3112}, {-48.2827, -1.0241}, {-47.0812, -0.2848}, {-44.2763, 0.6858}, {-41.8592, 0.8975}, {-41.5393, 3.9962}, {-43.8621, 4.6973}, {-46.4096, 6.2205}, {-47.4347, 7.1897}, {-46.8067, 9.395}, {-45.4248, 9.6788}, {-42.4571, 9.6315}, {-40.1134, 9.0037}, {-38.7529, 11.8061}, {-40.6959, 13.2594}, {-42.5687, 15.562}, {-43.2005, 16.8233}, {-41.8561, 18.6809}, {-40.4606, 18.4749}, {-37.688, 17.4154}, {-35.7003, 16.024}, {-33.4635, 18.192}, {-34.7922, 20.2222}, {-35.7645, 23.0265}, {-35.9268, 24.4278}, {-34.0282, 25.7135}, {-32.7872, 25.0427}, {-30.5442, 23.0988}, {-29.1523, 21.1115}, {-26.3089, 22.3837}, {-26.8631, 24.7459}, {-26.8177, 27.7137}, {-26.4909, 29.086}, {-24.267, 29.6448}, {-23.3303, 28.59}, {-21.8875, 25.9962}, {-21.2592, 23.6526}, {-18.1521, 23.8756}, {-17.865, 26.2849}, {-16.8073, 29.0581}, {-16.0308, 30.2359}, {-13.75, 30.0005}, {-13.2305, 28.6889}, {-12.7618, 25.758}, {-12.973, 23.3409}}, fillColor = {255, 160, 160}, fillPattern = FillPattern.Solid, lineThickness = 0.5, pattern = LinePattern.None), Line(points = {{-100, 0}, {-20, 0}}, thickness = 1)}),
          Icon(graphics={  Line(visible = useHeatPort, points = {{-100, -100}, {-100, -40}, {-60, -40}, {-42, -4}}, color = {191, 0, 0}, pattern = LinePattern.Dot)}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>In this model an ideal gear connection is modelled. It is based on the paper from van der Linden , <a href=\"http://dx.doi.org/10.3384/ecp12076303\">Modelling of Elastic Gearboxes Using a Generalized Gear Contact Model</a>. However, no gear elasticity is modelled.</p>
<p>The planar model of an internal gear wheel is used to build complex gear models. A <a href=\"http://dx.doi.org/10.3384/ecp12076681\">planar library</a> is used to create the constraints of the gearwheels. An example can be found in <a href=\"modelica://PlanarMechanics.GearComponents.Examples.SpurGear\">here</a>.</p>
<p>Using different parts from the planar library, it is possible to build complex gear systems. However, especially since no elasticity is included, kinematic loops can lead to complications and should be handled with care.</p>
<p>This model is suitable for: </p>
<ul>
<li>Kinematic analysis of gear systems and gear-like systems.</li>
<li>Modelling of multiple gear stage models with clutches.</li>
</ul>

<h4>Literature</h4>
<ol>
<li>van der Linden, F., Modelling of Elastic Gearboxes Using a Generalized Gear Contact Model, <i>Proceedings of the 9th International MODELICA Conference, Linkoping University Electronic Press, </i><b>2012</b>, 303-310 </li>
</ol>
</html>"));
      end RigidNoLossInternal;

      model RigidNoLossExternal "External rigid gear gonnection model"
        extends PlanarMechanics.Utilities.Icons.PlanarGearContactExternalL1;
        extends PlanarMechanics.Interfaces.PartialTwoFramesAndHeat;
        parameter SI.Distance r_a = 1 "Radius of gear A";
        parameter SI.Distance r_b = 1 "Radius of gear B";
        parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
          Evaluate = true,
          HideResult = true);
        parameter SI.Angle StartAngle_a = 0 "Start Angle of gear A" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        parameter SI.Angle StartAngle_b = 0 "Start Angle of gear B" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        parameter Integer Tooth_a(min = 1) = 20 "Number of Tooth" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        final parameter Integer Tooth_b(min = 1) = integer(PlanarMechanics.Utilities.Functions.round(Tooth_a / r_a * r_b)) "Number of Tooth" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        parameter Real RGB_a[3] = {195, 0, 0} "Color (RGB values)" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "If animation = true", enable = animate));
        parameter Real RGB_b[3] = {0, 0, 195} "Color (RGB values)" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "If animation = true", enable = animate));
        parameter SI.Distance z_offset = 0 "Offset of z-distance for simulation" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animate));
        SI.AngularVelocity w_a "Angular speed of gear A";
        SI.AngularVelocity w_b "Angular speed of gear B";
        SI.AngularVelocity w_gear "Angular speed of gear the overall gear contact";
        SI.Force F_n "Mesh normal force";
        SI.Velocity v_mesh "Mesh speed";
        SI.Acceleration a_mesh "Mesh acceleration";
        SI.Length xmesh_a "Mesh position of gear A";
        SI.Length xmesh_b "Mesh position of gear B";
        SI.Angle phi_gear "Gear angle";
      protected
        SI.Angle phi_gear2 = atan2(frame_b.y - frame_a.y, frame_b.x - frame_a.x) "Temporary Gear angle";
        Modelica.SIunits.AngularVelocity dphi_gear2 "Temporary Gear angle";
        //Visualization
        MB.Visualizers.Advanced.Shape pointA(shapeType = "cylinder", specularCoefficient = 0.5, length = 0.15, width = r_a / 10, height = r_a / 10, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -0.03}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, z_offset}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape pointB(shapeType = "cylinder", specularCoefficient = 0.5, length = 0.15, width = r_a / 10, height = r_a / 10, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -0.03}, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, z_offset}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape Gearwheel_a(shapeType = "gearwheel", color = RGB_a, specularCoefficient = 0, length = 0.1, width = r_a * 2, height = r_a * 2, lengthDirection = {0, 0, 1}, widthDirection = {0, 0, 1}, r_shape = {0, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, z_offset}) + planarWorld.r_0, extra = Tooth_a, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, frame_a.phi + StartAngle_a - mod(Tooth_a, 2) * 2 * Modelica.Constants.pi / Tooth_a / 4, 0))) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape Gearwheel_b(shapeType = "gearwheel", color = RGB_b, specularCoefficient = 0, length = 0.1, width = r_b * 2, height = r_b * 2, lengthDirection = {0, 0, 1}, widthDirection = {0, 0, 1}, r_shape = {0, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, z_offset}) + planarWorld.r_0, extra = Tooth_b, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, frame_b.phi + StartAngle_b + (1 - mod(Tooth_b, 2)) * 2 * Modelica.Constants.pi / Tooth_b / 4, 0))) if planarWorld.enableAnimation and animate;
        constant SI.Acceleration unitAcceleration = 1;
        constant SI.Force unitForce = 1;
      initial equation
        phi_gear = atan2(frame_b.y - frame_a.y, frame_b.x - frame_a.x);
      equation
        w_gear = der(phi_gear);
        dphi_gear2 = der(phi_gear2);
        der(phi_gear) = dphi_gear2;
        lossPower = 0;
        // Derivatives
        w_a = der(frame_a.phi);
        w_b = der(frame_b.phi);
        a_mesh = der(v_mesh);
        //  ********** Mesh position & speed ***************
        xmesh_a = frame_a.phi * r_a - phi_gear * r_a;
        xmesh_b = (-frame_b.phi * r_b) + phi_gear * r_b;
        xmesh_a - xmesh_b = 0;
        v_mesh = der(xmesh_a);
        // ***********  EOM *******************
        // Torques on the axis
        frame_a.t = F_n * r_a;
        frame_b.t = F_n * r_b;
        // Forces on the axis
        frame_a.fx = -sin(phi_gear) * F_n;
        frame_a.fy = cos(phi_gear) * F_n;
        // Force balace
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        annotation (
          defaultComponentName = "gear",
          Diagram(graphics = {Polygon(points = {{38.6901, 40.9645}, {39.3284, 42.8041}, {41.9096, 46.8087}, {44.5357, 46.0609}, {44.6201, 41.2972}, {44.1935, 39.3973}, {47.0172, 38.2626}, {48.024, 39.9292}, {51.3814, 43.3096}, {53.7946, 42.0322}, {52.8868, 37.3551}, {52.0745, 35.5854}, {54.6006, 33.8884}, {55.9319, 35.3093}, {59.9188, 37.9178}, {62.0136, 36.1665}, {60.1532, 31.7803}, {58.9908, 30.2182}, {61.1088, 28.0331}, {62.7064, 29.1461}, {67.1485, 30.8687}, {68.8335, 28.7202}, {66.1018, 24.8167}, {64.64, 23.5304}, {66.2574, 20.9526}, {68.0515, 21.7092}, {72.7547, 22.4705}, {73.9562, 20.0187}, {70.4725, 16.7684}, {68.7753, 15.8141}, {69.8213, 12.9564}, {71.7336, 13.3234}, {76.4923, 13.0903}, {77.1577, 10.4422}, {73.0745, 7.9872}, {71.2159, 7.4067}, {71.6449, 4.394}, {73.5917, 4.3554}, {78.198, 3.1379}, {78.2983, 0.4094}, {73.7938, -1.143}, {71.8552, -1.3244}, {71.6485, -4.3605}, {73.5446, -4.803}, {77.7971, -6.9516}, {77.3279, -9.6414}, {72.5992, -10.2233}, {70.6652, -9.9976}, {69.8317, -12.9244}, {71.5945, -13.7515}, {75.3073, -16.7372}, {74.2892, -19.2707}, {69.5428, -18.8567}, {67.6979, -18.2339}, {66.2742, -20.9235}, {67.8265, -22.099}, {70.8374, -25.7914}, {69.3147, -28.0578}, {64.7581, -26.6661}, {63.0831, -25.6733}, {61.1313, -28.0081}, {62.4052, -29.4806}, {64.5827, -33.7184}, {62.6221, -35.6187}, {58.4544, -33.31}, {57.0223, -31.9906}, {54.6278, -33.8686}, {55.5677, -35.5738}, {56.8165, -40.1717}, {54.5037, -41.6228}, {50.9071, -38.4981}, {49.7806, -36.9098}, {47.0479, -38.2489}, {47.6128, -40.1123}, {47.8783, -44.8693}, {45.3143, -45.8079}, {42.446, -42.0036}, {41.6744, -40.2159}, {38.723, -40.9575}, {38.8881, -42.8976}, {38.1588, -47.6059}, {35.4557, -47.9909}, {33.441, -43.6734}, {33.0579, -41.7643}, {30.0168, -41.8761}, {29.775, -43.8082}, {28.0827, -48.2619}, {25.3586, -48.0765}, {24.2856, -43.4344}, {24.3078, -41.4874}, {21.3099, -40.9645}, {20.6716, -42.8041}, {18.0904, -46.8087}, {15.4643, -46.0609}, {15.3799, -41.2972}, {15.8065, -39.3973}, {12.9828, -38.2626}, {11.976, -39.9292}, {8.6186, -43.3096}, {6.2054, -42.0322}, {7.1132, -37.3551}, {7.9255, -35.5854}, {5.3994, -33.8884}, {4.0681, -35.3093}, {0.0812, -37.9178}, {-2.0136, -36.1665}, {-0.1532, -31.7803}, {1.0092, -30.2182}, {-1.1088, -28.0331}, {-2.7064, -29.1461}, {-7.1485, -30.8687}, {-8.8335, -28.7202}, {-6.1018, -24.8167}, {-4.64, -23.5304}, {-6.2574, -20.9526}, {-8.0515, -21.7092}, {-12.7547, -22.4705}, {-13.9562, -20.0187}, {-10.4725, -16.7684}, {-8.7753, -15.8141}, {-9.8213, -12.9564}, {-11.7336, -13.3234}, {-16.4923, -13.0903}, {-17.1577, -10.4422}, {-13.0745, -7.9872}, {-11.2159, -7.4067}, {-11.6449, -4.394}, {-13.5917, -4.3554}, {-18.198, -3.1379}, {-18.2983, -0.40937}, {-13.7938, 1.143}, {-11.8552, 1.3244}, {-11.6485, 4.3605}, {-13.5446, 4.803}, {-17.7971, 6.9516}, {-17.3279, 9.6414}, {-12.5992, 10.2233}, {-10.6652, 9.9976}, {-9.8317, 12.9244}, {-11.5945, 13.7515}, {-15.3073, 16.7372}, {-14.2892, 19.2707}, {-9.5428, 18.8567}, {-7.6979, 18.2339}, {-6.2742, 20.9235}, {-7.8265, 22.099}, {-10.8374, 25.7914}, {-9.3147, 28.0578}, {-4.7581, 26.6661}, {-3.0831, 25.6733}, {-1.1313, 28.0081}, {-2.4052, 29.4806}, {-4.5827, 33.7184}, {-2.6221, 35.6187}, {1.5456, 33.31}, {2.9777, 31.9906}, {5.3722, 33.8686}, {4.4323, 35.5738}, {3.1835, 40.1717}, {5.4963, 41.6228}, {9.0929, 38.4981}, {10.2194, 36.9098}, {12.9521, 38.2489}, {12.3872, 40.1123}, {12.1217, 44.8693}, {14.6857, 45.8079}, {17.554, 42.0036}, {18.3256, 40.2159}, {21.277, 40.9575}, {21.1119, 42.8976}, {21.8412, 47.6059}, {24.5443, 47.9909}, {26.559, 43.6734}, {26.9421, 41.7643}, {29.9832, 41.8761}, {30.2251, 43.8082}, {31.9173, 48.2619}, {34.6414, 48.0765}, {35.7144, 43.4344}, {35.6922, 41.4874}}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, pattern = LinePattern.None), Polygon(points = {{-35.977, 22.4878}, {-34.8831, 24.6535}, {-32.9407, 26.8978}, {-31.8083, 27.739}, {-29.7455, 26.7376}, {-29.706, 25.3275}, {-30.2679, 22.413}, {-31.2931, 20.2139}, {-28.7696, 18.3875}, {-27.001, 20.0486}, {-24.4081, 21.4932}, {-23.0562, 21.8963}, {-21.4604, 20.2498}, {-21.9055, 18.9112}, {-23.4304, 16.3647}, {-25.1458, 14.6489}, {-23.3992, 12.0695}, {-21.1691, 13.0255}, {-18.2386, 13.4962}, {-16.8303, 13.4126}, {-15.8938, 11.3196}, {-16.77, 10.214}, {-19.0738, 8.3426}, {-21.2727, 7.3169}, {-20.5135, 4.2957}, {-18.091, 4.4313}, {-15.1762, 3.8713}, {-13.8815, 3.3112}, {-13.7173, 1.0241}, {-14.9188, 0.2848}, {-17.7237, -0.68579}, {-20.1408, -0.89754}, {-20.4607, -3.9962}, {-18.1379, -4.6973}, {-15.5904, -6.2205}, {-14.5653, -7.1897}, {-15.1933, -9.395}, {-16.5752, -9.6788}, {-19.5429, -9.6315}, {-21.8866, -9.0037}, {-23.2471, -11.8061}, {-21.3041, -13.2594}, {-19.4313, -15.562}, {-18.7995, -16.8233}, {-20.1439, -18.6809}, {-21.5394, -18.4749}, {-24.312, -17.4154}, {-26.2997, -16.024}, {-28.5365, -18.192}, {-27.2078, -20.2222}, {-26.2355, -23.0265}, {-26.0732, -24.4278}, {-27.9718, -25.7135}, {-29.2128, -25.0427}, {-31.4558, -23.0988}, {-32.8477, -21.1115}, {-35.6911, -22.3837}, {-35.1369, -24.7459}, {-35.1823, -27.7137}, {-35.5091, -29.086}, {-37.733, -29.6448}, {-38.6697, -28.59}, {-40.1125, -25.9962}, {-40.7408, -23.6526}, {-43.8479, -23.8756}, {-44.135, -26.2849}, {-45.1927, -29.0581}, {-45.9692, -30.2359}, {-48.25, -30.0005}, {-48.7695, -28.6889}, {-49.2382, -25.758}, {-49.027, -23.3409}, {-52.023, -22.4878}, {-53.1169, -24.6535}, {-55.0593, -26.8978}, {-56.1917, -27.739}, {-58.2545, -26.7376}, {-58.294, -25.3275}, {-57.7321, -22.413}, {-56.7069, -20.2139}, {-59.2304, -18.3875}, {-60.999, -20.0486}, {-63.5919, -21.4932}, {-64.9438, -21.8963}, {-66.5396, -20.2498}, {-66.0945, -18.9112}, {-64.5696, -16.3647}, {-62.8542, -14.6489}, {-64.6008, -12.0695}, {-66.8309, -13.0255}, {-69.7614, -13.4962}, {-71.1697, -13.4126}, {-72.1062, -11.3196}, {-71.23, -10.214}, {-68.9262, -8.3426}, {-66.7273, -7.3169}, {-67.4865, -4.2957}, {-69.909, -4.4313}, {-72.8238, -3.8713}, {-74.1185, -3.3112}, {-74.2827, -1.0241}, {-73.0812, -0.28477}, {-70.2763, 0.6858}, {-67.8592, 0.8975}, {-67.5393, 3.9962}, {-69.8621, 4.6973}, {-72.4096, 6.2205}, {-73.4347, 7.1897}, {-72.8067, 9.395}, {-71.4248, 9.6788}, {-68.4571, 9.6315}, {-66.1134, 9.0037}, {-64.7529, 11.8061}, {-66.6959, 13.2594}, {-68.5687, 15.562}, {-69.2005, 16.8233}, {-67.8561, 18.6809}, {-66.4606, 18.4749}, {-63.688, 17.4154}, {-61.7003, 16.024}, {-59.4635, 18.192}, {-60.7922, 20.2222}, {-61.7645, 23.0265}, {-61.9268, 24.4278}, {-60.0282, 25.7135}, {-58.7872, 25.0427}, {-56.5442, 23.0988}, {-55.1523, 21.1115}, {-52.3089, 22.3837}, {-52.8631, 24.7459}, {-52.8177, 27.7137}, {-52.4909, 29.086}, {-50.267, 29.6448}, {-49.3303, 28.59}, {-47.8875, 25.9962}, {-47.2592, 23.6526}, {-44.1521, 23.8756}, {-43.865, 26.2849}, {-42.8073, 29.0581}, {-42.0308, 30.2359}, {-39.75, 30.0005}, {-39.2305, 28.6889}, {-38.7618, 25.758}, {-38.973, 23.3409}}, fillColor = {255, 160, 160}, fillPattern = FillPattern.Solid, lineThickness = 0.5, pattern = LinePattern.None), Line(points = {{-100, 0}, {-44, 0}}, thickness = 1), Line(points = {{30, 0}, {100, 0}}, thickness = 1)}),
          Icon(graphics={  Line(visible = useHeatPort, points = {{-100, -100}, {-100, -40}, {-16, -40}, {-16, 0}}, color = {191, 0, 0}, pattern = LinePattern.Dot)}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>In this model an ideal gear connection is modelled. It is based on the paper from van der Linden: <a href=\"http://dx.doi.org/10.3384/ecp12076303\">Modelling of Elastic Gearboxes Using a Generalized Gear Contact Model</a>. However, no gear elasticity is modelled.</p>
<p>The planar model of an external gear wheel is used to build complex gear models. A <a href=\"http://dx.doi.org/10.3384/ecp12076681\">planar library</a> is used to create the constraints of the gearwheels. An example can be found in <a href=\"modelica://PlanarMechanics.GearComponents.Examples.SpurGear\">SpurGear</a>.</p>
<p>Using different parts from the planar library, it is possible to build complex gear systems. However, especially since no elasticity is included, kinematic loops can lead to complications and should be handled with care.</p>
<p>This model is suitable for: </p>
<ul>
<li>Kinematic analysis of gear systems and gear-like systems.</li>
<li>Modelling of multiple gear stage models with clutches.</li>
</ul>

<h4>Literature</h4>
<ol>
<li>van der Linden, F., Modelling of Elastic Gearboxes Using a Generalized Gear Contact Model, <i>Proceedings of the 9th International MODELICA Conference, Linkoping University Electronic Press, </i><b>2012</b>, 303-310 </li>
</ol>
</html>"));
      end RigidNoLossExternal;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This package contains gear components which can be used to construct planar gear connections from spur gears up to more advanced epicyclic types.</p>
</html>"));
    end GearComponents;

    package Interfaces "Connectors and partial models for 2-dim. mechanical components"
      extends Modelica.Icons.InterfacesPackage;

      connector Frame "General connector for planar mechanical components"
        SI.Position x "x-position";
        SI.Position y "y-position";
        SI.Angle phi "Angle (counter-clockwise)";
        flow SI.Force fx "Force in x-direction, resolved in planarWorld frame";
        flow SI.Force fy "Force in y-direction, resolved in planarWorld frame";
        flow SI.Torque t "Torque (clockwise)";
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Frame is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system and are resolved in planarWorld frame. Normally, this connector is fixed to a mechanical component. But this model is never used directly in a system. It is only for usage of inheritance.</p>
</html>"));
      end Frame;

      connector Frame_a "Coordinate system (2-dim.) fixed to the component with one cut-force and cut-torque (blue icon)"
        extends Frame;
        annotation (
          defaultComponentName = "frame_a",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16), graphics={  Rectangle(extent = {{-10, 10}, {10, -10}}, lineColor = {95, 95, 95},
                  lineThickness =                                                                                                                                                                                       0.5), Rectangle(extent = {{-34, 100}, {34, -100}}, lineColor = {95, 95, 95}, fillColor = {70, 163, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                  lineThickness =                                                                                                                                                                                                        0.5), Line(points = {{-20, 0}, {20, 0}}, color = {135, 197, 255}), Line(points = {{0, 20}, {0, -20}}, color = {135, 197, 255})}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16), graphics={  Rectangle(extent = {{-16, 50}, {16, -50}}, lineColor = {95, 95, 95}, fillColor = {70, 163, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                  lineThickness =                                                                                                                                                                                                        0.5), Line(points = {{-12, 0}, {12, 0}}, color = {135, 197, 255}), Line(points = {{0, 12}, {0, -12}}, color = {135, 197, 255}), Text(extent = {{-140, -50}, {140, -88}}, lineColor = {0, 0, 0}, textString = "%name")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Frame_a is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system and are resolved in planarWorld frame. Normally, this connector is fixed to a mechanical component. The same as <a href=\"modelica://PlanarMechanics.Interfaces.Frame_b\">Frame_b</a>.</p>
</html>"));
      end Frame_a;

      connector Frame_b "Coordinate system (2-dim.) fixed to the component with one cut-force and cut-torque (light blue icon)"
        extends Frame;
        annotation (
          defaultComponentName = "frame_b",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16), graphics={  Rectangle(extent = {{-10, 10}, {10, -10}}, lineColor = {95, 95, 95},
                  lineThickness =                                                                                                                                                                                       0.5), Rectangle(extent = {{-34, 100}, {34, -100}}, lineColor = {95, 95, 95}, fillColor = {225, 240, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                  lineThickness =                                                                                                                                                                                                        0.5), Line(points = {{-20, 0}, {20, 0}}, color = {135, 197, 255}), Line(points = {{0, 20}, {0, -20}}, color = {135, 197, 255})}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16), graphics={  Rectangle(extent = {{-16, 50}, {16, -50}}, lineColor = {95, 95, 95}, fillColor = {225, 240, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                  lineThickness =                                                                                                                                                                                                        0.5), Line(points = {{-12, 0}, {12, 0}}, color = {135, 197, 255}), Line(points = {{0, 12}, {0, -12}}, color = {135, 197, 255}), Text(extent = {{-140, -50}, {140, -88}}, lineColor = {0, 0, 0}, textString = "%name")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Frame_b is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system and are resolved in planarWorld frame. Normally, this connector is fixed to a mechanical component. The same as <a href=\"modelica://PlanarMechanics.Interfaces.Frame_a\">Frame_a</a>.</p>
</html>"));
      end Frame_b;

      connector Frame_resolve "Coordinate system fixed to the component used to express in which
                        coordinate system a vector is resolved (non-filled rectangular icon)"
        extends Frame;
        annotation (
          defaultComponentName = "frame_resolve",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16), graphics={  Rectangle(extent = {{-10, 10}, {10, -10}}, lineColor = {95, 95, 95}, pattern = LinePattern.Dot), Rectangle(extent = {{-34, 100}, {34, -100}}, lineColor = {95, 95, 95}, pattern = LinePattern.Dot, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-20, 0}, {20, 0}}, color = {135, 197, 255}), Line(points = {{0, 20}, {0, -20}}, color = {135, 197, 255})}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16), graphics={  Text(extent = {{-140, -50}, {140, -88}}, textString = "%name"), Rectangle(extent = {{-14, 50}, {14, -52}}, lineColor = {95, 95, 95}, pattern = LinePattern.Dot, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-12, 0}, {12, 0}}, color = {135, 197, 255}), Line(points = {{0, 12}, {0, -12}}, color = {135, 197, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
Basic definition of a coordinate system that is fixed to a mechanical
component. In the origin of the coordinate system the cut-force
and the cut-torque is acting. This coordinate system is used to
express in which coordinate system a vector is resolved.
A component that uses a Frame_resolve connector has to set the
cut-force and cut-torque of this frame to zero. When connecting
from a Frame_resolve connector to another frame connector,
by default the connecting line has line style \"dotted\".
This component has a non-filled rectangular icon.
</p>
</html>"));
      end Frame_resolve;

      partial model PartialTwoFrames "Partial model with two frames"
        Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque" annotation (
          Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
        Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque" annotation (
          Placement(transformation(extent = {{84, -16}, {116, 16}})));
      protected
        outer PlanarMechanics.PlanarWorld planarWorld "Planar world model";
      equation
        assert(cardinality(frame_a) > 0, "Connector frame_a of " + getInstanceName() + " is not connected");
        assert(cardinality(frame_b) > 0, "Connector frame_b of " + getInstanceName() + " is not connected");
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This is a partial model with two planar frames. It can be inherited to build up models with 2 planar flanges.</p>
<!--
This partial model provides two planar frame connectors, access to the world
object and an assert to check that both frame connectors are connected.
Therefore, inherit from this partial model if the two frame connectors are
needed and if the two frame connectors should be connected for a correct model.
-->
</html>"));
      end PartialTwoFrames;

      partial model PartialTwoFramesAndHeat "Partial model with two frames and HeatPort"
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        extends
          Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
        annotation (
          Documentation(info = "<html>
<p>The gear interface partial model is a model that has all the interfaces of each planar gear connection. Each planar gear model should be extended from this base level model.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end PartialTwoFramesAndHeat;

      model PlanarToMultiBody "This model enables to connect planar models to 3D Models"
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the fixed translation";
        //annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
        outer PlanarWorld planarWorld "planar world model";
        Frame_a frame_a "Frame connector in PlanarMechanics" annotation (
          Placement(transformation(extent = {{-56, -16}, {-24, 16}})));
        MB.Interfaces.Frame_b frame_b "Frame connector in MultiBody" annotation (
          Placement(transformation(extent = {{24, -16}, {56, 16}})));
      protected
        SI.Force fz "Normal Force";
        SI.Force f0[3] "Force vector";
      equation
        //connect the translatory position w.r.t inertial system
        frame_b.r_0 = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0;
        //Express 3D-rotation as planar rotation around z-axes
        frame_b.R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, frame_a.phi, der(frame_a.phi)));
        //define force vector in inertial system
        f0 = MB.Frames.resolve1(planarWorld.R, {frame_a.fx, frame_a.fy, fz});
        //the MulitBody force vector is resolved within the body system
        f0 * frame_b.R.T + frame_b.f = zeros(3);
        //connect the torque
        frame_a.t + MB.Frames.resolve2(planarWorld.R, frame_b.t) * {0, 0, 1} = 0;
        //This element determines the orientation matrix fully, hence it is a "root-element"
        Connections.root(frame_b.R);
        annotation (
          defaultComponentName = "adaptorPlanar2MBS",
          Icon(coordinateSystem(extent = {{-40, -20}, {40, 20}}, preserveAspectRatio = false), graphics={  Line(points = {{-42, 24}, {-26, 24}, {-26, -24}, {-42, -24}}, color = {95, 95, 95}, thickness = 0.5), Line(points = {{42, 24}, {26, 24}, {26, -24}, {42, -24}}, color = {95, 95, 95}, thickness = 0.5), Text(extent = {{-90, 60}, {90, 30}}, lineColor = {0, 0, 255}, textString = "%name"), Rectangle(extent = {{-26, 8}, {26, -8}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid)}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This component enables the connection between Planarmechanics and <a href=\"Modelica://Modelica.Mechanics.MultiBody\">MultiBody</a>.</p>
<p>The orientation and position of the 2D system within the 3D system are determined by the Multi-Body connector of the planar world model or zero rotation at zero position otherwise</p>
<p>The physical connection assumes the 2D world to be the root of the system, defining the orientation. All forces and torques acting outside the plane are assumed to be absorbed by the planar world system.. Beware! These forces are not transmitted by the Multi-Body connector of the planar world.</p>
</html>"),Diagram(coordinateSystem(extent = {{-40, -20}, {40, 20}}, preserveAspectRatio = false), graphics = {Line(points = {{-40, 0}, {40, 0}}, color = {95, 95, 95}, thickness = 0.5)}));
      end PlanarToMultiBody;

      model ZeroPosition "Set zero position vector and orientation object of frame_resolve"
        extends Modelica.Blocks.Icons.Block;
        Interfaces.Frame_resolve frame_resolve annotation (
          Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
      equation
        frame_resolve.x = 0;
        frame_resolve.y = 0;
        frame_resolve.phi = 0;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-74, 24}, {80, -20}}, textString = "r = 0")}),
          Documentation(info = "<html>
<p>
Set absolute position vector of <code>frame_resolve</code> to a zero
vector and its orientation object to a null rotation
</p>
</html>"));
      end ZeroPosition;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This package contains connectors and partial models (i.e., models that are only used to build other models) of the PlanarMechanics library.</p>
</html>"));
    end Interfaces;

    package Joints "Planar joint models"
      extends PlanarMechanics.Utilities.Icons.Joints;

      model Prismatic "A prismatic joint"
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        parameter Boolean useFlange = false "= true, if force flange enabled, otherwise implicitly grounded" annotation (
          Evaluate = true,
          HideResult = true,
          choices(checkBox = true));
        parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
          Evaluate = true,
          HideResult = true);
        parameter StateSelect stateSelect = StateSelect.default "Priority to use s and v as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        parameter SI.Position r[2] "Direction of the rod wrt. body system at phi=0";
        final parameter SI.Length l = sqrt(r * r) "Length of r";
        final parameter SI.Distance e[2] = r / l "Normalized r";
        Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a(f = f, s = s) if useFlange annotation (
          Placement(transformation(extent = {{-10, -110}, {10, -90}})));
        Modelica.Mechanics.Translational.Interfaces.Flange_b support if useFlange "1-dim. translational flange of the drive support (assumed to be fixed in the world frame, NOT in the joint)" annotation (
          Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {-60, -100})));
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of the prismatic joint box" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter SI.Distance boxWidth = l / planarWorld.defaultWidthFraction "Width of prismatic joint box" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        input PlanarMechanics.Types.Color boxColor = Types.Defaults.JointColor "Color of prismatic joint box" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate, colorSelector = true));
        input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        SI.Position s(final stateSelect = stateSelect, start = 0) "Elongation of the joint" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Velocity v(final stateSelect = stateSelect, start = 0) "Velocity of elongation" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Acceleration a(start = 0) "Acceleration of elongation" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Force f "Force in direction of elongation";
        Real e0[2] "Direction of the prismatic rod resolved wrt. inertial frame";
        SI.Position r0[2] "Translation vector of the prismatic rod resolved wrt. inertial frame";
        Real R[2, 2] "Rotation Matrix";
        //Visualization
        MB.Visualizers.Advanced.Shape box(shapeType = "box", color = boxColor, specularCoefficient = specularCoefficient, length = s, width = boxWidth, height = boxWidth, lengthDirection = {e0[1], e0[2], 0}, widthDirection = {0, 0, 1}, r_shape = {frame_a.x, frame_a.y, zPosition}, r = planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
      protected
        Modelica.Mechanics.Translational.Components.Fixed fixed annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -80})));
      equation
        //resolve the rod w.r.t. inertial system
        R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi), cos(frame_a.phi)}};
        e0 = R * e;
        r0 = e0 * s;
        //differential equations
        v = der(s);
        a = der(v);
        //actuation force
        if not useFlange then
          f = 0;
        end if;
        //rigidly connect positions
        frame_a.x + r0[1] = frame_b.x;
        frame_a.y + r0[2] = frame_b.y;
        frame_a.phi = frame_b.phi;
        //balance forces including lever principle
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        frame_a.t + frame_b.t + r0 * {frame_b.fy, -frame_b.fx} = 0;
        {frame_a.fx, frame_a.fy} * e0 = f;
        connect(fixed.flange, support) annotation (
          Line(points = {{-60, -80}, {-60, -100}}, color = {0, 127, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 40}, {-20, -40}}, lineColor = {0, 0, 0},
                  fillPattern =                                                                                                                                                                  FillPattern.Solid, fillColor = {175, 175, 175}), Rectangle(extent = {{-20, -20}, {100, 20}}, lineColor = {0, 0, 0},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, fillColor = {175, 175, 175}), Line(visible = useFlange, points = {{0, -90}, {0, -20}}, color = {0, 127, 0}), Text(extent = {{-140, -22}, {-104, -47}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{104, -22}, {140, -47}}, lineColor = {128, 128, 128}, textString = "b"), Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{-100, -50}, {100, -80}}, lineColor = {0, 0, 0}, textString = "r=%r"), Line(visible = useFlange, points = {{-92, -100}, {-30, -100}}), Line(visible = useFlange, points = {{-30, -80}, {-50, -100}}), Line(visible = useFlange, points = {{-50, -80}, {-70, -100}}), Line(visible = useFlange, points = {{-70, -80}, {-90, -100}})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Direction of the Joint is determined by <b>r[2]</b>, which is a vector pointing from <b>frame_a</b> to <b>frame_b</b>. </p>
<p>
Optionally, two additional 1-dimensional mechanical flanges
(flange \"flange_a\" represents the driving flange and
flange \"support\" represents the bearing) can be enabled via
parameter <strong>useFlange</strong>. The enabled flange_a flange can be
driven with elements of the
<a href=\"modelica://Modelica.Mechanics.Translational\">Modelica.Mechanics.Translational</a>
library.
</p>
<p>In the &quot;Initialization&quot; block, elongation of the joint <b>s</b>, velocity of elongation <b>v</b> as well as acceleration of elongation <b>a</b> can be initialized.</p>
<p>It can be defined via parameter (in &quot;advanced&quot; tab) <b>stateSelect</b> that the relative distance &quot;s&quot; and its derivative shall be definitely used as states by setting stateSelect=StateSelect.always. </p>
<p>In &quot;Animation&quot; group, animation parameters for this model can be set, where <b>zPosition</b> represents the model&apos;s position along the z axis in 3D animation. Some of the values can be preset by an outer PlanarWorld model.</p>
</html>"));
      end Prismatic;

      model Revolute "A revolute joint"
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        parameter Boolean useFlange = false "= true, if force flange enabled, otherwise implicitly grounded" annotation (
          Evaluate = true,
          HideResult = true,
          choices(checkBox = true));
        parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
          Evaluate = true,
          HideResult = true);
        parameter StateSelect stateSelect = StateSelect.default "Priority to use phi and w as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a(phi = phi, tau = t) if useFlange annotation (
          Placement(transformation(extent = {{-10, -110}, {10, -90}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b support if useFlange "1-dim. rotational flange of the drive support (assumed to be fixed in the world frame, NOT in the joint)" annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-60, -100})));
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the joint axis" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter SI.Distance cylinderLength = planarWorld.defaultJointLength "Length of cylinder representing the joint axis" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter SI.Distance cylinderDiameter = planarWorld.defaultJointWidth "Diameter of cylinder representing the joint axis" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        input PlanarMechanics.Types.Color cylinderColor = Types.Defaults.JointColor "Color of cylinder representing the joint axis" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animate));
        input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter Boolean extraLine = false "Enable black line in the cylinder to show the joint rotation" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate),
          choices(checkBox = true));
        SI.Angle phi(final stateSelect = stateSelect, start = 0) "Angular position" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularVelocity w(final stateSelect = stateSelect, start = 0) "Angular velocity" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularAcceleration z(start = 0) "Angular acceleration" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Torque t "Torque";
        // The following is defined in order to omit if-statements in visualization block (cylinder)
      protected
        final Modelica.Mechanics.MultiBody.Types.ShapeExtra extra = if extraLine then 1 else 0 "Extra parameter to visualize black line in visualized cylinder" annotation (
          HideResult = true);
        //Visualization
      public
        MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = cylinderLength, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -cylinderLength / 2}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, phi, w)), extra = extra) if planarWorld.enableAnimation and animate;
      protected
        Modelica.Mechanics.Rotational.Components.Fixed fixed "support flange is fixed to ground" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -80})));
      equation
        //Differential Equations
        w = der(phi);
        z = der(w);
        //actutation torque
        if not useFlange then
          t = 0;
        end if;
        //rigidly connect positions
        frame_a.x = frame_b.x;
        frame_a.y = frame_b.y;
        frame_a.phi + phi = frame_b.phi;
        //balance forces
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        frame_a.t + frame_b.t = 0;
        frame_a.t = t;
        connect(fixed.flange, support) annotation (
          Line(points = {{-60, -80}, {-60, -100}}));
        annotation (
          Icon(graphics={  Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255}), Rectangle(extent = {{-20, 20}, {20, -20}},
                  fillPattern =                                                                                                                                             FillPattern.HorizontalCylinder, fillColor = {175, 175, 175}, lineColor = {0, 0, 0}), Rectangle(extent = {{-100, 60}, {-20, -62}},
                  fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {175, 175, 175}, lineColor = {0, 0, 0}), Rectangle(extent = {{20, 60}, {100, -60}},
                  fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {175, 175, 175}, lineColor = {0, 0, 0}), Line(visible = useFlange, points = {{0, -100}, {0, -20}}), Text(extent = {{-140, -22}, {-104, -47}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{104, -22}, {140, -47}}, lineColor = {128, 128, 128}, textString = "b"), Line(visible = useFlange, points = {{-30, -80}, {-50, -100}}), Line(visible = useFlange, points = {{-50, -80}, {-70, -100}}), Line(visible = useFlange, points = {{-70, -80}, {-90, -100}}), Line(visible = useFlange, points = {{-92, -100}, {-30, -100}}), Rectangle(extent = {{-100, 60}, {-20, -62}}, lineColor = {0, 0, 0}), Rectangle(extent = {{20, 62}, {100, -60}}, lineColor = {0, 0, 0})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Joint where frame_b rotates around axis n which is fixed in frame_a. The two frames coincide when the rotation angle &quot;phi = 0&quot;.</p>
<p>
Optionally, two additional 1-dimensional mechanical flanges
(flange \"flange_a\" represents the driving flange and
flange \"support\" represents the bearing) can be enabled via
parameter <strong>useFlange</strong>. The enabled axis flange can be
driven with elements of the
<a href=\"modelica://Modelica.Mechanics.Rotational\">Modelica.Mechanics.Rotational</a>
library.
</p>
<p>In the &quot;Initialization&quot; block, angular position <b>phi</b>, angular velocity <b>w</b> as well as angular acceleration <b>z</b> can be initialized.</p>
<p>It can be defined via parameter (in &quot;advanced&quot; tab) <b>stateSelect</b> that the relative distance &quot;s&quot; and its derivative shall be definitely used as states by setting stateSelect=StateSelect.always. </p>
<p>In &quot;Animation&quot; group, animation parameters for this model can be set, where <b>zPosition</b> represents the model&apos;s position along the z axis in 3D animation. Some of the values can be preset by an outer PlanarWorld model.</p>
</html>"));
      end Revolute;

      model IdealRolling "A joint representing a wheel ideally rolling on the x-axis"
        Interfaces.Frame_a frame_a annotation (
          Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
        outer PlanarWorld planarWorld "Planar world model";
        parameter SI.Length R = 1.0 "Radius of the wheel";
        parameter StateSelect stateSelect = StateSelect.default "Priority to use phi, w and a as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        //parameter SI.Angle phi_start = 0;
        //parameter SI.AngularVelocity w_start = 0;
        parameter Boolean animate = true "Enable animation" annotation (
          Dialog(group = "Animation"));
        SI.Position x(stateSelect = stateSelect, start = 0) "Horizontal position" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Angle phi(stateSelect = stateSelect, start = 0) "Angular position" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularVelocity w(stateSelect = stateSelect, start = 0) "Angular velocity" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularAcceleration z(start = 0) "Angular acceleration" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Velocity vx(start = 0) "Velocity in x-direction" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        //Visualization
        MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = {255, 0, 0}, specularCoefficient = 0.5, length = 0.06, width = 2 * R, height = 2 * R, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -0.03}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, 0}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape rim1(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = 0.5, length = R * 2, width = 0.1, height = 0.1, lengthDirection = {1, 0, 0}, widthDirection = {0, 0, 1}, r_shape = {-R, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, 0}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, phi, 0))) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape rim2(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = 0.5, length = R * 2, width = 0.1, height = 0.1, lengthDirection = {1, 0, 0}, widthDirection = {0, 0, 1}, r_shape = {-R, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, 0}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, phi - Modelica.Constants.pi / 2, 0))) if planarWorld.enableAnimation and animate;
      initial equation

      equation
        //Differential Equations
        x = frame_a.x;
        phi = frame_a.phi;
        w = der(phi);
        z = der(w);
        vx = der(frame_a.x);
        //holonomic constraint
        frame_a.y = R;
        //non-holonomic constraint
        vx = -w * R;
        //balance forces
        frame_a.fx * R = frame_a.t;
        annotation (
          Icon(graphics={  Ellipse(extent = {{-80, 80}, {80, -80}}, pattern = LinePattern.None, fillColor = {95, 95, 95},
                  fillPattern =                                                                                                         FillPattern.Solid), Ellipse(extent = {{-70, 70}, {70, -70}}, pattern = LinePattern.None, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{0, 0}, {-100, 0}}, color = {0, 0, 255}), Ellipse(extent = {{-20, 20}, {20, -20}}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{-150, -90}, {150, -120}}, lineColor = {0, 0, 0}, textString = "R=%R")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Model IdealRolling contains only one connector frame_a lying at the center of the wheel, where it is assumed that no slip occurs between the wheel and ground.</p>
<p>The ground is hereby represented by the x-axis.</p>
</html>"));
      end IdealRolling;

      model DryFrictionBasedRolling "A joint representing a wheel with slip-based rolling (dry friction law) on the x-axis"
        extends
          Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(      final T = 293.15);
        Interfaces.Frame_a frame_a annotation (
          Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
        outer PlanarWorld planarWorld "Planar world model";
        parameter StateSelect stateSelect = StateSelect.default "Priority to use phi, w and a as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        parameter SI.Length R = 1.0 "Radius of the wheel";
        parameter SI.Velocity vAdhesion "Adhesion velocity";
        parameter SI.Velocity vSlide "Sliding velocity";
        parameter Real mu_A "Friction coefficient at adhesion";
        parameter Real mu_S "Friction coefficient at sliding";
        parameter Boolean animate = true "Enable animation" annotation (
          Dialog(group = "Animation"));
        SI.Position x(stateSelect = stateSelect, start = 0) "Horizontal position" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Angle phi(stateSelect = stateSelect, start = 0) "Angular position" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularVelocity w(stateSelect = stateSelect, start = 0) "Angular velocity" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularAcceleration z(start = 0) "Angular acceleration" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Velocity vx(stateSelect = stateSelect, start = 0) "Velocity in x-direction" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Force N "Normal force";
        SI.Velocity v_slip "Slip velocity";
        //Visualization
        MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = {255, 0, 0}, specularCoefficient = 0.5, length = 0.06, width = 2 * R, height = 2 * R, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -0.03}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, 0}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape rim1(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = 0.5, length = R * 2, width = 0.1, height = 0.1, lengthDirection = {1, 0, 0}, widthDirection = {0, 0, 1}, r_shape = {-R, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, 0}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, phi, 0))) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape rim2(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = 0.5, length = R * 2, width = 0.1, height = 0.1, lengthDirection = {1, 0, 0}, widthDirection = {0, 0, 1}, r_shape = {-R, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, 0}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, phi - Modelica.Constants.pi / 2, 0))) if planarWorld.enableAnimation and animate;
      initial equation
        //Initialization of Position and Velocity
      equation
        frame_a.x = x;
        phi = frame_a.phi;
        //Differential Equations
        w = der(phi);
        z = der(w);
        vx = der(frame_a.x);
        //holonomic constraint
        frame_a.y = R;
        //dry-friction law
        v_slip = vx + w * R;
        N = -frame_a.fy;
        frame_a.fx = N * noEvent(Utilities.Functions.limitByStriple(vAdhesion, vSlide, mu_A, mu_S, v_slip));
        //balance forces
        frame_a.fx * R = frame_a.t;
        lossPower = frame_a.fx * v_slip;
        annotation (
          Icon(graphics={  Ellipse(extent = {{-80, 80}, {80, -80}}, pattern = LinePattern.None, fillColor = {95, 95, 95},
                  fillPattern =                                                                                                         FillPattern.Solid), Ellipse(extent = {{-70, 70}, {70, -70}}, pattern = LinePattern.None, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{0, 0}, {-100, 0}}, color = {0, 0, 255}), Ellipse(extent = {{-20, 20}, {20, -20}}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255}), Line(visible = useHeatPort, points = {{-100, -100}, {-100, -80}, {0, -80}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Text(extent = {{-150, -90}, {150, -120}}, lineColor = {0, 0, 0}, textString = "R=%R")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Model SlipBasedRolling contains only one connector frame_a lying at the center of the wheel, where slip occurs between the wheel and ground and force caused by that is also taken into account.</p>
<p>The ground is hereby represented by the x-axis.</p>
</html>"));
      end DryFrictionBasedRolling;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This package contains idealized, massless <b>joint components </b>and<b> Rolling components</b>.</p>
</html>"));
    end Joints;

    package Parts "Components for rigid and elastic mechanical parts"
      extends PlanarMechanics.Utilities.Icons.Parts;

      model Body "Body component with mass and inertia"
        Interfaces.Frame_a frame_a annotation (
          Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
        outer PlanarWorld planarWorld "planar world model";
        parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
          Evaluate = true,
          HideResult = true,
          choices(checkBox = true));
        parameter StateSelect stateSelect = StateSelect.default "Priority to use phi, w and a as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        parameter SI.Mass m "Mass of the body";
        parameter SI.Inertia I "Inertia of the body with respect to the origin of frame_a along the z-axis of frame_a";
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of the body" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter SI.Diameter sphereDiameter = planarWorld.defaultBodyDiameter "Diameter of sphere" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter Boolean enableGravity = true "= true, if gravity effects should be taken into account" annotation (
          Evaluate = true,
          HideResult = true,
          choices(checkBox = true));
        input Types.Color sphereColor = Types.Defaults.BodyColor "Color of sphere" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animate));
        input Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        SI.Force f[2] "Force";
        SI.Position r[2](each final stateSelect = stateSelect, start = {0, 0}) "Translational position" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Velocity v[2](each final stateSelect = stateSelect, start = {0, 0}) "Velocity" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Acceleration a[2](start = {0, 0}) "Acceleration" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Angle phi(final stateSelect = stateSelect, start = 0) "Angle" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularVelocity w(final stateSelect = stateSelect, start = 0) "Angular velocity" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.AngularAcceleration z(start = 0) "Angular acceleration" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        //Visualization
        MB.Visualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, 0} - {0, 0, 1} * sphereDiameter / 2, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.axisRotation(3, frame_a.phi, w))) if planarWorld.enableAnimation and animate;
      equation
        //The velocity is a time-derivative of the position
        r = {frame_a.x, frame_a.y};
        v = der(r);
        phi = frame_a.phi;
        w = der(frame_a.phi);
        //The acceleration is a time-derivative of the velocity
        a = der(v);
        z = der(w);
        //Newton's law
        f = {frame_a.fx, frame_a.fy};
        if enableGravity then
          f + m * planarWorld.g = m * a;
        else
          f = m * a;
        end if;
        frame_a.t = I * z;
        annotation (
          Icon(graphics={  Rectangle(extent = {{-100, 40}, {-20, -40}}, fillColor = {85, 170, 255},
                  fillPattern =                                                                                   FillPattern.HorizontalCylinder), Ellipse(extent = {{-60, 60}, {60, -60}},
                  fillPattern =                                                                                                                                                                           FillPattern.Sphere, fillColor = {85, 170, 255}), Text(extent = {{150, -96}, {-150, -66}}, textString = "m=%m"), Text(extent = {{150, -130}, {-150, -100}}, textString = "I=%I", lineColor = {0, 0, 0}), Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Model <b>Body</b> is an ideal unlimited small point with mass and inertia.</p>
</html>"));
      end Body;

      model Fixed "Frame fixed in the planar world frame at a given position and orientation"
        parameter SI.Position r[2] = {0, 0} "Fixed absolute x,y-position, resolved in planarWorld frame";
        parameter SI.Angle phi = 0 "Fixed angle";
        Interfaces.Frame_b frame annotation (
          Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
      equation
        {frame.x, frame.y} = r;
        frame.phi = phi;
        annotation (
          Icon(graphics={  Line(points = {{-100, 0}, {0, 0}}), Line(points = {{0, 80}, {0, -80}}), Line(points = {{0, 40}, {80, 0}}), Line(points = {{0, 80}, {80, 40}}), Line(points = {{0, 0}, {80, -40}}), Line(points = {{0, -40}, {80, -80}}), Text(extent = {{-150, -90}, {150, -120}}, textString = "r=%r"), Text(extent = {{-150, 130}, {150, 90}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This component defines the x, y-position and angle of the frame connectors, to which this component is attached to.</p>
</html>"));
      end Fixed;

      model FixedRotation "A fixed translation between two components (rigid rod)"
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        parameter SI.Angle alpha "Fixed rotation angle";
        parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
          Dialog(group = "Animation"));
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the fixed rotation" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter SI.Length cylinderLength = planarWorld.defaultJointLength "Length of cylinder representing the fixed rotation" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter SI.Length cylinderDiameter = planarWorld.defaultJointWidth "Diameter of cylinder representing the fixed rotation" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        input PlanarMechanics.Types.Color color = Types.Defaults.RodColor "Color of cylinder representing the fixed rotation" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animate));
        input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        //Visualization
        MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = color, specularCoefficient = specularCoefficient, length = cylinderLength, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -0.05}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
      equation
        frame_a.x = frame_b.x;
        frame_a.y = frame_b.y;
        frame_a.phi + alpha = frame_b.phi;
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        frame_a.t + frame_b.t = 0;
        annotation (
          Icon(graphics={  Polygon(points = {{96, -8}, {96, 12}, {0, -30}, {-96, 12}, {-96, -6}, {0, -50}, {96, -8}}, fillColor = {175, 175, 175},
                  fillPattern =                                                                                                                                  FillPattern.Solid), Ellipse(extent = {{-20, -20}, {20, -60}}, fillColor = {175, 175, 175},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Ellipse(extent = {{-10, -30}, {10, -50}}, lineColor = {255, 255, 255}, fillColor = {175, 175, 175},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                  lineThickness =                                                                                                                                                                                                        0.5), Text(extent = {{-108, -24}, {-72, -49}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{72, -24}, {108, -49}}, lineColor = {128, 128, 128}, textString = "b"), Text(extent = {{-100, -70}, {100, -100}}, textString = "alpha=%alpha"), Text(extent = {{-150, 70}, {150, 30}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This component assures a static angle difference <b>alpha</b> between two frame connectors, to which <b>frame_a</b> and <b>frame_b</b> are connected.</p>
</html>"));
      end FixedRotation;

      model FixedTranslation "A fixed translation between two components (rigid rod)"
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        parameter SI.Length r[2] = {1, 0} "Fixed x,y-length of the rod resolved w.r.t to body frame_a at phi = 0";
        final parameter SI.Length l = Modelica.Math.Vectors.length(r) "Length of vector r";
        SI.Position r0[2] "Length of the rod resolved w.r.t to inertal frame";
        Real R[2, 2] "Rotation matrix";
        parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
          Dialog(group = "Animation"));
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the fixed translation" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        parameter SI.Distance width = l / planarWorld.defaultWidthFraction "Width of shape" annotation (
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        input PlanarMechanics.Types.Color color = Types.Defaults.RodColor "Color of shape" annotation (
          HideResult = true,
          Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animate));
        input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "if animation = true", enable = animate));
        //Visualization
        MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = color, specularCoefficient = specularCoefficient, length = l, width = width, height = width, lengthDirection = {r0[1] / l, r0[2] / l, 0}, widthDirection = {0, 0, 1}, r_shape = {0, 0, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
      equation
        //resolve the rod w.r.t inertial system
        //  sx0 = cos(frame_a.phi)*sx + sin(frame_a.phi)*sy;
        //  sy0 = -sin(frame_a.phi)*sx + cos(frame_a.phi)*sy;
        R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi), cos(frame_a.phi)}};
        r0 = R * r;
        //rigidly connect positions
        frame_a.x + r0[1] = frame_b.x;
        frame_a.y + r0[2] = frame_b.y;
        frame_a.phi = frame_b.phi;
        //balance forces including lever principle
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        //  frame_a.t + frame_b.t - sx0*frame_b.fy + sy0*frame_b.fx = 0;
        frame_a.t + frame_b.t + r0 * {frame_b.fy, -frame_b.fx} = 0;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 6}, {100, -6}},
                  fillPattern =                                                                                                                                         FillPattern.Solid, fillColor = {175, 175, 175}), Text(extent = {{-108, -24}, {-72, -49}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{72, -24}, {108, -49}}, lineColor = {128, 128, 128}, textString = "b"), Text(extent = {{-150, 80}, {150, 40}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{-100, -50}, {100, -80}}, lineColor = {0, 0, 0}, textString = "r=%r")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This component assures a static position difference <b>r</b> between two frame connectors, to which <b>frame_a</b> and <b>frame_b</b> are connected.</p>
</html>"));
      end FixedTranslation;

      model Damper "Linear (velocity dependent) damper"
        extends BaseClasses.TwoConnectorShapes;
        extends
          Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(      final T = 293.15);
        parameter StateSelect stateSelect = StateSelect.default "Priority to use phi and w as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        parameter SI.TranslationalDampingConstant d = 1 "Damping constant";
        SI.Length[2] r0(each final stateSelect = stateSelect, start = {0, 0});
        Real[2] d0;
        SI.Velocity vx(start = 0);
        SI.Velocity vy(start = 0);
        SI.Velocity v;
        SI.Force f;
        parameter SI.Position s_small = 1.E-10 "Prevent zero-division by regularization if distance between frame_a and frame_b is zero" annotation (
          Dialog(tab = "Advanced"));
        parameter Boolean enableAssert = false "Cause an assert when the distance between frame_a and frame_b < s_small" annotation (
          Dialog(tab = "Advanced"));
        //Visualization
        parameter SI.Length length_a = planarWorld.defaultForceLength "Length of cylinder at frame_a side" annotation (
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate));
        input SI.Diameter diameter_a = planarWorld.defaultForceWidth "Diameter of cylinder at frame_a side" annotation (
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate));
        input SI.Diameter diameter_b = 0.6 * diameter_a "Diameter of cylinder at frame_b side" annotation (
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate));
        input Types.Color color_a = {0, 127, 255} "Color of cylinder at frame_a side" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate, colorSelector = true));
        input Types.Color color_b = {0, 64, 200} "Color of cylinder at frame_b side" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate, colorSelector = true));
        SI.Distance length "Distance between the origin of frame_a and the origin of frame_b";
      protected
        SI.Position r0_b[3] = {d0[1], d0[2], 0} * noEvent(min(length_a, length));
        MB.Visualizers.Advanced.Shape cylinder_a(shapeType = "cylinder", color = color_a, specularCoefficient = specularCoefficient, length = noEvent(min(length_a, length)), width = diameter_a, height = diameter_a, lengthDirection = {d0[1], d0[2], 0}, widthDirection = {0, 0, 1}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape cylinder_b(shapeType = "cylinder", color = color_b, specularCoefficient = specularCoefficient, length = noEvent(max(length - length_a, 0)), width = diameter_b, height = diameter_b, lengthDirection = {r0[1], r0[2], 0}, widthDirection = {0, 0, 1}, r_shape = r0_b, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
      equation
        if enableAssert then
          assert(noEvent(length > s_small), "
 The distance between the origin of frame_a and the origin of frame_b
 of a Spring component became smaller as parameter s_small
 (= a small number, defined in the \"Advanced\" menu). The distance is
 set to s_small, although it is smaller, to avoid a division by zero
 when computing the direction of the line force. Possible reasons
 for this situation:
 - At initial time the distance may already be zero: Change the initial
   positions of the bodies connected by this element.
 - Hardware stops are not modeled or are modeled not stiff enough.
   Include stops, e.g., stiff springs, or increase the stiffness
   if already present.
 - Another error in your model may lead to unrealistically large forces
   and torques that would in reality destroy the stops.
 - The flange_b connector might be defined by a pre-defined motion,
   e.g., with Modelica.Mechanics.Translational.Position and the
   predefined flange_b.s is zero or negative.
 ");    end if;
        length = Modelica.Math.Vectors.length(r0);
        frame_a.x + r0[1] = frame_b.x;
        frame_a.y + r0[2] = frame_b.y;
        d0 = Modelica.Math.Vectors.normalize({r0[1], r0[2]}, s_small);
        der(frame_a.x) + vx = der(frame_b.x);
        der(frame_a.y) + vy = der(frame_b.y);
        v = {vx, vy} * d0;
        f = -d * v;
        frame_a.fx = d0[1] * f;
        frame_a.fy = d0[2] * f;
        frame_a.t = 0;
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        frame_a.t + frame_b.t = 0;
        lossPower = -f * v;
        annotation (
          Icon(graphics={  Line(points = {{-60, 30}, {60, 30}}), Line(points = {{-60, -30}, {60, -30}}), Line(points = {{-100, 0}, {100, 0}}), Rectangle(extent = {{-60, 30}, {30, -30}}, fillColor = {192, 192, 192},
                  fillPattern =                                                                                                                                                                                                      FillPattern.Solid), Text(extent = {{-150, 80}, {150, 40}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{-100, -50}, {100, -80}}, lineColor = {0, 0, 0}, textString = "d=%d"), Line(visible = useHeatPort, points = {{-100, -100}, {-100, -80}, {-18, 0}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Ellipse(extent = {{-90, 10}, {-70, -10}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Ellipse(extent = {{70, 10}, {90, -10}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid)}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This component is a <b>linear damper</b>, which acts as a line force between frame_a and frame_b. A <b>force f</b> is exerted on the origin of frame_b and with opposite sign on the origin of frame_a along the line from the origin of frame_a to the origin of frame_b according to the equation: </p>
<p><code>f = d*<b>der</b>(s);</code></p>
<p>where &quot;d&quot; is the damping constant, &quot;s&quot; is the distance between the origin of frame_a and origin of frame_b, and &quot;der(s)&quot; is the time derivative of &quot;s&quot;.</p>
<p>In the following figure a typical animation is shown where a mass is hanging on a damper.</p>

<blockquote>
<img src=\"modelica://PlanarMechanics/Resources/Images/Parts/Damper.png\" alt=\"Damper animation\">
</blockquote>
</html>"));
      end Damper;

      model Spring "Linear 2D translational spring"
        extends BaseClasses.TwoConnectorShapes;
        parameter StateSelect stateSelect = StateSelect.default "Priority to use phi and w as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        parameter SI.TranslationalSpringConstant c_x(final min = 0, start = 1) "Spring constant in x dir";
        parameter SI.TranslationalSpringConstant c_y(final min = 0, start = 1) "Spring constant in y dir";
        parameter SI.RotationalSpringConstant c_phi(final min = 0, start = 1.0e5) "Spring constant";
        parameter SI.Position s_relx0 = 0 "Unstretched spring length";
        parameter SI.Position s_rely0 = 0 "Unstretched spring length";
        parameter SI.Angle phi_rel0 = 0 "Unstretched spring angle";
        SI.Position s_relx(final stateSelect = stateSelect, start = 0) "Spring length" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Position s_rely(final stateSelect = stateSelect, start = 0) "Spring length" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Angle phi_rel(final stateSelect = stateSelect, start = 0) "Spring angle" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Force f_x "Force in x direction";
        SI.Force f_y "Force in y direction";
        parameter SI.Position s_small = 1.E-10 "Prevent zero-division if distance between frame_a and frame_b is zero" annotation (
          Dialog(tab = "Advanced"));
        parameter Boolean enableAssert = true "Cause an assert when the distance between frame_a and frame_b < s_small" annotation (
          Dialog(tab = "Advanced"));
        //Visualization
        parameter Integer numberOfWindings = 5 "Number of spring windings" annotation (
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate));
        input SI.Length width = planarWorld.defaultJointWidth "Width of spring" annotation (
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate));
        input SI.Length coilWidth = width / 10 "Width of spring coil" annotation (
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate));
        input Types.Color color = Types.Defaults.SpringColor "Color of spring" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate, colorSelector = true));
        SI.Length length "Distance between the origin of frame_a and the origin of frame_b";
        SI.Position r_rel_0[3] "Position vector (3D) from frame_a to frame_b resolved in multibody world frame";
        Real e_rel_0[3](each final unit = "1") "Unit vector (3D) in direction from frame_a to frame_b, resolved in multibody world frame";
      protected
        MB.Visualizers.Advanced.Shape shapeCoil(shapeType = "spring", color = color, specularCoefficient = specularCoefficient, length = length, width = width, height = coilWidth * 2, lengthDirection = e_rel_0, widthDirection = {0, 1, 0}, extra = numberOfWindings, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
      equation
        if enableAssert then
          assert(noEvent(length > s_small), "
The distance between the origin of frame_a and the origin of frame_b
of a Spring component became smaller as parameter s_small
(= a small number, defined in the \"Advanced\" menu). The distance is
set to s_small, although it is smaller, to avoid a division by zero
when computing the direction of the line force. Possible reasons
for this situation:
- At initial time the distance may already be zero: Change the initial
  positions of the bodies connected by this element.
- Hardware stops are not modeled or are modeled not stiff enough.
  Include stops, e.g., stiff springs, or increase the stiffness
  if already present.
- Another error in your model may lead to unrealistically large forces
  and torques that would in reality destroy the stops.
- The flange_b connector might be defined by a pre-defined motion,
  e.g., with Modelica.Mechanics.Translational.Position and the
  predefined flange_b.s is zero or negative.
          ", level = AssertionLevel.warning);
        end if;
        r_rel_0 = {s_relx, s_rely, 0};
        length = Modelica.Math.Vectors.length(r_rel_0);
        e_rel_0 = r_rel_0 / Modelica.Mechanics.MultiBody.Frames.Internal.maxWithoutEvent(length, s_small);
        phi_rel = frame_b.phi - frame_a.phi;
        frame_a.t = 0;
        frame_b.t = 0;
        s_relx = frame_b.x - frame_a.x;
        s_rely = frame_b.y - frame_a.y;
        f_x = c_x * (s_relx - s_relx0);
        f_y = c_y * (s_rely - s_rely0);
        frame_a.fx = -f_x;
        frame_b.fx = f_x;
        frame_a.fy = -f_y;
        frame_b.fy = f_y;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>A <i>linear translational spring</i>. x- and y direction stiffness can be parameterized.</p>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}, thickness = 0.5), Line(points = {{-70, -76}, {-70, 0}}, color = {128, 128, 128}), Line(points = {{70, -78}, {70, 0}}, color = {128, 128, 128}), Line(points = {{-70, -70}, {70, -70}}, color = {128, 128, 128}), Polygon(points = {{60, -67}, {70, -70}, {60, -73}, {60, -67}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-40, -66}, {40, -42}}, lineColor = {128, 128, 128}, textString = "phi_rel"), Text(extent = {{-150, 80}, {150, 40}}, textString = "%name", lineColor = {0, 0, 255})}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics = {Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}, thickness = 0.5), Line(points = {{-68, 0}, {-68, 65}}, color = {128, 128, 128}), Line(points = {{72, 0}, {72, 65}}, color = {128, 128, 128}), Line(points = {{-68, 60}, {72, 60}}, color = {128, 128, 128}), Polygon(points = {{62, 63}, {72, 60}, {62, 57}, {62, 63}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid), Text(extent = {{-20, 40}, {20, 65}}, lineColor = {0, 0, 255}, textString = "phi_rel")}));
      end Spring;

      model SpringDamper "Linear 2D translational spring damper model"
        extends BaseClasses.TwoConnectorShapes;
        extends
          Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(      final T = 293.15);
        parameter StateSelect stateSelect = StateSelect.default "Priority to use phi, w and a as states" annotation (
          HideResult = true,
          Dialog(tab = "Advanced"));
        parameter SI.TranslationalSpringConstant c_x(final min = 0, start = 1) "Spring constant in x dir";
        parameter SI.TranslationalSpringConstant c_y(final min = 0, start = 1) "Spring constant in y dir";
        parameter SI.RotationalSpringConstant c_phi(final min = 0, start = 1.0e5) "Spring constant in phi dir";
        parameter SI.TranslationalDampingConstant d_x(final min = 0, start = 1) "Damping constant in x dir";
        parameter SI.TranslationalDampingConstant d_y(final min = 0, start = 1) "Damping constant in y dir";
        parameter SI.RotationalDampingConstant d_phi(final min = 0, start = 1) "Damping constant in phi dir";
        parameter SI.Position s_relx0 = 0 "Unstretched spring length";
        parameter SI.Position s_rely0 = 0 "Unstretched spring length";
        parameter SI.Angle phi_rel0 = 0 "Unstretched spring angle";
        SI.Velocity v_relx "Spring velocity";
        SI.Velocity v_rely "Spring velocity";
        SI.AngularVelocity w_rel(start = 0) "Spring anglular velocity" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Position s_relx(final stateSelect = stateSelect) "Spring length";
        SI.Position s_rely(final stateSelect = stateSelect) "Spring length";
        SI.Angle phi_rel(start = 0, final stateSelect = stateSelect) "Spring angle" annotation (
          Dialog(group = "Initialization", showStartAttribute = true));
        SI.Force f_x "Force in x direction";
        SI.Force f_y "Force in y direction";
        SI.Torque tau "Torque between frames (= frame_b.f)";
        parameter SI.Position s_small = 1.E-10 "Prevent zero-division if distance between frame_a and frame_b is zero" annotation (
          Dialog(tab = "Advanced"));
        parameter Boolean enableAssert = true "Cause an assert when the distance between frame_a and frame_b < s_small" annotation (
          Dialog(tab = "Advanced"));
        //Visualization
        parameter Integer numberOfWindings = 5 "Number of spring windings" annotation (
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate));
        input SI.Length width = planarWorld.defaultJointWidth "Width of spring" annotation (
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate));
        input SI.Length coilWidth = width / 10 "Width of spring coil" annotation (
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate));
        input Types.Color color = Types.Defaults.SpringColor "Color of spring" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "Spring coil (if animation = true)", enable = animate, colorSelector = true));
        parameter SI.Length length_a = planarWorld.defaultForceLength "Length of cylinder at frame_a side" annotation (
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate));
        input SI.Diameter diameter_a = planarWorld.defaultForceWidth "Diameter of cylinder at frame_a side" annotation (
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate));
        input SI.Diameter diameter_b = 0.6 * diameter_a "Diameter of cylinder at frame_b side" annotation (
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate));
        input Types.Color color_a = {100, 100, 100} "Color of cylinder at frame_a side" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate, colorSelector = true));
        input Types.Color color_b = {155, 155, 155} "Color of cylinder at frame_b side" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "Damper cylinders (if animation = true)", enable = animate, colorSelector = true));
        SI.Length length "Distance between the origin of frame_a and the origin of frame_b";
        SI.Position r_rel_0[3] "Position vector (3D) from frame_a to frame_b resolved in multibody world frame";
        Real e_rel_0[3](each final unit = "1") "Unit vector (3D) in direction from frame_a to frame_b, resolved in multibody world frame";
      protected
        MB.Visualizers.Advanced.Shape shapeCoil(shapeType = "spring", color = color, specularCoefficient = specularCoefficient, length = length, width = width, height = coilWidth * 2, lengthDirection = e_rel_0, widthDirection = {0, 1, 0}, extra = numberOfWindings, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape cylinderDamper_a(shapeType = "cylinder", color = color_a, specularCoefficient = specularCoefficient, length = noEvent(min(length_a, length)), width = diameter_a, height = diameter_a, lengthDirection = {s_relx, s_rely, 0}, widthDirection = {0, 1, 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
        MB.Visualizers.Advanced.Shape cylinderDamper_b(shapeType = "cylinder", color = color_b, specularCoefficient = specularCoefficient, length = noEvent(max(length - length_a, 0)), width = diameter_b, height = diameter_b, lengthDirection = {s_relx, s_rely, 0}, widthDirection = {0, 1, 0}, r_shape = Modelica.Math.Vectors.normalize(r_rel_0) * noEvent(min(length_a, length)), r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
      equation
        if enableAssert then
          assert(noEvent(length > s_small), "
 The distance between the origin of frame_a and the origin of frame_b
 of a Spring component became smaller as parameter s_small
 (= a small number, defined in the \"Advanced\" menu). The distance is
 set to s_small, although it is smaller, to avoid a division by zero
 when computing the direction of the line force. Possible reasons
 for this situation:
 - At initial time the distance may already be zero: Change the initial
   positions of the bodies connected by this element.
 - Hardware stops are not modeled or are modeled not stiff enough.
   Include stops, e.g., stiff springs, or increase the stiffness
   if already present.
 - Another error in your model may lead to unrealistically large forces
   and torques that would in reality destroy the stops.
 - The flange_b connector might be defined by a pre-defined motion,
   e.g., with Modelica.Mechanics.Translational.Position and the
   predefined flange_b.s is zero or negative.
          ", level = AssertionLevel.warning);
        end if;
        r_rel_0 = {s_relx, s_rely, 0};
        length = Modelica.Math.Vectors.length(r_rel_0);
        e_rel_0 = r_rel_0 / MB.Frames.Internal.maxWithoutEvent(length, s_small);
        s_relx = frame_b.x - frame_a.x;
        s_rely = frame_b.y - frame_a.y;
        v_relx = der(s_relx);
        v_rely = der(s_rely);
        w_rel = der(phi_rel);
        phi_rel = frame_b.phi - frame_a.phi;
        tau = c_phi * (phi_rel - phi_rel0) + d_phi * w_rel;
        frame_a.t = -tau;
        frame_b.t = tau;
        f_x = c_x * (s_relx - s_relx0) + d_x * v_relx;
        f_y = c_y * (s_rely - s_rely0) + d_y * v_rely;
        frame_a.fx = -f_x;
        frame_b.fx = f_x;
        frame_a.fy = -f_y;
        frame_b.fy = f_y;
        lossPower = d_x * v_relx * v_relx + d_y * v_rely * v_rely;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>A <i>linear translational spring-damper</i>. x- and y direction stiffness and damping can be parameterized.</p>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Line(points = {{-80, 40}, {-58, 40}, {-43, 10}, {-13, 70}, {17, 10}, {47, 70}, {62, 40}, {80, 40}}, color = {0, 0, 0}), Line(points = {{-70, -106}, {-70, -41}}, color = {128, 128, 128}), Line(points = {{70, -106}, {70, -41}}, color = {128, 128, 128}), Line(points = {{-70, -100}, {70, -100}}, color = {128, 128, 128}), Polygon(points = {{60, -97}, {70, -100}, {60, -103}, {60, -97}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-40, -96}, {40, -76}}, lineColor = {128, 128, 128}, textString = "phi_rel"), Rectangle(extent = {{-50, -10}, {40, -70}}, fillColor = {192, 192, 192},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-50, -70}, {54, -70}}), Line(points = {{-50, -10}, {54, -10}}), Line(points = {{40, -40}, {80, -40}}), Line(points = {{-80, -40}, {-50, -40}}), Line(points = {{-80, 40}, {-80, -40}}), Line(points = {{80, 40}, {80, -40}}), Line(points = {{-96, 0}, {-80, 0}}), Line(points = {{96, 0}, {80, 0}}), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255}), Line(visible = useHeatPort, points = {{-100, -100}, {-100, -60}, {-40, -60}, {-20, -40}}, color = {191, 0, 0}, pattern = LinePattern.Dot)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics = {Line(points = {{-80, 32}, {-58, 32}, {-43, 2}, {-13, 62}, {17, 2}, {47, 62}, {62, 32}, {80, 32}}, thickness = 0.5), Line(points = {{-68, 32}, {-68, 97}}, color = {128, 128, 128}), Line(points = {{72, 32}, {72, 97}}, color = {128, 128, 128}), Line(points = {{-68, 92}, {72, 92}}, color = {128, 128, 128}), Polygon(points = {{62, 95}, {72, 92}, {62, 89}, {62, 95}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid), Text(extent = {{-20, 72}, {20, 97}}, lineColor = {0, 0, 255}, textString = "phi_rel"), Rectangle(extent = {{-50, -20}, {40, -80}}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-50, -80}, {68, -80}}), Line(points = {{-50, -20}, {68, -20}}), Line(points = {{40, -50}, {80, -50}}), Line(points = {{-80, -50}, {-50, -50}}), Line(points = {{-80, 32}, {-80, -50}}), Line(points = {{80, 32}, {80, -50}}), Line(points = {{-100, 0}, {-80, 0}}), Line(points = {{100, 0}, {80, 0}})}));
      end SpringDamper;

      package BaseClasses "Collection of base classes for 'Parts'"
        extends Modelica.Icons.BasesPackage;

        partial model TwoConnectorShapes "Partial base class containing two frames and cylinder shapes at these frames"
          extends PlanarMechanics.Interfaces.PartialTwoFrames;
          parameter Boolean animate = true "Enable animation" annotation (
            Dialog(group = "Animation"));
          parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the fixed translation" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          input Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
            HideResult = true,
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          parameter Boolean animateConnectors = true "=true, if connectors a and b should be animated as cylinders" annotation (
            Dialog(tab = "Animation", group = "Connectors (if animation = true)", enable = animate));
          parameter SI.Diameter diameterConnector_a = planarWorld.defaultJointWidth "Diameter of connector at frame_a" annotation (
            Dialog(tab = "Animation", group = "Connectors (if animation = true)", enable = animate and animateConnectors));
          parameter SI.Diameter diameterConnector_b = planarWorld.defaultJointWidth "Diameter of connector at frame_b" annotation (
            Dialog(tab = "Animation", group = "Connectors (if animation = true)", enable = animate and animateConnectors));
          input Types.Color colorConnector_a = Types.Defaults.RodColor "Color of connector at frame_a" annotation (
            HideResult = true,
            Dialog(colorSelector = true, tab = "Animation", group = "Connectors (if animation = true)", enable = animate and animateConnectors));
          input Types.Color colorConnector_b = Types.Defaults.RodColor "Color of connector at frame_a" annotation (
            HideResult = true,
            Dialog(colorSelector = true, tab = "Animation", group = "Connectors (if animation = true)", enable = animate and animateConnectors));
        protected
          MB.Visualizers.Advanced.Shape contactA(shapeType = "cylinder", color = colorConnector_a, specularCoefficient = specularCoefficient, length = planarWorld.defaultJointLength, width = diameterConnector_a, height = diameterConnector_a, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {frame_a.x, frame_a.y, zPosition} + {0, 0, -planarWorld.defaultJointLength / 2}, r = planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate and animateConnectors;
          MB.Visualizers.Advanced.Shape contactB(shapeType = "cylinder", color = colorConnector_b, specularCoefficient = specularCoefficient, length = planarWorld.defaultJointLength, width = diameterConnector_b, height = diameterConnector_b, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {frame_b.x, frame_b.y, zPosition} + {0, 0, -planarWorld.defaultJointLength / 2}, r = planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate and animateConnectors;
          annotation (
            Icon(graphics={  Text(extent = {{-108, -24}, {-72, -49}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{72, -24}, {108, -49}}, lineColor = {128, 128, 128}, textString = "b")}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This partial base class contains two frames and optionally enables to visualize a cylinder at place of each of these frames. This class should be extended to create a proper model, see e.g. <a href=\"modelica://PlanarMechanics.Parts.Spring\">Spring</a> model. </p>
</html>"));
        end TwoConnectorShapes;
        annotation (
          Documentation(info = "<html>
<p>
A collection of base classes for planar parts.
These partial classes should be extended to create proper models.
</p>
</html>"));
      end BaseClasses;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Package <b>Parts</b> contains rigid components and spring/damper components for planar mechanical systems, which could be used to build up a complex planar system.</p>
</html>"));
    end Parts;

    model PlanarWorld "Planar world coordinate system + gravity field + default animation definition"
      SI.Position r_0[3] "Position vector from world frame to the connector frame origin, resolved in world frame";
      MB.Frames.Orientation R "Orientation object to rotate the world frame into the connector frame";
      parameter SI.Acceleration[2] constantGravity = {0, -9.81} "Constant gravity acceleration vector resolved in world frame" annotation (
        Dialog(group = "Gravity"));
      parameter Boolean enableAnimation = true "= true, if animation of all components is enabled" annotation (
        Evaluate = true,
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Animation (General)"));
      parameter Boolean animateWorld = true "= true, if world coordinate system shall be visualized" annotation (
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Animation (General)", enable = enableAnimation));
      parameter Boolean animateGravity = true "= true, if gravity field shall be visualized (acceleration vector or field center)" annotation (
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Animation (General)", enable = enableAnimation));
      parameter String label1 = "x" "Label of horizontal axis in icon" annotation (
        Dialog(group = "Animation (General)"));
      parameter String label2 = "y" "Label of vertical axis in icon" annotation (
        Dialog(group = "Animation (General)"));
      SI.Acceleration[2] g "Constant gravity acceleration vector resolved in world frame";
      parameter SI.Length axisLength = nominalLength / 2 "Length of world axes arrows" annotation (
        Dialog(tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter SI.Diameter axisDiameter = axisLength / defaultFrameDiameterFraction "Diameter of world axes arrows" annotation (
        Dialog(tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Boolean axisShowLabels = true "= true, if labels shall be shown" annotation (
        Dialog(tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Types.Color axisColor_x = Types.Defaults.FrameColor "Color of x-arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Types.Color axisColor_y = axisColor_x "Color of y-arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Types.Color axisColor_z = axisColor_x "Color of z-arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter SI.Position gravityArrowTail[2] = {0, 0} "Position vector from origin of world frame to arrow tail, resolved in world frame" annotation (
        Dialog(tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter SI.Length gravityArrowLength = axisLength / 2 "Length of gravity arrow" annotation (
        Dialog(tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter SI.Diameter gravityArrowDiameter = gravityArrowLength / defaultWidthFraction "Diameter of gravity arrow" annotation (
        Dialog(tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter Types.Color gravityArrowColor = {0, 180, 0} "Color of gravity arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter SI.Length defaultZPosition = 0 "Default for z positions of all the elements" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length nominalLength = 1 "\"Nominal\" length of PlanarMechanics" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultJointLength = nominalLength / 10 "Default for the fixed length of a shape representing a joint" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultJointWidth = nominalLength / 10 "Default for the fixed width of a shape representing a joint" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Diameter defaultBodyDiameter = nominalLength / 9 "Default for diameter of sphere representing the center of mass of a body" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultWidthFraction = 20 "Default for shape width as a fraction of shape length (e.g., for Parts.FixedTranslation)" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Diameter defaultArrowDiameter = nominalLength / 40 "Default for arrow diameter (e.g., of forces, torques, sensors)" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultForceLength = nominalLength / 10 "Default for the fixed length of a shape representing a force (e.g., damper)" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultForceWidth = nominalLength / 20 "Default for the fixed width of a shape represening a force (e.g., spring, bushing)" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultFrameDiameterFraction = 40 "Default for arrow diameter of a coordinate system as a fraction of axis length" annotation (
        Dialog(tab = "Defaults"));
      parameter PlanarMechanics.Types.SpecularCoefficient defaultSpecularCoefficient(min = 0) = 0.7 "Default reflection of ambient light (= 0: light is completely absorbed)" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultN_to_m(unit = "N/m", min = 0) = 1000 "Default scaling of force arrows (length = force/defaultN_to_m)" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultNm_to_m(unit = "N.m/m", min = 0) = 1000 "Default scaling of torque arrows (length = torque/defaultNm_to_m)" annotation (
        Dialog(tab = "Defaults"));
    protected
      parameter Integer ndim = if enableAnimation and animateWorld then 1 else 0;
      parameter Integer ndim2 = if enableAnimation and animateWorld and axisShowLabels then 1 else 0;
      // Parameters to define axes labels
      parameter SI.Length scaledLabel = Modelica.Mechanics.MultiBody.Types.Defaults.FrameLabelHeightFraction * axisDiameter;
      parameter SI.Length labelStart = 1.05 * axisLength;
      // coordinate system
    protected
      Visualizers.Internal.CoordinateSystem coordinateSystem(r = r_0, R = R, r_shape = zeros(3), axisLength = axisLength, axisDiameter = axisDiameter, axisShowLabels = axisShowLabels, scaledLabel = scaledLabel, labelStart = labelStart, color_x = axisColor_x, color_y = axisColor_y, color_z = axisColor_z) if enableAnimation and animateWorld;
      // gravity visualization
      Visualizers.Internal.Arrow gravityArrow(r = r_0, R = R, r_tail = {gravityArrowTail[1], gravityArrowTail[2], 0}, r_head = gravityArrowLength * Modelica.Math.Vectors.normalize({g[1], g[2], 0}), diameter = gravityArrowDiameter, color = gravityArrowColor, specularCoefficient = 0) if enableAnimation and animateGravity;
    equation
      r_0 = {0, 0, 0};
      R = MB.Frames.nullRotation();
      g = constantGravity;
      annotation (
        defaultComponentName = "planarWorld",
        defaultComponentPrefixes = "inner",
        missingInnerMessage = "No \"world\" component is defined. A default world
component with the default gravity field will be used
(g=9.81 in negative y-axis). If this is not desired,
drag PlanarMechanics.PlanarWorld into the top level of your model.",
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Rectangle(extent = {{-100, 100}, {100, -100}}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-100, -118}, {-100, 61}}, thickness = 0.5), Polygon(points = {{-100, 100}, {-120, 60}, {-80, 60}, {-100, 100}, {-100, 100}},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-119, -100}, {59, -100}}, thickness = 0.5), Polygon(points = {{99, -100}, {59, -80}, {59, -120}, {99, -100}},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 145}, {150, 105}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{95, -113}, {144, -162}}, textString = "%label1"), Text(extent = {{-170, 127}, {-119, 77}}, textString = "%label2"), Line(points = {{-56, 60}, {-56, -26}}, color = {0, 0, 255}), Polygon(points = {{-68, -8}, {-56, -48}, {-44, -8}, {-68, -8}}, fillColor = {0, 0, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Line(points = {{2, 60}, {2, -26}}, color = {0, 0, 255}), Polygon(points = {{-10, -8}, {2, -48}, {14, -8}, {-10, -8}}, fillColor = {0, 0, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Line(points = {{66, 60}, {66, -26}}, color = {0, 0, 255}), Polygon(points = {{54, -8}, {66, -48}, {78, -8}, {54, -8}}, fillColor = {0, 0, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Text(extent = {{-80, 90}, {80, 60}}, lineColor = {0, 0, 0}, textString = "2-dim."), Text(extent = {{-100, -50}, {100, -80}}, lineColor = {0, 0, 0}, textString = "g=%constantGravity")}),
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Model <b>PlanarWorld</b> defines all possible general parameters to make parameterization of models much more convenient. It has the following functionalites.</p>
<ol>
<li>It defines the global coordinate system fixed in ground and shows the x, y, z axes in animation if wanted.</li>
<li>It contains all default parameters for animation, e.g. axis diameter, default joint length etc, which can still be overwritten by setting parameters in these models.</li>
<li>It provides the default gravity definition and its animation.</li>
</ol>
<p><br>The pure planar world cannot be coupled to the 3D world. It shall be used when no outer 3D world is available.</p>
</html>"));
    end PlanarWorld;

    model PlanarWorldIn3D "Planar world coordinate system + gravity field + default animation definition"
      MB.Interfaces.Frame_a MBFrame_a if connectToMultiBody annotation (
        Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
      SI.Position r_0[3] "Position vector from world frame to the connector frame origin, resolved in world frame";
      MB.Frames.Orientation R "Orientation object to rotate the world frame into the connector frame";
      parameter Boolean inheritGravityFromMultiBody = false "=true if gravity vector shall be inherited from 3D world model" annotation (
        Evaluate = true,
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Gravity"));
      parameter SI.Acceleration[2] constantGravity = {0, -9.81} "Constant gravity acceleration vector resolved in world frame" annotation (
        Dialog(group = "Gravity", enable = not inheritGravityFromMultiBody));
      parameter Boolean connectToMultiBody = false "= true when visualization of the planar world shall be connected to a 3D multibody system" annotation (
        Evaluate = true,
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Animation (General)"));
      parameter Boolean enableAnimation = true "= true, if animation of all components is enabled" annotation (
        Evaluate = true,
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Animation (General)"));
      parameter Boolean animateWorld = true "= true, if world coordinate system shall be visualized" annotation (
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Animation (General)", enable = enableAnimation));
      parameter Boolean animateGravity = true "= true, if gravity field shall be visualized (acceleration vector or field center)" annotation (
        HideResult = true,
        choices(checkBox = true),
        Dialog(group = "Animation (General)", enable = enableAnimation));
      parameter String label1 = "x" "Label of horizontal axis in icon" annotation (
        Dialog(group = "Animation (General)"));
      parameter String label2 = "y" "Label of vertical axis in icon" annotation (
        Dialog(group = "Animation (General)"));
      SI.Acceleration[2] g "Constant gravity acceleration vector resolved in world frame";
      parameter SI.Length axisLength = nominalLength / 2 "Length of world axes arrows" annotation (
        Dialog(tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter SI.Diameter axisDiameter = axisLength / defaultFrameDiameterFraction "Diameter of world axes arrows" annotation (
        Dialog(tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Boolean axisShowLabels = true "= true, if labels shall be shown" annotation (
        Dialog(tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Types.Color axisColor_x = Types.Defaults.FrameColor "Color of x-arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Types.Color axisColor_y = axisColor_x "Color of y-arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter Types.Color axisColor_z = axisColor_x "Color of z-arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateWorld = true", enable = enableAnimation and animateWorld));
      parameter SI.Position gravityArrowTail[2] = {0, 0} "Position vector from origin of world frame to arrow tail, resolved in world frame" annotation (
        Dialog(tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter SI.Length gravityArrowLength = axisLength / 2 "Length of gravity arrow" annotation (
        Dialog(tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter SI.Diameter gravityArrowDiameter = gravityArrowLength / defaultWidthFraction "Diameter of gravity arrow" annotation (
        Dialog(tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter Types.Color gravityArrowColor = {0, 180, 0} "Color of gravity arrow" annotation (
        HideResult = true,
        Dialog(colorSelector = true, tab = "Animation", group = "If animateGravity = true", enable = enableAnimation and animateGravity));
      parameter SI.Length defaultZPosition = 0 "Default for z positions of all the elements" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length nominalLength = 1 "\"Nominal\" length of PlanarMechanics" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultJointLength = nominalLength / 10 "Default for the fixed length of a shape representing a joint" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultJointWidth = nominalLength / 10 "Default for the fixed width of a shape representing a joint" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Diameter defaultBodyDiameter = nominalLength / 9 "Default for diameter of sphere representing the center of mass of a body" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultWidthFraction = 20 "Default for shape width as a fraction of shape length (e.g., for Parts.FixedTranslation)" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Diameter defaultArrowDiameter = nominalLength / 40 "Default for arrow diameter (e.g., of forces, torques, sensors)" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultForceLength = nominalLength / 10 "Default for the fixed length of a shape representing a force (e.g., damper)" annotation (
        Dialog(tab = "Defaults"));
      parameter SI.Length defaultForceWidth = nominalLength / 20 "Default for the fixed width of a shape represening a force (e.g., spring, bushing)" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultFrameDiameterFraction = 40 "Default for arrow diameter of a coordinate system as a fraction of axis length" annotation (
        Dialog(tab = "Defaults"));
      parameter PlanarMechanics.Types.SpecularCoefficient defaultSpecularCoefficient(min = 0) = 0.7 "Default reflection of ambient light (= 0: light is completely absorbed)" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultN_to_m(unit = "N/m", min = 0) = 1000 "Default scaling of force arrows (length = force/defaultN_to_m)" annotation (
        Dialog(tab = "Defaults"));
      parameter Real defaultNm_to_m(unit = "N.m/m", min = 0) = 1000 "Default scaling of torque arrows (length = torque/defaultNm_to_m)" annotation (
        Dialog(tab = "Defaults"));
    protected
      MB.Interfaces.Frame MBFrame;
      outer Modelica.Mechanics.MultiBody.World world;
      SI.Acceleration gz "Auxiliary gravity acc. in z-direction";
      parameter Integer ndim = if enableAnimation and animateWorld then 1 else 0;
      parameter Integer ndim2 = if enableAnimation and animateWorld and axisShowLabels then 1 else 0;
      // Parameters to define axes labels
      parameter SI.Length scaledLabel = Modelica.Mechanics.MultiBody.Types.Defaults.FrameLabelHeightFraction * axisDiameter;
      parameter SI.Length labelStart = 1.05 * axisLength;
      // coordinate system
    protected
      Visualizers.Internal.CoordinateSystem coordinateSystem(r = r_0, R = R, r_shape = zeros(3), axisLength = axisLength, axisDiameter = axisDiameter, axisShowLabels = axisShowLabels, scaledLabel = scaledLabel, labelStart = labelStart, color_x = axisColor_x, color_y = axisColor_y, color_z = axisColor_z) if enableAnimation and animateWorld;
      // gravity visualization
      Visualizers.Internal.Arrow gravityArrow(r = r_0, R = R, r_tail = {gravityArrowTail[1], gravityArrowTail[2], 0}, r_head = gravityArrowLength * Modelica.Math.Vectors.normalize({g[1], g[2], 0}), diameter = gravityArrowDiameter, color = gravityArrowColor, specularCoefficient = 0) if enableAnimation and animateGravity;
    equation
      if connectToMultiBody then
        connect(MBFrame_a, MBFrame);
      else
        MBFrame.r_0 = {0, 0, 0};
        MBFrame.R = MB.Frames.nullRotation();
        //    Connections.root(MBFrame.R);
      end if;
      r_0 = MBFrame.r_0;
      R = MBFrame.R;
      if inheritGravityFromMultiBody then
        {g[1], g[2], gz} = MB.Frames.resolve2(R, world.gravityAcceleration(MBFrame.r_0));
      else
        gz = 0;
        g = constantGravity;
      end if;
      //  MBFrame.f = {0,0,0};
      //  MBFrame.t = {0,0,0};
      annotation (
        defaultComponentName = "planarWorld",
        defaultComponentPrefixes = "inner",
        missingInnerMessage = "No \"world\" component is defined. A default world
component with the default gravity field will be used
(g=9.81 in negative y-axis). If this is not desired,
drag PlanarMechanics.PlanarWorld into the top level of your model.",
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Rectangle(extent = {{-100, 100}, {100, -100}}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-100, -118}, {-100, 61}}, thickness = 0.5), Polygon(points = {{-100, 100}, {-120, 60}, {-80, 60}, {-100, 100}, {-100, 100}},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-119, -100}, {59, -100}}, thickness = 0.5), Polygon(points = {{99, -100}, {59, -80}, {59, -120}, {99, -100}},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 145}, {150, 105}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{95, -113}, {144, -162}}, textString = "%label1"), Text(extent = {{-170, 127}, {-119, 77}}, textString = "%label2"), Line(points = {{-56, 60}, {-56, -26}}, color = {0, 0, 255}), Polygon(points = {{-68, -8}, {-56, -48}, {-44, -8}, {-68, -8}}, fillColor = {0, 0, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Line(points = {{2, 60}, {2, -26}}, color = {0, 0, 255}), Polygon(points = {{-10, -8}, {2, -48}, {14, -8}, {-10, -8}}, fillColor = {0, 0, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Line(points = {{66, 60}, {66, -26}}, color = {0, 0, 255}), Polygon(points = {{54, -8}, {66, -48}, {78, -8}, {54, -8}}, fillColor = {0, 0, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, lineColor = {0, 0, 255}), Text(extent = {{-80, 90}, {80, 60}}, lineColor = {0, 0, 0}, textString = "2-dim."), Text(visible = not inheritGravityFromMultiBody, extent = {{-100, -50}, {100, -80}}, lineColor = {0, 0, 0}, textString = "g=%constantGravity"), Text(visible = inheritGravityFromMultiBody, extent = {{-100, -50}, {100, -80}}, lineColor = {0, 0, 0}, textString = "g inherited from 3D")}),
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Model <b>PlanarWorld</b> defines all possible general parameters to make parameterization of models much more convenient. It has the following functionalites.</p>
<ol>
<li>It defines the global coordinate system fixed in ground and shows the x, y, z axes in animation if wanted.</li>
<li>It contains all default parameters for animation, e.g. axis diameter, default joint length etc, which can still be overwritten by setting parameters in these models.</li>
<li>It provides the default gravity definition and its animation.</li>
</ol>
<p><br>The planar world can optionaly be coupled to a <a href=\"Modelica.Mechanics.MultiBody.Interfaces.Frame_a\">3D-Multibody connector</a>. This will affect visualization mainly. Beware! The physics of the planar world presume the inertial system to be non-accelerated. When connecting to an accelerated MultiBody connector the physical forces going along with this acceleration are thus neglected.</p>
<p>For physical coupling between 2D and 3D system use <a href=\"PlanarMechanics.Interfaces.PlanarToMultiBody\">Interfaces.PlanarToMultiBody</a></p>
<p>The gravity vector can be inherited from the <a href=\"Modelica.Mechanics.MultiBody.World\">MultiBody world component</a>. In this case, the vector is determined once for the origin of the planar world system and then applied to all body components (if enabled there, as default).</p>
</html>"));
    end PlanarWorldIn3D;

    package Sensors "Sensors to measure variables in 2D mechanical components"
      extends Modelica.Icons.SensorsPackage;

      model AbsolutePosition "Measure absolute position vector of the origin of a frame connector"
        extends Internal.PartialAbsoluteSensor;
        Modelica.Blocks.Interfaces.RealOutput r[3] "Absolute position vector resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {110, 0})));
        Interfaces.Frame_resolve frame_resolve if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve "Coordinate system in which output vector r is optionally resolved" annotation (
          Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which output vector r shall be resolved (1: world, 2: frame_a, 3:frame_resolve)";
      protected
        Internal.BasicAbsolutePosition position(resolveInFrame = resolveInFrame) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Interfaces.ZeroPosition zeroPosition if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve annotation (
          Placement(transformation(extent = {{20, -40}, {40, -20}})));
      equation
        connect(position.frame_resolve, frame_resolve) annotation (
          Line(points = {{0, -10}, {0, -32.5}, {0, -32.5}, {0, -55}, {0, -100}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(zeroPosition.frame_resolve, position.frame_resolve) annotation (
          Line(points = {{20, -30}, {0, -30}, {0, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(position.r, r) annotation (
          Line(points = {{11, 0}, {35.75, 0}, {35.75, 0}, {60.5, 0}, {60.5, 0}, {110, 0}}, color = {0, 0, 127}));
        connect(position.frame_a, frame_a) annotation (
          Line(points = {{-10, 0}, {-32.5, 0}, {-32.5, 0}, {-55, 0}, {-55, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{70, 0}, {100, 0}}, color = {0, 0, 127}), Text(extent = {{62, 46}, {146, 16}}, textString = "r"), Text(extent = {{15, -67}, {146, -92}}, lineColor = {95, 95, 95}, textString = "resolve"), Line(points = {{0, -100}, {0, -70}}, pattern = LinePattern.Dot), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The absolute position and angle vector<b> [x,y,phi]</b> of the origin of frame_a is determined and provided at the output signal connector <b>r</b>.</p>
<p>Via parameter <b>resolveInFrame</b> it is defined, in which frame the position and angle vector is resolved: </p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><h4>resolveInFrame =</h4></p><p align=\"center\">Types.ResolveInFrameA.</p></td>
<td><p align=\"center\"><h4>Meaning</h4></p></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameA.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and r is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected.</p>
<p>Example: If <code>resolveInFrame = Types.ResolveInFrameA.frame_resolve</code>, the output vector is computed as: </p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-dcUlfcwL.png\" alt=\"r =transpose([cos(frame_resolve.phi), -sin(frame_resolve.phi), 0; sin(frame_resolve.phi),cos(frame_resolve.phi), 0;0, 0, 1]) * [frame_a.x;frame_a.y;frame_a.phi] - [0;0;frame_resolve.phi]\"/></p>
</html>"));
      end AbsolutePosition;

      model RelativePosition "Measure relative position vector between the origins of two frame connectors"
        extends Internal.PartialRelativeSensor;
        Modelica.Blocks.Interfaces.RealOutput r_rel[3] "Relative position vector resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
        Interfaces.Frame_resolve frame_resolve if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which r_rel is optionally resolved" annotation (
          Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 65}, {116, 97}})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector r_rel shall be resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
      protected
        Internal.BasicRelativePosition relativePosition(resolveInFrame = resolveInFrame) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Interfaces.ZeroPosition zeroPosition if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve annotation (
          Placement(transformation(extent = {{52, 20}, {72, 40}})));
      equation
        connect(relativePosition.frame_a, frame_a) annotation (
          Line(points = {{-10, 0}, {-32.5, 0}, {-32.5, 0}, {-55, 0}, {-55, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativePosition.frame_b, frame_b) annotation (
          Line(points = {{10, 0}, {32.5, 0}, {32.5, 0}, {55, 0}, {55, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativePosition.frame_resolve, frame_resolve) annotation (
          Line(points = {{10, 8.1}, {26, 8.1}, {26, 8}, {36, 8}, {36, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(zeroPosition.frame_resolve, relativePosition.frame_resolve) annotation (
          Line(points = {{52, 30}, {36, 30}, {36, 8.1}, {10, 8.1}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(relativePosition.r_rel, r_rel) annotation (
          Line(points = {{0, -11}, {0, -35.75}, {0, -35.75}, {0, -60.5}, {0, -60.5}, {0, -110}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, -70}, {0, -100}}, color = {0, 0, 127}), Text(extent = {{18, -80}, {102, -110}}, textString = "r_rel"), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The relative position and angle vector<b> [x,y,phi]</b> between the origins of frame_a and frame_b are determined and provided at the output signal connector <b>r_rel</b>.</p>
<p>Via parameter <b>resolveInFrame</b> it is defined, in which frame the position vector is resolved: </p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><h4>resolveInFrame =</h4></p><p align=\"center\">Types.ResolveInFrameAB.</p></td>
<td><p align=\"center\"><h4>Meaning</h4></p></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_b</p></td>
<td valign=\"top\"><p>Resolve vector in frame_b</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and r_rel is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected.</p>
<p>Example: If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the output vector is computed as: </p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-LrRs4SXG.png\" alt=\"r_rel = transpose([cos(frame_resolve.phi), -sin(frame_resolve.phi), 0; sin(frame_resolve.phi),cos(frame_resolve.phi), 0;0,0,1]) * [frame_b.x - frame_a.x;frame_b.y - frame_a.y;frame_b.phi - frame_a.phi]\"/></p>
</html>"));
      end RelativePosition;

      model AbsoluteVelocity "Measure absolute velocity vector of origin of frame connector"
        extends Internal.PartialAbsoluteSensor;
        Modelica.Blocks.Interfaces.RealOutput v[3] "Absolute velocity vector resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {110, 0})));
        Interfaces.Frame_resolve frame_resolve if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve "Coordinate system in which output vector v is optionally resolved" annotation (
          Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which output vector v shall be resolved (1: world, 2: frame_a, 3: frame_resolve)";
      protected
        Internal.BasicAbsolutePosition position(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation (
          Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
        Modelica.Blocks.Continuous.Der der1[3] annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        TransformAbsoluteVector transformAbsoluteVector(frame_r_in = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world, frame_r_out = resolveInFrame) annotation (
          Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {50, 0})));
        Interfaces.ZeroPosition zeroPosition annotation (
          Placement(transformation(extent = {{-60, -60}, {-80, -40}})));
        Interfaces.ZeroPosition zeroPosition1 if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve annotation (
          Placement(transformation(extent = {{60, -60}, {80, -40}})));
      equation
        connect(position.r, der1.u) annotation (
          Line(points = {{-39, 0}, {-12, 0}}, color = {0, 0, 127}));
        connect(position.frame_a, frame_a) annotation (
          Line(points = {{-60, 0}, {-70, 0}, {-70, 0}, {-80, 0}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(der1.y, transformAbsoluteVector.r_in) annotation (
          Line(points = {{11, 0}, {38, 0}}, color = {0, 0, 127}));
        connect(transformAbsoluteVector.r_out, v) annotation (
          Line(points = {{61, 0}, {56, 0}, {56, 0}, {110, 0}}, color = {0, 0, 127}));
        connect(zeroPosition.frame_resolve, position.frame_resolve) annotation (
          Line(points = {{-60, -50}, {-50, -50}, {-50, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(transformAbsoluteVector.frame_a, frame_a) annotation (
          Line(points = {{50, 10}, {50, 20}, {-70, 20}, {-70, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(transformAbsoluteVector.frame_resolve, zeroPosition1.frame_resolve) annotation (
          Line(points = {{49.9, -10}, {50, -10}, {50, -50}, {60, -50}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(transformAbsoluteVector.frame_resolve, frame_resolve) annotation (
          Line(points = {{49.9, -10}, {50, -10}, {50, -50}, {0, -50}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{70, 0}, {100, 0}}, color = {0, 0, 127}), Text(extent = {{58, 48}, {142, 18}}, textString = "v"), Text(extent = {{15, -67}, {146, -92}}, lineColor = {95, 95, 95}, textString = "resolve"), Line(points = {{0, -70}, {0, -95}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The absolute velocity vector of the origin of frame_a is determined and provided at the output signal connector <b>v</b>.</p>
<p>Via parameter <b>resolveInFrame</b> it is defined, in which frame the velocity vector is resolved: </p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><h4>resolveInFrame =</h4></p><p align=\"center\">Types.ResolveInFrameA.</p></td>
<td><p align=\"center\"><h4>Meaning</h4></p></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameA.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and v is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected.</p>
<p>Example: If <code>resolveInFrame = Types.ResolveInFrameA.frame_resolve</code>, the output vector is computed as: </p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-x2brh9fX.png\" alt=\"
v0 = der([x,y,phi])\"></p>
<p><br/><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-Kgd1NoyE.png\" alt=\"v = [cos(frame_resolve.phi), sin(frame_resolve.phi),0;-sin(frame_resolve.phi),cos(frame_resolve.phi),0;0,0,1] * [v0[1];v0[2];v0[3]]\"></p>
<p>where <img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-zBL2JSRi.png\" alt=\"[x,y,phi]\"> is position and angle vector of origin of frame_a resolved in world coordinate.</p>
</html>"));
      end AbsoluteVelocity;

      model RelativeVelocity "Measure relative velocity vector between the origins of two frame connectors"
        extends Internal.PartialRelativeSensor;
        Modelica.Blocks.Interfaces.RealOutput v_rel[3] "Relative velocity vector resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
        Interfaces.Frame_resolve frame_resolve if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which v_rel is optionally resolved" annotation (
          Placement(transformation(extent = {{84, 64}, {116, 96}})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector v_rel shall be resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
      protected
        RelativePosition relativePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Interfaces.ZeroPosition zeroPosition if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve annotation (
          Placement(transformation(extent = {{40, -60}, {60, -40}})));
        Modelica.Blocks.Continuous.Der der_r_rel[3] annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -30})));
        TransformRelativeVector transformRelativeVector(frame_r_in = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a, frame_r_out = resolveInFrame) annotation (
          Placement(transformation(extent = {{-10, -80}, {10, -60}})));
      equation
        connect(relativePosition.frame_a, frame_a) annotation (
          Line(points = {{-10, 0}, {-32.5, 0}, {-32.5, 0}, {-55, 0}, {-55, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativePosition.frame_b, frame_b) annotation (
          Line(points = {{10, 0}, {32.5, 0}, {32.5, 0}, {55, 0}, {55, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativePosition.r_rel, der_r_rel.u) annotation (
          Line(points = {{0, -11}, {0, -18}, {1.77636e-15, -18}}, color = {0, 0, 127}));
        connect(der_r_rel.y, transformRelativeVector.r_in) annotation (
          Line(points = {{-1.77636e-15, -41}, {-1.77636e-15, -50}, {0, -50}, {0, -58}}, color = {0, 0, 127}));
        connect(transformRelativeVector.r_out, v_rel) annotation (
          Line(points = {{0, -81}, {0, -88.25}, {0, -88.25}, {0, -95.5}, {0, -95.5}, {0, -110}}, color = {0, 0, 127}));
        connect(transformRelativeVector.frame_a, frame_a) annotation (
          Line(points = {{-10, -70}, {-70, -70}, {-70, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(transformRelativeVector.frame_b, frame_b) annotation (
          Line(points = {{10, -70}, {70, -70}, {70, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(transformRelativeVector.frame_resolve, frame_resolve) annotation (
          Line(points = {{10, -61.9}, {30, -61.9}, {30, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(zeroPosition.frame_resolve, transformRelativeVector.frame_resolve) annotation (
          Line(points = {{40, -50}, {30, -50}, {30, -61.9}, {10, -61.9}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, -70}, {0, -100}}, color = {0, 0, 127}), Text(extent = {{18, -80}, {102, -110}}, textString = "v_rel"), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The relative velocity vector between the origins of frame_a and of frame_b are determined and provided at the output signal connector <b>v_rel</b>.</p>
<p><code>Via parameter <b>resolveInFrame</b> it is defined, in which frame the velocity vector is resolved: </code></p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><code><b>resolveInFrame =</b>Types.ResolveInFrameAB.</code></p></td>
<td><pre><b>Meaning</b></pre></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_b</p></td>
<td valign=\"top\"><p>Resolve vector in frame_b</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and v_rel is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected. Example: If resolveInFrame = Types.ResolveInFrameAB.frame_resolve, the output vector is computed as: </p>
<pre><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-LZbFgA50.png\" alt=\"r_rel = transpose([cos(frame_resolve.phi), -sin(frame_resolve.phi), 0; sin(frame_resolve.phi),cos(frame_resolve.phi), 0;0,0,1]) * [frame_b.x - frame_a.x;frame_b.y - frame_a.y;frame_b.phi - frame_a.phi]\"/>
<img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-b53N2SsO.png\" alt=\"v_rela = der(r_rel)\"/></pre>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-PGBmAMb7.png\" alt=\"v_rel = transpose([cos(frame_resolve.phi), -sin(frame_resolve.phi),0;sin(frame_resolve.phi),cos(frame_resolve.phi),0;0,0,1]) * [cos(frame_a.phi),-sin(frame_a.phi), 0;sin(frame_a.phi), cos(frame_a.phi),0;0,0,1] * r_rela\"/></p>
</html>"));
      end RelativeVelocity;

      model AbsoluteAcceleration "Measure absolute acceleration vector of origin of frame connector"
        extends Internal.PartialAbsoluteSensor;
        Modelica.Blocks.Interfaces.RealOutput a[3] "Absolute acceleration vector resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {110, 0})));
        Interfaces.Frame_resolve frame_resolve if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve "Coordinate system in which output vector v is optionally resolved" annotation (
          Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which output vector v shall be resolved (1: world, 2: frame_a, 3: frame_resolve)";
      protected
        Internal.BasicAbsolutePosition position(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation (
          Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
        Modelica.Blocks.Continuous.Der der1[3] annotation (
          Placement(transformation(extent = {{-20, -10}, {0, 10}})));
        TransformAbsoluteVector transformAbsoluteVector(frame_r_in = position.resolveInFrame, frame_r_out = resolveInFrame) annotation (
          Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {70, 0})));
        Interfaces.ZeroPosition zeroPosition annotation (
          Placement(transformation(extent = {{-60, -60}, {-80, -40}})));
        Interfaces.ZeroPosition zeroPosition1 if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve annotation (
          Placement(transformation(extent = {{60, -60}, {40, -40}})));
        Modelica.Blocks.Continuous.Der der2[3] annotation (
          Placement(transformation(extent = {{20, -10}, {40, 10}})));
      equation
        connect(position.r, der1.u) annotation (
          Line(points = {{-39, 0}, {-22, 0}}, color = {0, 0, 127}));
        connect(position.frame_a, frame_a) annotation (
          Line(points = {{-60, 0}, {-70, 0}, {-70, 0}, {-80, 0}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(zeroPosition.frame_resolve, position.frame_resolve) annotation (
          Line(points = {{-60, -50}, {-50, -50}, {-50, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dash));
        connect(transformAbsoluteVector.frame_a, frame_a) annotation (
          Line(points = {{70, 10}, {70, 20}, {-70, 20}, {-70, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(transformAbsoluteVector.frame_resolve, zeroPosition1.frame_resolve) annotation (
          Line(points = {{69.9, -10}, {70, -10}, {70, -50}, {60, -50}}, color = {95, 95, 95}, pattern = LinePattern.Dash));
        connect(transformAbsoluteVector.frame_resolve, frame_resolve) annotation (
          Line(points = {{69.9, -10}, {70, -10}, {70, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dash));
        connect(der1.y, der2.u) annotation (
          Line(points = {{1, 0}, {18, 0}}, color = {0, 0, 127}));
        connect(der2.y, transformAbsoluteVector.r_in) annotation (
          Line(points = {{41, 0}, {58, 0}}, color = {0, 0, 127}));
        connect(transformAbsoluteVector.r_out, a) annotation (
          Line(points = {{81, 0}, {110, 0}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{70, 0}, {100, 0}}, color = {0, 0, 127}), Text(extent = {{58, 48}, {142, 18}}, textString = "acc", lineColor = {0, 0, 0}), Text(extent = {{15, -67}, {146, -92}}, lineColor = {95, 95, 95}, textString = "resolve"), Line(points = {{0, -70}, {0, -95}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The absolute acceleration vector of the origin of frame_a is determined and provided at the output signal connector <b>a</b>.</p>
<p>Via parameter <b>resolveInFrame</b> it is defined, in which frame the velocity vector is resolved: </p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><h4>resolveInFrame =</h4></p><p align=\"center\">Types.ResolveInFrameA.</p></td>
<td><p align=\"center\"><h4>Meaning</h4></p></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameA.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and v is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected.</p>
<p>Example: If <code>resolveInFrame = Types.ResolveInFrameA.frame_resolve</code>, the output vector is computed as: </p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-x2brh9fX.png\" alt=\"
v0 = der([x,y,phi])\"/></p><p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-Kgd1NoyE.png\" alt=\"v = [cos(frame_resolve.phi), sin(frame_resolve.phi),0;-sin(frame_resolve.phi),cos(frame_resolve.phi),0;0,0,1] * [v0[1];v0[2];v0[3]]\"/></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-FoL9Qy7b.png\" alt=\"a = der(v)\"/></p>
<p>where <img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-zBL2JSRi.png\" alt=\"[x,y,phi]\"/> is position and angle vector of origin of frame_a resolved in world coordinate.</p>
</html>"));
      end AbsoluteAcceleration;

      model RelativeAcceleration "Measure relative acceleration vector between the origins of two frame connectors"
        extends Internal.PartialRelativeSensor;
        Modelica.Blocks.Interfaces.RealOutput a_rel[3] "Relative acceleration vector resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
        Interfaces.Frame_resolve frame_resolve if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which v_rel is optionally resolved" annotation (
          Placement(transformation(extent = {{84, 64}, {116, 96}})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector v_rel shall be resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
      protected
        RelativePosition relativePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a) annotation (
          Placement(transformation(extent = {{-10, 30}, {10, 50}})));
        Interfaces.ZeroPosition zeroPosition if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve annotation (
          Placement(transformation(extent = {{40, -30}, {60, -10}})));
        Modelica.Blocks.Continuous.Der der_r_rel[3] annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, 0})));
        TransformRelativeVector transformRelativeVector(frame_r_in = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a, frame_r_out = resolveInFrame) annotation (
          Placement(transformation(extent = {{-10, -50}, {10, -30}})));
        Modelica.Blocks.Continuous.Der der_r_rel1[3] annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -70})));
      equation
        connect(relativePosition.frame_a, frame_a) annotation (
          Line(points = {{-10, 40}, {-70, 40}, {-70, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativePosition.frame_b, frame_b) annotation (
          Line(points = {{10, 40}, {70, 40}, {70, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(relativePosition.r_rel, der_r_rel.u) annotation (
          Line(points = {{0, 29}, {0, 12}, {2.22045e-15, 12}}, color = {0, 0, 127}));
        connect(transformRelativeVector.frame_a, frame_a) annotation (
          Line(points = {{-10, -40}, {-70, -40}, {-70, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(transformRelativeVector.frame_b, frame_b) annotation (
          Line(points = {{10, -40}, {70, -40}, {70, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(transformRelativeVector.frame_resolve, frame_resolve) annotation (
          Line(points = {{10, -31.9}, {30, -31.9}, {30, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(zeroPosition.frame_resolve, transformRelativeVector.frame_resolve) annotation (
          Line(points = {{40, -20}, {30, -20}, {30, -31.9}, {10, -31.9}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(der_r_rel.y, transformRelativeVector.r_in) annotation (
          Line(points = {{-2.22045e-15, -11}, {-2.22045e-15, -14}, {0, -14}, {0, -28}}, color = {0, 0, 127}));
        connect(transformRelativeVector.r_out, der_r_rel1.u) annotation (
          Line(points = {{0, -51}, {0, -58}, {2.22045e-15, -58}}, color = {0, 0, 127}));
        connect(der_r_rel1.y, a_rel) annotation (
          Line(points = {{-2.22045e-15, -81}, {-2.22045e-15, -110}, {0, -110}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, -70}, {0, -100}}, color = {0, 0, 127}), Text(extent = {{18, -80}, {102, -110}}, textString = "a_rel", lineColor = {0, 0, 0}), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The relative acceleration vector between the origins of frame_a and of frame_b are determined and provided at the output signal connector <b>a_rel</b>.</p>
<p><code>Via parameter <b>resolveInFrame</b> it is defined, in which frame the velocity vector is resolved: </code></p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><code><b>resolveInFrame =</b>Types.ResolveInFrameAB.</code></p></td>
<td><h4>Meaning</h4></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_b</p></td>
<td valign=\"top\"><p>Resolve vector in frame_b</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and a_rel is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected.
</p>
<p>
Example: If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the output vector is computed as:</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-LZbFgA50.png\" alt=\"r_rel = transpose([cos(frame_resolve.phi), -sin(frame_resolve.phi), 0; sin(frame_resolve.phi),cos(frame_resolve.phi), 0;0,0,1]) * [frame_b.x - frame_a.x;frame_b.y - frame_a.y;frame_b.phi - frame_a.phi]\"/></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-b53N2SsO.png\" alt=\"v_rela = der(r_rel)\"/></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-PGBmAMb7.png\" alt=\"v_rel = transpose([cos(frame_resolve.phi), -sin(frame_resolve.phi),0;sin(frame_resolve.phi),cos(frame_resolve.phi),0;0,0,1]) * [cos(frame_a.phi),-sin(frame_a.phi), 0;sin(frame_a.phi), cos(frame_a.phi),0;0,0,1] * r_rela\"/></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/equations/equation-NK9IGjAY.png\" alt=\"a_rel = der(v_rel)\"/></p>
</html>"));
      end RelativeAcceleration;

      model TransformAbsoluteVector "Transform absolute vector in to another frame"
        extends Modelica.Icons.RotationalSensor;
        Interfaces.Frame_a frame_a "Coordinate system from which absolute kinematic quantities are measured" annotation (
          Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
        Interfaces.Frame_resolve frame_resolve if frame_r_in == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve or frame_r_out == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve "Coordinate system in which r_in or r_out is optionally resolved" annotation (
          Placement(transformation(extent = {{84, -16}, {116, 16}}), iconTransformation(extent = {{84, -15}, {116, 17}})));
        Modelica.Blocks.Interfaces.RealInput r_in[3] "Input vector resolved in frame defined by frame_r_in" annotation (
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {0, 120})));
        Modelica.Blocks.Interfaces.RealOutput r_out[3] "Input vector r_in resolved in frame defined by frame_r_out" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA frame_r_in = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which vector r_in is resolved (1: world, 2: frame_a, 3: frame_resolve)";
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA frame_r_out = frame_r_in "Frame in which vector r_in shall be resolved and provided as r_out (1: world, 2: frame_a, 3: frame_resolve)";
      protected
        Internal.BasicTransformAbsoluteVector basicTransformVector(frame_r_in = frame_r_in, frame_r_out = frame_r_out) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Interfaces.ZeroPosition zeroPosition if not (frame_r_in == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve or frame_r_out == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve) annotation (
          Placement(transformation(extent = {{40, 20}, {60, 40}})));
      equation
        connect(basicTransformVector.frame_a, frame_a) annotation (
          Line(points = {{-10, 0}, {-32.5, 0}, {-32.5, 0}, {-55, 0}, {-55, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(basicTransformVector.frame_resolve, frame_resolve) annotation (
          Line(points = {{10, 0}, {32.5, 0}, {32.5, 0}, {55, 0}, {55, 0}, {100, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(zeroPosition.frame_resolve, basicTransformVector.frame_resolve) annotation (
          Line(points = {{40, 30}, {30, 30}, {30, 0}, {10, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(basicTransformVector.r_out, r_out) annotation (
          Line(points = {{0, -11}, {0, -35.75}, {0, -35.75}, {0, -60.5}, {0, -60.5}, {0, -110}}, color = {0, 0, 127}));
        connect(basicTransformVector.r_in, r_in) annotation (
          Line(points = {{0, 12}, {0, 39}, {0, 39}, {0, 66}, {0, 66}, {0, 120}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, -70}, {0, -100}}, color = {0, 0, 127}), Line(points = {{0, 100}, {0, 70}}, color = {0, 0, 127}), Text(extent = {{-104, 124}, {-18, 96}}, textString = "r_in"), Text(extent = {{-124, -76}, {2, -104}}, textString = "r_out"), Line(points = {{95, 0}, {95, 0}, {70, 0}, {70, 0}}, pattern = LinePattern.Dot), Text(extent = {{58, 47}, {189, 22}}, lineColor = {95, 95, 95}, textString = "resolve"), Line(points = {{-70, 0}, {-96, 0}, {-96, 0}}), Text(extent = {{-116, 45}, {-80, 20}}, lineColor = {95, 95, 95}, textString = "a")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
The input vector \"Real r_in[3]\" is assumed to be an absolute kinematic quantity
of frame_a that is defined to be resolved in the frame defined
with parameter \"frame_r_in\". This model resolves vector r_in in the
coordinate system defined with parameter \"frame_r_out\" and returns the
transformed output vector as \"Real r_out[3]\";
</p>
</html>"));
      end TransformAbsoluteVector;

      model TransformRelativeVector "Transform relative vector in to another frame"
        extends Internal.PartialRelativeSensor;
        Interfaces.Frame_resolve frame_resolve if frame_r_in == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve or frame_r_out == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which r_in or r_out is optionally resolved" annotation (
          Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 65}, {116, 97}})));
        Modelica.Blocks.Interfaces.RealInput r_in[3] "Input vector resolved in frame defined by frame_r_in" annotation (
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {0, 120})));
        Modelica.Blocks.Interfaces.RealOutput r_out[3] "Input vector r_in resolved in frame defined by frame_r_out" annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB frame_r_in = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a "Frame in which vector r_in is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB frame_r_out = frame_r_in "Frame in which vector r_in shall be resolved and provided as r_out (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
      protected
        Internal.BasicTransformRelativeVector basicTransformVector(frame_r_in = frame_r_in, frame_r_out = frame_r_out) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Interfaces.ZeroPosition zeroPosition if not (frame_r_in == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve or frame_r_out == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve) annotation (
          Placement(transformation(extent = {{40, 20}, {60, 40}})));
      equation
        connect(basicTransformVector.frame_a, frame_a) annotation (
          Line(points = {{-10, 0}, {-32.5, 0}, {-32.5, 0}, {-55, 0}, {-55, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(basicTransformVector.frame_b, frame_b) annotation (
          Line(points = {{10, 0}, {32.5, 0}, {32.5, 0}, {55, 0}, {55, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(basicTransformVector.frame_resolve, frame_resolve) annotation (
          Line(points = {{10, 8.1}, {30, 8.1}, {30, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(zeroPosition.frame_resolve, basicTransformVector.frame_resolve) annotation (
          Line(points = {{40, 30}, {30, 30}, {30, 8.1}, {10, 8.1}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(basicTransformVector.r_out, r_out) annotation (
          Line(points = {{0, -11}, {0, -35.75}, {0, -35.75}, {0, -60.5}, {0, -60.5}, {0, -110}}, color = {0, 0, 127}));
        connect(basicTransformVector.r_in, r_in) annotation (
          Line(points = {{0, 12}, {0, 39}, {0, 39}, {0, 66}, {0, 66}, {0, 120}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, -70}, {0, -100}}, color = {0, 0, 127}), Line(points = {{0, 100}, {0, 70}}, color = {0, 0, 127}), Text(extent = {{-104, 124}, {-18, 96}}, textString = "r_in"), Text(extent = {{-124, -76}, {2, -104}}, textString = "r_out")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
The input vector \"Real r_in[3]\" is assumed to be a relative kinematic quantity
between frame_a and frame_b
that is defined to be resolved in the frame defined
with parameter \"frame_r_in\". This model resolves vector r_in in the
coordinate system defined with parameter \"frame_r_out\" and returns the
transformed output vector as \"Real r_out[3]\";
</p>
</html>"));
      end TransformRelativeVector;

      model CutForce "Measure cut force vector"
        import SI = Modelica.SIunits;
        Modelica.Blocks.Interfaces.RealOutput force[2](each final quantity = "Force", each final unit = "N") "Cut force resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        parameter Boolean animation = true "= true, if animation shall be enabled (show arrow)";
        parameter Boolean positiveSign = true "= true, if force with positive sign is returned (= frame_a.f), otherwise with negative sign (= frame_b.f)";
        input Real N_to_m(unit = "N/m") = planarWorld.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input SI.Diameter forceDiameter = planarWorld.defaultArrowDiameter "Diameter of force arrow" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input Types.Color forceColor = Modelica.Mechanics.MultiBody.Types.Defaults.ForceColor "Color of force arrow" annotation (
          HideResult = true,
          Dialog(colorSelector = true, group = "if animation = true", enable = animation));
        input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(group = "if animation = true", enable = animation));
        extends Internal.PartialCutForceSensor;
      protected
        inner Modelica.Mechanics.MultiBody.World world;
        SI.Position f_in_m[3] = {frame_a.fx, frame_a.fy, 0} * (if positiveSign then +1 else -1) / N_to_m "Force mapped from N to m for animation";
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Arrow forceArrow(diameter = forceDiameter, color = forceColor, specularCoefficient = specularCoefficient, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, 0}) + planarWorld.r_0, r_tail = f_in_m, r_head = -f_in_m, R = planarWorld.R) if planarWorld.enableAnimation and animation;
        Internal.BasicCutForce cutForce(resolveInFrame = resolveInFrame, positiveSign = positiveSign) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Interfaces.ZeroPosition zeroPosition if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve annotation (
          Placement(transformation(extent = {{40, -40}, {60, -20}})));
      equation
        connect(cutForce.frame_a, frame_a) annotation (
          Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutForce.frame_b, frame_b) annotation (
          Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutForce.frame_resolve, frame_resolve) annotation (
          Line(points = {{8, -10}, {8, -60}, {80, -60}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(cutForce.force, force) annotation (
          Line(points = {{-8, -11}, {-8, -60}, {-80, -60}, {-80, -110}}, color = {0, 0, 127}));
        connect(zeroPosition.frame_resolve, cutForce.frame_resolve) annotation (
          Line(points = {{40, -30}, {8, -30}, {8, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-190, -70}, {-74, -96}}, textString = "force"), Line(points = {{-80, -100}, {-80, 0}}, color = {0, 0, 127})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The cut-force acting between the two frames to which this model is connected, is determined and provided at the output signal connector <b>force</b> (= frame_a.f). If parameter <b>positiveSign</b> = <b>false</b>, the negative cut-force is provided (= frame_b.f).</p>
<p>Via parameter <b>resolveInFrame</b> it is defined, in which frame the force vector is resolved: </p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><h4>resolveInFrame =</h4></p><p align=\"center\">Types.ResolveInFrameAB.</p></td>
<td><p align=\"center\"><h4>Meaning</h4></p></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_b</p></td>
<td valign=\"top\"><p>Resolve vector in frame_b</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and output force is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected.</p>
<p>In the following figure the modeling and animation of a CutForce sensor is shown.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Sensors/CutForce2.png\" alt=\"Modelica diagram\">
<img src=\"modelica://PlanarMechanics/Resources/Images/Sensors/CutForce.png\" alt=\"CutForce animation\"></p>
</html>"));
      end CutForce;

      model CutTorque "Measure cut torque vector"
        extends Internal.PartialCutTorqueSensor;
        import SI = Modelica.SIunits;
        Modelica.Blocks.Interfaces.RealOutput torque(final quantity = "Torque", final unit = "N.m") "Cut torque resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        parameter Boolean animation = true "= true, if animation shall be enabled (show arrow)";
        parameter Boolean positiveSign = true "= true, if torque with positive sign is returned (= frame_a.t), otherwise with negative sign (= frame_b.t)";
        input Real Nm_to_m(unit = "N.m/m") = planarWorld.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input SI.Diameter torqueDiameter = planarWorld.defaultArrowDiameter "Diameter of torque arrow" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input Types.Color torqueColor = Modelica.Mechanics.MultiBody.Types.Defaults.TorqueColor "Color of torque arrow" annotation (
          HideResult = true,
          Dialog(colorSelector = true, group = "if animation = true", enable = animation));
        input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(group = "if animation = true", enable = animation));
      protected
        inner Modelica.Mechanics.MultiBody.World world;
        SI.Position t_in_m[3] = {0, 0, frame_a.t} * (if positiveSign then +1 else -1) / Nm_to_m "Torque mapped from Nm to m for animation";
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.DoubleArrow torqueArrow(diameter = torqueDiameter, color = torqueColor, specularCoefficient = specularCoefficient, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, 0}) + planarWorld.r_0, r_tail = t_in_m, r_head = -t_in_m, R = planarWorld.R) if planarWorld.enableAnimation and animation;
        //R=Modelica.Mechanics.MultiBody.Frames.planarRotation({0,0,1},frame_b.phi,0),
        Internal.BasicCutTorque cutTorque(positiveSign = positiveSign) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        //   Interfaces.ZeroPosition zeroPosition if
        //     not (resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve)
        //     annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
      equation
        connect(cutTorque.frame_a, frame_a) annotation (
          Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutTorque.frame_b, frame_b) annotation (
          Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutTorque.torque, torque) annotation (
          Line(points = {{-8, -11}, {-8, -60}, {-80, -60}, {-80, -110}}, color = {0, 0, 127}));
        //   connect(cutTorque.frame_resolve, frame_resolve) annotation (Line(
        //       points={{-44,-10},{-44,-74},{80,-74},{80,-100}},
        //       color={95,95,95},
        //       pattern=LinePattern.Dot,
        //       ));
        //  connect(zeroPosition.frame_resolve, cutTorque.frame_resolve) annotation (Line(
        //       points={{-20,-30},{-44,-30},{-44,-10}},
        //       color={95,95,95},
        //       pattern=LinePattern.Dot,
        //       ));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-180, -72}, {-64, -98}}, textString = "torque"), Line(points = {{-80, -100}, {-80, 0}}, color = {0, 0, 127})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>The cut-torque acting between the two frames to which this model is connected, is determined and provided at the output signal connector <b>torque</b> (= frame_a.t). If parameter <b>positiveSign</b> = <b>false</b>, the negative cut-torque is provided (= frame_b.t).</p>
<p>Via parameter <b>resolveInFrame</b> it is defined, in which frame the torque vector is resolved: </p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<td><p align=\"center\"><h4>resolveInFrame =</h4></p><p align=\"center\">Types.ResolveInFrameAB.</p></td>
<td><p align=\"center\"><h4>Meaning</h4></p></td>
</tr>
<tr>
<td valign=\"top\"><p>world</p></td>
<td valign=\"top\"><p>Resolve vector in world frame</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_a</p></td>
<td valign=\"top\"><p>Resolve vector in frame_a</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_b</p></td>
<td valign=\"top\"><p>Resolve vector in frame_b</p></td>
</tr>
<tr>
<td valign=\"top\"><p>frame_resolve</p></td>
<td valign=\"top\"><p>Resolve vector in frame_resolve</p></td>
</tr>
</table>
<p>If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the conditional connector &quot;frame_resolve&quot; is enabled and output torque is resolved in the frame, to which frame_resolve is connected. Note, if this connector is enabled, it must be connected.</p>
<p>In the following figure the modeling and animation of a CutTorque sensor is shown.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/Sensors/CutTorque1.png\" alt=\"Modelica diagram\">
<img src=\"modelica://PlanarMechanics/Resources/Images/Sensors/CutTorque2.png\" alt=\"CutTorque2 animation\"></p>
</html>"));
      end CutTorque;

      model CutForceAndTorque "Measure cut force and cut torque vector"
        import SI = Modelica.SIunits;
        import Modelica.Mechanics.MultiBody.Types;
        Modelica.Blocks.Interfaces.RealOutput force[2](each final quantity = "Force", each final unit = "N") "Cut force resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealOutput torque(final quantity = "Torque", final unit = "N.m") "Cut torque resolved in frame defined by resolveInFrame" annotation (
          Placement(transformation(origin = {0, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        parameter Boolean animation = true "= true, if animation shall be enabled (show force and torque arrow)";
        parameter Boolean positiveSign = true "= true, if force and torque with positive sign is returned (= frame_a.f/.t), otherwise with negative sign (= frame_b.f/.t)";
        input Real N_to_m(unit = "N/m") = planarWorld.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input Real Nm_to_m(unit = "N.m/m") = planarWorld.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input SI.Diameter forceDiameter = planarWorld.defaultArrowDiameter "Diameter of force arrow" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input SI.Diameter torqueDiameter = forceDiameter "Diameter of torque arrow" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input Types.Color forceColor = Modelica.Mechanics.MultiBody.Types.Defaults.ForceColor "Color of force arrow" annotation (
          HideResult = true,
          Dialog(colorSelector = true, group = "if animation = true", enable = animation));
        input Types.Color torqueColor = Modelica.Mechanics.MultiBody.Types.Defaults.TorqueColor "Color of torque arrow" annotation (
          HideResult = true,
          Dialog(colorSelector = true, group = "if animation = true", enable = animation));
        input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(group = "if animation = true", enable = animation));
        extends Internal.PartialCutForceSensor;
      protected
        inner Modelica.Mechanics.MultiBody.World world;
        parameter Integer csign = if positiveSign then +1 else -1;
        SI.Position f_in_m[3] = {frame_a.fx, frame_a.fy, 0} * csign / N_to_m "Force mapped from N to m for animation";
        SI.Position t_in_m[3] = {0, 0, frame_a.t} * csign / Nm_to_m "Torque mapped from Nm to m for animation";
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Arrow forceArrow(diameter = forceDiameter, color = forceColor, specularCoefficient = specularCoefficient, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, 0}) + planarWorld.r_0, r_tail = f_in_m, r_head = -f_in_m, R = planarWorld.R) if planarWorld.enableAnimation and animation;
        //R=Modelica.Mechanics.MultiBody.Frames.planarRotation({0,0,1},frame_b.phi,0),
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.DoubleArrow torqueArrow(diameter = torqueDiameter, color = torqueColor, specularCoefficient = specularCoefficient, r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, 0}) + planarWorld.r_0, r_tail = t_in_m, r_head = -t_in_m, R = planarWorld.R) if planarWorld.enableAnimation and animation;
        //R=Modelica.Mechanics.MultiBody.Frames.planarRotation({0,0,1},frame_b.phi,0),
        Internal.BasicCutForce cutForce(resolveInFrame = resolveInFrame, positiveSign = positiveSign) annotation (
          Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
        Internal.BasicCutTorque cutTorque(positiveSign = positiveSign) annotation (
          Placement(transformation(extent = {{40, -10}, {60, 10}})));
        Interfaces.ZeroPosition zeroPosition if not resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve annotation (
          Placement(transformation(extent = {{-20, -40}, {0, -20}})));
      equation
        connect(cutForce.frame_a, frame_a) annotation (
          Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutForce.frame_b, cutTorque.frame_a) annotation (
          Line(points = {{-40, 0}, {40, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutTorque.frame_b, frame_b) annotation (
          Line(points = {{60, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(cutForce.force, force) annotation (
          Line(points = {{-58, -11}, {-58, -60}, {-80, -60}, {-80, -110}}, color = {0, 0, 127}));
        connect(cutTorque.torque, torque) annotation (
          Line(points = {{42, -11}, {42, -79.75}, {0, -79.75}, {0, -110}}, color = {0, 0, 127}));
        connect(zeroPosition.frame_resolve, cutForce.frame_resolve) annotation (
          Line(points = {{-20, -30}, {-42, -30}, {-42, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(cutForce.frame_resolve, frame_resolve) annotation (
          Line(points = {{-42, -10}, {-42, -60}, {80, -60}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{-80, -100}, {-80, 0}}, color = {0, 0, 127}), Line(points = {{0, -100}, {0, -70}}, color = {0, 0, 127}), Text(extent = {{-188, -70}, {-72, -96}}, textString = "force"), Text(extent = {{-56, -70}, {60, -96}}, textString = "torque")}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
The cut-force and cut-torque acting between the two frames to which this
model is connected, are determined and provided at the output signal connectors
<b>force</b> (= frame_a.f) and <b>torque</b> (= frame_a.t).
If parameter <b>positiveSign</b> =
<b>false</b>, the negative cut-force and cut-torque is provided
(= frame_b.f, frame_b.t).

<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the two vectors are resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>resolveInFrame =<br>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve vectors in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve vectors in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve vectors in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve vectors in frame_resolve</td></tr>
</table>

<p>
If <code>resolveInFrame = Types.ResolveInFrameAB.frame_resolve</code>, the conditional connector
\"frame_resolve\" is enabled and the output vectors force and torque are resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
In the following figure the animation of a CutForceAndTorque
sensor is shown. The dark blue coordinate system is frame_b,
and the green arrows are the cut force and the cut torque,
respectively, acting at frame_b and
with negative sign at frame_a.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Sensors/CutForceAndTorque.png\" alt=\"CutForceAndTorque animation\">
</p>
</html>"));
      end CutForceAndTorque;

      model Power "Measure power flowing from frame_a to frame_b"
        extends Modelica.Icons.RotationalSensor;
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        Modelica.Blocks.Interfaces.RealOutput power(final quantity = "Power", final unit = "W") "Power at frame_a as output signal" annotation (
          Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      equation
        //Connections.branch(frame_a.R, frame_b.R);
        //frame_a.r_0 = frame_b.r_0;
        {frame_a.x, frame_a.y} = {frame_b.x, frame_b.y};
        frame_a.phi = frame_b.phi;
        //frame_a.R = frame_b.R;
        zeros(2) = {frame_a.fx, frame_a.fy} + {frame_b.fx, frame_b.fy};
        0 = frame_a.t + frame_b.t;
        //power = frame_a.f*Frames.resolve2(frame_a.R, der(frame_a.r_0)) + frame_a.t*Frames.angularVelocity2(frame_a.R);
        power = {frame_a.fx, frame_a.fy} * der({frame_a.x, frame_a.y}) + frame_a.t * der(frame_a.phi);
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{-70, 0}, {-101, 0}}), Line(points = {{70, 0}, {100, 0}}), Line(points = {{-80, 0}, {-80, -100}}, color = {0, 0, 127}), Text(extent = {{-60, -92}, {16, -114}}, textString = "power"), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<HTML>
<p>
This component provides the power flowing from frame_a to frame_b
as output signal <b>power</b>.
</p>
</HTML>"));
      end Power;

      model Distance "Measure the distance between the origins of two frame connectors"
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        extends Modelica.Icons.TranslationalSensor;
        import Modelica.Mechanics.MultiBody.Frames;
        import Modelica.Mechanics.MultiBody.Types;
        Modelica.Blocks.Interfaces.RealOutput distance(final quantity = "Position", final unit = "m", min = 0) "Distance between the origin of frame_a and the origin of frame_b" annotation (
          Placement(transformation(origin = {0, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        parameter Boolean animation = true "= true, if animation shall be enabled (show arrow)";
        input SI.Diameter arrowDiameter = planarWorld.defaultArrowDiameter "Diameter of relative arrow from frame_a to frame_b" annotation (
          Dialog(group = "if animation = true", enable = animation));
        input Types.Color arrowColor = Modelica.Mechanics.MultiBody.Types.Defaults.SensorColor "Color of relative arrow from frame_a to frame_b" annotation (
          HideResult = true,
          Dialog(colorSelector = true, group = "if animation = true", enable = animation));
        input Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(group = "if animation = true", enable = animation));
        input SI.Position s_small(min = sqrt(Modelica.Constants.small)) = 1.E-10 "Prevent zero-division if distance between frame_a and frame_b is zero" annotation (
          Dialog(tab = "Advanced"));
      protected
        inner Modelica.Mechanics.MultiBody.World world;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Arrow arrow(r = {frame_a.x, frame_a.y, 0}, r_head = {frame_b.x, frame_b.y, 0} - {frame_a.x, frame_a.y, 0}, diameter = arrowDiameter, color = arrowColor, specularCoefficient = specularCoefficient) if planarWorld.enableAnimation and animation;
      protected
        SI.Position r_rel_0[3] = {frame_b.x, frame_b.y, 0} - {frame_a.x, frame_a.y, 0} "Position vector from frame_a to frame_b resolved in world frame";
        SI.Area L2 = r_rel_0 * r_rel_0;
        SI.Area s_small2 = s_small ^ 2;
      equation
        {frame_a.fx, frame_a.fy} = zeros(2);
        {frame_b.fx, frame_b.fy} = zeros(2);
        frame_a.t = 0;
        frame_b.t = 0;
        distance = smooth(1, if noEvent(L2 > s_small2) then sqrt(L2) else L2 / (2 * s_small) * (3 - L2 / s_small2));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, -60}, {0, -100}}, color = {0, 0, 127}), Line(points = {{-70, 0}, {-101, 0}}), Line(points = {{70, 0}, {100, 0}}), Text(extent = {{-150, 80}, {150, 40}}, textString = "%name", lineColor = {0, 0, 255})}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-22, 70}, {20, 46}}, textString = "s", lineColor = {0, 0, 255}), Line(points = {{-98, 40}, {88, 40}}, color = {0, 0, 255}), Polygon(points = {{102, 40}, {87, 46}, {87, 34}, {102, 40}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid)}),
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
The <b>distance</b> between the origins of frame_a
and of frame_b are determined and provided at the
output signal connector <b>distance</b>. This
distance is always positive. <b>Derivatives</b> of this
signal can be easily obtained by connecting the
block
<a href=\"modelica://Modelica.Blocks.Continuous.Der\">Modelica.Blocks.Continuous.Der</a>
to \"distance\" (this block performs analytic differentiation
of the input signal using the <code><strong>der</strong>(&hellip;)</code> operator).
</p>
<p>
In the following figure the animation of a Distance
sensor is shown. The light blue coordinate system is
frame_a, the dark blue coordinate system is frame_b, and
the yellow arrow is the animated sensor.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Sensors/Distance.png\" alt=\"Distance animation\">
</p>

<p>
If the distance is smaller as parameter <b>s_small</b> (in the \"advanced\" menu),
it is approximated such that its derivative is
finite for zero distance. Without such an approximation, the derivative would
be infinite and a division by zero would occur. The approximation is performed
in the following way: If distance > s_small, it is computed as sqrt(r*r) where
r is the position vector from the origin of frame_a to the origin of frame_b.
If the distance becomes smaller as s_small, the \"sqrt()\" function is approximated
by a second order polynomial, such that the function value and its first derivative
are identical for sqrt() and the polynomial at s_small. Furthermore, the polynomial
passes through zero. The effect is, that the distance function is continuous and
differentiable everywhere. The derivative at zero distance is 3/(2*s_small).
</p>
</html>"));
      end Distance;

      package Internal "Internal package for sensors, should not be used by user"
        extends Modelica.Icons.InternalPackage;

        partial model PartialAbsoluteSensor "Partial absolute sensor model for sensors defined by components"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system at which the kinematic quantities are measured" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-108, 43}, {-72, 18}}, lineColor = {128, 128, 128}, textString = "a"), Line(points = {{-70, 0}, {-96, 0}, {-96, 0}})}));
        end PartialAbsoluteSensor;

        partial model PartialRelativeSensor "Partial relative sensor model for sensors defined by components"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system a" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_b frame_b "Coordinate system b" annotation (
            Placement(transformation(extent = {{84, -16}, {116, 16}})));
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
          assert(cardinality(frame_b) > 0, "Connector frame_b must be connected at least once");
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-108, 43}, {-72, 18}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{72, 41}, {108, 16}}, lineColor = {128, 128, 128}, textString = "b"), Line(points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(points = {{96, 0}, {70, 0}, {70, 0}}), Line(points = {{60, 36}, {60, 36}, {60, 80}, {95, 80}}, pattern = LinePattern.Dot)}));
        end PartialRelativeSensor;

        model BasicAbsolutePosition "Measure absolute position vector (same as Sensors.AbsolutePosition, but frame_resolve is not conditional and must be connected)"
          import Modelica.Mechanics.MultiBody.Types.ResolveInFrameA;
          extends Sensors.Internal.PartialAbsoluteBaseSensor;
          Modelica.Blocks.Interfaces.RealOutput r[3] "Absolute position vector frame_a.r_0 resolved in frame defined by resolveInFrame" annotation (
            Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which output vector r is resolved (1: world, 2: frame_a, 3: frame_resolve)";
        equation
          if resolveInFrame == ResolveInFrameA.world then
            r = {frame_a.x, frame_a.y, frame_a.phi};
          elseif resolveInFrame == ResolveInFrameA.frame_a then
            r = transpose({{cos(frame_a.phi), -sin(frame_a.phi), 0}, {sin(frame_a.phi), cos(frame_a.phi), 0}, {0, 0, 1}}) * {frame_a.x, frame_a.y, frame_a.phi} - {0, 0, frame_a.phi};
          elseif resolveInFrame == ResolveInFrameA.frame_resolve then
            r = transpose({{cos(frame_resolve.phi), -sin(frame_resolve.phi), 0}, {sin(frame_resolve.phi), cos(frame_resolve.phi), 0}, {0, 0, 1}}) * {frame_a.x, frame_a.y, frame_a.phi} - {0, 0, frame_resolve.phi};
          else
            assert(false, "Wrong value for parameter resolveInFrame");
            r = zeros(3);
          end if;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Text(extent = {{61, 47}, {145, 17}}, textString = "r"), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}));
        end BasicAbsolutePosition;

        model BasicRelativePosition "Measure relative position vector (same as Sensors.RelativePosition, but frame_resolve is not conditional and must be connected)"
          import Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB;
          extends Sensors.Internal.PartialRelativeBaseSensor;
          Modelica.Blocks.Interfaces.RealOutput r_rel[3] "Relative position vector frame_b.r_0 - frame_a.r_0 resolved in frame defined by resolveInFrame" annotation (
            Placement(transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector r_rel is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
        equation
          if resolveInFrame == ResolveInFrameAB.frame_a then
            r_rel = transpose({{cos(frame_a.phi), -sin(frame_a.phi), 0}, {sin(frame_a.phi), cos(frame_a.phi), 0}, {0, 0, 1}}) * {frame_b.x - frame_a.x, frame_b.y - frame_a.y, frame_b.phi - frame_a.phi};
          elseif resolveInFrame == ResolveInFrameAB.frame_b then
            r_rel = transpose({{cos(frame_b.phi), -sin(frame_b.phi), 0}, {sin(frame_b.phi), cos(frame_b.phi), 0}, {0, 0, 1}}) * {frame_b.x - frame_a.x, frame_b.y - frame_a.y, frame_b.phi - frame_a.phi};
          elseif resolveInFrame == ResolveInFrameAB.world then
            r_rel = {frame_b.x - frame_a.x, frame_b.y - frame_a.y, frame_b.phi - frame_a.phi};
          elseif resolveInFrame == ResolveInFrameAB.frame_resolve then
            r_rel = transpose({{cos(frame_resolve.phi), -sin(frame_resolve.phi), 0}, {sin(frame_resolve.phi), cos(frame_resolve.phi), 0}, {0, 0, 1}}) * {frame_b.x - frame_a.x, frame_b.y - frame_a.y, frame_b.phi - frame_a.phi};
            //r_rel = Frames.resolve2(frame_resolve.R, frame_b.r_0 - frame_a.r_0);
          else
            assert(false, "Wrong value for parameter resolveInFrame");
            r_rel = zeros(3);
          end if;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Text(extent = {{12, -76}, {96, -106}}, textString = "r_rel"), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255})}));
        end BasicRelativePosition;

        model PartialAbsoluteBaseSensor "Partial absolute sensor models for sensors defined by equations (frame_resolve must be connected exactly once)"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system from which kinematic quantities are measured" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_resolve frame_resolve "Coordinate system in which vector is optionally resolved" annotation (
            Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {10, -100}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100})));
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
          assert(cardinality(frame_resolve) == 1, "Connector frame_resolve must be connected exactly once");
          frame_a.fx = 0;
          frame_a.fy = 0;
          frame_a.t = 0;
          frame_resolve.fx = 0;
          frame_resolve.fy = 0;
          frame_resolve.t = 0;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Text(extent = {{-108, 43}, {-72, 18}}, lineColor = {95, 95, 95}, textString = "a"), Line(points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(points = {{0, 15}, {0, -15}}, color = {0, 0, 127}, origin = {85, 0}, rotation = 90), Line(points = {{0, -95}, {0, -95}, {0, -70}, {0, -70}}, pattern = LinePattern.Dot), Text(extent = {{0, -75}, {131, -100}}, lineColor = {95, 95, 95}, textString = "resolve")}));
        end PartialAbsoluteBaseSensor;

        model PartialRelativeBaseSensor "Partial relative sensor models for sensors defined by equations (frame_resolve must be connected exactly once)"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system a (measurement is between frame_a and frame_b)" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_b frame_b "Coordinate system b (measurement is between frame_a and frame_b)" annotation (
            Placement(transformation(extent = {{84, -16}, {116, 16}})));
          Interfaces.Frame_resolve frame_resolve "Coordinate system in which vector is optionally resolved" annotation (
            Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 65}, {116, 97}})));
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
          assert(cardinality(frame_b) > 0, "Connector frame_b must be connected at least once");
          assert(cardinality(frame_resolve) == 1, "Connector frame_resolve must be connected exactly once");
          frame_a.fx = 0;
          frame_a.fy = 0;
          frame_a.t = 0;
          frame_b.fx = 0;
          frame_b.fy = 0;
          frame_b.t = 0;
          frame_resolve.fx = 0;
          frame_resolve.fy = 0;
          frame_resolve.t = 0;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-108, 43}, {-72, 18}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{72, 41}, {108, 16}}, lineColor = {128, 128, 128}, textString = "b"), Line(points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(points = {{96, 0}, {70, 0}, {70, 0}}), Line(points = {{0, -70}, {0, -100}}, color = {0, 0, 127}), Line(points = {{60, 36}, {60, 36}, {60, 80}, {95, 80}}, pattern = LinePattern.Dot)}));
        end PartialRelativeBaseSensor;

        model BasicTransformAbsoluteVector "Transform absolute vector in to another frame"
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Mechanics.MultiBody.Types.ResolveInFrameA;
          extends Modelica.Icons.RotationalSensor;
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA frame_r_in = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which vector r_in is resolved (1: world, 2: frame_a, 3: frame_resolve)";
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA frame_r_out = frame_r_in "Frame in which vector r_out (= r_in in other frame) is resolved (1: world, 2: frame_a, 3: frame_resolve)";
          Interfaces.Frame_a frame_a "Coordinate system from which absolute kinematic quantities are measured" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_resolve frame_resolve "Coordinate system in which vector is optionally resolved" annotation (
            Placement(transformation(extent = {{-16, -16}, {16, 16}}, origin = {100, 0}), iconTransformation(extent = {{-16, -16}, {16, 16}}, origin = {100, 0})));
          Modelica.Blocks.Interfaces.RealInput r_in[3] "Input vector resolved in frame defined by frame_r_in" annotation (
            Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {0, 120})));
          Modelica.Blocks.Interfaces.RealOutput r_out[3] "Input vector r_in resolved in frame defined by frame_r_out" annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
        protected
          Real R1[3, 4] "Orientation object from frame in which r_in is resolved to world frame";
          Real r_temp[3] "Temporary vector in transformation calculation";
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
          assert(cardinality(frame_resolve) == 1, "Connector frame_resolve must be connected exactly once");
          frame_a.fx = 0;
          frame_a.fy = 0;
          frame_a.t = 0;
          frame_resolve.fx = 0;
          frame_resolve.fy = 0;
          frame_resolve.t = 0;
          r_temp = R1 * {r_in[1], r_in[2], r_in[3], 1};
          if frame_r_out == frame_r_in then
            r_out = r_in;
            R1 = {{cos(0), -sin(0), 0, 0}, {sin(0), cos(0), 0, 0}, {0, 0, 1, 0}};
          else
            if frame_r_in == ResolveInFrameA.world then
              R1 = {{cos(0), -sin(0), 0, 0}, {sin(0), cos(0), 0, 0}, {0, 0, 1, 0}};
            elseif frame_r_in == ResolveInFrameA.frame_a then
              R1 = {{cos(frame_a.phi), -sin(frame_a.phi), 0, 0}, {sin(frame_a.phi), cos(frame_a.phi), 0, 0}, {0, 0, 1, frame_a.phi}};
            elseif frame_r_in == ResolveInFrameA.frame_resolve then
              R1 = {{cos(frame_resolve.phi), -sin(frame_resolve.phi), 0, 0}, {sin(frame_resolve.phi), cos(frame_resolve.phi), 0, 0}, {0, 0, 1, frame_resolve.phi}};
            else
              assert(false, "Wrong value for parameter frame_r_in");
              R1 = {{cos(0), -sin(0), 0, 0}, {sin(0), cos(0), 0, 0}, {0, 0, 1, 0}};
            end if;
            if frame_r_out == ResolveInFrameA.world then
              r_out = r_temp;
            elseif frame_r_out == ResolveInFrameA.frame_a then
              r_out = {{cos(frame_a.phi), sin(frame_a.phi), 0}, {-sin(frame_a.phi), cos(frame_a.phi), 0}, {0, 0, 1}} * {r_temp[1], r_temp[2], r_temp[3]};
            elseif frame_r_out == ResolveInFrameA.frame_resolve then
              r_out = {{cos(frame_resolve.phi), sin(frame_resolve.phi), 0}, {-sin(frame_resolve.phi), cos(frame_resolve.phi), 0}, {0, 0, 1}} * {r_temp[1], r_temp[2], r_temp[3]};
            else
              assert(false, "Wrong value for parameter frame_r_out");
              r_out = zeros(3);
            end if;
          end if;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {1, 1}), graphics={  Text(extent = {{-128, -84}, {-2, -112}}, textString = "r_out"), Text(extent = {{-108, 137}, {-22, 109}}, textString = "r_in"), Line(points = {{0, 100}, {0, 70}}, color = {0, 0, 127}), Line(points = {{0, -70}, {0, -100}}, color = {0, 0, 127}), Text(extent = {{58, 47}, {189, 22}}, lineColor = {95, 95, 95}, textString = "resolve"), Text(extent = {{-116, 45}, {-80, 20}}, lineColor = {95, 95, 95}, textString = "a"), Line(points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(points = {{95, 0}, {95, 0}, {70, 0}, {70, 0}}, pattern = LinePattern.Dot)}));
        end BasicTransformAbsoluteVector;

        model BasicTransformRelativeVector "Transform relative vector in to another frame"
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB;
          extends Internal.PartialRelativeBaseSensor;
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB frame_r_in = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a "Frame in which vector r_in is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB frame_r_out = frame_r_in "Frame in which vector r_out (= r_in in other frame) is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
          Modelica.Blocks.Interfaces.RealInput r_in[3] "Input vector resolved in frame defined by frame_r_in" annotation (
            Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {0, 120})));
          Modelica.Blocks.Interfaces.RealOutput r_out[3] "Input vector r_in resolved in frame defined by frame_r_out" annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
          // protected
          //   Modelica.Mechanics.MultiBody.Frames.Orientation R1
          //     "Orientation object from world frame to frame in which r_in is resolved";
          Real R1[3, 3];
        equation
          if frame_r_out == frame_r_in then
            r_out = r_in;
            //R1 = Frames.nullRotation();
            R1 = {{cos(0), -sin(0), 0}, {sin(0), cos(0), 0}, {0, 0, 1}};
          else
            if frame_r_in == ResolveInFrameAB.world then
              R1 = {{cos(0), -sin(0), 0}, {sin(0), cos(0), 0}, {0, 0, 1}};
            elseif frame_r_in == ResolveInFrameAB.frame_a then
              R1 = {{cos(frame_a.phi), -sin(frame_a.phi), 0}, {sin(frame_a.phi), cos(frame_a.phi), 0}, {0, 0, 1}};
            elseif frame_r_in == ResolveInFrameAB.frame_b then
              R1 = {{cos(frame_b.phi), -sin(frame_b.phi), 0}, {sin(frame_b.phi), cos(frame_b.phi), 0}, {0, 0, 1}};
            else
              R1 = {{cos(frame_resolve.phi), -sin(frame_resolve.phi), 0}, {sin(frame_resolve.phi), cos(frame_resolve.phi), 0}, {0, 0, 1}};
            end if;
            if frame_r_out == ResolveInFrameAB.world then
              r_out = R1 * r_in;
            elseif frame_r_out == ResolveInFrameAB.frame_a then
              r_out = transpose({{cos(frame_a.phi), -sin(frame_a.phi), 0}, {sin(frame_a.phi), cos(frame_a.phi), 0}, {0, 0, 1}}) * (R1 * r_in);
            elseif frame_r_out == ResolveInFrameAB.frame_b then
              r_out = transpose({{cos(frame_b.phi), -sin(frame_b.phi), 0}, {sin(frame_b.phi), cos(frame_b.phi), 0}, {0, 0, 1}}) * (R1 * r_in);
            else
              r_out = transpose({{cos(frame_resolve.phi), -sin(frame_resolve.phi), 0}, {sin(frame_resolve.phi), cos(frame_resolve.phi), 0}, {0, 0, 1}}) * (R1 * r_in);
            end if;
          end if;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-128, -92}, {-2, -120}}, textString = "r_out"), Text(extent = {{-108, 144}, {-22, 116}}, textString = "r_in"), Line(points = {{0, 100}, {0, 70}}, color = {0, 0, 127})}));
        end BasicTransformRelativeVector;

        partial model PartialCutForceSensor "Base model to measure the cut force and/or torque between two frames, defined by components"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system a" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_b frame_b "Coordinate system b" annotation (
            Placement(transformation(extent = {{84, -16}, {116, 16}})));
          Interfaces.Frame_resolve frame_resolve if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve "Output vectors are optionally resolved in this frame (cut-force/-torque are set to zero)" annotation (
            Placement(transformation(origin = {80, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 270)));
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which output vector(s) is/are resolved (1: world, 2: frame_a, 3: frame_resolve)";
        protected
          outer PlanarWorld planarWorld;
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a of cut-force/-torque sensor object is not connected");
          assert(cardinality(frame_b) > 0, "Connector frame_b of cut-force/-torque sensor object is not connected");
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This is a base class for 3-dim. mechanical components with two frames
and one output port in order to measure the cut-force and/or
cut-torque acting between the two frames and
to provide the measured signals as output for further processing
with the blocks of package Modelica.Blocks.
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Line(points = {{-70, 0}, {-101, 0}}), Line(points = {{70, 0}, {100, 0}}), Text(extent = {{-118, 55}, {-82, 30}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{83, 55}, {119, 30}}, lineColor = {128, 128, 128}, textString = "b"), Text(extent = {{70, -66}, {201, -91}}, lineColor = {95, 95, 95}, textString = "resolve"), Line(points = {{80, 0}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}));
        end PartialCutForceSensor;

        partial model PartialCutTorqueSensor "Base model to measure the cut force and/or torque between two frames, defined by components"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system a" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_b frame_b "Coordinate system b" annotation (
            Placement(transformation(extent = {{84, -16}, {116, 16}})));
          //   Interfaces.Frame_resolve frame_resolve if
          //          resolveInFrame==Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve
          //     "Output vectors are optionally resolved in this frame (cut-force/-torque are set to zero)"
          //     annotation (Placement(transformation(
          //         origin={80,-100},
          //         extent={{-16,-16},{16,16}},
          //         rotation=270)));
          //
          //   parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA
          //     resolveInFrame=
          //   Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a
          //     "Frame in which output vector(s) is/are resolved (1: world, 2: frame_a, 3: frame_resolve)";
        protected
          outer PlanarWorld planarWorld;
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a of cut-force/-torque sensor object is not connected");
          assert(cardinality(frame_b) > 0, "Connector frame_b of cut-force/-torque sensor object is not connected");
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This is a base class for 3-dim. mechanical components with two frames
and one output port in order to measure the cut-force and/or
cut-torque acting between the two frames and
to provide the measured signals as output for further processing
with the blocks of package Modelica.Blocks.
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Line(points = {{-70, 0}, {-101, 0}}), Line(points = {{70, 0}, {100, 0}}), Text(extent = {{-118, 55}, {-82, 30}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{83, 55}, {119, 30}}, lineColor = {128, 128, 128}, textString = "b"), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}));
        end PartialCutTorqueSensor;

        partial model PartialCutForceBaseSensor "Base model to measure the cut force and/or torque between two frames, defined by equations (frame_resolve must be connected exactly once)"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system a" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_b frame_b "Coordinate system b" annotation (
            Placement(transformation(extent = {{84, -16}, {116, 16}})));
          Interfaces.Frame_resolve frame_resolve "The output vector is optionally resolved in this frame (cut-force/-torque are set to zero)" annotation (
            Placement(transformation(origin = {80, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 270)));
          parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a "Frame in which output vector is resolved (1: world, 2: frame_a, 3: frame_resolve)";
        protected
          outer PlanarWorld planarWorld;
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a of cut-force/-torque sensor object is not connected");
          assert(cardinality(frame_b) > 0, "Connector frame_b of cut-force/-torque sensor object is not connected");
          // frame_a and frame_b are identical
          {frame_a.x, frame_a.y} = {frame_b.x, frame_b.y};
          frame_a.phi = frame_b.phi;
          // force and torque balance
          zeros(2) = {frame_a.fx, frame_a.fy} + {frame_b.fx, frame_b.fy};
          0 = frame_a.t + frame_b.t;
          frame_resolve.fx = 0;
          frame_resolve.fy = 0;
          frame_resolve.t = 0;
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This is a base class for 3-dim. mechanical components with two frames
and one output port in order to measure the cut-force and/or
cut-torque acting between the two frames and
to provide the measured signals as output for further processing
with the blocks of package Modelica.Blocks.
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Line(points = {{-70, 0}, {-101, 0}}), Line(points = {{70, 0}, {100, 0}}), Text(extent = {{-118, 55}, {-82, 30}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{83, 55}, {119, 30}}, lineColor = {128, 128, 128}, textString = "b"), Text(extent = {{70, -66}, {201, -91}}, lineColor = {95, 95, 95}, textString = "resolve"), Line(points = {{80, 0}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}));
        end PartialCutForceBaseSensor;

        partial model PartialCutTorqueBaseSensor "Base model to measure the cut force and/or torque between two frames, defined by equations (frame_resolve must be connected exactly once)"
          extends Modelica.Icons.RotationalSensor;
          Interfaces.Frame_a frame_a "Coordinate system a" annotation (
            Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
          Interfaces.Frame_b frame_b "Coordinate system b" annotation (
            Placement(transformation(extent = {{84, -16}, {116, 16}})));
          //   Interfaces.Frame_resolve frame_resolve
          //     "The output vector is optionally resolved in this frame (cut-force/-torque are set to zero)"
          //     annotation (Placement(transformation(
          //         origin={80,-100},
          //         extent={{-16,-16},{16,16}},
          //         rotation=270)));
          //
          //   parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA
          //     resolveInFrame=
          //   Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a
          //     "Frame in which output vector is resolved (1: world, 2: frame_a, 3: frame_resolve)";
        protected
          outer PlanarWorld planarWorld;
        equation
          assert(cardinality(frame_a) > 0, "Connector frame_a of cut-force/-torque sensor object is not connected");
          assert(cardinality(frame_b) > 0, "Connector frame_b of cut-force/-torque sensor object is not connected");
          // frame_a and frame_b are identical
          {frame_a.x, frame_a.y} = {frame_b.x, frame_b.y};
          frame_a.phi = frame_b.phi;
          // force and torque balance
          zeros(2) = {frame_a.fx, frame_a.fy} + {frame_b.fx, frame_b.fy};
          0 = frame_a.t + frame_b.t;
          //   frame_resolve.fx = 0;
          //   frame_resolve.fy = 0;
          //   frame_resolve.t = 0;
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This is a base class for 3-dim. mechanical components with two frames
and one output port in order to measure the cut-force and/or
cut-torque acting between the two frames and
to provide the measured signals as output for further processing
with the blocks of package Modelica.Blocks.
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Line(points = {{-70, 0}, {-101, 0}}), Line(points = {{70, 0}, {100, 0}}), Text(extent = {{-118, 55}, {-82, 30}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{83, 55}, {119, 30}}, lineColor = {128, 128, 128}, textString = "b"), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}));
        end PartialCutTorqueBaseSensor;

        model BasicCutForce "Measure cut force vector (frame_resolve must be connected)"
          import SI = Modelica.SIunits;
          import Modelica.Mechanics.MultiBody.Types.ResolveInFrameA;
          import Modelica.Mechanics.MultiBody.Frames;
          extends Internal.PartialCutForceBaseSensor;
          Modelica.Blocks.Interfaces.RealOutput force[2](each final quantity = "Force", each final unit = "N") "Cut force resolved in frame defined by resolveInFrame" annotation (
            Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
          parameter Boolean positiveSign = true "= true, if force with positive sign is returned (= frame_a.f), otherwise with negative sign (= frame_b.f)";
        protected
          parameter Integer csign = if positiveSign then +1 else -1;
        equation
          if resolveInFrame == ResolveInFrameA.world then
            //force = Frames.resolve1(frame_a.R, frame_a.f)*csign;
            force = {frame_a.fx, frame_a.fy} * csign;
          elseif resolveInFrame == ResolveInFrameA.frame_a then
            force = {{cos(frame_a.phi), sin(frame_a.phi)}, {-sin(frame_a.phi), cos(frame_a.phi)}} * {frame_a.fx, frame_a.fy} * csign;
          elseif resolveInFrame == ResolveInFrameA.frame_resolve then
            //force = Frames.resolveRelative(frame_a.f, frame_a.R, frame_resolve.R)*csign;
            force = {{cos(frame_resolve.phi), sin(frame_resolve.phi)}, {-sin(frame_resolve.phi), cos(frame_resolve.phi)}} * {frame_a.fx, frame_a.fy} * csign;
          else
            assert(false, "Wrong value for parameter resolveInFrame");
            force = zeros(2);
          end if;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {1, 1}), graphics={  Text(extent = {{-190, -70}, {-74, -96}}, textString = "force"), Line(points = {{-80, -100}, {-80, 0}}, color = {0, 0, 127})}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<HTML>

</HTML>"));
        end BasicCutForce;

        model BasicCutTorque "Measure cut torque vector (frame_resolve must be connected)"
          import SI = Modelica.SIunits;
          import Modelica.Mechanics.MultiBody.Types.ResolveInFrameA;
          import Modelica.Mechanics.MultiBody.Frames;
          extends Internal.PartialCutTorqueBaseSensor;
          Modelica.Blocks.Interfaces.RealOutput torque(final quantity = "Torque", final unit = "N.m") "Cut torque resolved in frame defined by resolveInFrame" annotation (
            Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
          parameter Boolean positiveSign = true "= true, if torque with positive sign is returned (= frame_a.t), otherwise with negative sign (= frame_b.t)";
        protected
          parameter Integer csign = if positiveSign then +1 else -1;
        equation
          torque = frame_a.t * csign;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-190, -70}, {-74, -96}}, textString = "torque"), Line(points = {{-80, -100}, {-80, 0}}, color = {0, 0, 127})}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<HTML>

</HTML>"));
        end BasicCutTorque;
        annotation (
          Documentation(info = "<html>
<p>
A collection of internal material for sensor models. Generally, the material collected here should not be used by a common user.
</p>
</html>"));
      end Internal;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Package <b>Sensors</b> contains <b>ideal measurement</b> components to determine absolute and relative kinematic quantities, as well as cut-forces, cut-torques and power. All measured quantities can be provided in every desired coordinate system.</p>
</html>"));
    end Sensors;

    package Sources "Sources to drive 2D mechanical components"
      extends Modelica.Icons.SourcesPackage;

      model RelativeForce "Input signal acting as force and torque on two frames"
        extends PlanarMechanics.Interfaces.PartialTwoFrames;
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector r_rel shall be resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
        parameter Boolean animation = true "= true, if animation shall be enabled";
        parameter Real N_to_m(unit = "N/m") = planarWorld.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        parameter Real Nm_to_m(unit = "N.m/m") = planarWorld.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        input SI.Diameter diameter = planarWorld.defaultArrowDiameter "Diameter of force arrow" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the fixed translation" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        input Types.Color color = PlanarMechanics.Types.Defaults.ForceColor "Color of arrow" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", colorSelector = true, enable = animation));
        input Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        Modelica.Blocks.Interfaces.RealInput force[3] annotation (
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, origin = {-60, -120}, rotation = 90)));
        Real R[2, 2] "Rotation matrix";
        SI.Angle phi "Rotation angle of the additional frame_c";
        Interfaces.Frame_resolve frame_resolve(fx = 0, fy = 0, t = 0, phi = phi) if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which vector is optionally resolved, if useExtraFrame is true" annotation (
          Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 90, origin = {40, -100})));
      protected
        SI.Position f_in_m[3] = {force[1], force[2], 0} / N_to_m "Force mapped from N to m for animation";
        SI.Position t_in_m[3] = {0, 0, force[3]} / Nm_to_m "Torque mapped from N.m to m for animation";
        PlanarMechanics.Visualizers.Advanced.Arrow arrow(diameter = diameter, color = color, specularCoefficient = specularCoefficient, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, phi, der(phi))), r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, zPosition}) + planarWorld.r_0, r_tail = -f_in_m, r_head = f_in_m) if planarWorld.enableAnimation and animation;
        PlanarMechanics.Visualizers.Advanced.DoubleArrow doubleArrow(diameter = diameter, color = color, specularCoefficient = specularCoefficient, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 1, 0}, Modelica.Constants.pi, 0)), r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, zPosition}) + planarWorld.r_0, r_tail = t_in_m, r_head = -t_in_m) if planarWorld.enableAnimation and animation;
      equation
        if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_a then
          phi = frame_a.phi;
          //R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi),cos(frame_a.phi)}};
        elseif resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b then
          phi = frame_b.phi;
          //R = {{cos(frame_b.phi), -sin(frame_b.phi)}, {sin(frame_b.phi),cos(frame_b.phi)}};
          //R = {{cos(frame_resolve.phi), -sin(frame_resolve.phi)}, {sin(frame_resolve.phi),cos(frame_resolve.phi)}};
        elseif resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.world then
          phi = 0;
        end if;
        R = {{cos(phi), -sin(phi)}, {sin(phi), cos(phi)}};
        {frame_b.fx, frame_b.fy} + R * {force[1], force[2]} = {0, 0};
        frame_b.t + force[3] = 0;
        frame_a.fx + frame_b.fx = 0;
        frame_a.fy + frame_b.fy = 0;
        frame_a.t + frame_b.t = 0;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{30, 0}, {30, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(points = {{-60, -100}, {30, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Polygon(points = {{14, 10}, {44, 10}, {44, 40}, {94, 0}, {44, -40}, {44, -10}, {14, -10}, {14, 10}}, lineColor = {0, 127, 0}, fillColor = {215, 215, 215},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{-14, 10}, {-44, 10}, {-44, 40}, {-94, 0}, {-44, -40}, {-44, -10}, {-14, -10}, {-14, 10}}, lineColor = {0, 127, 0}, fillColor = {215, 215, 215},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 110}, {150, 70}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{-108, -24}, {-72, -49}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{72, -24}, {108, -49}}, lineColor = {128, 128, 128}, textString = "b")}),
          Documentation(info = "<html>
<p>
The <b>3</b> signals of the <b>force</b> connector contain force and torque.
The first and second signal are interpreted as the x- and y-coordinates of
a <b>force</b> and the third is torque, acting between two frame connectors
to which frame_a and frame_b are attached respectively.
Note that torque is a scalar quantity, which is exerted perpendicular
to the x-y plane.
</p>
<p>
Parameter <code><b>resolveInFrame</b></code> defines in which frame the input
force shall be resolved.
</p>

<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\">
<tr>
<th>Types.ResolveInFrameB.</th>
<th>Meaning</th>
</tr>
<tr>
<td valign=\"top\">world</td>
<td valign=\"top\">Resolve input force in world frame (= default)</td>
</tr>
<tr>
<td valign=\"top\">frame_a</td>
<td valign=\"top\">Resolve input force in frame_a</td>
</tr>
<tr>
<td valign=\"top\">frame_b</td>
<td valign=\"top\">Resolve input force in frame_b</td>
</tr>
<tr>
<td valign=\"top\">frame_resolve</td>
<td valign=\"top\">Resolve input force in frame_resolve (frame_resolve must be connected)</td>
</tr>
</table>

<p>
If resolveInFrame&nbsp;=&nbsp;Types.ResolveInFrameAB.frame_resolve,
the force coordinates shall be resolved in the frame, which is
connected to <b>frame_resolve</b>.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end RelativeForce;

      model WorldForce "External force and torque acting at frame_b, defined by 3 input signals and resolved in world frame"
        outer PlanarWorld planarWorld "Planar world model";
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameB resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b "Frame in which output vector r_rel shall be resolved (1: world, 2: frame_b, 3: frame_resolve)";
        parameter Boolean animation = true "= true, if animation shall be enabled";
        parameter Real N_to_m(unit = "N/m") = planarWorld.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        parameter Real Nm_to_m(unit = "N.m/m") = planarWorld.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        input SI.Diameter diameter = planarWorld.defaultArrowDiameter "Diameter of force arrow" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the fixed translation" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        input Types.Color color = PlanarMechanics.Types.Defaults.ForceColor "Color of arrow" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", colorSelector = true, enable = animation));
        input Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque" annotation (
          Placement(transformation(extent = {{84, -16}, {116, 16}})));
        Modelica.Blocks.Interfaces.RealInput force[3] "x-, y-coordinates of force and torque resolved in world frame" annotation (
          Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
        Interfaces.Frame_resolve frame_resolve(fx = 0, fy = 0, t = 0, phi = phi) if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve "Coordinate system in which vector is optionally resolved, if useExtraFrame is true" annotation (
          Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 90, origin = {0, -100})));
        Real R[2, 2] "Rotation matrix";
        SI.Angle phi "Rotation angle of the additional frame_c";
      protected
        SI.Position f_in_m[3] = {force[1], force[2], 0} / N_to_m "Force mapped from N to m for animation";
        SI.Position t_in_m[3] = {0, 0, force[3]} / Nm_to_m "Torque mapped from N.m to m for animation";
        PlanarMechanics.Visualizers.Advanced.Arrow arrow(diameter = diameter, color = color, specularCoefficient = specularCoefficient, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 0, 1}, phi, der(phi))), r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, zPosition}) + planarWorld.r_0, r_tail = -f_in_m, r_head = f_in_m) if planarWorld.enableAnimation and animation;
        PlanarMechanics.Visualizers.Advanced.DoubleArrow doubleArrow(diameter = diameter, color = color, specularCoefficient = specularCoefficient, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({0, 1, 0}, Modelica.Constants.pi, 0)), r = MB.Frames.resolve1(planarWorld.R, {frame_b.x, frame_b.y, zPosition}) + planarWorld.r_0, r_tail = t_in_m, r_head = -t_in_m) if planarWorld.enableAnimation and animation;
      equation
        if resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b then
          phi = frame_b.phi;
        elseif resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world then
          phi = 0;
        end if;
        R = {{cos(phi), -sin(phi)}, {sin(phi), cos(phi)}};
        {frame_b.fx, frame_b.fy} + R * {force[1], force[2]} = {0, 0};
        frame_b.t + force[3] = 0;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, 0}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot, visible = resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve), Polygon(points = {{-100, 10}, {20, 10}, {20, 41}, {90, 0}, {20, -41}, {20, -10}, {-100, -10}, {-100, 10}}, lineColor = {0, 127, 0}, fillColor = {215, 215, 215},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 80}, {150, 40}},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Sphere, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(info = "<html>
<p>
The <b>3</b> signals of the <b>force</b> connector contain force and torque.
The first and second signal are interpreted as the x- and y-coordinates of
a <b>force</b> and the third is torque, acting at the frame connector
to which <b>frame_b</b> of this component is attached.
Note that torque is a scalar quantity, which is exerted perpendicular
to the x-y plane.
</p>
<p>
An example of this model is given in the following figure:
</p>

<blockquote>
<img src=\"modelica://PlanarMechanics/Resources/Images/Sources/WorldForce.png\" alt=\"Modelica diagram\">
</blockquote>

<p>
The parameter resolveInFrame defines in which frame the input force shall be resolved.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end WorldForce;

      model QuadraticSpeedDependentForce "External force and torque acting at frame_b, defined by 3 input signals and resolved in world frame"
        outer PlanarWorld planarWorld "Planar world model";
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world "Frame in which output vector r_rel shall be resolved (1: world, 2: frame_a, 3: frame_resolve)";
      protected
        parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameB resolveInFrameB = PlanarMechanics.Utilities.Conversions.fromFrameAtoFrameB(resolveInFrame) "Conversion from frame A to B";
      public
        parameter Modelica.SIunits.Force F_nominal "Nominal force (if negative, torque is acting as load)";
        parameter Modelica.SIunits.Velocity v_nominal(min = Modelica.Constants.eps) "Nominal speed";
        parameter Modelica.SIunits.Torque tau_nominal "Nominal torque (if negative, torque is acting as load)";
        parameter Modelica.SIunits.AngularVelocity w_nominal(min = Modelica.Constants.eps) "Nominal speed";
        parameter Boolean animation = true "= true, if animation shall be enabled";
        parameter Real N_to_m(unit = "N/m") = planarWorld.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        parameter Real Nm_to_m(unit = "N.m/m") = planarWorld.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        input SI.Diameter diameter = planarWorld.defaultArrowDiameter "Diameter of force arrow" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of cylinder representing the fixed translation" annotation (
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        input Types.Color color = PlanarMechanics.Types.Defaults.ForceColor "Color of arrow" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", colorSelector = true, enable = animation));
        input Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
          HideResult = true,
          Dialog(tab = "Animation", group = "If animation = true", enable = animation));
        SI.Force force[3] = worldForce.force;
        Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque" annotation (
          Placement(transformation(extent = {{84, -16}, {116, 16}})));
        Interfaces.Frame_resolve frame_resolve(fx = 0, fy = 0, t = 0) if resolveInFrameB == Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve "Coordinate system in which vector is optionally resolved, if useExtraFrame is true" annotation (
          Placement(transformation(extent = {{-16, 16}, {16, -16}}, rotation = 90, origin = {0, -100})));
      public
        Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = resolveInFrame) annotation (
          Placement(transformation(extent = {{40, 30}, {20, 50}})));
        WorldForce worldForce(animation = animation, N_to_m = N_to_m, Nm_to_m = Nm_to_m, diameter = diameter, zPosition = zPosition, color = color, specularCoefficient = specularCoefficient, resolveInFrame = resolveInFrameB) annotation (
          Placement(transformation(extent = {{20, -10}, {40, 10}})));
        Modelica.Blocks.Math.MatrixGain normalizeSpeeds(K = [1 / v_nominal, 0, 0; 0, 1 / v_nominal, 0; 0, 0, 1 / w_nominal]) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-10, 40})));
        Modelica.Blocks.Math.MatrixGain scaleForces(K = [F_nominal, 0, 0; 0, F_nominal, 0; 0, 0, tau_nominal]) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-50, 0})));
        Utilities.Blocks.SquaretimesSign square(blockSize = 3) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-50, 40})));
      equation
        connect(worldForce.frame_b, frame_b) annotation (
          Line(points = {{40, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(absoluteVelocity.frame_a, frame_b) annotation (
          Line(points = {{40, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
        connect(worldForce.frame_resolve, frame_resolve) annotation (
          Line(points = {{30, -10}, {0, -10}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(absoluteVelocity.frame_resolve, frame_resolve) annotation (
          Line(points = {{30, 30}, {30, 20}, {0, 20}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
        connect(absoluteVelocity.v, normalizeSpeeds.u) annotation (
          Line(points = {{19, 40}, {2, 40}}, color = {0, 0, 127}));
        connect(square.u, normalizeSpeeds.y) annotation (
          Line(points = {{-38, 40}, {-21, 40}}, color = {0, 0, 127}));
        connect(square.y, scaleForces.u) annotation (
          Line(points = {{-61, 40}, {-80, 40}, {-80, 0}, {-62, 0}}, color = {0, 0, 127}));
        connect(scaleForces.y, worldForce.force) annotation (
          Line(points = {{-39, 0}, {18, 0}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{0, 0}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot, visible = resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve), Polygon(points = {{-100, 10}, {20, 10}, {20, 41}, {90, 0}, {20, -41}, {20, -10}, {-100, -10}, {-100, 10}}, lineColor = {0, 127, 0}, fillColor = {215, 215, 215},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-100, -100}, {-80, -98}, {-60, -92}, {-40, -82}, {-20, -68}, {0, -50}, {20, -28}, {40, -2}, {60, 28}, {80, 62}, {100, 100}}, color = {0, 0, 127}, smooth = Smooth.Bezier), Line(visible = resolveInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve, points = {{0, -10}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Text(extent = {{-150, 80}, {150, 40}}, textString = "%name", lineColor = {0, 0, 255}), Text(extent = {{72, -24}, {108, -49}}, lineColor = {128, 128, 128}, textString = "b")}),
          Documentation(info = "<html>
<p>
Model of a force quadratic dependent on the velocity of the flange.
The force can be resolved in a world frame, or a relative speed can be used
by selecting resolve_frame to use the extra frame_resolve.
</p>
<p>
This model is e.g. suitable to simulate aerodynamic drag forces.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end QuadraticSpeedDependentForce;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This package contains components that exert forces and torques between two frame connectors, e.g., between two parts.</p>
</html>"));
    end Sources;

    package Types "Type definitions used across the library"
      extends Modelica.Icons.TypesPackage;
      type Color = Modelica.Icons.TypeInteger[3](each min = 0, each max = 255) "RGB representation of color (will be improved with a color editor)" annotation (
        Dialog(colorSelector),
        choices(choice = {0, 0, 0} "{0,0,0}       \"black\"", choice = {155, 0, 0} "{155,0,0}     \"dark red\"", choice = {255, 0, 0} "{255,0,0 }    \"red\"", choice = {255, 65, 65} "{255,65,65}   \"light red\"", choice = {0, 128, 0} "{0,128,0}     \"dark green\"", choice = {0, 180, 0} "{0,180,0}     \"green\"", choice = {0, 230, 0} "{0,230,0}     \"light green\"", choice = {0, 0, 200} "{0,0,200}     \"dark blue\"", choice = {0, 0, 255} "{0,0,255}     \"blue\"", choice = {0, 128, 255} "{0,128,255}   \"light blue\"", choice = {255, 255, 0} "{255,255,0}   \"yellow\"", choice = {255, 0, 255} "{255,0,255}   \"pink\"", choice = {100, 100, 100} "{100,100,100} \"dark grey\"", choice = {155, 155, 155} "{155,155,155} \"grey\"", choice = {255, 255, 255} "{255,255,255} \"white\""),
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
Type <b>Color</b> is an Integer vector with 3 elements,
{r, g, b}, and specifies the color of a shape.
{r,g,b} are the \"red\", \"green\" and \"blue\" color parts.
Note, r g, b are given in the range 0 .. 255.
</p>
</html>"));
      type SpecularCoefficient = Modelica.Icons.TypeReal(min = 0) "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
        choices(choice = 0 "\"0.0 (dull)\"", choice = 0.7 "\"0.7 (medium)\"", choice = 1 "\"1.0 (glossy)\""),
        Documentation(info = "<html>
<p>
Type <b>SpecularCoefficient</b> defines the reflection of
ambient light on shape surfaces. If value = 0, the light
is completely absorbed. Often, 0.7 is a reasonable value.
It might be that from some viewing directions, a body is no
longer visible, if the SpecularCoefficient value is too high.
In the following image, the different values of SpecularCoefficient
are shown for a cylinder:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/SpecularCoefficient.png\" alt=\"SpecularCoefficient animation\">
</p>
</html>"));

      package Defaults "Default settings of the library via constants"
        extends Modelica.Icons.Package;
        // Color defaults
        constant Types.Color BodyColor = {0, 103, 200} "Default color for body shapes that have mass (light blue)";
        constant Types.Color RodColor = {115, 115, 115} "Default color for massless rod shapes (grey)";
        constant Types.Color JointColor = {200, 0, 0} "Default color for elementary joints (red)";
        constant Types.Color ForceColor = {0, 100, 0} "Default color for force arrow (dark green)";
        constant Types.Color TorqueColor = {0, 100, 0} "Default color for torque arrow (dark green)";
        constant Types.Color SpringColor = {0, 0, 200} "Default color for a spring (blue)";
        constant Types.Color SensorColor = {200, 200, 0} "Default color for sensors (yellow)";
        constant Types.Color FrameColor = {85, 85, 200} "Default color for frame axes and labels (blue)";
        constant Types.Color ArrowColor = {0, 0, 200} "Default color for arrows and double arrows (blue)";
        // Arrow and frame defaults
        constant Real FrameHeadLengthFraction = 5.0 "Frame arrow head length / arrow diameter";
        constant Real FrameHeadWidthFraction = 3.0 "Frame arrow head width / arrow diameter";
        constant Real FrameLabelHeightFraction = 3.0 "Height of frame label / arrow diameter";
        constant Real ArrowHeadLengthFraction = 4.0 "Arrow head length / arrow diameter";
        constant Real ArrowHeadWidthFraction = 3.0 "Arrow head width / arrow diameter";
        // Other defaults
        constant SI.Diameter BodyCylinderDiameterFraction = 3 "Default for body cylinder diameter as a fraction of body sphere diameter";
        constant Real JointRodDiameterFraction = 2 "Default for rod diameter as a fraction of joint sphere diameter attached to rod";
        /*
                                  constant Real N_to_m(unit="N/m") = 1000
                                    "Default force arrow scaling (length = force/N_to_m_default)";
                                  constant Real Nm_to_m(unit="N.m/m") = 1000
                                    "Default torque arrow scaling (length = torque/Nm_to_m_default)";
                                */
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This package contains constants used as default setting
in the library.
</p>
</html>"));
      end Defaults;
      annotation (
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>In this package <b>types</b> and <b>constants</b> are defined that are used in the PlanarMechanics library. The types have additional annotation choices definitions that define the menus to be built up in the graphical user interface when the type is used as parameter in a declaration.</p>
</html>"));
    end Types;

    package UsersGuide "User's Guide"
      extends Modelica.Icons.Information;

      package Tutorial "Tutorial"
        extends Modelica.Icons.Information;

        class OverView "Overview of PlanarMechanics library"
          extends Modelica.Icons.Information;
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Library <b>PlanarMechanics</b> is a <b>free</b> Modelica package providing 2-dimensional mechanical components to model <b>mechanical systems</b>, such as robots, mechanisms, vehicles, where MultiBody library is too complex to use. The main features of the library are:</p>
<ul>
<li>Much more <b>compact</b> than MultiBody library, which means fewer parameters to be set, shorter time to build up a model-based system, in the meanwhile containing important information as much as possible.</li>
<li>A <b>PlanarWorld</b> model could be used to set up almost all global parameters, such as visualization of global coordinate system, animation parameters of joints, parts, sources etc., and gravity definition as well as its animation. Note that, in most cases the animation parameters set in PlanarWorld model can be also overwritten in individual model.</li>
<li><b>Built-in animation properties</b> of all components, such as joints, forces, bodies, sensors. It enables an easy visual check of the constructed model. What&#39;s more, in all models animation can be disabled respectively, while in the PlanarWorld model animations of all models are able to be switched off.</li>
</ul>
</html>"));
        end OverView;

        class FirstExample "A first example"
          extends Modelica.Icons.Information;
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Here seveal steps will be listed to demonstrate how to build up, simulate and animate a <b>simple pendulum</b>, which consists of a fixed point, a planar world model, a revolute joint, a fixed translation and a body.</p>
<ul>
<li><b>Building up Modelica composition diagram</b>.</li>
</ul>
<p>The diagram is showed as following.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/UsersGuide/Tutorial/FirstExample/FirstExample1.png\" alt=\"Modelica diagram\"></p>
<p>where component <b>Fixed</b>, <b>Body</b>, <b>FixedTranslation </b>can be found in <b>Parts</b> package, component <b>Revolute</b> in <b>Joints</b>, <b>PlanarWorld</b> directly under PlanarMechanics.</p>
<p>Every model having components from PlanarMechanics library must include an instance of component PlanarWorld on the highest level. The reason is that PlanarWorld component defines the default gravity for the model, includes default settings of animation parameters of almost every components.</p>
<ul>
<li><h4>Setting up initial values and parameters.</h4></li>
</ul>
<p>In this step, we only need to double click the relevant component and write initial values and parameters in the blanks. Default gravity force in is {0,-9.81}. In this example, phi.start in revolute component is set to be 50&deg;, m and I in Body are respectively 1kg and 0.1kgm<sup>2</sup>; all other settings remain default.</p>
<ul>
<li><h4>Translating and simulating the simple pendulum model.</h4></li>
</ul>
<p>With the above settings, animation is as following:</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/UsersGuide/Tutorial/FirstExample/FirstExample2.png\" alt=\"First example animation\"></p>
</html>"));
        end FirstExample;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This tutorial provides an introduction into the PlanarMechanics library.</p>
<ol>
<li><a href=\"modelica://PlanarMechanics.UsersGuide.Tutorial.OverView\">Overview of PlanarMechanics library</a> shows the most important aspects of the library.</li>
<li><a href=\"Modelica://PlanarMechanics.UsersGuide.Tutorial.FirstExample\">A first example</a> demonstrates how to build up, simulate and animate a model by using a simple pendulum as example.</li>
</ol>
</html>"));
      end Tutorial;

      class ReleaseNotes "Release notes"
        extends Modelica.Icons.ReleaseNotes;
        annotation (
          Documentation(info = "<html>
<h4>Version 1.4.1, 2019-03-29</h4>
<p>
This version requires the <b>Modelica&nbsp;3.2.3</b> Library.
It is backwards compatible to previous library versions.
</p>

<ul>
<li> The license has been changed to BSD 3-clause, visit:
     <a href=\"https://modelica.org/licenses/modelica-3-clause-bsd\">https://modelica.org/licenses/modelica-3-clause-bsd</a>.</li>
</ul>

<p>Improvements in this version:</p>
<ul>
<li>PNG files moved to folders which exactly represent the package structure
    (see also <a href=\"modelica://Modelica.UsersGuide.Conventions.Documentation.Format.Figures\">Modelica.UsersGuide.Conventions.Documentation.Format.Figures</a>).</li>
<li>Documentation of some classes.</li>
<li>Improved icons of some classes.</li>
</ul>

<!--
<p>New features</p>
<ul>
<li>...</li>
</ul>
-->

<h4>Version 1.4.0, 2017-01-12</h4>
<p>
Various improvements and bug fixes of components and documentation.
</p>


<h4>Version 1.3.0, 2014-06-13</h4>
<p>
Various improvements and bug fixes of components and documentation.
</p>


<h4>Version 1.2.0, 2014-06-02</h4>
<p>
First official release of the PlanarMechanics library.
</p>
</html>"));
      end ReleaseNotes;

      class Contact "Contact"
        extends Modelica.Icons.Contact;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<dl>
<dt><b>Library Officer:</b></dt>
<dd><a href=\"http://www.robotic.dlr.de/Dirk.Zimmer/\">Dirk Zimmer</a></dd>
<dd>Deutsches Zentrum f&uuml;r Luft und Raumfahrt e.V. (DLR)</dd>
<dd>Robotic and Mechatronic Centre</dd>
<dd><a href=\"https://www.dlr.de/sr/en\">Institute of Sytem Dynamics and Control</<></dd>
<dd>M&uuml;nchner Strasse 20 D-82234 Wessling</dd>
<dd>Germany</dd>
<dd>email: <a href=\"mailto:Dirk.Zimmer@dlr.de\">Dirk.Zimmer@dlr.de</a></dd>
</dl>

<h4>Acknowledgements:</h4>
<ul>
<li>The origin of this library is a planar mechanical library conceived for
    teaching purposes at the TUM. This library was designed by Dirk Zimmer.</li>
<li>Further component development and testing by practical applications was
    performed by Franciscus van der Linden.</li>
<li>A refinement of the library and the modeling of sensors and sources was
    performed by Zheng Qu.</li>
<li>Components for gear wheels have been added by Franciscus van der Linden.</li>
</ul>
</html>"));
      end Contact;
      annotation (
        DocumentationClass = true,
        Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>Library <b>PlanarMechanics</b> is a <b>free</b> Modelica package providing 2-dimensional mechanical components to model mechanical systems, such as robots, mechanisms, vehicles, where MultiBody library is unnecessarily complex. This package contains the User&#39;s Guide for the PlanarMechanics library.</p>
<ol>
<li><a href=\"modelica://PlanarMechanics.UsersGuide.Tutorial\">Tutorial</a> gives an introduction about how to use the library.</li>
<li><a href=\"modelica://PlanarMechanics.UsersGuide.Contact\">Contact</a> provides information about the author of the library as well as acknowledgments.</li>
</ol>
</html>"));
    end UsersGuide;

    package Utilities "Utility elements used across the library"
      extends Modelica.Icons.UtilitiesPackage;

      package Blocks "Blocks to process signals"
        block LimiterS "Limit the range of output y using S-form transition"
          extends Modelica.Blocks.Interfaces.SISO;
          parameter Real x_min = 0;
          parameter Real x_max = 1;
          parameter Real y_min = 0;
          parameter Real y_max = 1;
        equation
          y = Functions.limitBySform(x_min, x_max, y_min, y_max, u);
          annotation (
            Icon(graphics={  Line(points = {{-70, -78}, {-70, 78}}, color = {192, 192, 192}), Polygon(points = {{-70, 100}, {-78, 78}, {-62, 78}, {-70, 100}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                    fillPattern =                                                                                                                                                                                                      FillPattern.Solid), Polygon(points = {{100, -70}, {78, -62}, {78, -78}, {100, -70}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-80, -70}, {78, -70}}, color = {192, 192, 192}), Text(extent = {{2, 6}, {74, -42}}, lineColor = {192, 192, 192}, textString = "S"), Line(points = {{-70, -70}, {-62, -70}, {-50, -66}, {-40, -58}, {-30, -40}, {-18, -12}, {-2, 22}, {10, 40}, {22, 52}, {32, 60}, {42, 64}, {56, 68}, {70, 68}})}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end LimiterS;

        block SquaretimesSign "Output the squared input and retain the same sign of it"
          parameter Integer blockSize;
          extends Modelica.Blocks.Interfaces.MIMO(final nin = blockSize, final nout = blockSize);
        equation
          for i in 1:size(u, 1) loop
            y[i] = smooth(1, u[i] ^ 2 * sign(u[i]));
          end for;
          annotation (
            Documentation(info = "<html>
<p>
This block outputs squared input real signal whereby the sign
of the output is the same as input.
The size of input&nbsp;u and output&nbsp;y are defined by
a parameter blockSize, thus
</p>

<blockquote><pre>
<strong>for</strong> i <strong>in</strong> 1:blockSize <strong>loop</strong>
  y[i] = sign(u[i]) * u[i]^2;
<strong>end for</strong>;
</pre></blockquote>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-100, -98}, {100, -68}}, lineColor = {160, 160, 164}, textString = "sign(u) * u^2"), Polygon(points = {{0, 90}, {-8, 68}, {8, 68}, {0, 90}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{0, 68}, {0, -80}}, color = {192, 192, 192}), Line(points = {{-90, 0}, {68, 0}}, color = {192, 192, 192}), Polygon(points = {{90, 0}, {68, 8}, {68, -8}, {90, 0}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{-80, -50}, {80, 50}}, color = {95, 95, 95}), Line(points = {{0, 0}, {-2, 0}, {-16, -2}, {-34, -14}, {-54, -42}, {-64, -72}}, color = {0, 0, 127}, smooth = Smooth.Bezier), Line(points = {{0, 0}, {2, 0}, {16, 2}, {34, 14}, {56, 46}, {70, 90}}, color = {0, 0, 127}, smooth = Smooth.Bezier)}),
            Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-90, -60}, {90, 60}}, lineColor = {160, 160, 164}, textString = "sign(u) * u^2")}));
        end SquaretimesSign;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This package contains input/output blocks to build up block diagrams.</p>
<placeholder to reach necessary length of documentation (100 strings)>
</html>"));
      end Blocks;

      package Functions "Library for the atan3 function which is double differentiable"
        extends Modelica.Icons.Package;

        function atan3b "Four quadrant inverse tangent (select solution that is closest to given angle y0)"
          import Modelica.Math;
          extends Modelica.Math.Icons.AxisCenter;
          input Real u1;
          input Real u2;
          input Modelica.SIunits.Angle y0 = 0 "y shall be in the range: -pi < y-y0 < pi";
          output Modelica.SIunits.Angle y;
        protected
          Real pi = Modelica.Constants.pi;
          Real w;
        algorithm
          w := Math.atan2(u1, u2);
          y := w + 2 * pi * div(abs(w - y0) + pi, 2 * pi) * (if y0 > w then +1 else -1);
          annotation (
            derivative(noDerivative = y0) = atan3b_der,
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics={  Line(points = {{-82, -36}, {-46, -32}, {-30, -28}, {-18, -22}, {-8, -14}, {0, 0}, {8, 14}, {18, 22}, {30, 28}, {46, 32}, {82, 36}}), Line(points = {{82, -40}, {46, -44}, {30, -48}, {18, -54}, {8, -62}, {0, -76}}), Line(points = {{-82, 40}, {-46, 44}, {-30, 48}, {-18, 54}, {-8, 62}, {0, 76}}), Line(points = {{-90, 0}, {68, 0}}, color = {192, 192, 192}), Polygon(points = {{90, 0}, {68, 8}, {68, -8}, {90, 0}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-90, -46}, {-18, -94}}, lineColor = {192, 192, 192}, textString = "atan3")}),
            Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics = {Line(points = {{-100, -86}, {84, -86}}, color = {95, 95, 95}), Polygon(points = {{98, -86}, {82, -80}, {82, -92}, {98, -86}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Line(points = {{0, -80}, {8.93, -67.2}, {17.1, -59.3}, {27.3, -53.6}, {42.1, -49.4}, {69.9, -45.8}, {80, -45.1}}, color = {0, 0, 255}, thickness = 0.5), Line(points = {{-80, -34.9}, {-46.1, -31.4}, {-29.4, -27.1}, {-18.3, -21.5}, {-10.3, -14.5}, {-2.03, -3.17}, {7.97, 11.6}, {15.5, 19.4}, {24.3, 25}, {39, 30}, {62.1, 33.5}, {80, 34.9}}, color = {0, 0, 255}, thickness = 0.5), Line(points = {{-80, 45.1}, {-45.9, 48.7}, {-29.1, 52.9}, {-18.1, 58.6}, {-10.2, 65.8}, {-1.82, 77.2}, {0, 80}}, color = {0, 0, 255}, thickness = 0.5), Text(extent = {{-56, 82}, {-12, 72}}, textString = "(2*N-1)*pi", lineColor = {0, 0, 255}), Text(extent = {{-52, -72}, {-10, -88}}, textString = "(2*N-3)*pi", lineColor = {0, 0, 255}), Line(points = {{0, 40}, {-8, 40}}, color = {192, 192, 192}), Line(points = {{0, -40}, {-8, -40}}, color = {192, 192, 192}), Text(extent = {{38, -68}, {78, -84}}, lineColor = {95, 95, 95}, textString = "u1, u2, y0"), Line(points = {{-84, 40}, {88, 40}}, color = {175, 175, 175}), Line(points = {{-84, -40}, {88, -40}}, color = {175, 175, 175})}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This function returns y = <b>atan3</b>(u1,u2,y0) such that
<b>tan</b>(y) = u1/u2 and
y is in the range: -pi &lt; y-y0 &lt; pi.<br>
u2 may be zero, provided u1 is not zero. The difference to
<a href=\"modelica://Modelica.Math.atan3\">Modelica.Math.atan3</a>(&hellip;)
is that the derivatives of atan3 are explicitely defined here.
</p>


<h4>See also</h4>
<p>
<a href=\"modelica://PlanarMechanics.Utilities.Functions.atan3b_der\">atan3b_der</a>
for 1st derivative and
<a href=\"modelica://PlanarMechanics.Utilities.Functions.atan3b_dder\">atan3b_dder</a>
for 2nd derivative of this function.
</p>
</html>"));
        end atan3b;

        function atan3b_der "First deviation of atan3"
          extends Modelica.Icons.Function;
          import Modelica.Math;
          input Real u1;
          input Real u2;
          input Modelica.SIunits.Angle y = 0 "y shall be in the range: -pi < y-y0 < pi";
          input Real u1_der;
          input Real u2_der;
          output Modelica.SIunits.AngularVelocity y_der;
        algorithm
          y_der := u2 / (u1 * u1 + u2 * u2) * u1_der - u1 / (u1 * u1 + u2 * u2) * u2_der;
          annotation (
            derivative = atan3b_dder,
            Documentation(info = "<html>
<p>
The first derivative of the function
<a href=\"modelica://PlanarMechanics.Utilities.Functions.atan3b\">atan3b</a>.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end atan3b_der;

        function atan3b_dder "Second deviation of atan3"
          extends Modelica.Icons.Function;
          import Modelica.Math;
          input Real u1;
          input Real u2;
          input Modelica.SIunits.Angle y_d = 0 "y shall be in the range: -pi < y-y0 < pi";
          input Real u1_der;
          input Real u2_der;
          input Real u1_dder;
          input Real u2_dder;
          output Modelica.SIunits.AngularAcceleration y_dder;
        algorithm
          y_dder := 2 * u1 * u2 / ((u1 * u1 + u2 * u2) * (u1 * u1 + u2 * u2)) * u2_der - 2 * u1 * u2 / ((u1 * u1 + u2 * u2) * (u1 * u1 + u2 * u2)) * u1_der;
          annotation (
            Documentation(info = "<html>
<p>
The first derivative of function
<a href=\"modelica://PlanarMechanics.Utilities.Functions.atan3b_der\">atan3b_der</a>,
i.e. the second derivative of the function
<a href=\"modelica://PlanarMechanics.Utilities.Functions.atan3b\">atan3b</a>.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end atan3b_dder;

        function limitBySform "Returns a S-shaped transition"
          extends Modelica.Icons.Function;
          input Real x_min "Abscissa for y_min";
          input Real x_max "Abscissa for y_max";
          input Real y_min "First value of y";
          input Real y_max "Second value of y";
          input Real x "Current abscissa value";
          output Real y "Current ordinate";
        protected
          Real x2;
        algorithm
          x2 := x - x_max / 2 - x_min / 2;
          x2 := x2 * 2 / (x_max - x_min);
          if x2 > 1 then
            y := 1;
          elseif x2 < (-1) then
            y := -1;
          else
            y := (-0.5 * x2 ^ 3) + 1.5 * x2;
          end if;
          y := y * (y_max - y_min) / 2;
          y := y + y_max / 2 + y_min / 2;
          annotation (
            smoothOrder = 1,
            Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
y = Functions.<strong>limitBySform</strong>(x_min, x_max, y_min, y_max, x);
</pre></blockquote>

<h4>Description</h4>
<p>
A smooth transition between points (x_min,&nbsp;y_min) and (x_max,&nbsp;y_max).
The transition is done in such a way that the 1st function&#039;s derivative
is continuous for all&nbsp;<em>x</em>.
The higher derivatives are, in contrast, discontinuous at input points.
</p>

<p>
The figure below shows the function&nbsp;<em>y</em> and its 1st derivative&nbsp;<em>dy/dx</em>
for the following input:
x_max&nbsp;=&nbsp;-0.4,
x_sat&nbsp;=&nbsp;0.6,
y_max&nbsp;=&nbsp;1.4,
y_sat&nbsp;=&nbsp;1.2.
</p>

<blockquote>
<img src=\"modelica://PlanarMechanics/Resources/Images/Utilities/Functions/limitBySform.png\">
</blockquote>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end limitBySform;

        function limitByStriple "Returns a point-symmetric Triple S-Function"
          extends Modelica.Icons.Function;
          input Real x_max "Abscissa for y_max";
          input Real x_sat "Abscissa for y_sat";
          input Real y_max "Peak ordinate";
          input Real y_sat "Saturated ordinate";
          input Real x "Current abscissa value";
          output Real y "Current ordinate";
        algorithm
          if x > x_max then
            y := Functions.limitBySform(x_max, x_sat, y_max, y_sat, x);
          elseif x < (-x_max) then
            y := Functions.limitBySform(-x_max, -x_sat, -y_max, -y_sat, x);
          else
            y := Functions.limitBySform(-x_max, x_max, -y_max, y_max, x);
          end if;
          annotation (
            smoothOrder = 1,
            Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
y = Functions.<strong>limitByStriple</strong>(x_max, x_sat, y_max, y_sat, x);
</pre></blockquote>

<h4>Description</h4>
<p>
A point symmetric interpolation between points (0,&nbsp;0), (x_max,&nbsp;y_max)
and (x_sat,&nbsp;y_sat), provided x_max&nbsp;&lt;&nbsp;x_sat. The approximation
is done in such a way that the 1st function&#039;s derivative is zero at points
points (x_max,&nbsp;y_max) and (x_sat,&nbsp;y_sat).
Thus, the 1st function&#039;s derivative is continuous for all&nbsp;<em>x</em>.
The higher derivatives are, in contrast, discontinuous at these points.
</p>

<p>
The figure below shows the function&nbsp;<em>y</em> and its 1st derivative&nbsp;<em>dy/dx</em>
for the following input:
x_max&nbsp;=&nbsp;0.2,
x_sat&nbsp;=&nbsp;0.5,
y_max&nbsp;=&nbsp;1.4,
y_sat&nbsp;=&nbsp;1.2.
</p>

<blockquote>
<img src=\"modelica://PlanarMechanics/Resources/Images/Utilities/Functions/limitByStriple.png\">
</blockquote>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end limitByStriple;

        function round "Round to nearest Integer"
          extends Modelica.Icons.Function;
          input Real v "Real number";
          output Integer i "Real number v rounded to nearest integer";
        algorithm
          i := if v >= 0 then integer(floor(v + 0.5)) else integer(ceil(v - 0.5));
          annotation (
            Documentation(info = "<html>
<p>
Returns the input argument rounded to the nearest Integer. Examples:
</p>
 
<pre>
   round(2.4)  ->  2
   round(2.6)  ->  3
   round(-1.3) -> -1
   round(-1.6) -> -2
</pre>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end round;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end Functions;

      package Conversions "Conversion functions"
        function fromFrameAtoFrameB "Convert from frameA to frameB"
          extends Modelica.SIunits.Icons.Conversion;
          input Modelica.Mechanics.MultiBody.Types.ResolveInFrameA FrameA;
          output Modelica.Mechanics.MultiBody.Types.ResolveInFrameB FrameB;
          // Integer temp;
        algorithm
          if FrameA == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world then
            FrameB := Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world;
          elseif FrameA == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a then
            FrameB := Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b;
          else
            FrameB := Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve;
          end if;
          annotation (
            Inline = true,
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-20, 100}, {-102, 56}}, lineColor = {0, 0, 0}, textString = "A"), Text(extent = {{100, -56}, {0, -102}}, lineColor = {0, 0, 0}, textString = "B")}));
        end fromFrameAtoFrameB;
        annotation (
          Documentation(info = "<html>
<p>This package provides conversion functions for various purposes.</p>
</html>"));
      end Conversions;

      package Icons "Icons used for library components"
        extends Modelica.Icons.IconsPackage;

        model PlanarGearContactExternalL1 "Icon for an external planar gear contact"
          annotation (
            Icon(graphics={  Polygon(points = {{38.6901, 40.9645}, {39.3284, 42.8041}, {41.9096, 46.8087}, {44.5357, 46.0609}, {44.6201, 41.2972}, {44.1935, 39.3973}, {47.0172, 38.2626}, {48.024, 39.9292}, {51.3814, 43.3096}, {53.7946, 42.0322}, {52.8868, 37.3551}, {52.0745, 35.5854}, {54.6006, 33.8884}, {55.9319, 35.3093}, {59.9188, 37.9178}, {62.0136, 36.1665}, {60.1532, 31.7803}, {58.9908, 30.2182}, {61.1088, 28.0331}, {62.7064, 29.1461}, {67.1485, 30.8687}, {68.8335, 28.7202}, {66.1018, 24.8167}, {64.64, 23.5304}, {66.2574, 20.9526}, {68.0515, 21.7092}, {72.7547, 22.4705}, {73.9562, 20.0187}, {70.4725, 16.7684}, {68.7753, 15.8141}, {69.8213, 12.9564}, {71.7336, 13.3234}, {76.4923, 13.0903}, {77.1577, 10.4422}, {73.0745, 7.9872}, {71.2159, 7.4067}, {71.6449, 4.394}, {73.5917, 4.3554}, {78.198, 3.1379}, {78.2983, 0.4094}, {73.7938, -1.143}, {71.8552, -1.3244}, {71.6485, -4.3605}, {73.5446, -4.803}, {77.7971, -6.9516}, {77.3279, -9.6414}, {72.5992, -10.2233}, {70.6652, -9.9976}, {69.8317, -12.9244}, {71.5945, -13.7515}, {75.3073, -16.7372}, {74.2892, -19.2707}, {69.5428, -18.8567}, {67.6979, -18.2339}, {66.2742, -20.9235}, {67.8265, -22.099}, {70.8374, -25.7914}, {69.3147, -28.0578}, {64.7581, -26.6661}, {63.0831, -25.6733}, {61.1313, -28.0081}, {62.4052, -29.4806}, {64.5827, -33.7184}, {62.6221, -35.6187}, {58.4544, -33.31}, {57.0223, -31.9906}, {54.6278, -33.8686}, {55.5677, -35.5738}, {56.8165, -40.1717}, {54.5037, -41.6228}, {50.9071, -38.4981}, {49.7806, -36.9098}, {47.0479, -38.2489}, {47.6128, -40.1123}, {47.8783, -44.8693}, {45.3143, -45.8079}, {42.446, -42.0036}, {41.6744, -40.2159}, {38.723, -40.9575}, {38.8881, -42.8976}, {38.1588, -47.6059}, {35.4557, -47.9909}, {33.441, -43.6734}, {33.0579, -41.7643}, {30.0168, -41.8761}, {29.775, -43.8082}, {28.0827, -48.2619}, {25.3586, -48.0765}, {24.2856, -43.4344}, {24.3078, -41.4874}, {21.3099, -40.9645}, {20.6716, -42.8041}, {18.0904, -46.8087}, {15.4643, -46.0609}, {15.3799, -41.2972}, {15.8065, -39.3973}, {12.9828, -38.2626}, {11.976, -39.9292}, {8.6186, -43.3096}, {6.2054, -42.0322}, {7.1132, -37.3551}, {7.9255, -35.5854}, {5.3994, -33.8884}, {4.0681, -35.3093}, {0.0812, -37.9178}, {-2.0136, -36.1665}, {-0.1532, -31.7803}, {1.0092, -30.2182}, {-1.1088, -28.0331}, {-2.7064, -29.1461}, {-7.1485, -30.8687}, {-8.8335, -28.7202}, {-6.1018, -24.8167}, {-4.64, -23.5304}, {-6.2574, -20.9526}, {-8.0515, -21.7092}, {-12.7547, -22.4705}, {-13.9562, -20.0187}, {-10.4725, -16.7684}, {-8.7753, -15.8141}, {-9.8213, -12.9564}, {-11.7336, -13.3234}, {-16.4923, -13.0903}, {-17.1577, -10.4422}, {-13.0745, -7.9872}, {-11.2159, -7.4067}, {-11.6449, -4.394}, {-13.5917, -4.3554}, {-18.198, -3.1379}, {-18.2983, -0.40937}, {-13.7938, 1.143}, {-11.8552, 1.3244}, {-11.6485, 4.3605}, {-13.5446, 4.803}, {-17.7971, 6.9516}, {-17.3279, 9.6414}, {-12.5992, 10.2233}, {-10.6652, 9.9976}, {-9.8317, 12.9244}, {-11.5945, 13.7515}, {-15.3073, 16.7372}, {-14.2892, 19.2707}, {-9.5428, 18.8567}, {-7.6979, 18.2339}, {-6.2742, 20.9235}, {-7.8265, 22.099}, {-10.8374, 25.7914}, {-9.3147, 28.0578}, {-4.7581, 26.6661}, {-3.0831, 25.6733}, {-1.1313, 28.0081}, {-2.4052, 29.4806}, {-4.5827, 33.7184}, {-2.6221, 35.6187}, {1.5456, 33.31}, {2.9777, 31.9906}, {5.3722, 33.8686}, {4.4323, 35.5738}, {3.1835, 40.1717}, {5.4963, 41.6228}, {9.0929, 38.4981}, {10.2194, 36.9098}, {12.9521, 38.2489}, {12.3872, 40.1123}, {12.1217, 44.8693}, {14.6857, 45.8079}, {17.554, 42.0036}, {18.3256, 40.2159}, {21.277, 40.9575}, {21.1119, 42.8976}, {21.8412, 47.6059}, {24.5443, 47.9909}, {26.559, 43.6734}, {26.9421, 41.7643}, {29.9832, 41.8761}, {30.2251, 43.8082}, {31.9173, 48.2619}, {34.6414, 48.0765}, {35.7144, 43.4344}, {35.6922, 41.4874}}, fillColor = {255, 0, 0},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, pattern = LinePattern.None), Polygon(points = {{-35.977, 22.4878}, {-34.8831, 24.6535}, {-32.9407, 26.8978}, {-31.8083, 27.739}, {-29.7455, 26.7376}, {-29.706, 25.3275}, {-30.2679, 22.413}, {-31.2931, 20.2139}, {-28.7696, 18.3875}, {-27.001, 20.0486}, {-24.4081, 21.4932}, {-23.0562, 21.8963}, {-21.4604, 20.2498}, {-21.9055, 18.9112}, {-23.4304, 16.3647}, {-25.1458, 14.6489}, {-23.3992, 12.0695}, {-21.1691, 13.0255}, {-18.2386, 13.4962}, {-16.8303, 13.4126}, {-15.8938, 11.3196}, {-16.77, 10.214}, {-19.0738, 8.3426}, {-21.2727, 7.3169}, {-20.5135, 4.2957}, {-18.091, 4.4313}, {-15.1762, 3.8713}, {-13.8815, 3.3112}, {-13.7173, 1.0241}, {-14.9188, 0.2848}, {-17.7237, -0.68579}, {-20.1408, -0.89754}, {-20.4607, -3.9962}, {-18.1379, -4.6973}, {-15.5904, -6.2205}, {-14.5653, -7.1897}, {-15.1933, -9.395}, {-16.5752, -9.6788}, {-19.5429, -9.6315}, {-21.8866, -9.0037}, {-23.2471, -11.8061}, {-21.3041, -13.2594}, {-19.4313, -15.562}, {-18.7995, -16.8233}, {-20.1439, -18.6809}, {-21.5394, -18.4749}, {-24.312, -17.4154}, {-26.2997, -16.024}, {-28.5365, -18.192}, {-27.2078, -20.2222}, {-26.2355, -23.0265}, {-26.0732, -24.4278}, {-27.9718, -25.7135}, {-29.2128, -25.0427}, {-31.4558, -23.0988}, {-32.8477, -21.1115}, {-35.6911, -22.3837}, {-35.1369, -24.7459}, {-35.1823, -27.7137}, {-35.5091, -29.086}, {-37.733, -29.6448}, {-38.6697, -28.59}, {-40.1125, -25.9962}, {-40.7408, -23.6526}, {-43.8479, -23.8756}, {-44.135, -26.2849}, {-45.1927, -29.0581}, {-45.9692, -30.2359}, {-48.25, -30.0005}, {-48.7695, -28.6889}, {-49.2382, -25.758}, {-49.027, -23.3409}, {-52.023, -22.4878}, {-53.1169, -24.6535}, {-55.0593, -26.8978}, {-56.1917, -27.739}, {-58.2545, -26.7376}, {-58.294, -25.3275}, {-57.7321, -22.413}, {-56.7069, -20.2139}, {-59.2304, -18.3875}, {-60.999, -20.0486}, {-63.5919, -21.4932}, {-64.9438, -21.8963}, {-66.5396, -20.2498}, {-66.0945, -18.9112}, {-64.5696, -16.3647}, {-62.8542, -14.6489}, {-64.6008, -12.0695}, {-66.8309, -13.0255}, {-69.7614, -13.4962}, {-71.1697, -13.4126}, {-72.1062, -11.3196}, {-71.23, -10.214}, {-68.9262, -8.3426}, {-66.7273, -7.3169}, {-67.4865, -4.2957}, {-69.909, -4.4313}, {-72.8238, -3.8713}, {-74.1185, -3.3112}, {-74.2827, -1.0241}, {-73.0812, -0.28477}, {-70.2763, 0.6858}, {-67.8592, 0.8975}, {-67.5393, 3.9962}, {-69.8621, 4.6973}, {-72.4096, 6.2205}, {-73.4347, 7.1897}, {-72.8067, 9.395}, {-71.4248, 9.6788}, {-68.4571, 9.6315}, {-66.1134, 9.0037}, {-64.7529, 11.8061}, {-66.6959, 13.2594}, {-68.5687, 15.562}, {-69.2005, 16.8233}, {-67.8561, 18.6809}, {-66.4606, 18.4749}, {-63.688, 17.4154}, {-61.7003, 16.024}, {-59.4635, 18.192}, {-60.7922, 20.2222}, {-61.7645, 23.0265}, {-61.9268, 24.4278}, {-60.0282, 25.7135}, {-58.7872, 25.0427}, {-56.5442, 23.0988}, {-55.1523, 21.1115}, {-52.3089, 22.3837}, {-52.8631, 24.7459}, {-52.8177, 27.7137}, {-52.4909, 29.086}, {-50.267, 29.6448}, {-49.3303, 28.59}, {-47.8875, 25.9962}, {-47.2592, 23.6526}, {-44.1521, 23.8756}, {-43.865, 26.2849}, {-42.8073, 29.0581}, {-42.0308, 30.2359}, {-39.75, 30.0005}, {-39.2305, 28.6889}, {-38.7618, 25.758}, {-38.973, 23.3409}}, fillColor = {255, 160, 160},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                    lineThickness =                                                                                                                                                                                                        0.5, pattern = LinePattern.None), Line(points = {{-100, 0}, {-44, 0}}, thickness = 1), Line(points = {{30, 0}, {100, 0}}, thickness = 1), Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255})}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This partial class is intended to design a <em>default icon for an external planar gear contact</em>.
<p>
</html>"));
        end PlanarGearContactExternalL1;

        model PlanarGearContactInternalL1 "Icon for an internal planar gear contact"
          annotation (
            Icon(graphics={  Line(points = {{38, 0}, {98, 0}}, thickness = 1), Polygon(points = {{8.6901, 40.9645}, {9.3284, 42.8041}, {11.9096, 46.8087}, {14.5357, 46.0609}, {14.6201, 41.2972}, {14.1935, 39.3973}, {17.0172, 38.2626}, {18.024, 39.9292}, {21.3814, 43.3096}, {23.7946, 42.0322}, {22.8868, 37.3551}, {22.0745, 35.5854}, {24.6006, 33.8884}, {25.9319, 35.3093}, {29.9188, 37.9178}, {32.0136, 36.1665}, {30.1532, 31.7803}, {28.9908, 30.2182}, {31.1088, 28.0331}, {32.7064, 29.1461}, {37.1485, 30.8687}, {38.8335, 28.7202}, {36.1018, 24.8167}, {34.64, 23.5304}, {36.2574, 20.9526}, {38.0515, 21.7092}, {42.7547, 22.4705}, {43.9562, 20.0187}, {40.4725, 16.7684}, {38.7753, 15.8141}, {39.8213, 12.9564}, {41.7336, 13.3234}, {46.4923, 13.0903}, {47.1577, 10.4422}, {43.0745, 7.9872}, {41.2159, 7.4067}, {41.6449, 4.394}, {43.5917, 4.3554}, {48.198, 3.1379}, {48.2983, 0.4094}, {43.7938, -1.143}, {41.8552, -1.3244}, {41.6485, -4.3605}, {43.5446, -4.803}, {47.7971, -6.9516}, {47.3279, -9.6414}, {42.5992, -10.2233}, {40.6652, -9.9976}, {39.8317, -12.9244}, {41.5945, -13.7515}, {45.3073, -16.7372}, {44.2892, -19.2707}, {39.5428, -18.8567}, {37.6979, -18.2339}, {36.2742, -20.9235}, {37.8265, -22.099}, {40.8374, -25.7914}, {39.3147, -28.0578}, {34.7581, -26.6661}, {33.0831, -25.6733}, {31.1313, -28.0081}, {32.4052, -29.4806}, {34.5827, -33.7184}, {32.6221, -35.6187}, {28.4544, -33.31}, {27.0223, -31.9906}, {24.6278, -33.8686}, {25.5677, -35.5738}, {26.8165, -40.1717}, {24.5037, -41.6228}, {20.9071, -38.4981}, {19.7806, -36.9098}, {17.0479, -38.2489}, {17.6128, -40.1123}, {17.8783, -44.8693}, {15.3143, -45.8079}, {12.446, -42.0036}, {11.6744, -40.2159}, {8.723, -40.9575}, {8.8881, -42.8976}, {8.1588, -47.6059}, {5.4557, -47.9909}, {3.441, -43.6734}, {3.0579, -41.7643}, {0.0168, -41.8761}, {-0.225, -43.8082}, {-1.9173, -48.2619}, {-4.6414, -48.0765}, {-5.7144, -43.4344}, {-5.6922, -41.4874}, {-8.6901, -40.9645}, {-9.3284, -42.8041}, {-11.9096, -46.8087}, {-14.5357, -46.0609}, {-14.6201, -41.2972}, {-14.1935, -39.3973}, {-17.0172, -38.2626}, {-18.024, -39.9292}, {-21.3814, -43.3096}, {-23.7946, -42.0322}, {-22.8868, -37.3551}, {-22.0745, -35.5854}, {-24.6006, -33.8884}, {-25.9319, -35.3093}, {-29.9188, -37.9178}, {-32.0136, -36.1665}, {-30.1532, -31.7803}, {-28.9908, -30.2182}, {-31.1088, -28.0331}, {-32.7064, -29.1461}, {-37.1485, -30.8687}, {-38.8335, -28.7202}, {-36.1018, -24.8167}, {-34.64, -23.5304}, {-36.2574, -20.9526}, {-38.0515, -21.7092}, {-42.7547, -22.4705}, {-43.9562, -20.0187}, {-40.4725, -16.7684}, {-38.7753, -15.8141}, {-39.8213, -12.9564}, {-41.7336, -13.3234}, {-46.4923, -13.0903}, {-47.1577, -10.4422}, {-43.0745, -7.9872}, {-41.2159, -7.4067}, {-41.6449, -4.394}, {-43.5917, -4.3554}, {-48.198, -3.1379}, {-48.2983, -0.4094}, {-43.7938, 1.143}, {-41.8552, 1.3244}, {-41.6485, 4.3605}, {-43.5446, 4.803}, {-47.7971, 6.9516}, {-47.3279, 9.6414}, {-42.5992, 10.2233}, {-40.6652, 9.9976}, {-39.8317, 12.9244}, {-41.5945, 13.7515}, {-45.3073, 16.7372}, {-44.2892, 19.2707}, {-39.5428, 18.8567}, {-37.6979, 18.2339}, {-36.2742, 20.9235}, {-37.8265, 22.099}, {-40.8374, 25.7914}, {-39.3147, 28.0578}, {-34.7581, 26.6661}, {-33.0831, 25.6733}, {-31.1313, 28.0081}, {-32.4052, 29.4806}, {-34.5827, 33.7184}, {-32.6221, 35.6187}, {-28.4544, 33.31}, {-27.0223, 31.9906}, {-24.6278, 33.8686}, {-25.5677, 35.5738}, {-26.8165, 40.1717}, {-24.5037, 41.6228}, {-20.9071, 38.4981}, {-19.7806, 36.9098}, {-17.0479, 38.2489}, {-17.6128, 40.1123}, {-17.8783, 44.8693}, {-15.3143, 45.8079}, {-12.446, 42.0036}, {-11.6744, 40.2159}, {-8.723, 40.9575}, {-8.8881, 42.8976}, {-8.1588, 47.6059}, {-5.4557, 47.9909}, {-3.441, 43.6734}, {-3.0579, 41.7643}, {-0.0168, 41.8761}, {0.2251, 43.8082}, {1.9173, 48.2619}, {4.6414, 48.0765}, {5.7144, 43.4344}, {5.6922, 41.4874}}, fillColor = {255, 0, 0},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, pattern = LinePattern.None), Polygon(points = {{-9.977, 22.4878}, {-8.8831, 24.6535}, {-6.9407, 26.8978}, {-5.8083, 27.739}, {-3.7455, 26.7376}, {-3.706, 25.3275}, {-4.2679, 22.413}, {-5.2931, 20.2139}, {-2.7696, 18.3875}, {-1.001, 20.0486}, {1.5919, 21.4932}, {2.9438, 21.8963}, {4.5396, 20.2498}, {4.0945, 18.9112}, {2.5696, 16.3647}, {0.8542, 14.6489}, {2.6008, 12.0695}, {4.8309, 13.0255}, {7.7614, 13.4962}, {9.1697, 13.4126}, {10.1062, 11.3196}, {9.23, 10.214}, {6.9262, 8.3426}, {4.7273, 7.3169}, {5.4865, 4.2957}, {7.909, 4.4313}, {10.8238, 3.8713}, {12.1185, 3.3112}, {12.2827, 1.0241}, {11.0812, 0.2848}, {8.2763, -0.6858}, {5.8592, -0.8975}, {5.5393, -3.9962}, {7.8621, -4.6973}, {10.4096, -6.2205}, {11.4347, -7.1897}, {10.8067, -9.395}, {9.4248, -9.6788}, {6.4571, -9.6315}, {4.1134, -9.0037}, {2.7529, -11.8061}, {4.6959, -13.2594}, {6.5687, -15.562}, {7.2005, -16.8233}, {5.8561, -18.6809}, {4.4606, -18.4749}, {1.688, -17.4154}, {-0.2997, -16.024}, {-2.5365, -18.192}, {-1.2078, -20.2222}, {-0.2355, -23.0265}, {-0.0732, -24.4278}, {-1.9718, -25.7135}, {-3.2128, -25.0427}, {-5.4558, -23.0988}, {-6.8477, -21.1115}, {-9.6911, -22.3837}, {-9.1369, -24.7459}, {-9.1823, -27.7137}, {-9.5091, -29.086}, {-11.733, -29.6448}, {-12.6697, -28.59}, {-14.1125, -25.9962}, {-14.7408, -23.6526}, {-17.8479, -23.8756}, {-18.135, -26.2849}, {-19.1927, -29.0581}, {-19.9692, -30.2359}, {-22.25, -30.0005}, {-22.7695, -28.6889}, {-23.2382, -25.758}, {-23.027, -23.3409}, {-26.023, -22.4878}, {-27.1169, -24.6535}, {-29.0593, -26.8978}, {-30.1917, -27.739}, {-32.2545, -26.7376}, {-32.294, -25.3275}, {-31.7321, -22.413}, {-30.7069, -20.2139}, {-33.2304, -18.3875}, {-34.999, -20.0486}, {-37.5919, -21.4932}, {-38.9438, -21.8963}, {-40.5396, -20.2498}, {-40.0945, -18.9112}, {-38.5696, -16.3647}, {-36.8542, -14.6489}, {-38.6008, -12.0695}, {-40.8309, -13.0255}, {-43.7614, -13.4962}, {-45.1697, -13.4126}, {-46.1062, -11.3196}, {-45.23, -10.214}, {-42.9262, -8.3426}, {-40.7273, -7.3169}, {-41.4865, -4.2957}, {-43.909, -4.4313}, {-46.8238, -3.8713}, {-48.1185, -3.3112}, {-48.2827, -1.0241}, {-47.0812, -0.2848}, {-44.2763, 0.6858}, {-41.8592, 0.8975}, {-41.5393, 3.9962}, {-43.8621, 4.6973}, {-46.4096, 6.2205}, {-47.4347, 7.1897}, {-46.8067, 9.395}, {-45.4248, 9.6788}, {-42.4571, 9.6315}, {-40.1134, 9.0037}, {-38.7529, 11.8061}, {-40.6959, 13.2594}, {-42.5687, 15.562}, {-43.2005, 16.8233}, {-41.8561, 18.6809}, {-40.4606, 18.4749}, {-37.688, 17.4154}, {-35.7003, 16.024}, {-33.4635, 18.192}, {-34.7922, 20.2222}, {-35.7645, 23.0265}, {-35.9268, 24.4278}, {-34.0282, 25.7135}, {-32.7872, 25.0427}, {-30.5442, 23.0988}, {-29.1523, 21.1115}, {-26.3089, 22.3837}, {-26.8631, 24.7459}, {-26.8177, 27.7137}, {-26.4909, 29.086}, {-24.267, 29.6448}, {-23.3303, 28.59}, {-21.8875, 25.9962}, {-21.2592, 23.6526}, {-18.1521, 23.8756}, {-17.865, 26.2849}, {-16.8073, 29.0581}, {-16.0308, 30.2359}, {-13.75, 30.0005}, {-13.2305, 28.6889}, {-12.7618, 25.758}, {-12.973, 23.3409}}, fillColor = {255, 160, 160},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                    lineThickness =                                                                                                                                                                                                        0.5, pattern = LinePattern.None), Line(points = {{-100, 0}, {-20, 0}}, thickness = 1), Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255})}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This partial class is intended to design a <em>default icon for an internal planar gear contact</em>.
<p>
</html>"));
        end PlanarGearContactInternalL1;

        package GearConnections "Icon for a package class which contains gear connetion elements"
          extends Modelica.Icons.Package;
          annotation (
            Icon(graphics={  Polygon(points = {{26.6901, 34.9645}, {27.3284, 36.8041}, {29.9096, 40.8087}, {32.5357, 40.0609}, {32.6201, 35.2972}, {32.1935, 33.3973}, {35.0172, 32.2626}, {36.024, 33.9292}, {39.3814, 37.3096}, {41.7946, 36.0322}, {40.8868, 31.3551}, {40.0745, 29.5854}, {42.6006, 27.8884}, {43.9319, 29.3093}, {47.9188, 31.9178}, {50.0136, 30.1665}, {48.1532, 25.7803}, {46.9908, 24.2182}, {49.1088, 22.0331}, {50.7064, 23.1461}, {55.1485, 24.8687}, {56.8335, 22.7202}, {54.1018, 18.8167}, {52.64, 17.5304}, {54.2574, 14.9526}, {56.0515, 15.7092}, {60.7547, 16.4705}, {61.9562, 14.0187}, {58.4725, 10.7684}, {56.7753, 9.8141}, {57.8213, 6.9564}, {59.7336, 7.3234}, {64.4923, 7.0903}, {65.1577, 4.4422}, {61.0745, 1.9872}, {59.2159, 1.4067}, {59.6449, -1.606}, {61.5917, -1.6446}, {66.198, -2.8621}, {66.2983, -5.5906}, {61.7938, -7.143}, {59.8552, -7.3244}, {59.6485, -10.3605}, {61.5446, -10.803}, {65.7971, -12.9516}, {65.3279, -15.6414}, {60.5992, -16.2233}, {58.6652, -15.9976}, {57.8317, -18.9244}, {59.5945, -19.7515}, {63.3073, -22.7372}, {62.2892, -25.2707}, {57.5428, -24.8567}, {55.6979, -24.2339}, {54.2742, -26.9235}, {55.8265, -28.099}, {58.8374, -31.7914}, {57.3147, -34.0578}, {52.7581, -32.6661}, {51.0831, -31.6733}, {49.1313, -34.0081}, {50.4052, -35.4806}, {52.5827, -39.7184}, {50.6221, -41.6187}, {46.4544, -39.31}, {45.0223, -37.9906}, {42.6278, -39.8686}, {43.5677, -41.5738}, {44.8165, -46.1717}, {42.5037, -47.6228}, {38.9071, -44.4981}, {37.7806, -42.9098}, {35.0479, -44.2489}, {35.6128, -46.1123}, {35.8783, -50.8693}, {33.3143, -51.8079}, {30.446, -48.0036}, {29.6744, -46.2159}, {26.723, -46.9575}, {26.8881, -48.8976}, {26.1588, -53.6059}, {23.4557, -53.9909}, {21.441, -49.6734}, {21.0579, -47.7643}, {18.0168, -47.8761}, {17.775, -49.8082}, {16.0827, -54.2619}, {13.3586, -54.0765}, {12.2856, -49.4344}, {12.3078, -47.4874}, {9.3099, -46.9645}, {8.6716, -48.8041}, {6.0904, -52.8087}, {3.4643, -52.0609}, {3.3799, -47.2972}, {3.8065, -45.3973}, {0.9828, -44.2626}, {-0.024, -45.9292}, {-3.3814, -49.3096}, {-5.7946, -48.0322}, {-4.8868, -43.3551}, {-4.0745, -41.5854}, {-6.6006, -39.8884}, {-7.9319, -41.3093}, {-11.9188, -43.9178}, {-14.0136, -42.1665}, {-12.1532, -37.7803}, {-10.9908, -36.2182}, {-13.1088, -34.0331}, {-14.7064, -35.1461}, {-19.1485, -36.8687}, {-20.8335, -34.7202}, {-18.1018, -30.8167}, {-16.64, -29.5304}, {-18.2574, -26.9526}, {-20.0515, -27.7092}, {-24.7547, -28.4705}, {-25.9562, -26.0187}, {-22.4725, -22.7684}, {-20.7753, -21.8141}, {-21.8213, -18.9564}, {-23.7336, -19.3234}, {-28.4923, -19.0903}, {-29.1577, -16.4422}, {-25.0745, -13.9872}, {-23.2159, -13.4067}, {-23.6449, -10.394}, {-25.5917, -10.3554}, {-30.198, -9.1379}, {-30.2983, -6.40937}, {-25.7938, -4.857}, {-23.8552, -4.6756}, {-23.6485, -1.6395}, {-25.5446, -1.197}, {-29.7971, 0.9516}, {-29.3279, 3.6414}, {-24.5992, 4.2233}, {-22.6652, 3.9976}, {-21.8317, 6.9244}, {-23.5945, 7.7515}, {-27.3073, 10.7372}, {-26.2892, 13.2707}, {-21.5428, 12.8567}, {-19.6979, 12.2339}, {-18.2742, 14.9235}, {-19.8265, 16.099}, {-22.8374, 19.7914}, {-21.3147, 22.0578}, {-16.7581, 20.6661}, {-15.0831, 19.6733}, {-13.1313, 22.0081}, {-14.4052, 23.4806}, {-16.5827, 27.7184}, {-14.6221, 29.6187}, {-10.4544, 27.31}, {-9.0223, 25.9906}, {-6.6278, 27.8686}, {-7.5677, 29.5738}, {-8.8165, 34.1717}, {-6.5037, 35.6228}, {-2.9071, 32.4981}, {-1.7806, 30.9098}, {0.9521, 32.2489}, {0.3872, 34.1123}, {0.1217, 38.8693}, {2.6857, 39.8079}, {5.554, 36.0036}, {6.3256, 34.2159}, {9.277, 34.9575}, {9.1119, 36.8976}, {9.8412, 41.6059}, {12.5443, 41.9909}, {14.559, 37.6734}, {14.9421, 35.7643}, {17.9832, 35.8761}, {18.2251, 37.8082}, {19.9173, 42.2619}, {22.6414, 42.0765}, {23.7144, 37.4344}, {23.6922, 35.4874}}, fillColor = {255, 0, 0},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, pattern = LinePattern.None), Polygon(points = {{-47.977, 16.4878}, {-46.8831, 18.6535}, {-44.9407, 20.8978}, {-43.8083, 21.739}, {-41.7455, 20.7376}, {-41.706, 19.3275}, {-42.2679, 16.413}, {-43.2931, 14.2139}, {-40.7696, 12.3875}, {-39.001, 14.0486}, {-36.4081, 15.4932}, {-35.0562, 15.8963}, {-33.4604, 14.2498}, {-33.9055, 12.9112}, {-35.4304, 10.3647}, {-37.1458, 8.6489}, {-35.3992, 6.0695}, {-33.1691, 7.0255}, {-30.2386, 7.4962}, {-28.8303, 7.4126}, {-27.8938, 5.3196}, {-28.77, 4.214}, {-31.0738, 2.3426}, {-33.2727, 1.3169}, {-32.5135, -1.7043}, {-30.091, -1.5687}, {-27.1762, -2.1287}, {-25.8815, -2.6888}, {-25.7173, -4.9759}, {-26.9188, -5.7152}, {-29.7237, -6.68579}, {-32.1408, -6.89754}, {-32.4607, -9.9962}, {-30.1379, -10.6973}, {-27.5904, -12.2205}, {-26.5653, -13.1897}, {-27.1933, -15.395}, {-28.5752, -15.6788}, {-31.5429, -15.6315}, {-33.8866, -15.0037}, {-35.2471, -17.8061}, {-33.3041, -19.2594}, {-31.4313, -21.562}, {-30.7995, -22.8233}, {-32.1439, -24.6809}, {-33.5394, -24.4749}, {-36.312, -23.4154}, {-38.2997, -22.024}, {-40.5365, -24.192}, {-39.2078, -26.2222}, {-38.2355, -29.0265}, {-38.0732, -30.4278}, {-39.9718, -31.7135}, {-41.2128, -31.0427}, {-43.4558, -29.0988}, {-44.8477, -27.1115}, {-47.6911, -28.3837}, {-47.1369, -30.7459}, {-47.1823, -33.7137}, {-47.5091, -35.086}, {-49.733, -35.6448}, {-50.6697, -34.59}, {-52.1125, -31.9962}, {-52.7408, -29.6526}, {-55.8479, -29.8756}, {-56.135, -32.2849}, {-57.1927, -35.0581}, {-57.9692, -36.2359}, {-60.25, -36.0005}, {-60.7695, -34.6889}, {-61.2382, -31.758}, {-61.027, -29.3409}, {-64.023, -28.4878}, {-65.1169, -30.6535}, {-67.0593, -32.8978}, {-68.1917, -33.739}, {-70.2545, -32.7376}, {-70.294, -31.3275}, {-69.7321, -28.413}, {-68.7069, -26.2139}, {-71.2304, -24.3875}, {-72.999, -26.0486}, {-75.5919, -27.4932}, {-76.9438, -27.8963}, {-78.5396, -26.2498}, {-78.0945, -24.9112}, {-76.5696, -22.3647}, {-74.8542, -20.6489}, {-76.6008, -18.0695}, {-78.8309, -19.0255}, {-81.7614, -19.4962}, {-83.1697, -19.4126}, {-84.1062, -17.3196}, {-83.23, -16.214}, {-80.9262, -14.3426}, {-78.7273, -13.3169}, {-79.4865, -10.2957}, {-81.909, -10.4313}, {-84.8238, -9.8713}, {-86.1185, -9.3112}, {-86.2827, -7.0241}, {-85.0812, -6.28477}, {-82.2763, -5.3142}, {-79.8592, -5.1025}, {-79.5393, -2.0038}, {-81.8621, -1.3027}, {-84.4096, 0.2205}, {-85.4347, 1.1897}, {-84.8067, 3.395}, {-83.4248, 3.6788}, {-80.4571, 3.6315}, {-78.1134, 3.0037}, {-76.7529, 5.8061}, {-78.6959, 7.2594}, {-80.5687, 9.562}, {-81.2005, 10.8233}, {-79.8561, 12.6809}, {-78.4606, 12.4749}, {-75.688, 11.4154}, {-73.7003, 10.024}, {-71.4635, 12.192}, {-72.7922, 14.2222}, {-73.7645, 17.0265}, {-73.9268, 18.4278}, {-72.0282, 19.7135}, {-70.7872, 19.0427}, {-68.5442, 17.0988}, {-67.1523, 15.1115}, {-64.3089, 16.3837}, {-64.8631, 18.7459}, {-64.8177, 21.7137}, {-64.4909, 23.086}, {-62.267, 23.6448}, {-61.3303, 22.59}, {-59.8875, 19.9962}, {-59.2592, 17.6526}, {-56.1521, 17.8756}, {-55.865, 20.2849}, {-54.8073, 23.0581}, {-54.0308, 24.2359}, {-51.75, 24.0005}, {-51.2305, 22.6889}, {-50.7618, 19.758}, {-50.973, 17.3409}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid,
                    lineThickness =                                                                                                                                                                                                        0.5, pattern = LinePattern.None)}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This partial class is intended to design a <em>default icon for a package class</em> which contains different <em>gear connection models</em>.
<p>
</html>"));
        end GearConnections;

        model PlanetaryGear "Icon for planetary gears"
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 100}, {80, -100}},
                    fillPattern =                                                                                                                                            FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{-100, 100}, {68, 90}}, pattern = LinePattern.None,
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Rectangle(extent = {{-100, -90}, {68, -100}}, pattern = LinePattern.None,
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Rectangle(extent = {{68, 100}, {80, -100}}, pattern = LinePattern.None,
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Rectangle(extent = {{-60, 38}, {8, -38}}, fillColor = {0, 0, 255},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, pattern = LinePattern.None), Rectangle(extent = {{-60, 88}, {8, 40}}, fillColor = {0, 127, 0},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, pattern = LinePattern.None), Rectangle(extent = {{-60, -40}, {8, -88}}, fillColor = {0, 127, 0},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, pattern = LinePattern.None), Rectangle(extent = {{8, 70}, {24, 60}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {255, 255, 0}), Rectangle(extent = {{8, -60}, {24, -70}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {255, 255, 0}), Rectangle(extent = {{44, 10}, {100, -10}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {255, 255, 0}), Rectangle(extent = {{24, 82}, {44, -82}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {255, 255, 0}), Rectangle(extent = {{-100, 10}, {-60, -10}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {0, 0, 255}), Text(extent = {{-140, 140}, {140, 100}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Sphere, fillColor = {85, 170, 255}, textString = "%name")}),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
This partial class is intended to design a <em>default icon for a planetary gear models</em>.
<p>
</html>"));
        end PlanetaryGear;

        package Parts "Icon for a package class which contains parts"
          extends Modelica.Icons.Package;
          annotation (
            Icon(graphics={  Rectangle(extent = {{-82, 22}, {0, -22}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                              FillPattern.HorizontalCylinder, fillColor = {215, 215, 215}, radius = 10), Ellipse(extent = {{-10, 46}, {84, -48}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Sphere, fillColor = {215, 215, 215})}),
            Documentation(info = "<html>
<p>
This partial class is intended to design a <em>default icon for a package class</em> which contains different <em>2-dim. parts</em>.
<p>
</html>"));
        end Parts;

        package Joints "Icon for a package class which contains joints"
          extends Modelica.Icons.Package;
          annotation (
            Icon(graphics={  Ellipse(extent = {{-4, 10}, {42, -36}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                            FillPattern.Sphere, fillColor = {95, 95, 95}), Polygon(points = {{14, 8}, {36, 0}, {62, 82}, {40, 88}, {14, 8}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Sphere, fillColor = {255, 255, 255}), Polygon(points = {{-4, -16}, {8, -34}, {-62, -82}, {-74, -64}, {-4, -16}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Sphere, fillColor = {255, 255, 255})}),
            Documentation(info = "<html>
<p>
This partial class is intended to design a <em>default icon for a package class</em> which contains different <em>joint models</em>.
<p>
</html>"));
        end Joints;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>
A collection of basic icons to be used for different elements of the library.
</p>
<placeholder to comply minimal number of characters for documentation>
</html>"));
      end Icons;
      annotation (
        Documentation(info = "<html>
<p>This package contains auxiliary packages and elements to be used in context with the library.
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
    end Utilities;

    package VehicleComponents "Vehicle relevant 2-dim. components"
      extends Modelica.Icons.Package;

      package Examples "Collection of simulatable models involving vehicle components"
        extends Modelica.Icons.ExamplesPackage;

        model TestIdealWheel "Test an ideal wheel"
          extends Modelica.Icons.Example;
          VehicleComponents.Wheels.IdealWheelJoint idealWheelJoint(radius = 0.3, r = {1, 0}, animate = true) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 30})));
          Joints.Prismatic prismatic(r = {0, 1}, s(start = 1, fixed = true)) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -10})));
          Joints.Revolute revolute(phi(fixed = true), w(fixed = false), stateSelect = StateSelect.always) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -40})));
          Parts.Fixed fixed annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -70})));
          Modelica.Mechanics.Rotational.Sources.ConstantTorque engineTorque(tau_constant = 2) annotation (
            Placement(transformation(extent = {{-32, 60}, {-12, 80}})));
          Parts.Body body(m = 10, I = 1, animate = false) annotation (
            Placement(transformation(extent = {{20, 0}, {40, 20}})));
          Modelica.Mechanics.Rotational.Components.Inertia inertia(phi(fixed = true, start = 0), w(fixed = true, start = 0), J = 1) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {0, 60})));
          inner PlanarWorld planarWorld(enableAnimation = true, constantGravity = {0, 0}) annotation (
            Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        equation
          connect(idealWheelJoint.frame_a, prismatic.frame_b) annotation (
            Line(points = {{0, 26}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(prismatic.frame_a, revolute.frame_b) annotation (
            Line(points = {{0, -20}, {0, -20}, {0, -30}}, color = {95, 95, 95}, thickness = 0.5));
          connect(revolute.frame_a, fixed.frame) annotation (
            Line(points = {{0, -50}, {0, -50}, {0, -60}}, color = {95, 95, 95}, thickness = 0.5));
          connect(engineTorque.flange, inertia.flange_a) annotation (
            Line(points = {{-12, 70}, {0, 70}}));
          connect(inertia.flange_b, idealWheelJoint.flange_a) annotation (
            Line(points = {{0, 50}, {0, 40}}));
          connect(body.frame_a, prismatic.frame_b) annotation (
            Line(points = {{20, 10}, {0, 10}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>It introduces one non-holonomic constraint. Difficult for index-reduction.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/TestIdealWheel_1.png\" alt=\"Diagram TestIdealWheel_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/TestIdealWheel_2.png\" alt=\"Diagram TestIdealWheel_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>inertia.phi</li>
<li>prismatic.s</li>
<li>revolute.phi</li>
<li>revolute.w</li>
</ul>
</html>"),  experiment(StopTime = 10));
        end TestIdealWheel;

        model TestDryFrictionWheel "Dry friction wheel"
          extends Modelica.Icons.Example;
          Joints.Prismatic prismatic(r = {0, 1}, s(start = 1, fixed = true), v(fixed = true)) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -10})));
          Joints.Revolute revolute(phi(fixed = true), w(fixed = true)) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -40})));
          Parts.Fixed fixed annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -70})));
          Modelica.Mechanics.Rotational.Sources.ConstantTorque engineTorque(tau_constant = 2) annotation (
            Placement(transformation(extent = {{-32, 60}, {-12, 80}})));
          Parts.Body body(m = 10, I = 1) annotation (
            Placement(transformation(extent = {{20, 0}, {40, 20}})));
          Modelica.Mechanics.Rotational.Components.Inertia inertia(phi(fixed = true, start = 0), J = 1, w(start = 0, fixed = true)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {0, 60})));
          VehicleComponents.Wheels.DryFrictionWheelJoint dryFrictionWheelJoint(radius = 0.3, r = {1, 0}, N = 100, vAdhesion = 0.1, vSlide = 0.3, mu_A = 0.8, mu_S = 0.4, w_roll(fixed = false, start = 10)) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 30})));
          inner PlanarWorld planarWorld(constantGravity = {0, 0}) annotation (
            Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        equation
          connect(prismatic.frame_a, revolute.frame_b) annotation (
            Line(points = {{0, -20}, {0, -20}, {0, -30}}, color = {95, 95, 95}, thickness = 0.5));
          connect(revolute.frame_a, fixed.frame) annotation (
            Line(points = {{0, -50}, {0, -50}, {0, -60}}, color = {95, 95, 95}, thickness = 0.5));
          connect(engineTorque.flange, inertia.flange_a) annotation (
            Line(points = {{-12, 70}, {0, 70}}));
          connect(body.frame_a, prismatic.frame_b) annotation (
            Line(points = {{20, 10}, {0, 10}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(dryFrictionWheelJoint.frame_a, prismatic.frame_b) annotation (
            Line(points = {{0, 26}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(dryFrictionWheelJoint.flange_a, inertia.flange_b) annotation (
            Line(points = {{0, 40}, {0, 40}, {0, 50}}));
          annotation (
            experiment(StopTime = 20),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/TestDryFrictionWheel_1.png\" alt=\"Diagram TestDryFrictionWheel_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/TestDryFrictionWheel_2.png\" alt=\"Diagram TestDryFrictionWheel_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>inertia.phi</li>
<li>inertia.w</li>
<li>prismatic.s</li>
<li>prismatic.v</li>
<li>revolute.phi</li>
<li>revolute.w</li>
</ul>
</html>"));
        end TestDryFrictionWheel;

        model TestSlipBasedWheel "A slip-based wheel"
          extends Modelica.Icons.Example;
          Joints.Prismatic prismatic(r = {0, 1}, s(start = 1, fixed = true), v(fixed = true)) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -10})));
          Joints.Revolute revolute(phi(fixed = true), w(fixed = true)) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -40})));
          Parts.Fixed fixed annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -70})));
          Modelica.Mechanics.Rotational.Sources.ConstantTorque engineTorque(tau_constant = 2) annotation (
            Placement(transformation(extent = {{-32, 60}, {-12, 80}})));
          Parts.Body body(m = 10, I = 1) annotation (
            Placement(transformation(extent = {{20, 0}, {40, 20}})));
          Modelica.Mechanics.Rotational.Components.Inertia inertia(phi(fixed = true, start = 0), J = 1, w(fixed = true, start = 0)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {0, 60})));
          VehicleComponents.Wheels.SlipBasedWheelJoint slipBasedWheelJoint(radius = 0.3, r = {1, 0}, mu_A = 0.8, mu_S = 0.4, N = 100, sAdhesion = 0.04, sSlide = 0.12, vAdhesion_min = 0.05, vSlide_min = 0.15, w_roll(fixed = false, start = 10)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 90, origin = {0, 30})));
          Modelica.Blocks.Sources.Constant const(k = 0) annotation (
            Placement(transformation(extent = {{-60, 20}, {-40, 40}})));
          inner PlanarWorld planarWorld(constantGravity = {0, 0}) annotation (
            Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        equation
          connect(prismatic.frame_a, revolute.frame_b) annotation (
            Line(points = {{0, -20}, {0, -20}, {0, -30}}, color = {95, 95, 95}, thickness = 0.5));
          connect(revolute.frame_a, fixed.frame) annotation (
            Line(points = {{0, -50}, {0, -50}, {0, -60}}, color = {95, 95, 95}, thickness = 0.5));
          connect(engineTorque.flange, inertia.flange_a) annotation (
            Line(points = {{-12, 70}, {0, 70}}));
          connect(body.frame_a, prismatic.frame_b) annotation (
            Line(points = {{20, 10}, {0, 10}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(slipBasedWheelJoint.frame_a, prismatic.frame_b) annotation (
            Line(points = {{0, 26}, {0, 26}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
          connect(slipBasedWheelJoint.flange_a, inertia.flange_b) annotation (
            Line(points = {{0, 40}, {0, 40}, {0, 50}}));
          connect(const.y, slipBasedWheelJoint.dynamicLoad) annotation (
            Line(points = {{-39, 30}, {-10, 30}}, color = {0, 0, 127}));
          annotation (
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/TestSlipBasedWheel_1.png\" alt=\"Diagram TestSlipBasedWheel_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/TestSlipBasedWheel_2.png\" alt=\"Diagram TestSlipBasedWheel_2\"></p>
<p>Selected continuous time states</p>
<ul>
<li>inertia.phi</li>
<li>inertia.w</li>
<li>prismatic.s</li>
<li>prismatic.v</li>
<li>revolute.phi</li>
<li>revolute.w</li>
</ul>
</html>"),  experiment(StopTime = 20));
        end TestSlipBasedWheel;

        model SingleTrackWithEngine "Single track model"
          extends Modelica.Icons.Example;
          Parts.Body bodyFront(I = 0.1, m = 2, enableGravity = false) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {40, 50})));
          VehicleComponents.Wheels.IdealWheelJoint idealWheelFront(r = {0, 1}, radius = 0.3, phi_roll(fixed = true)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {0, 50})));
          Parts.FixedTranslation chassis(r = {0, 1}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {20, -40})));
          Parts.Body bodyRear(I = 0.1, m = 10, phi(fixed = true), w(fixed = true), v(each fixed = false), r(each fixed = true), enableGravity = false) annotation (
            Placement(transformation(extent = {{30, -90}, {50, -70}})));
          VehicleComponents.Wheels.IdealWheelJoint idealWheelRear(r = {0, 1}, radius = 0.3, w_roll(fixed = true, start = 0), phi_roll(fixed = true), stateSelect = StateSelect.default) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {0, -80})));
          Joints.Revolute revolute(w(fixed = false, start = 0), stateSelect = StateSelect.always, phi(fixed = true, start = 0.69813170079773)) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {20, 0})));
          Modelica.Mechanics.Rotational.Sources.ConstantTorque engineTorque(tau_constant = 2) annotation (
            Placement(transformation(extent = {{-40, -90}, {-20, -70}})));
          Parts.FixedTranslation trail(r = {0, -0.1}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {20, 30})));
          inner PlanarWorld planarWorld(defaultWidthFraction = 10, defaultZPosition = 0, constantGravity = {0, 0}) annotation (
            Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
        equation
          connect(idealWheelFront.frame_a, bodyFront.frame_a) annotation (
            Line(points = {{4, 50}, {30, 50}}, color = {95, 95, 95}, thickness = 0.5));
          connect(chassis.frame_a, idealWheelRear.frame_a) annotation (
            Line(points = {{20, -50}, {20, -80}, {4, -80}}, color = {95, 95, 95}, thickness = 0.5));
          connect(bodyRear.frame_a, chassis.frame_a) annotation (
            Line(points = {{30, -80}, {20, -80}, {20, -50}}, color = {95, 95, 95}, thickness = 0.5));
          connect(revolute.frame_a, chassis.frame_b) annotation (
            Line(points = {{20, -10}, {20, -30}}, color = {95, 95, 95}, thickness = 0.5));
          connect(engineTorque.flange, idealWheelRear.flange_a) annotation (
            Line(points = {{-20, -80}, {-10, -80}}));
          connect(trail.frame_a, revolute.frame_b) annotation (
            Line(points = {{20, 20}, {20, 10}}, color = {95, 95, 95}, thickness = 0.5));
          connect(trail.frame_b, idealWheelFront.frame_a) annotation (
            Line(points = {{20, 40}, {20, 50}, {4, 50}}, color = {95, 95, 95}, thickness = 0.5));
          annotation (
            experiment(StopTime = 6),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>An ideal rolling single track model of a car.
There is dynamic state selection applied. It might be avoided by picking Rear.v_long as state.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/SingleTrackWithEngine_1.png\" alt=\"Diagram SingleTrackWithEngine_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/SingleTrackWithEngine_2.png\" alt=\"Diagram SingleTrackWithEngine_2\"></p>
</html>"));
        end SingleTrackWithEngine;

        model TwoTrackWithDifferentialGear "Double track model"
          extends Modelica.Icons.Example;
          Parts.Body body(m = 100, I = 1, enableGravity = false) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-40, 90})));
          VehicleComponents.Wheels.DryFrictionWheelJoint wheelJoint1(vAdhesion = 0.1, r = {0, 1}, vSlide = 0.3, mu_A = 1, mu_S = 0.7, radius = 0.25, N = 1000, phi_roll(fixed = false)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-64, 70})));
          Parts.FixedTranslation fixedTranslation1(r = {0, 2}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -10})));
          Parts.Body body1(I = 0.1, m = 300, r(each fixed = true), v(each fixed = true), phi(fixed = true), w(fixed = true), enableGravity = false) annotation (
            Placement(transformation(extent = {{12, -40}, {32, -20}})));
          VehicleComponents.Wheels.DryFrictionWheelJoint wheelJoint2(r = {0, 1}, vAdhesion = 0.1, vSlide = 0.3, mu_A = 1, mu_S = 0.7, radius = 0.25, N = 1500, phi_roll(fixed = false)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-64, -50})));
          Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque1(tau_constant = 25) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-50, -90})));
          Modelica.Mechanics.Rotational.Components.Inertia inertia(phi(fixed = true, start = 0), w(fixed = true, start = 0), J = 1) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-90, 70})));
          Modelica.Mechanics.Rotational.Components.Inertia inertia1(phi(fixed = true, start = 0), w(fixed = true, start = 0), J = 1) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-92, -50})));
          Parts.FixedTranslation fixedTranslation2(r = {0.75, 0}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-20, -50})));
          Parts.FixedTranslation fixedTranslation3(r = {-0.75, 0}) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {20, -50})));
          VehicleComponents.Wheels.DryFrictionWheelJoint wheelJoint3(r = {0, 1}, vAdhesion = 0.1, vSlide = 0.3, mu_A = 1, mu_S = 0.7, radius = 0.25, N = 1500) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {64, -50})));
          Modelica.Mechanics.Rotational.Components.Inertia inertia2(J = 1, phi(fixed = true, start = 0), w(fixed = true, start = 0)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {90, -50})));
          Parts.FixedTranslation fixedTranslation4(r = {0.75, 0}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-20, 12})));
          Parts.FixedTranslation fixedTranslation5(r = {-0.75, 0}) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {20, 12})));
          VehicleComponents.Wheels.DryFrictionWheelJoint wheelJoint4(vAdhesion = 0.1, r = {0, 1}, vSlide = 0.3, mu_A = 1, mu_S = 0.7, radius = 0.25, N = 1000) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {64, 70})));
          Modelica.Mechanics.Rotational.Components.Inertia inertia3(phi(fixed = true, start = 0), w(fixed = true, start = 0), J = 1) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {90, 70})));
          Parts.Body body2(m = 100, I = 1, enableGravity = false) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {40, 90})));
          Modelica.Mechanics.Rotational.Sources.Torque torque annotation (
            Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {-10, 50})));
          Modelica.Blocks.Sources.Pulse pulse(period = 2, offset = 0, startTime = 1, width = 30, amplitude = -2) annotation (
            Placement(transformation(extent = {{20, 70}, {0, 90}})));
          VehicleComponents.DifferentialGear differentialGear annotation (
            Placement(transformation(extent = {{-10, -82}, {10, -62}})));
          Parts.FixedTranslation leftTrail(r = {0., -0.05}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-40, 60})));
          Parts.FixedTranslation rightTrail(r = {0., -0.05}) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {40, 60})));
          inner PlanarWorld planarWorld(defaultWidthFraction = 10, constantGravity = {0, 0}) annotation (
            Placement(transformation(extent = {{-100, 0}, {-80, 20}})));
          Joints.Revolute revolute2(useFlange = true, phi(fixed = true, start = -0.43633231299858), w(fixed = true)) annotation (
            Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {-40, 30})));
          Joints.Revolute revolute(useFlange = true) annotation (
            Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {40, 30})));
        equation
          connect(wheelJoint2.flange_a, inertia1.flange_b) annotation (
            Line(points = {{-74, -50}, {-82, -50}}));
          connect(inertia.flange_b, wheelJoint1.flange_a) annotation (
            Line(points = {{-80, 70}, {-74, 70}}));
          connect(fixedTranslation2.frame_b, fixedTranslation1.frame_a) annotation (
            Line(points = {{-10, -50}, {0, -50}, {0, -20}}, color = {95, 95, 95}, thickness = 0.5));
          connect(fixedTranslation2.frame_a, wheelJoint2.frame_a) annotation (
            Line(points = {{-30, -50}, {-60, -50}}, color = {95, 95, 95}, thickness = 0.5));
          connect(fixedTranslation3.frame_b, fixedTranslation1.frame_a) annotation (
            Line(points = {{10, -50}, {0, -50}, {0, -20}}, color = {95, 95, 95}, thickness = 0.5));
          connect(wheelJoint3.frame_a, fixedTranslation3.frame_a) annotation (
            Line(points = {{60, -50}, {30, -50}}, color = {95, 95, 95}, thickness = 0.5));
          connect(inertia2.flange_b, wheelJoint3.flange_a) annotation (
            Line(points = {{80, -50}, {74, -50}}));
          connect(body1.frame_a, fixedTranslation1.frame_a) annotation (
            Line(points = {{12, -30}, {0, -30}, {0, -20}}, color = {95, 95, 95}, thickness = 0.5));
          connect(fixedTranslation1.frame_b, fixedTranslation4.frame_b) annotation (
            Line(points = {{0, 0}, {0, 0}, {0, 12}, {-10, 12}}, color = {95, 95, 95}, thickness = 0.5));
          connect(fixedTranslation1.frame_b, fixedTranslation5.frame_b) annotation (
            Line(points = {{0, 0}, {0, 0}, {0, 12}, {10, 12}}, color = {95, 95, 95}, thickness = 0.5));
          connect(inertia3.flange_b, wheelJoint4.flange_a) annotation (
            Line(points = {{80, 70}, {74, 70}}));
          connect(pulse.y, torque.tau) annotation (
            Line(points = {{-1, 80}, {-10, 80}, {-10, 62}}, color = {0, 0, 127}));
          connect(differentialGear.flange_right, wheelJoint3.flange_a) annotation (
            Line(points = {{10, -72}, {74, -72}, {74, -50}}));
          connect(differentialGear.flange_left, wheelJoint2.flange_a) annotation (
            Line(points = {{-10, -72}, {-74, -72}, {-74, -50}}));
          connect(constantTorque1.flange, differentialGear.flange_b) annotation (
            Line(points = {{-40, -90}, {0, -90}, {0, -82}}));
          connect(body.frame_a, leftTrail.frame_b) annotation (
            Line(points = {{-40, 80}, {-40, 70}}, color = {95, 95, 95}, thickness = 0.5));
          connect(leftTrail.frame_b, wheelJoint1.frame_a) annotation (
            Line(points = {{-40, 70}, {-60, 70}}, color = {95, 95, 95}, thickness = 0.5));
          connect(body2.frame_a, rightTrail.frame_b) annotation (
            Line(points = {{40, 80}, {40, 70}}, color = {95, 95, 95}, thickness = 0.5));
          connect(wheelJoint4.frame_a, rightTrail.frame_b) annotation (
            Line(points = {{60, 70}, {40, 70}}, color = {95, 95, 95}, thickness = 0.5));
          connect(leftTrail.frame_a, revolute2.frame_a) annotation (
            Line(points = {{-40, 50}, {-40, 40}}, color = {95, 95, 95}, thickness = 0.5));
          connect(revolute2.frame_b, fixedTranslation4.frame_a) annotation (
            Line(points = {{-40, 20}, {-40, 12}, {-30, 12}}, color = {95, 95, 95}, thickness = 0.5));
          connect(torque.flange, revolute2.flange_a) annotation (
            Line(points = {{-10, 40}, {-10, 30}, {-30, 30}}));
          connect(revolute.flange_a, revolute2.flange_a) annotation (
            Line(points = {{30, 30}, {30, 30}, {-30, 30}}));
          connect(revolute.frame_a, rightTrail.frame_a) annotation (
            Line(points = {{40, 40}, {40, 50}}, color = {95, 95, 95}, thickness = 0.5));
          connect(revolute.frame_b, fixedTranslation5.frame_a) annotation (
            Line(points = {{40, 20}, {40, 12}, {30, 12}}, color = {95, 95, 95}, thickness = 0.5));
          annotation (
            experiment(StopTime = 10),
            Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>A double track model of a car.</p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/SimpleCarWithDifferentialGear_1.png\" alt=\"Diagram SimpleCarWithDifferentialGear_1\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/SimpleCarWithDifferentialGear_2.png\" alt=\"Diagram SimpleCarWithDifferentialGear_2\"></p>
<p><img src=\"modelica://PlanarMechanics/Resources/Images/VehicleComponents/Examples/SimpleCarWithDifferentialGear_3.png\" alt=\"Diagram SimpleCarWithDifferentialGear_3\"></p>

<p>Selected continuous time states</p>
<ul>
<li>actuatedRevolute.phi</li>
<li>actuatedRevolute.w</li>
<li>body.v[1]</li>
<li>body1.frame_a.phi</li>
<li>body1.r[1]</li>
<li>body1.r[2]</li>
<li>body1.w</li>
<li>body2.v[2]</li>
<li>inertia.phi</li>
<li>inertia.w</li>
<li>inertia1.phi</li>
<li>inertia1.w</li>
<li>inertia2.phi</li>
<li>inertia2.w</li>
<li>inertia3.phi</li>
<li>inertia3.w</li>
</ul>
</html>"));
        end TwoTrackWithDifferentialGear;
        annotation (
          Documentation(revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>", info = "<html>
<p>This package contains examples of wheels and vehicles rolling on a x,y-plane</p>
</html>"));
      end Examples;

      model DifferentialGear "Simple Model of a differential gear"
        Modelica.Mechanics.Rotational.Components.IdealPlanetary idealPlanetary(ratio = -2) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -52})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation (
          Placement(transformation(extent = {{-10, -110}, {10, -90}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_left annotation (
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_right annotation (
          Placement(transformation(extent = {{90, -10}, {110, 10}})));
      equation
        connect(flange_b, idealPlanetary.ring) annotation (
          Line(points = {{0, -100}, {0, -62}, {0, -62}}));
        connect(idealPlanetary.carrier, flange_right) annotation (
          Line(points = {{4, -42}, {4, 0}, {100, 0}}));
        connect(idealPlanetary.sun, flange_left) annotation (
          Line(points = {{0, -42}, {0, -42}, {0, 0}, {-100, 0}}));
        annotation (
          Icon(graphics={  Rectangle(extent = {{-60, 50}, {40, -50}}, fillColor = {175, 175, 175},
                  fillPattern =                                                                                  FillPattern.Solid, pattern = LinePattern.None), Rectangle(extent = {{-48, 40}, {40, -40}}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, pattern = LinePattern.None), Polygon(points = {{40, -60}, {60, -80}, {60, 80}, {40, 60}, {40, -60}}, pattern = LinePattern.None, fillColor = {175, 175, 175},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{20, -60}, {40, -80}, {-40, -80}, {-20, -60}, {20, -60}}, pattern = LinePattern.None, fillColor = {175, 175, 175},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{14, 10}, {34, -10}, {-34, -10}, {-14, 10}, {14, 10}}, pattern = LinePattern.None, fillColor = {135, 135, 135},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, origin = {-30, 0}, rotation = 270), Polygon(points = {{14, 10}, {34, -10}, {-32, -10}, {-12, 10}, {14, 10}}, pattern = LinePattern.None, fillColor = {135, 135, 135},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, origin = {-4, -26}, rotation = 360), Polygon(points = {{16, 10}, {36, -10}, {-32, -10}, {-12, 10}, {16, 10}}, pattern = LinePattern.None, fillColor = {135, 135, 135},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, origin = {24, -2}, rotation = 90), Rectangle(extent = {{-100, 10}, {-40, -10}}, pattern = LinePattern.None, fillColor = {135, 135, 135},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Rectangle(extent = {{34, 10}, {102, -10}}, pattern = LinePattern.None, fillColor = {135, 135, 135},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Rectangle(extent = {{-10, -100}, {10, -80}}, pattern = LinePattern.None, fillColor = {175, 175, 175},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Rectangle(extent = {{-16, -36}, {10, -40}}, pattern = LinePattern.None, fillColor = {175, 175, 175},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 120}, {150, 80}}, textString = "%name", lineColor = {0, 0, 255})}),
          Documentation(info = "<html>
<p>The differential gear is a 1D-rotational component. It is a variant of a planetary gear and can be used to distribute the torque equally among the wheels on one axis.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
      end DifferentialGear;

      package Wheels "Wheel and tire models"
        extends Modelica.Icons.Package;

        model IdealWheelJoint "Ideal wheel joint"
          Interfaces.Frame_a frame_a annotation (
            Placement(transformation(extent = {{-56, -16}, {-24, 16}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(extent = {{90, -8}, {110, 12}}), iconTransformation(extent = {{90, -10}, {110, 10}})));
          outer PlanarWorld planarWorld "planar world model";
          parameter StateSelect stateSelect = StateSelect.default "Priority to use acceleration as states" annotation (
            HideResult = true,
            Dialog(tab = "Advanced"));
          parameter SI.Length radius "Radius of the wheel";
          parameter SI.Length r[2] "Driving direction of the wheel at angle phi = 0";
          final parameter SI.Length l = sqrt(r * r) "Length of vector r";
          final parameter Real e[2] = r / l "Normalized direction";
          Real e0[2] "Normalized direction w.r.t inertial system";
          Real R[2, 2] "Rotation Matrix";
          SI.Angle phi_roll(start = 0) "Roll angle of the wheel" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.AngularVelocity w_roll(final stateSelect = stateSelect, start = 0) "Roll velocity of wheel" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v[2] "Velocity";
          SI.Velocity v_long "Driving velocity in (longitudinal) driving direction";
          SI.Acceleration a(stateSelect = stateSelect, start = 0) "Acceleration of driving velocity" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Force f_long "Longitudinal force";
          parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
            Dialog(group = "Animation"));
          parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of the body" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          parameter SI.Length diameter = 0.1 "Diameter of the rims" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          parameter SI.Length width = diameter * 0.6 "Width of the wheel" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = {63, 63, 63}, specularCoefficient = specularCoefficient, length = width, width = radius * 2, height = radius * 2, lengthDirection = {-e0[2], e0[1], 0}, widthDirection = {0, 0, 1}, r_shape = -0.03 * {-e0[2], e0[1], 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
          MB.Visualizers.Advanced.Shape rim1(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = specularCoefficient, length = radius * 2, width = diameter, height = diameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -radius}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({-e0[2], e0[1], 0}, flange_a.phi, 0))) if planarWorld.enableAnimation and animate;
          MB.Visualizers.Advanced.Shape rim2(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = specularCoefficient, length = radius * 2, width = diameter, height = diameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -radius}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({-e0[2], e0[1], 0}, flange_a.phi + Modelica.Constants.pi / 2, 0))) if planarWorld.enableAnimation and animate;
        equation
          R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi), cos(frame_a.phi)}};
          e0 = R * e;
          v = der({frame_a.x, frame_a.y});
          v = v_long * e0;
          phi_roll = flange_a.phi;
          w_roll = der(phi_roll);
          v_long = radius * w_roll;
          a = der(v_long);
          -f_long * radius = flange_a.tau;
          frame_a.t = 0;
          {frame_a.fx, frame_a.fy} * e0 = f_long;
          annotation (
            Icon(graphics={  Rectangle(extent = {{-40, 100}, {40, -100}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                                 FillPattern.HorizontalCylinder, fillColor = {231, 231, 231}), Line(points = {{-40, 30}, {40, 30}}, color = {95, 95, 95}), Line(points = {{-40, -30}, {40, -30}}, color = {95, 95, 95}), Line(points = {{-40, 60}, {40, 60}}, color = {95, 95, 95}), Line(points = {{-40, 80}, {40, 80}}, color = {95, 95, 95}), Line(points = {{-40, 90}, {40, 90}}, color = {95, 95, 95}), Line(points = {{-40, 100}, {40, 100}}, color = {95, 95, 95}), Line(points = {{-40, -80}, {40, -80}}, color = {95, 95, 95}), Line(points = {{-40, -90}, {40, -90}}, color = {95, 95, 95}), Line(points = {{-40, -100}, {40, -100}}, color = {95, 95, 95}), Line(points = {{-40, -60}, {40, -60}}, color = {95, 95, 95}), Rectangle(extent = {{100, 10}, {40, -10}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {231, 231, 231}), Text(extent = {{-150, -110}, {150, -140}}, lineColor = {0, 0, 0}, textString = "radius=%radius"), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255})}),
            Documentation(info = "<html>
<p>The ideal wheel joint enforces the constraints of ideal rolling on the x,y-plane.</p>
<p>The constraint is that the velocity of the virtual point of contact shall be zero. This constrains is split into two components:</p>
<ul>
<li>no lateral velocity</li>
<li>the longitudinal velocity has to equal the rolling velocity times the radius.</li>
</ul>
<p>The radius of the wheel can be specified by the parameter <b>radius</b>. The driving direction (for phi=0) can be specified by the parameter <b>r</b>.</p>
<p>The wheel contains a 2D frame connector for the steering on the plane. The rolling motion of the wheel can be actuated by the 1D flange connector.</p>
<p>For examples of usage see the local <a href=\"modelica://PlanarMechanics.VehicleComponents.Examples\">Examples package</a>.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end IdealWheelJoint;

        model DryFrictionWheelJoint "Dry-Friction based wheel joint"
          extends
            Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(      final T = 293.15);
          Interfaces.Frame_a frame_a annotation (
            Placement(transformation(extent = {{-56, -16}, {-24, 16}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(extent = {{90, -8}, {110, 12}}), iconTransformation(extent = {{90, -10}, {110, 10}})));
          outer PlanarWorld planarWorld "planar world model";
          parameter StateSelect stateSelect = StateSelect.default "Priority to use acceleration as states" annotation (
            HideResult = true,
            Dialog(tab = "Advanced"));
          parameter SI.Length radius "Radius of the wheel";
          parameter SI.Length r[2] "Driving direction of the wheel at angle phi = 0";
          parameter SI.Force N "Normal force";
          parameter SI.Velocity vAdhesion "Adhesion velocity";
          parameter SI.Velocity vSlide "Sliding velocity";
          parameter Real mu_A "Friction coefficient at adhesion";
          parameter Real mu_S "Friction coefficient at sliding";
          final parameter SI.Length l = sqrt(r * r) "Length of vector r";
          final parameter Real e[2] = r / l "Normalized direction";
          Real e0[2] "Normalized direction w.r.t inertial system";
          Real R[2, 2] "Rotation Matrix";
          SI.Angle phi_roll(stateSelect = stateSelect, start = 0) "Roll angle of the wheel" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.AngularVelocity w_roll(final stateSelect = stateSelect, start = 0) "Roll velocity of wheel" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v[2] "Velocity";
          SI.Velocity v_lat(start = 0) "Driving in lateral direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_long(start = 0) "Velocity in longitudinal direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_slip_long(start = 0) "Slip velocity in longitudinal direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_slip_lat(start = 0) "Slip velocity in lateral direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_slip "Slip velocity";
          SI.Force f "Longitudinal force";
          SI.Force f_lat "Longitudinal force";
          SI.Force f_long "Longitudinal force";
          parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
            Dialog(group = "Animation"));
          parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of the body" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          parameter SI.Length diameter = 0.1 "Diameter of the rims" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          parameter SI.Length width = diameter * 0.6 "Width of the wheel" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = {63, 63, 63}, specularCoefficient = specularCoefficient, length = width, width = radius * 2, height = radius * 2, lengthDirection = {-e0[2], e0[1], 0}, widthDirection = {0, 0, 1}, r_shape = -0.03 * {-e0[2], e0[1], 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
          MB.Visualizers.Advanced.Shape rim1(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = specularCoefficient, length = radius * 2, width = diameter, height = diameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -radius}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({-e0[2], e0[1], 0}, flange_a.phi, 0))) if planarWorld.enableAnimation and animate;
          MB.Visualizers.Advanced.Shape rim2(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = specularCoefficient, length = radius * 2, width = diameter, height = diameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -radius}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({-e0[2], e0[1], 0}, flange_a.phi + Modelica.Constants.pi / 2, 0))) if planarWorld.enableAnimation and animate;
        equation
          R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi), cos(frame_a.phi)}};
          e0 = R * e;
          v = der({frame_a.x, frame_a.y});
          phi_roll = flange_a.phi;
          w_roll = der(phi_roll);
          v_long = v * e0;
          v_lat = (-v[1] * e0[2]) + v[2] * e0[1];
          v_slip_lat = v_lat - 0;
          v_slip_long = v_long - radius * w_roll;
          v_slip = sqrt(v_slip_long ^ 2 + v_slip_lat ^ 2) + 0.0001;
          -f_long * radius = flange_a.tau;
          frame_a.t = 0;
          f = N * noEvent(Utilities.Functions.limitByStriple(vAdhesion, vSlide, mu_A, mu_S, v_slip));
          f_long = f * v_slip_long / v_slip;
          f_lat = f * v_slip_lat / v_slip;
          f_long = {frame_a.fx, frame_a.fy} * e0;
          f_lat = {frame_a.fy, -frame_a.fx} * e0;
          lossPower = f * v_slip;
          annotation (
            Icon(graphics={  Rectangle(extent = {{-40, 100}, {40, -100}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                                 FillPattern.HorizontalCylinder, fillColor = {231, 231, 231}), Line(points = {{-40, 30}, {40, 30}}, color = {95, 95, 95}), Line(points = {{-40, -30}, {40, -30}}, color = {95, 95, 95}), Line(points = {{-40, 60}, {40, 60}}, color = {95, 95, 95}), Line(points = {{-40, 80}, {40, 80}}, color = {95, 95, 95}), Line(points = {{-40, 90}, {40, 90}}, color = {95, 95, 95}), Line(points = {{-40, 100}, {40, 100}}, color = {95, 95, 95}), Line(points = {{-40, -80}, {40, -80}}, color = {95, 95, 95}), Line(points = {{-40, -90}, {40, -90}}, color = {95, 95, 95}), Line(points = {{-40, -100}, {40, -100}}, color = {95, 95, 95}), Line(points = {{-40, -60}, {40, -60}}, color = {95, 95, 95}), Rectangle(extent = {{100, 10}, {40, -10}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {231, 231, 231}), Line(visible = useHeatPort, points = {{-100, -100}, {-100, -90}, {0, -90}}, color = {191, 0, 0}, pattern = LinePattern.Dot, smooth = Smooth.None), Text(extent = {{-150, -110}, {150, -140}}, lineColor = {0, 0, 0}, textString = "radius=%radius"), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255})}),
            Documentation(info = "<html>
<p>The ideal wheel joint models the behavior of a wheel rolling on a x,y-plane whose contact patch has dry-friction characteristics. This is an approximation for stiff wheels without a tire.</p>
<p>The force depends with dry-friction characteristics on the slip velocity. The slip velocity is split into two components:</p>
<ul>
<li>the lateral velocity</li>
<li>the longitudinal velocity minus the rolling velocity times the radius.</li>
</ul>
<p>The radius of the wheel can be specified by the parameter <b>radius</b>. The driving direction (for phi=0) can be specified by the parameter <b>r</b>. The normal load is set by <b>N</b>.</p>
<p>The wheel contains a 2D connector <b>frame_a</b> for the steering on the plane. The rolling motion of the wheel can be actuated by the 1D  connector <b>flange_a</b>.</p>
<p>For examples of usage see the local <a href=\"modelica://PlanarMechanics.VehicleComponents.Examples\">Examples package</a>.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end DryFrictionWheelJoint;

        model SlipBasedWheelJoint "Slip-Friction based wheel joint"
          extends
            Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(      final T = 293.15);
          Interfaces.Frame_a frame_a annotation (
            Placement(transformation(extent = {{-56, -16}, {-24, 16}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(extent = {{90, -10}, {110, 10}})));
          Modelica.Blocks.Interfaces.RealInput dynamicLoad(unit = "N") annotation (
            Placement(transformation(extent = {{20, -20}, {-20, 20}}, rotation = 270, origin = {0, -100})));
          outer PlanarWorld planarWorld "planar world model";
          parameter StateSelect stateSelect = StateSelect.default "Priority to use acceleration as states" annotation (
            HideResult = true,
            Dialog(tab = "Advanced"));
          parameter SI.Length radius "Radius of the wheel";
          parameter SI.Length r[2] "Driving direction of the wheel at angle phi = 0";
          parameter SI.Force N "Base normal load";
          parameter SI.Velocity vAdhesion_min "Minimum adhesion velocity";
          parameter SI.Velocity vSlide_min "Minimum sliding velocity";
          parameter Real sAdhesion "Adhesion slippage";
          parameter Real sSlide "Sliding slippage";
          parameter Real mu_A "Friction coefficient at adhesion";
          parameter Real mu_S "Friction coefficient at sliding";
          final parameter SI.Length l = sqrt(r * r) "Length of vector r";
          final parameter Real e[2] = r / l "Normalized direction";
          Real e0[2] "Normalized direction w.r.t inertial system";
          Real R[2, 2] "Rotation Matrix";
          SI.Angle phi_roll(stateSelect = stateSelect, start = 0) "Roll angle of the wheel" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.AngularVelocity w_roll(final stateSelect = stateSelect, start = 0) "Roll velocity of wheel" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v[2] "velocity";
          SI.Velocity v_lat(start = 0) "Driving in lateral direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_long(start = 0) "Velocity in longitudinal direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_slip_long(start = 0) "Slip velocity in longitudinal direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_slip_lat(start = 0) "Slip velocity in lateral direction" annotation (
            Dialog(group = "Initialization", showStartAttribute = true));
          SI.Velocity v_slip "Slip velocity";
          SI.Force f "Longitudinal force";
          SI.Force f_lat "Longitudinal force";
          SI.Force f_long "Longitudinal force";
          SI.Force fN "Base normal load";
          SI.Velocity vAdhesion "Adhesion velocity";
          SI.Velocity vSlide "Sliding velocity";
          parameter Boolean animate = true "= true, if animation shall be enabled" annotation (
            Dialog(group = "Animation"));
          parameter SI.Length zPosition = planarWorld.defaultZPosition "Position z of the body" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          parameter SI.Length diameter = 0.1 "Diameter of the rims" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          parameter SI.Length width = diameter * 0.6 "Width of the wheel" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation (
            Dialog(tab = "Animation", group = "if animation = true", enable = animate));
          MB.Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = {63, 63, 63}, specularCoefficient = specularCoefficient, length = width, width = radius * 2, height = radius * 2, lengthDirection = {-e0[2], e0[1], 0}, widthDirection = {0, 0, 1}, r_shape = -0.03 * {-e0[2], e0[1], 0}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = planarWorld.R) if planarWorld.enableAnimation and animate;
          MB.Visualizers.Advanced.Shape rim1(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = specularCoefficient, length = radius * 2, width = diameter, height = diameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -radius}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({-e0[2], e0[1], 0}, flange_a.phi, 0))) if planarWorld.enableAnimation and animate;
          MB.Visualizers.Advanced.Shape rim2(shapeType = "cylinder", color = {195, 195, 195}, specularCoefficient = specularCoefficient, length = radius * 2, width = diameter, height = diameter, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, -radius}, r = MB.Frames.resolve1(planarWorld.R, {frame_a.x, frame_a.y, zPosition}) + planarWorld.r_0, R = MB.Frames.absoluteRotation(planarWorld.R, MB.Frames.planarRotation({-e0[2], e0[1], 0}, flange_a.phi + Modelica.Constants.pi / 2, 0))) if planarWorld.enableAnimation and animate;
        equation
          R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi), cos(frame_a.phi)}};
          e0 = R * e;
          v = der({frame_a.x, frame_a.y});
          phi_roll = flange_a.phi;
          w_roll = der(phi_roll);
          v_long = v * e0;
          v_lat = (-v[1] * e0[2]) + v[2] * e0[1];
          v_slip_lat = v_lat - 0;
          v_slip_long = v_long - radius * w_roll;
          v_slip = sqrt(v_slip_long ^ 2 + v_slip_lat ^ 2) + 0.0001;
          -f_long * radius = flange_a.tau;
          frame_a.t = 0;
          vAdhesion = noEvent(max(sAdhesion * abs(radius * w_roll), vAdhesion_min));
          vSlide = noEvent(max(sSlide * abs(radius * w_roll), vSlide_min));
          fN = max(0, N + dynamicLoad);
          f = fN * noEvent(Utilities.Functions.limitByStriple(vAdhesion, vSlide, mu_A, mu_S, v_slip));
          f_long = f * v_slip_long / v_slip;
          f_lat = f * v_slip_lat / v_slip;
          f_long = {frame_a.fx, frame_a.fy} * e0;
          f_lat = {frame_a.fy, -frame_a.fx} * e0;
          lossPower = f * v_slip;
          annotation (
            Icon(graphics={  Rectangle(extent = {{-40, 100}, {40, -100}}, lineColor = {95, 95, 95},
                    fillPattern =                                                                                 FillPattern.HorizontalCylinder, fillColor = {231, 231, 231}), Line(points = {{-40, 30}, {40, 30}}, color = {95, 95, 95}), Line(points = {{-40, -30}, {40, -30}}, color = {95, 95, 95}), Line(points = {{-40, 60}, {40, 60}}, color = {95, 95, 95}), Line(points = {{-40, 80}, {40, 80}}, color = {95, 95, 95}), Line(points = {{-40, 90}, {40, 90}}, color = {95, 95, 95}), Line(points = {{-40, 100}, {40, 100}}, color = {95, 95, 95}), Line(points = {{-40, -80}, {40, -80}}, color = {95, 95, 95}), Line(points = {{-40, -90}, {40, -90}}, color = {95, 95, 95}), Line(points = {{-40, -100}, {40, -100}}, color = {95, 95, 95}), Rectangle(extent = {{100, 10}, {40, -10}},
                    fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {231, 231, 231}), Line(visible = useHeatPort, points = {{-100, -100}, {-100, -90}, {-30, -90}}, color = {191, 0, 0}, pattern = LinePattern.Dot, smooth = Smooth.None), Text(extent = {{-150, -40}, {150, -70}}, lineColor = {0, 0, 0}, textString = "radius=%radius"), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255})}),
            Documentation(info = "<html>
<p>The ideal wheel joint models the behavior of a wheel rolling on a x,y-plane whose contact patch has slip-dependent friction characteristics. This is an approximation for wheels with a rim and a rupper tire.</p>
<p>The force depends with friction characteristics on the <b>slip</b>. The <b>slip</b> is split into two components:</p>
<ul>
<li>lateral slip: the lateral velocity divided by the rolling velocity.</li>
<li>longitudinal slip: the longitudinal slip velocity divided by the rolling velocity.</li>
</ul>
<p>For low rolling velocity this definitions become ill-conditioned. Hence a dry-friction model is used for low rolling velocities.</p>
<p>The radius of the wheel can be specified by the parameter <b>radius</b>. The driving direction (for phi=0) can be specified by the parameter <b>r</b>. The normal load is set by <b>N</b>.</p>
<p>The wheel contains a 2D connector <b>frame_a</b> for the steering on the plane. The rolling motion of the wheel can be actuated by the 1D  connector <b>flange_a</b>.</p>
<p>In addition there is an input for a dynamic component of the normal load.</p>
<p>For examples of usage see the local <a href=\"modelica://PlanarMechanics.VehicleComponents.Examples\">Examples package</a>.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
        end SlipBasedWheelJoint;
        annotation (
          Documentation(info = "<html>
<p>This package contains wheel models that roll on the x,y plane. The wheel models are hereby represented as a mass-free element similar to a joint component.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"),Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(extent = {{-70, 70}, {70, -70}}, fillColor = {215, 215, 215},
                  fillPattern =                                                                                                                                                                    FillPattern.Sphere), Ellipse(extent = {{-50, 50}, {50, -50}}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, pattern = LinePattern.None), Ellipse(extent = {{-10, 10}, {10, -10}}, lineColor = {0, 0, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid)}));
      end Wheels;
      annotation (
        Documentation(info = "<html>
<p>This package contains elements useful for modeling vehicles that roll on the xy-plane.
Essentially, there are various wheel models and a differential gear.</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
    end VehicleComponents;

    package Visualizers "Visualization components of the library"
      extends Modelica.Icons.Package;

      package Advanced "Visualizers that require advanced knowledge in order to use them properly"
        extends Modelica.Icons.Package;

        model Arrow "Visualizing an arrow with variable size; all data have to be set as modifiers (see info layer)"
          input MB.Frames.Orientation R = MB.Frames.nullRotation() "Orientation object to rotate the planarWorld frame into the arrow frame" annotation (
            Dialog);
          input SI.Position r[3] = {0, 0, 0} "Position vector from origin of planarWorld frame to origin of arrow frame, resolved in planarWorld frame" annotation (
            Dialog);
          input SI.Position r_tail[3] = {0, 0, 0} "Position vector from origin of arrow frame to arrow tail, resolved in arrow frame" annotation (
            Dialog);
          input SI.Position r_head[3] = {0, 0, 0} "Position vector from arrow tail to the head of the arrow, resolved in arrow frame" annotation (
            Dialog);
          input SI.Diameter diameter = planarWorld.defaultArrowDiameter "Diameter of arrow line" annotation (
            Dialog);
          input PlanarMechanics.Types.Color color = PlanarMechanics.Types.Defaults.ArrowColor "Color of arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Material property describing the reflecting of ambient light (= 0 means, that light is completely absorbed)" annotation (
            HideResult = true,
            Dialog);
        protected
          outer PlanarWorld planarWorld;
          Internal.Arrow arrow(R = R, r = r, r_tail = r_tail, r_head = r_head, diameter = diameter, color = color, specularCoefficient = specularCoefficient) if planarWorld.enableAnimation;
          annotation (
            Documentation(info = "<html>
<p>
Model <b>Arrow</b> defines an arrow that is dynamically
visualized at the defined location (see variables below).
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Arrow.png\" ALT=\"model Visualizers.Advanced.Arrow\">
</p>

<p>
The variables under heading <b>Parameters</b> below
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where an <b>Arrow</b> instance is used, e.g., in the form
</p>
<blockquote><pre>
Visualizers.Advanced.Arrow arrow(diameter = sin(time));
</pre></blockquote>

<p>
Variable <b>color</b> is a RGB color space given in the range 0 .. 255.
The predefined type <a href=\"modelica://PlanarMechanics.Types.Color\">Types.Color</a>
contains a menu definition of the colors used in the library</a>
(will be replaced by a color editor).
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 30}, {20, -30}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                    fillPattern =                                                                                                                                                                                                   FillPattern.Solid), Polygon(points = {{20, 60}, {100, 0}, {20, -60}, {20, 60}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255})}));
        end Arrow;

        model DoubleArrow "Visualizing a double arrow with variable size; all data have to be set as modifiers (see info layer)"
          import Modelica.Mechanics.MultiBody.Types;
          import Modelica.Mechanics.MultiBody.Frames;
          import T = Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
          import Modelica.SIunits.Conversions.to_unit1;
          input Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the planarWorld frame into the arrow frame" annotation (
            Dialog);
          input SI.Position r[3] = {0, 0, 0} "Position vector from origin of planarWorld frame to origin of arrow frame, resolved in planarWorld frame" annotation (
            Dialog);
          input SI.Position r_tail[3] = {0, 0, 0} "Position vector from origin of arrow frame to double arrow tail, resolved in arrow frame" annotation (
            Dialog);
          input SI.Position r_head[3] = {0, 0, 0} "Position vector from double arrow tail to the head of the double arrow, resolved in arrow frame" annotation (
            Dialog);
          input SI.Diameter diameter = planarWorld.defaultArrowDiameter "Diameter of arrow line" annotation (
            Dialog);
          input PlanarMechanics.Types.Color color = PlanarMechanics.Types.Defaults.ArrowColor "Color of double arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = planarWorld.defaultSpecularCoefficient "Material property describing the reflecting of ambient light (= 0 means, that light is completely absorbed)" annotation (
            HideResult = true,
            Dialog);
        protected
          outer PlanarWorld planarWorld;
          SI.Length length = Modelica.Math.Vectors.length(r_head) "Length of arrow";
          Real e_x[3](each final unit = "1", start = {1, 0, 0}) = noEvent(if length < 1.e-10 then {1, 0, 0} else r_head / length);
          Real rxvisobj[3](each final unit = "1") = transpose(R.T) * e_x "X-axis unit vector of shape, resolved in planarWorld frame" annotation (
            HideResult = true);
          SI.Position rvisobj[3] = r + T.resolve1(R.T, r_tail) "Position vector from planarWorld frame to shape frame, resolved in planarWorld frame" annotation (
            HideResult = true);
          SI.Length headLength = noEvent(max(0, length - arrowLength)) annotation (
            HideResult = true);
          SI.Length headWidth = noEvent(max(0, diameter * Types.Defaults.ArrowHeadWidthFraction)) annotation (
            HideResult = true);
          SI.Length arrowLength = noEvent(max(0, length - 1.5 * diameter * Types.Defaults.ArrowHeadLengthFraction)) annotation (
            HideResult = true);
          MB.Visualizers.Advanced.Shape arrowLine(length = arrowLength, width = diameter, height = diameter, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cylinder", color = color, specularCoefficient = specularCoefficient, r_shape = r_tail, r = r, R = R) if planarWorld.enableAnimation;
          MB.Visualizers.Advanced.Shape arrowHead1(length = 2 / 3 * headLength, width = headWidth, height = headWidth, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cone", color = color, specularCoefficient = specularCoefficient, r = rvisobj + rxvisobj * arrowLength, R = R) if planarWorld.enableAnimation;
          MB.Visualizers.Advanced.Shape arrowHead2(length = 2 / 3 * headLength, width = headWidth, height = headWidth, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cone", color = color, specularCoefficient = specularCoefficient, r = rvisobj + rxvisobj * (arrowLength + headLength / 3), R = R) if planarWorld.enableAnimation;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 30}, {0, -30}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                    fillPattern =                                                                                                                                                                                                  FillPattern.Solid), Polygon(points = {{40, 60}, {100, 0}, {40, -60}, {40, 60}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{0, 60}, {60, 0}, {0, -60}, {0, 60}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255})}),
            Documentation(info = "<html>
<p>
Model <b>DoubleArrow</b> defines a double arrow that is dynamically
visualized at the defined location (see variables below).
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/DoubleArrow.png\" ALT=\"model Visualizers.Advanced.DoubleArrow\">
</p>

<p>
The variables under heading <b>Parameters</b> below
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where an <b>Arrow</b> instance is used, e.g., in the form
</p>
<pre>
    Visualizers.Advanced.DoubleArrow doubleArrow(diameter = sin(time));
</pre>
<p>
Variable <b>color</b> is a RGB color space given in the range 0 .. 255.
The predefined type <a href=\"modelica://PlanarMechanics.Types.Color\">Types.Color</a>
contains a menu definition of the colors used in the library</a>
(will be replaced by a color editor).
</p>
</html>"));
        end DoubleArrow;

        model CoordinateSystem "Visualizing an orthogonal coordinate system of three axes"
          input SI.Position r[3] = {0, 0, 0} "Position vector from origin of world frame to origin of object frame, resolved in world frame" annotation (
            Dialog);
          input MB.Frames.Orientation R = MB.Frames.nullRotation() "Orientation object to rotate the world frame into the object frame" annotation (
            Dialog);
          input SI.Position r_shape[3] = {0, 0, 0} "Position vector from origin of object frame to shape origin, resolved in object frame" annotation (
            Dialog);
          parameter SI.Length axisLength = planarWorld.nominalLength / 2 "Length of world axes arrows";
          parameter SI.Diameter axisDiameter = axisLength / planarWorld.defaultFrameDiameterFraction "Diameter of world axes arrows";
          parameter PlanarMechanics.Types.Color color_x = PlanarMechanics.Types.Defaults.FrameColor "Color of x-arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          parameter PlanarMechanics.Types.Color color_y = color_x "Color of y-arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          parameter PlanarMechanics.Types.Color color_z = color_x "Color of z-arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          parameter Boolean axisShowLabels = true "True, if labels shall be shown" annotation (
            HideResult = true,
            Dialog(group = "Axes labels"));
          parameter SI.Length labelStart = 1.05 * axisLength annotation (
            Dialog(group = "Axes labels", enable = axisShowLabels));
          parameter SI.Length scaledLabel = PlanarMechanics.Types.Defaults.FrameLabelHeightFraction * axisDiameter annotation (
            Dialog(group = "Axes labels", enable = axisShowLabels));
        protected
          outer .Praktikum11.PlanarMechanics.PlanarWorld planarWorld;
          Internal.CoordinateSystem coordinateSystem(final r = r, final R = R, final r_shape = r_shape, final axisLength = axisLength, final axisDiameter = axisDiameter, final color_x = color_x, final color_y = color_y, final color_z = color_z, final axisShowLabels = axisShowLabels, final labelStart = labelStart, final scaledLabel = scaledLabel) if planarWorld.enableAnimation;
          annotation (
            Documentation(info = "<html>
<p>
This element enbles visualization of an <b>orthogonal coordinate system</b>
as shown in the following picture.
</p>

<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/Visualizers/Advanced/CoordinateSystem.png\" ALT=\"model Visualizers.Advanced.CoordinateSystem\">
</p>

<p>
The variables <code>r</code>, <code>R</code> and <code>r_shape</code>
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where a <b>CoordinateSystem</b> instance is used, e.g., in the form
</p>
<blockquote><pre>
PlanarMechanics.Visualizers.Advanced.CoordinateSystem coordinateSystem(r = {sin(time), 0, 0.3});
</pre></blockquote>

<p>
<b>Color</b> of each axis can be set individually using RGB color space given in the range 0 .. 255.
The predefined type <a href=\"modelica://PlanarMechanics.Types.Color\">Types.Color</a>
contains a menu definition of the colors used in the library</a>
(will be replaced by a color editor).
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-150, 100}, {150, 60}}, lineColor = {0, 0, 255}, textString = "%name"), Polygon(points = {{-50, -90}, {0, -72}, {50, -90}, {90, -70}, {26, -16}, {8, 40}, {-8, 40}, {-28, -16}, {-90, -70}, {-50, -90}}, lineColor = {255, 255, 255}, smooth = Smooth.None, fillColor = {255, 255, 255},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{0, -30}, {0, 40}}, color = {135, 135, 135}, thickness = 0.5), Line(points = {{0, -30}, {74, -66}}, color = {135, 135, 135}, thickness = 0.5), Line(points = {{-70, -66}, {0, -30}}, thickness = 0.5, color = {135, 135, 135}), Text(extent = {{-69, -54}, {-20, -84}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, textString = "x"), Polygon(points = {{-96, -80}, {-64, -72}, {-76, -58}, {-96, -80}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{100, -82}, {76, -56}, {66, -72}, {100, -82}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{25, -54}, {74, -84}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, textString = "z"), Text(extent = {{10, 60}, {61, 30}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, textString = "y"), Polygon(points = {{-14, 22}, {14, 22}, {0, 62}, {-14, 22}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid)}));
        end CoordinateSystem;
        annotation (
          Documentation(info = "<html>
<p>
This package contains components to visualize 2-dimensional shapes with dynamical sizes in 3-dimensional world. None of the components
has a frame connector. The position and orientation is set via modifiers. Advanced knowledge of Modelica
is needed in order to utilize the components of this package.
</p>
</html>"));
      end Advanced;

      package Internal "Collection of internal visualizers, should not be used by user"
        extends Modelica.Icons.InternalPackage;

        model Arrow "Visualizing an arrow with variable size; all data have to be set as modifiers (see info layer)"
          import T = Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
          import Modelica.SIunits.Conversions.to_unit1;
          input MB.Frames.Orientation R = MB.Frames.nullRotation() "Orientation object to rotate the planarWorld frame into the arrow frame" annotation (
            Dialog(enable = true));
          input SI.Position r[3] = {0, 0, 0} "Position vector from origin of planarWorld frame to origin of arrow frame, resolved in planarWorld frame" annotation (
            Dialog(enable = true));
          input SI.Position r_tail[3] = {0, 0, 0} "Position vector from origin of arrow frame to arrow tail, resolved in arrow frame" annotation (
            Dialog(enable = true));
          input SI.Position r_head[3] = {0, 0, 0} "Position vector from arrow tail to the head of the arrow, resolved in arrow frame" annotation (
            Dialog(enable = true));
          input SI.Diameter diameter = 1 / 40 "Diameter of arrow line" annotation (
            Dialog(enable = true));
          input PlanarMechanics.Types.Color color = PlanarMechanics.Types.Defaults.ArrowColor "Color of arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          input PlanarMechanics.Types.SpecularCoefficient specularCoefficient = 0.7 "Material property describing the reflecting of ambient light (= 0 means, that light is completely absorbed)" annotation (
            HideResult = true,
            Dialog(enable = true));
        protected
          SI.Length length = Modelica.Math.Vectors.length(r_head) "Length of arrow";
          Real e_x[3](each final unit = "1", start = {1, 0, 0}) = noEvent(if length < 1.e-10 then {1, 0, 0} else r_head / length);
          Real rxvisobj[3](each final unit = "1") = transpose(R.T) * e_x "X-axis unit vector of shape, resolved in planarWorld frame" annotation (
            HideResult = true);
          SI.Position rvisobj[3] = r + T.resolve1(R.T, r_tail) "Position vector from planarWorld frame to shape frame, resolved in planarWorld frame" annotation (
            HideResult = true);
          SI.Length headLength = noEvent(max(0, length - arrowLength)) annotation (
            HideResult = true);
          SI.Length headWidth = noEvent(max(0, diameter * PlanarMechanics.Types.Defaults.ArrowHeadWidthFraction)) annotation (
            HideResult = true);
          SI.Length arrowLength = noEvent(max(0, length - diameter * PlanarMechanics.Types.Defaults.ArrowHeadLengthFraction)) annotation (
            HideResult = true);
          MB.Visualizers.Advanced.Shape arrowLine(length = arrowLength, width = diameter, height = diameter, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cylinder", color = color, specularCoefficient = specularCoefficient, r_shape = r_tail, r = r, R = R);
          MB.Visualizers.Advanced.Shape arrowHead(length = headLength, width = headWidth, height = headWidth, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cone", color = color, specularCoefficient = specularCoefficient, r = rvisobj + rxvisobj * arrowLength, R = R);
          annotation (
            Documentation(info = "<html>
<p>
<b>Note</b>: This element is intended to be used in <a href=\"PlanarMechanics.Visualizers.Internal.CoordinateSystem\">CoordinateSystem</a> only!
To visualize an arrow in your model, the best solution is usually to use the visualizer <a href=\"PlanarMechanics.Visualizers.Advanced.Arrow\">Advanced.Arrow</a>.
The only difference between this two visualizers is that the current one does not utilizes <b>outer planarWorld</b>, 
whereas the other does.
</p>
<p>
Model <b>Arrow</b> defines an arrow that is dynamically
visualized at the defined location (see variables below).
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Arrow.png\" ALT=\"model Visualizers.Advanced.Arrow\">
</p>

<p>
The variables under heading <b>Parameters</b> below
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where an <b>Arrow</b> instance is used, e.g., in the form
</p>
<blockquote><pre>
Visualizers.Advanced.Arrow arrow(diameter = sin(time));
</pre></blockquote>

<p>
Variable <b>color</b> is a RGB color space given in the range 0 .. 255.
The predefined type <a href=\"modelica://PlanarMechanics.Types.Color\">Types.Color</a>
contains a menu definition of the colors used in the library</a>
(will be replaced by a color editor).
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 28}, {20, -30}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                    fillPattern =                                                                                                                                                                                                   FillPattern.Solid), Polygon(points = {{20, 60}, {100, 0}, {20, -60}, {20, 60}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{-150, 100}, {150, 60}}, textString = "%name", lineColor = {0, 0, 255})}));
        end Arrow;

        model CoordinateSystem "Visualizing an orthogonal coordinate system of three axes"
          input SI.Position r[3] = {0, 0, 0} "Position vector from origin of world frame to origin of object frame, resolved in world frame" annotation (
            Dialog);
          input MB.Frames.Orientation R = MB.Frames.nullRotation() "Orientation object to rotate the world frame into the object frame" annotation (
            Dialog);
          input SI.Position r_shape[3] = {0, 0, 0} "Position vector from origin of object frame to shape origin, resolved in object frame" annotation (
            Dialog);
          parameter SI.Length axisLength = 0.5 "Length of world axes arrows";
          parameter SI.Diameter axisDiameter = axisLength / 40 "Diameter of world axes arrows";
          parameter PlanarMechanics.Types.Color color_x = PlanarMechanics.Types.Defaults.FrameColor "Color of x-arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          parameter PlanarMechanics.Types.Color color_y = color_x "Color of y-arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          parameter PlanarMechanics.Types.Color color_z = color_x "Color of z-arrow" annotation (
            HideResult = true,
            Dialog(colorSelector = true));
          parameter Boolean axisShowLabels = true "True, if labels shall be shown" annotation (
            HideResult = true,
            Dialog(group = "Axes labels"));
          parameter SI.Length labelStart = 1.05 * axisLength annotation (
            Dialog(group = "Axes labels", enable = axisShowLabels));
          parameter SI.Length scaledLabel = PlanarMechanics.Types.Defaults.FrameLabelHeightFraction * axisDiameter annotation (
            Dialog(group = "Axes labels", enable = axisShowLabels));
        protected
          Arrow x_arrow(R = R, r = r, r_tail = {0, 0, 0}, r_head = axisLength * {1, 0, 0}, diameter = axisDiameter, color = color_x, specularCoefficient = 0);
          MB.Visualizers.Internal.Lines x_label(R = R, r = r, r_lines = {labelStart, 0, 0}, lines = scaledLabel * {[0, 0; 1, 1], [0, 1; 1, 0]}, diameter = axisDiameter, n_x = {1, 0, 0}, n_y = {0, 1, 0}, color = color_x, specularCoefficient = 0) if axisShowLabels;
          Arrow y_arrow(R = R, r = r, r_tail = {0, 0, 0}, r_head = axisLength * {0, 1, 0}, diameter = axisDiameter, color = color_y, specularCoefficient = 0);
          MB.Visualizers.Internal.Lines y_label(R = R, r = r, r_lines = {0, labelStart, 0}, lines = scaledLabel * {[0, 0; 1, 1.5], [0, 1.5; 0.5, 0.75]}, diameter = axisDiameter, n_x = {0, 1, 0}, n_y = {-1, 0, 0}, color = color_y, specularCoefficient = 0) if axisShowLabels;
          Arrow z_arrow(R = R, r = r, r_tail = {0, 0, 0}, r_head = axisLength * {0, 0, 1}, diameter = axisDiameter, color = color_z, specularCoefficient = 0);
          MB.Visualizers.Internal.Lines z_label(R = R, r = r, r_lines = {0, 0, labelStart}, lines = scaledLabel * {[0, 0; 1, 0], [0, 1; 1, 1], [0, 1; 1, 0]}, diameter = axisDiameter, n_x = {0, 0, 1}, n_y = {0, 1, 0}, color = color_z, specularCoefficient = 0) if axisShowLabels;
          annotation (
            Documentation(info = "<html>
<p>
<b>Note</b>: This element is intended to be used in <a href=\"PlanarMechanics.PlanarWorld\">PlanarWorld</a> and its derivatives only!
To visualize a coordinate system in your model, the best solution is usually to use the visualizer <a href=\"PlanarMechanics.Visualizers.Advanced.CoordinateSystem\">Advanced.CoordinateSystem</a>.
The only difference between this two visualizers is that the current one does not utilizes <b>outer planarWorld</b>, 
whereas the other does.
</p>
<p>
This element enbles visualization of an <b>orthogonal coordinate system</b>
as shown in the following picture.
</p>

<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/Visualizers/Advanced/CoordinateSystem.png\" ALT=\"model Visualizers.Advanced.CoordinateSystem\">
</p>

<p>
The variables <code>r</code>, <code>R</code> and <code>r_shape</code>
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where a <b>CoordinateSystem</b> instance is used, e.g., in the form
</p>
<blockquote><pre>
PlanarMechanics.Visualizers.Advanced.CoordinateSystem coordinateSystem(r = {sin(time), 0, 0.3});
</pre></blockquote>

<p>
<b>Color</b> of each axis can be set individually using RGB color space given in the range 0 .. 255.
The predefined type <a href=\"modelica://PlanarMechanics.Types.Color\">Types.Color</a>
contains a menu definition of the colors used in the library</a>
(will be replaced by a color editor).
</p>
</html>"),  Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent = {{-150, 100}, {150, 60}}, lineColor = {0, 0, 255}, textString = "%name"), Polygon(points = {{-50, -90}, {0, -72}, {50, -90}, {90, -70}, {26, -16}, {8, 40}, {-8, 40}, {-28, -16}, {-90, -70}, {-50, -90}}, lineColor = {255, 255, 255}, smooth = Smooth.None, fillColor = {255, 255, 255},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{0, -30}, {0, 40}}, color = {135, 135, 135}, thickness = 0.5), Line(points = {{0, -30}, {74, -66}}, color = {135, 135, 135}, thickness = 0.5), Line(points = {{-70, -66}, {0, -30}}, thickness = 0.5, color = {135, 135, 135}), Text(extent = {{-69, -54}, {-20, -84}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, textString = "x"), Polygon(points = {{-96, -80}, {-64, -72}, {-76, -58}, {-96, -80}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{100, -82}, {76, -56}, {66, -72}, {100, -82}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent = {{25, -54}, {74, -84}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, textString = "z"), Text(extent = {{10, 60}, {61, 30}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid, textString = "y"), Polygon(points = {{-14, 22}, {14, 22}, {0, 62}, {-14, 22}}, lineColor = {135, 135, 135}, fillColor = {135, 135, 135},
                    fillPattern =                                                                                                                                                                                                        FillPattern.Solid)}));
        end CoordinateSystem;
        annotation (
          Documentation(info = "<html>
<p>
A collection of internal material for visualizers. Generally, the material collected here should not be used by a common user.
</p>
</html>"));
      end Internal;
      annotation (
        Documentation(info = "<html>
<p>This package provides common animation components which are not integrated in other components.</p>
</html>"));
    end Visualizers;
    annotation (
      preferredView = "info",
      versionBuild = 1,
      versionDate = "2019-03-29",
      dateModified = "2019-03-29 12:00:00Z",
      conversion(noneFromVersion = "1.4.0", from(version = "1.3.0", script = "modelica://PlanarMechanics/Resources/Scripts/Dymola/convertFromPlanarMechanics1_3_0.mos")),
      Documentation(info = "<html>
<p>
Library <b>PlanarMechanics</b> is a <b>free</b> Modelica package providing
2-dimensional mechanical components to model mechanical systems, such as
robots, mechanisms, vehicles, where
<a href=\"modelica://Modelica.Mechanics.MultiBody\">MultiBody</a> library is unnecessarily
complex.
</p>
<p>
In order to know how the library works, first have a look at:
</p>
<ul>
<li><a href=\"modelica://PlanarMechanics.UsersGuide\">UsersGuide</a>
    describes the principle ways to use the library.</li>
<li><a href=\"modelica://PlanarMechanics.Examples\">Examples</a>
    contains examples that demonstrate the usage of the library.</li>
</ul>


<h4>Licensed by DLR e.V. under the 3-Clause BSD License</h4>
<p>
Copyright &copy; 2010-2019, Deutsches Zentrum fuer Luft- und Raumfahrt e.V.
</p>
<p>
<em>This Modelica package is <u>free</u> software and the use is completely
at <u>your own risk</u>; it can be redistributed and/or modified under
the terms of the 3-Clause BSD license. For license conditions (including
the disclaimer of warranty) visit
<a href=\"https://modelica.org/licenses/modelica-3-clause-bsd\">https://modelica.org/licenses/modelica-3-clause-bsd</a>.</em>
</p>
</html>", revisions = "<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"),
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(extent = {{-46, 10}, {-26, -10}},
              fillPattern =                                                                                                           FillPattern.Solid, pattern = LinePattern.None), Ellipse(extent = {{-74, -40}, {-54, -60}},
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, pattern = LinePattern.None), Ellipse(extent = {{18, -34}, {38, -54}},
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, pattern = LinePattern.None), Line(points = {{-64, -50}, {-36, 0}, {24, -42}})}));
  end PlanarMechanics;

  model DriveTrain2D
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = 6) annotation (
      Placement(transformation(extent = {{-6, -10}, {14, 10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
      Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b VR annotation (
      Placement(visible = true, transformation(extent = {{88, -12}, {108, 8}}, rotation = 0), iconTransformation(extent = {{90, 10}, {110, 30}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b VL annotation (
      Placement(transformation(extent = {{90, 50}, {110, 70}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b HL annotation (
      Placement(transformation(extent = {{90, -30}, {110, -10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b HR annotation (
      Placement(transformation(extent = {{90, -70}, {110, -50}})));
  Praktikum11.Differential differential annotation (Placement(visible=true,
          transformation(
          origin={62,30},
          extent={{-10,-10},{10,10}},
          rotation=90)));
  equation
    connect(idealGear.flange_a, flange_a) annotation (
      Line(points = {{-6, 0}, {-100, 0}}, color = {0, 0, 0}));
    connect(HL, HL) annotation (
      Line(points = {{100, -20}, {100, -20}}, color = {0, 0, 0}));
    connect(HR, HL) annotation (
      Line(points = {{100, -60}, {80, -60}, {80, -20}, {100, -20}}, color = {0, 0, 0}));
  connect(idealGear.flange_b, differential.flange_a) annotation (
      Line(points={{14,0},{32,0},{32,30},{52.2,30}}));
  connect(differential.flange_b1, VL) annotation (
      Line(points = {{62, 40}, {62, 60}, {100, 60}}));
  connect(differential.flange_b, VR) annotation (
      Line(points = {{62, 20}, {62, -2}, {98, -2}}));
  annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end DriveTrain2D;

  model Chassis2D
    import SI = Modelica.SIunits;
    parameter SI.Length wheelBase;
    parameter SI.Length trackWidth;
    parameter SI.Length wheelRadius;
    parameter SI.Mass carMass;
    Modelica.Mechanics.Rotational.Interfaces.Flange_a HR annotation (
      Placement(transformation(extent = {{-110, -70}, {-90, -50}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a VL annotation (
      Placement(transformation(extent = {{-110, 50}, {-90, 70}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a VR annotation (
      Placement(transformation(extent = {{-110, 10}, {-90, 30}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a HL annotation (
      Placement(transformation(extent = {{-110, -30}, {-90, -10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a Steering annotation (
      Placement(visible = true, transformation(extent = {{60, 92}, {80, 112}}, rotation = 0), iconTransformation(extent = {{44, 90}, {64, 110}}, rotation = 0)));
    PlanarMechanics.Interfaces.Frame frame annotation (
      Placement(transformation(extent = {{72, -8}, {88, 8}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 1.055) annotation (
      Placement(transformation(extent = {{-78, 50}, {-58, 70}})));
    Praktikum11.PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint dryFrictionWheelJoint1(
      radius=wheelRadius,
      r={1,0},
      N=(carMass/4 + 19)*9.81,
      vAdhesion=0.01,
      vSlide=0.1,
      mu_A=0.85,
      mu_S=0.5) annotation (Placement(visible=true, transformation(extent={{-30,
              10},{-50,30}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1.055) annotation (
      Placement(visible = true, transformation(extent = {{-80, 10}, {-60, 30}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia2(J = 1.055) annotation (
      Placement(transformation(extent = {{-80, -30}, {-60, -10}})));
    PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint dryFrictionWheelJoint3(radius = wheelRadius, r = {1, 0}, N = (carMass / 4 + 19) * 9.81, vAdhesion(displayUnit = "m/s") = 0.01, vSlide = 0.1, mu_A = 0.85, mu_S = 0.5) annotation (
      Placement(transformation(extent = {{-30, -70}, {-50, -50}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia3(J = 1.055) annotation (
      Placement(transformation(extent = {{-80, -70}, {-60, -50}})));
    Praktikum11.PlanarMechanics.Parts.FixedTranslation fixedTranslation1(r={0,-
          trackWidth/2}) annotation (Placement(visible=true, transformation(
          origin={48,20},
          extent={{10,-10},{-10,10}},
          rotation=0)));
    Praktikum11.PlanarMechanics.Parts.FixedTranslation fixedTranslation3(r={0,-
          trackWidth/2}) annotation (Placement(visible=true, transformation(
          origin={46,-60},
          extent={{10,-10},{-10,10}},
          rotation=0)));
    Praktikum11.PlanarMechanics.Parts.Body body1(m=19, I=0.653) annotation (
        Placement(visible=true, transformation(
          origin={-20,42},
          extent={{-8,-8},{8,8}},
          rotation=90)));
    Praktikum11.PlanarMechanics.Parts.Body body3(
      I=0.653,
      animate=true,
      enableGravity=true,
      m=19) annotation (Placement(visible=true, transformation(
          origin={-20,-38},
          extent={{-8,-8},{8,8}},
          rotation=90)));
    PlanarMechanics.Parts.FixedTranslation fixedTranslation4(r = {(-wheelBase) / 2, 0}) annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {80, -30})));
    Praktikum11.PlanarMechanics.Parts.FixedTranslation fixedTranslation5(r={
          wheelBase/2,0}) annotation (Placement(visible=true, transformation(
          origin={80,36},
          extent={{10,-10},{-10,10}},
          rotation=-90)));
    Praktikum11.PlanarMechanics.Joints.Revolute revolute(useFlange=true, phi(
          start=0)) annotation (Placement(visible=true, transformation(
          origin={10,20},
          extent={{10,-10},{-10,10}},
          rotation=0)));
    PlanarMechanics.Parts.FixedTranslation fixedTranslation(r = {0, trackWidth / 2}) annotation (
      Placement(visible = true, transformation(origin = {48, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    PlanarMechanics.Joints.Revolute revolute1(useFlange=true)   annotation (
      Placement(visible = true, transformation(extent={{20,50},{0,70}},      rotation = 0)));
    PlanarMechanics.Parts.Body body(I = 0.653, m = 19) annotation (
      Placement(visible = true, transformation(origin = {-20, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint dryFrictionWheelJoint(N = (carMass / 4 + 19) * 9.81, mu_A = 0.85, mu_S = 0.5, r = {1, 0}, radius = wheelRadius, vAdhesion(displayUnit = "m/s") = 0.01, vSlide = 0.1) annotation (
      Placement(visible = true, transformation(extent = {{-30, 50}, {-50, 70}}, rotation = 0)));
    Praktikum11.PlanarMechanics.Parts.FixedTranslation fixedTranslation2(r={0,
          trackWidth/2}) annotation (Placement(visible=true, transformation(
          origin={46,-20},
          extent={{10,-10},{-10,10}},
          rotation=0)));
    PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint dryFrictionWheelJoint2(N = (carMass / 4 + 19) * 9.81, mu_A = 0.85, mu_S = 0.5, r = {1, 0}, radius = wheelRadius, vAdhesion(displayUnit = "m/s") = 0.01, vSlide(displayUnit = "m/s") = 0.1) annotation (
      Placement(visible = true, transformation(extent = {{-30, -30}, {-50, -10}}, rotation = 0)));
    PlanarMechanics.Parts.Body body2(I = 0.653, m = 19) annotation (
      Placement(visible = true, transformation(origin = {-20, 2}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));
  equation
    connect(VL, inertia.flange_a) annotation (
      Line(points = {{-100, 60}, {-78, 60}}, color = {0, 0, 0}));
    connect(inertia1.flange_b, dryFrictionWheelJoint1.flange_a) annotation (
      Line(points = {{-60, 20}, {-50, 20}}));
    connect(inertia1.flange_a, VR) annotation (
      Line(points = {{-80, 20}, {-100, 20}}));
    connect(inertia2.flange_a, HL) annotation (
      Line(points = {{-80, -20}, {-100, -20}}, color = {0, 0, 0}));
    connect(inertia3.flange_b, dryFrictionWheelJoint3.flange_a) annotation (
      Line(points = {{-60, -60}, {-50, -60}}, color = {0, 0, 0}));
    connect(HR, HR) annotation (
      Line(points = {{-100, -60}, {-100, -60}}, color = {0, 0, 0}));
    connect(HR, inertia3.flange_a) annotation (
      Line(points = {{-100, -60}, {-80, -60}}, color = {0, 0, 0}));
    connect(frame, frame) annotation (
      Line(points = {{80, 0}, {80, 0}}, color = {0, 0, 0}));
    connect(frame, fixedTranslation4.frame_a) annotation (
      Line(points = {{80, 0}, {80, -20}}));
    connect(dryFrictionWheelJoint1.frame_a, dryFrictionWheelJoint1.frame_a) annotation (
      Line(points = {{-36, 20}, {-36, 20}}));
    connect(fixedTranslation5.frame_a, frame) annotation (
      Line(points = {{80, 26}, {80, 0}}));
    connect(fixedTranslation1.frame_a, fixedTranslation5.frame_b) annotation (
      Line(points = {{58, 20}, {64, 20}, {64, 60}, {80, 60}, {80, 46}}));
    connect(fixedTranslation4.frame_b, fixedTranslation3.frame_a) annotation (
      Line(points = {{80, -40}, {80, -60}, {56, -60}}, color = {95, 95, 95}));
    connect(fixedTranslation3.frame_b, dryFrictionWheelJoint3.frame_a) annotation (
      Line(points = {{36, -60}, {-36, -60}}, color = {95, 95, 95}));
    connect(dryFrictionWheelJoint3.frame_a, body3.frame_a) annotation (
      Line(points = {{-36, -60}, {-20, -60}, {-20, -46}}, color = {95, 95, 95}));
    connect(fixedTranslation.frame_a, fixedTranslation5.frame_b) annotation (
      Line(points = {{58, 60}, {80, 60}, {80, 46}}));
    connect(inertia.flange_b, dryFrictionWheelJoint.flange_a) annotation (
      Line(points = {{-58, 60}, {-50, 60}}));
    connect(fixedTranslation2.frame_a, fixedTranslation4.frame_b) annotation (
      Line(points = {{56, -20}, {60, -20}, {60, -60}, {80, -60}, {80, -40}}, color = {95, 95, 95}));
    connect(dryFrictionWheelJoint2.frame_a, fixedTranslation2.frame_b) annotation (
      Line(points = {{-36, -20}, {36, -20}}, color = {95, 95, 95}));
    connect(inertia2.flange_b, dryFrictionWheelJoint2.flange_a) annotation (
      Line(points = {{-60, -20}, {-50, -20}}));
    connect(dryFrictionWheelJoint2.frame_a, body2.frame_a) annotation (
      Line(points = {{-36, -20}, {-20, -20}, {-20, -6}}, color = {95, 95, 95}));
    connect(body1.frame_a, dryFrictionWheelJoint1.frame_a) annotation (Line(
        points={{-20,34},{-28,34},{-28,20},{-36,20}},
        color={95,95,95},
        thickness=0.5));
    connect(body.frame_a, dryFrictionWheelJoint.frame_a) annotation (Line(
        points={{-20,74},{-28,74},{-28,60},{-36,60}},
        color={95,95,95},
        thickness=0.5));
    connect(dryFrictionWheelJoint.frame_a, revolute1.frame_b) annotation (Line(
        points={{-36,60},{0,60}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute1.frame_a, fixedTranslation.frame_b) annotation (Line(
        points={{20,60},{38,60}},
        color={95,95,95},
        thickness=0.5));
    connect(dryFrictionWheelJoint1.frame_a, revolute.frame_b) annotation (Line(
        points={{-36,20},{0,20}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute.frame_a, fixedTranslation1.frame_b) annotation (Line(
        points={{20,20},{38,20}},
        color={95,95,95},
        thickness=0.5));
    connect(Steering, revolute.flange_a) annotation (Line(points={{70,102},{70,
            6},{10,6},{10,10}}, color={0,0,0}));
    connect(revolute1.flange_a, Steering)
      annotation (Line(points={{10,50},{70,50},{70,102}}, color={0,0,0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Chassis2D;

  model Body2D
    PlanarMechanics.Interfaces.Frame frame annotation (
      Placement(transformation(extent = {{-94, -10}, {-74, 10}})));
    PlanarMechanics.Parts.Body body(m = 1376, I = 1137,
      phi(start=0, fixed=true))                         annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {10, 56})));
    PlanarMechanics.Sources.QuadraticSpeedDependentForce quadraticSpeedDependentForce1(F_nominal = -35.478, v_nominal = 2.7777777777778, tau_nominal = 0,
      w_nominal=0)                                                                                                                                                       annotation (
      Placement(transformation(extent = {{80, -10}, {60, 10}})));
    PlanarMechanics.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation (
      Placement(transformation(extent = {{-6, -44}, {14, -24}})));
    Modelica.Blocks.Interfaces.RealOutput speed annotation (
      Placement(transformation(extent = {{96, -48}, {116, -28}})));
    PlanarMechanics.Sensors.AbsolutePosition absolutePosition(resolveInFrame=
          Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)                                                                       annotation (
      Placement(transformation(extent={{-14,76},{6,96}})));
    Modelica.Blocks.Interfaces.RealOutput angle annotation (
      Placement(transformation(extent = {{72, 72}, {92, 92}})));
  equation
    connect(frame, body.frame_a) annotation (
      Line(points = {{-84, 0}, {10, 0}, {10, 46}}, color = {0, 0, 0}));
    connect(quadraticSpeedDependentForce1.frame_b, body.frame_a) annotation (
      Line(points = {{60, 0}, {10, 0}, {10, 46}}, color = {95, 95, 95}, thickness = 0.5));
    connect(absoluteVelocity.frame_a, frame) annotation (
      Line(points = {{-6, -34}, {-46, -34}, {-46, 0}, {-84, 0}}, color = {95, 95, 95}, thickness = 0.5));
    connect(absoluteVelocity.v[1], speed) annotation (
      Line(points={{15,-34.6667},{32,-34.6667},{32,-34},{50,-34},{50,-38},{106,
            -38}},                                                                                 color = {0, 0, 127}));
    connect(absolutePosition.frame_a, frame) annotation (
      Line(points={{-14,86},{-80,86},{-80,0},{-84,0}},          color = {95, 95, 95}, thickness = 0.5));
    connect(absolutePosition.r[3], angle) annotation (
      Line(points={{7,86.6667},{42,86.6667},{42,82},{82,82}},          color = {0, 0, 127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Body2D;

  model HybrdiCar2D
    Driver driver(fileName=
          "C:/Users/sturmdan/Documents/Dymola/sort1_steering.txt",                  k = 50) annotation (
      Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
    ElectricDrive electricDrive annotation (
      Placement(transformation(extent = {{-36, 4}, {-16, 24}})));
    CombustionEngineDrive combustionEngineDrive(omegaStart = 0) annotation (
      Placement(transformation(extent = {{-36, -28}, {-16, -8}})));
    Modelica.Blocks.Math.Gain gain(k = 0.5) annotation (
      Placement(transformation(extent = {{-52, -4}, {-44, 4}})));
    DriveTrain2D driveTrain2D annotation (
      Placement(transformation(extent = {{2, -10}, {22, 10}})));
    Chassis2D chassis2D(wheelBase = 2.578, trackWidth = 1.549, wheelRadius = 0.31, carMass = 1376) annotation (
      Placement(transformation(extent={{28,-10},{48,10}})));
    Body2D body2D annotation (
      Placement(transformation(extent={{56,-10},{76,10}})));
    inner PlanarMechanics.PlanarWorld planarWorld(constantGravity = {0, 0}) annotation (
      Placement(transformation(extent = {{-80, -74}, {-60, -54}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport=false)
      annotation (Placement(transformation(extent={{-16,46},{4,66}})));
    SteeringLimiter steeringLimiter
      annotation (Placement(transformation(extent={{18,46},{38,66}})));
  equation
    connect(gain.u, driver.car_torque) annotation (
      Line(points = {{-52.8, 0}, {-63.6, 0}}, color = {0, 0, 127}));
    connect(gain.y, combustionEngineDrive.u) annotation (
      Line(points = {{-43.6, 0}, {-42, 0}, {-42, -18}, {-36.2, -18}}, color = {0, 0, 127}));
    connect(electricDrive.u, gain.y) annotation (
      Line(points = {{-36.4, 14}, {-42, 14}, {-42, 0}, {-43.6, 0}}, color = {0, 0, 127}));
    connect(electricDrive.flange_a, driveTrain2D.flange_a) annotation (
      Line(points = {{-16.2, 14}, {-8, 14}, {-8, 0}, {2, 0}}, color = {0, 0, 0}));
    connect(combustionEngineDrive.flange_a, driveTrain2D.flange_a) annotation (
      Line(points = {{-16, -18}, {-8, -18}, {-8, 0}, {2, 0}}, color = {0, 0, 0}));
    connect(driveTrain2D.VL, chassis2D.VL) annotation (
      Line(points = {{22, 6}, {28, 6}}, color = {0, 0, 0}));
    connect(driveTrain2D.VR, chassis2D.VR) annotation (
      Line(points = {{22, 2}, {28, 2}}, color = {0, 0, 0}));
    connect(driveTrain2D.HL, chassis2D.HL) annotation (
      Line(points = {{22, -2}, {28, -2}}, color = {0, 0, 0}));
    connect(driveTrain2D.HR, chassis2D.HR) annotation (
      Line(points = {{22, -6}, {28, -6}}, color = {0, 0, 0}));
    connect(chassis2D.frame, body2D.frame) annotation (
      Line(points={{46,0},{57.6,0}},      color = {0, 0, 0}));
    connect(body2D.speed, driver.car_speed) annotation (
      Line(points={{76.6,-3.8},{96,-3.8},{96,-40},{-92,-40},{-92,-3},{-78.4,-3}},              color = {0, 0, 127}));
    connect(body2D.angle, driver.driving_angle_deg) annotation (Line(points={{74.2,
            8.2},{98,8.2},{98,-44},{-96,-44},{-96,8.8},{-76.8,8.8}},      color=
           {0,0,127}));
    connect(torque.flange, steeringLimiter.flange_a)
      annotation (Line(points={{4,56},{18,56}}, color={0,0,0}));
    connect(steeringLimiter.flange_b, chassis2D.Steering)
      annotation (Line(points={{38,56},{43.4,56},{43.4,10}}, color={0,0,0}));
    connect(driver.steering_angle, torque.tau) annotation (Line(points={{-61.2,
            5.6},{-61.2,56},{-18,56}}, color={0,0,127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)),
      experiment(StopTime=150, __Dymola_Algorithm="Dassl"));
  end HybrdiCar2D;

  model Differential
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
      Placement(transformation(extent = {{-10, 88}, {10, 108}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation (
      Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1 annotation (
      Placement(transformation(extent = {{90, -10}, {110, 10}})));
  equation
    2 * der(flange_a.phi) = der(flange_b.phi) + der(flange_b1.phi);
    flange_b.tau = flange_b1.tau;
    flange_a.tau = - 2 * flange_b.tau;
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Differential;

  model DifferentialGearTest
    Differential differential annotation (
      Placement(transformation(extent = {{-20, -6}, {0, 14}})));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(tau_constant = 100) annotation (
      Placement(transformation(extent = {{-58, 48}, {-38, 68}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 100) annotation (
      Placement(transformation(extent = {{-64, -6}, {-44, 14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 100) annotation (
      Placement(transformation(extent = {{30, -6}, {50, 14}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed annotation (
      Placement(transformation(extent = {{62, -42}, {82, -22}})));
  equation
    connect(differential.flange_a, constantTorque.flange) annotation (
      Line(points = {{-10, 13.8}, {-10, 58}, {-38, 58}}, color = {0, 0, 0}));
    connect(inertia.flange_b, differential.flange_b) annotation (
      Line(points = {{-44, 4}, {-20, 4}}, color = {0, 0, 0}));
    connect(differential.flange_b1, inertia1.flange_a) annotation (
      Line(points = {{0, 4}, {30, 4}}, color = {0, 0, 0}));
    connect(inertia1.flange_b, fixed.flange) annotation (
      Line(points = {{50, 4}, {62, 4}, {62, -32}, {72, -32}}, color = {0, 0, 0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end DifferentialGearTest;

  model ElectricDriveTest
  ElectricDrive electricDrive annotation (
      Placement(visible = true, transformation(origin = {6, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression realExpression(y = 1500)  annotation (
      Placement(visible = true, transformation(origin = {-68, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(electricDrive.u, realExpression.y) annotation (
      Line(points={{-4.4,0},{-57,0}},    color = {0, 0, 127}));
  end ElectricDriveTest;

model SteeringLimiter
  parameter Modelica.SIunits.RotationalSpringConstant springConstant = 100;
  parameter Modelica.SIunits.RotationalDampingConstant damperConstant = 10;
  parameter Modelica.SIunits.Angle maxAngle = (45/360)*2*3.14;
  Real spring_tau;
  Real damper_tau;

  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));

equation
    flange_a.phi = flange_b.phi;
    flange_b.tau = - flange_a.tau + sign(flange_a.phi) * (spring_tau) + sign(spring_tau) * damper_tau;
    spring_tau = max(0, (abs(flange_a.phi)- maxAngle) * springConstant);
    damper_tau = der(flange_a.phi) * damperConstant;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SteeringLimiter;

  model SteeringLimiterTest
    Praktikum11.SteeringLimiter steeringLimiter
      annotation (Placement(transformation(extent={{-8,-12},{12,8}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
      annotation (Placement(transformation(extent={{34,-12},{54,8}})));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
        tau_constant=1)
      annotation (Placement(transformation(extent={{-56,-12},{-36,8}})));
  equation
    connect(constantTorque.flange, steeringLimiter.flange_a)
      annotation (Line(points={{-36,-2},{-8,-2}}, color={0,0,0}));
    connect(steeringLimiter.flange_b, inertia.flange_a)
      annotation (Line(points={{12,-2},{34,-2}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(__Dymola_NumberOfIntervals=5000, __Dymola_Algorithm="Dassl"));
  end SteeringLimiterTest;

  package HybridCar3D
    model Body3D
      import SI = Modelica.SIunits;
      parameter SI.Length wheelRadius;
      Modelica.Blocks.Interfaces.RealOutput speed annotation (
        Placement(transformation(extent={{96,-56},{116,-36}})));
      Modelica.Blocks.Interfaces.RealOutput angle_Deg
        annotation (Placement(transformation(extent={{72,72},{92,92}})));
      QuadraticSpeedDependentForce quadraticSpeedDependentForce(
        resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world,
        F_nominal=-35.478,
        v_nominal(displayUnit="m/s") = 2.7777777777778,
        tau_nominal=0,
        w_nominal=1)
        annotation (Placement(transformation(extent={{58,-10},{38,10}})));
      Modelica.Mechanics.MultiBody.Parts.Body body1(
        r_CM={0,0,0},
        m=1300,
        I_11=284.25,
        I_22=284.25,
        I_33=1137,
        r_0(start={0,0,wheelRadius}, fixed=true),
        angles_start={0,0,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={6,52})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame frame
        annotation (Placement(transformation(extent={{-88,-8},{-68,12}})));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity1(
          resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a)
        annotation (Placement(transformation(extent={{14,-56},{34,-36}})));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles
        annotation (Placement(transformation(extent={{-18,76},{2,96}})));
    equation
      connect(quadraticSpeedDependentForce.frame, frame) annotation (Line(
          points={{38,0},{-18,0},{-18,2},{-78,2}},
          color={95,95,95},
          thickness=0.5));
      connect(body1.frame_a, frame) annotation (Line(
          points={{6,42},{6,0},{-18,0},{-18,2},{-78,2}},
          color={95,95,95},
          thickness=0.5));
      connect(absoluteVelocity1.frame_a, frame) annotation (Line(
          points={{14,-46},{-16,-46},{-16,2},{-78,2}},
          color={95,95,95},
          thickness=0.5));
      connect(absoluteAngles.frame_a, frame) annotation (Line(
          points={{-18,86},{-46,86},{-46,2},{-78,2}},
          color={95,95,95},
          thickness=0.5));
      connect(absoluteAngles.angles[3], angle_Deg) annotation (Line(points={{3,
              86.6667},{40,86.6667},{40,82},{82,82}}, color={0,0,127}));
      connect(absoluteVelocity1.v[1], speed) annotation (Line(points={{35,
              -46.6667},{72,-46.6667},{72,-46},{106,-46}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end Body3D;

    model QuadraticSpeedDependentForce
      "External force and torque acting at frame_b, defined by 3 input signals and resolved in world frame"

      import SI=Modelica.SIunits;

      parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveInFrame=
        Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world;

       final parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameB resolveInFrameB=
        Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world
        "Conversion from frame A to B";

    public
      parameter Modelica.SIunits.Force F_nominal
        "Nominal force (if negative, torque is acting as load)";
      parameter Modelica.SIunits.Velocity v_nominal(min=Modelica.Constants.eps)
        "Nominal speed";
      parameter Modelica.SIunits.Torque tau_nominal
        "Nominal torque (if negative, torque is acting as load)";
      parameter Modelica.SIunits.AngularVelocity w_nominal(min=Modelica.Constants.eps)
        "Nominal speed";

      Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(
          resolveInFrame=resolveInFrame) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={56,40})));
      Modelica.Mechanics.MultiBody.Forces.WorldForce force1(resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b)
        annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame
        annotation (Placement(transformation(extent={{84,-16},{116,16}})));
      Modelica.Blocks.Math.MatrixGain normalizeSpeeds(K=[1/v_nominal,0,0; 0,1/
            v_nominal,0; 0,0,1/v_nominal])
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=180,
            origin={8,40})));
      Modelica.Blocks.Math.MatrixGain scaleForces(K=[F_nominal,0,0; 0,F_nominal,
            0; 0,0,F_nominal])
        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            origin={-58,0})));
      SquaretimesSign squaretimesSign(blockSize=3)
        annotation (Placement(transformation(extent={{-40,30},{-60,50}})));
    equation
      connect(absoluteVelocity.frame_a, frame) annotation (Line(
          points={{66,40},{88,40},{88,0},{100,0}},
          color={95,95,95},
          thickness=0.5));
      connect(normalizeSpeeds.u, absoluteVelocity.v)
        annotation (Line(points={{20,40},{45,40}}, color={0,0,127}));
      connect(scaleForces.y, force1.force)
        annotation (Line(points={{-47,0},{-10,0}}, color={0,0,127}));
      connect(force1.frame_b, frame) annotation (Line(
          points={{12,0},{100,0}},
          color={95,95,95},
          thickness=0.5));
      connect(squaretimesSign.u, normalizeSpeeds.y)
        annotation (Line(points={{-38,40},{-3,40}}, color={0,0,127}));
      connect(squaretimesSign.y, scaleForces.u) annotation (Line(points={{-61,40},{-80,
              40},{-80,0},{-70,0}}, color={0,0,127}));
      annotation (Icon(graphics={
            Polygon(
              points={{-100,10},{20,10},{20,41},{90,0},{20,-41},{20,-10},{-100,-10},
                  {-100,10}},
              lineColor={0,127,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{-80,-98},{-60,-92},{-40,-82},{-20,-68},{0,-50},{20,
                  -28},{40,-2},{60,28},{80,62},{100,100}},
              color={0,0,127}, smooth=Smooth.Bezier),
            Text(
              extent={{-150,80},{150,40}},
              textString="%name",
              lineColor={0,0,255}),
            Text(
              extent={{72,-24},{108,-49}},
              lineColor={128,128,128},
              textString="b")}));
    end QuadraticSpeedDependentForce;

    block SquaretimesSign
      "Output the squared input and retain the same sign of it"
      parameter Integer blockSize;
      extends Modelica.Blocks.Interfaces.MIMO(final nin=blockSize, final nout=blockSize);
    equation
      for i in 1:size(u,1) loop
        y[i] = smooth(1,u[i]^2*sign(u[i]));
      end for;
      annotation (
        Documentation(info="<html>
<p>
This block outputs squared input real signal whereby the sign
of the output is the same as input.
The size of input&nbsp;u and output&nbsp;y are defined by
a parameter blockSize, thus
</p>

<blockquote><pre>
<strong>for</strong> i <strong>in</strong> 1:blockSize <strong>loop</strong>
  y[i] = sign(u[i]) * u[i]^2;
<strong>end for</strong>;
</pre></blockquote>
</html>"),     Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}), graphics={Text(
          extent={{-100,-98},{100,-68}},
          lineColor={160,160,164},
              textString="sign(u) * u^2"),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,68},{0,-80}},     color={192,192,192}),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-50},{80,50}}, color={95,95,95}),
            Line(
              points={{0,0},{-2,0},{-16,-2},{-34,-14},{-54,-42},{-64,-72}},
              color={0,0,127},
              smooth=Smooth.Bezier),
            Line(
              points={{0,0},{2,0},{16,2},{34,14},{56,46},{70,90}},
              color={0,0,127},
              smooth=Smooth.Bezier)}),
        Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}), graphics={Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-90,-60},{90,60}},
          lineColor={160,160,164},
              textString="sign(u) * u^2")}));
    end SquaretimesSign;

    model Chassis3D
      import SI = Modelica.SIunits;
      parameter SI.Length wheelBase;
      parameter SI.Length trackWidth;
      parameter SI.Length wheelRadius;
      parameter SI.Mass carMass;
      Modelica.Mechanics.Rotational.Interfaces.Flange_a HR annotation (
        Placement(transformation(extent={{-110,-94},{-90,-74}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a VL annotation (
        Placement(transformation(extent={{-110,30},{-90,50}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a VR annotation (
        Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a HL annotation (
        Placement(transformation(extent={{-110,-60},{-90,-40}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a Steering annotation (
        Placement(visible = true, transformation(extent = {{60, 92}, {80, 112}}, rotation = 0), iconTransformation(extent = {{44, 90}, {64, 110}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame frame
        annotation (Placement(transformation(extent={{76,-10},{88,2}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation6(r={
            (-wheelBase)/2,0,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={82,-40})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r={
            0,trackWidth/2,0})
        annotation (Placement(transformation(extent={{52,-36},{32,-16}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r={
            0,-trackWidth/2,0})
        annotation (Placement(transformation(extent={{52,-76},{32,-56}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r={
            0,-trackWidth/2,0})
        annotation (Placement(transformation(extent={{52,10},{32,30}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation7(r={
            0,trackWidth/2,0})
        annotation (Placement(transformation(extent={{52,50},{32,70}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r={
            wheelBase/2,0,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={82,26})));
      Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape2(
        animateSphere=false,
        r={0,-0.1,0},
        r_CM={0,0,0},
        m=19,
        I_11=0.653,
        I_22=1.055,
        I_33=0.653,
        shapeType="cylinder",
        widthDirection(displayUnit="1") = {0,1,0},
        width=0.62,
        color={255,170,170})
                    annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,20})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute2(useAxisFlange=true)
        annotation (Placement(transformation(extent={{20,10},{0,30}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute3(useAxisFlange=true)
        annotation (Placement(transformation(extent={{20,50},{0,70}})));
      DryFrictionWheelJoint3D dryFrictionWheelJoint3D(
        radius=0.31,
        vAdhesion=0.01,
        vSlide=0.1,
        mu_A=0.85,
        mu_S=0.5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-40,20})));
      DryFrictionWheelJoint3D dryFrictionWheelJoint3D1(
        radius=0.31,
        vAdhesion=0.01,
        vSlide=0.1,
        mu_A=0.85,
        mu_S=0.5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-40,-26})));
      DryFrictionWheelJoint3D dryFrictionWheelJoint3D2(
        radius=0.31,
        vAdhesion=0.01,
        vSlide=0.1,
        mu_A=0.85,
        mu_S=0.5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-40,-66})));
      Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape1(
        animateSphere=false,
        r={0,0.1,0},
        r_CM={0,0,0},
        m=19,
        I_11=0.653,
        I_22=1.055,
        I_33=0.653,
        shapeType="cylinder",
        widthDirection(displayUnit="1") = {0,1,0},
        width=0.62,
        color={0,128,255})
                    annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,-26})));
      Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape3(
        animateSphere=false,
        r={0,-0.1,0},
        r_CM={0,0,0},
        m=19,
        I_11=0.653,
        I_22=1.055,
        I_33=0.653,
        shapeType="cylinder",
        widthDirection(displayUnit="1") = {0,1,0},
        width=0.62,
        color={170,213,255})
                    annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,-66})));
      DryFrictionWheelJoint3D dryFrictionWheelJoint3D3(
        radius=0.31,
        vAdhesion=0.01,
        vSlide=0.1,
        mu_A=0.85,
        mu_S=0.5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-40,60})));
      Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape4(
        animateSphere=false,
        r={0,0.1,0},
        r_CM={0,0,0},
        m=19,
        I_11=0.653,
        I_22=1.055,
        I_33=0.653,
        shapeType="cylinder",
        widthDirection(displayUnit="1") = {0,1,0},
        width=0.62,
        color={255,0,0})
                    annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,60})));
      WheelSuspensionRight wheelSuspension
        annotation (Placement(transformation(extent={{-24,54},{-4,74}})));
      WheelSuspensionRight wheelSuspension1
        annotation (Placement(transformation(extent={{-26,14},{-6,34}})));
      WheelSuspensionRight wheelSuspension2
        annotation (Placement(transformation(extent={{-12,-30},{8,-10}})));
      WheelSuspensionRight wheelSuspension3
        annotation (Placement(transformation(extent={{-12,-68},{8,-48}})));
    equation
      connect(HR, HR) annotation (
        Line(points={{-100,-84},{-100,-84}},      color = {0, 0, 0}));
      connect(frame, fixedTranslation6.frame_a) annotation (Line(points={{82,-4},
              {82,-30}},                       color={0,0,0}));
      connect(frame, fixedTranslation.frame_a) annotation (Line(points={{82,-4},
              {82,16}},                   color={0,0,0}));
      connect(fixedTranslation3.frame_a, fixedTranslation.frame_b) annotation (
          Line(
          points={{52,20},{60,20},{60,44},{82,44},{82,36}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation7.frame_a, fixedTranslation.frame_b) annotation (
          Line(
          points={{52,60},{60,60},{60,44},{82,44},{82,36}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.frame_a, fixedTranslation3.frame_b) annotation (Line(
          points={{20,20},{32,20}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute3.frame_a, fixedTranslation7.frame_b) annotation (Line(
          points={{20,60},{32,60}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.axis, Steering) annotation (Line(points={{10,30},{10,38},
              {70,38},{70,102}}, color={0,0,0}));
      connect(revolute3.axis, Steering) annotation (Line(points={{10,70},{10,86},
              {70,86},{70,102}}, color={0,0,0}));
      connect(bodyShape3.frame_a, dryFrictionWheelJoint3D2.frame_wheel)
        annotation (Line(
          points={{-64,-66},{-50,-66}},
          color={95,95,95},
          thickness=0.5));
      connect(HR, dryFrictionWheelJoint3D2.flange) annotation (Line(points={{
              -100,-84},{-37,-84},{-37,-76}}, color={0,0,0}));
      connect(HL, dryFrictionWheelJoint3D1.flange) annotation (Line(points={{
              -100,-50},{-37,-50},{-37,-36}}, color={0,0,0}));
      connect(bodyShape1.frame_a, dryFrictionWheelJoint3D1.frame_wheel)
        annotation (Line(
          points={{-64,-26},{-50,-26}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation2.frame_a, fixedTranslation6.frame_b) annotation (
         Line(
          points={{52,-66},{60,-66},{60,-54},{82,-54},{82,-50},{82,-50}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation4.frame_a, fixedTranslation6.frame_b) annotation (
         Line(
          points={{52,-26},{60,-26},{60,-54},{82,-54},{82,-50}},
          color={95,95,95},
          thickness=0.5));
      connect(VR, dryFrictionWheelJoint3D.flange)
        annotation (Line(points={{-100,0},{-37,0},{-37,10}}, color={0,0,0}));
      connect(bodyShape2.frame_a, dryFrictionWheelJoint3D.frame_wheel)
        annotation (Line(
          points={{-64,20},{-50,20}},
          color={95,95,95},
          thickness=0.5));
      connect(bodyShape4.frame_a, dryFrictionWheelJoint3D3.frame_wheel)
        annotation (Line(
          points={{-64,60},{-50,60}},
          color={95,95,95},
          thickness=0.5));
      connect(VL, dryFrictionWheelJoint3D3.flange)
        annotation (Line(points={{-100,40},{-37,40},{-37,50}}, color={0,0,0}));
      connect(wheelSuspension.frame, dryFrictionWheelJoint3D3.frame_chassis)
        annotation (Line(points={{-24,60},{-28,60},{-28,60},{-30,60}},
            color={0,0,0}));
      connect(wheelSuspension.frame1, revolute3.frame_b) annotation (Line(
            points={{-4.2,56.2},{-2.6,56.2},{-2.6,60},{0,60}}, color={0,0,0}));
      connect(dryFrictionWheelJoint3D.frame_chassis, wheelSuspension1.frame)
        annotation (Line(
          points={{-30,20},{-28,20},{-28,20},{-26,20}},
          color={95,95,95},
          thickness=0.5));
      connect(wheelSuspension1.frame1, revolute2.frame_b) annotation (Line(
            points={{-6.2,16.2},{-3.6,16.2},{-3.6,20},{0,20}}, color={0,0,0}));
      connect(wheelSuspension3.frame, dryFrictionWheelJoint3D2.frame_chassis)
        annotation (Line(points={{-12,-62},{-22,-62},{-22,-66},{-30,-66}},
            color={0,0,0}));
      connect(wheelSuspension2.frame, dryFrictionWheelJoint3D1.frame_chassis)
        annotation (Line(points={{-12,-24},{-22,-24},{-22,-26},{-30,-26}},
            color={0,0,0}));
      connect(wheelSuspension2.frame1, fixedTranslation4.frame_b) annotation (
          Line(points={{7.8,-27.8},{19.4,-27.8},{19.4,-26},{32,-26}}, color={0,
              0,0}));
      connect(wheelSuspension3.frame1, fixedTranslation2.frame_b) annotation (
          Line(points={{7.8,-65.8},{19.4,-65.8},{19.4,-66},{32,-66}}, color={0,
              0,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end Chassis3D;

    function limitByStriple "Returns a point-symmetric Triple S-Function"
      extends Modelica.Icons.Function;

      input Real x_max "Abscissa for y_max";
      input Real x_sat "Abscissa for y_sat";
      input Real y_max "Peak ordinate";
      input Real y_sat "Saturated ordinate";
      input Real x "Current abscissa value";
      output Real y "Current ordinate";
    algorithm
      if x > x_max then
        y := limitBySform(
          x_max,
          x_sat,
          y_max,
          y_sat,
          x);
      elseif x < -x_max then
        y := limitBySform(
          -x_max,
          -x_sat,
          -y_max,
          -y_sat,
          x);
      else
        y := limitBySform(
          -x_max,
          x_max,
          -y_max,
          y_max,
          x);
      end if;

      annotation (
        smoothOrder=1,
        Documentation(
          info="<html>
<h4>Syntax</h4>
<blockquote><pre>
y = Functions.<strong>limitByStriple</strong>(x_max, x_sat, y_max, y_sat, x);
</pre></blockquote>

<h4>Description</h4>
<p>
A point symmetric interpolation between points (0,&nbsp;0), (x_max,&nbsp;y_max)
and (x_sat,&nbsp;y_sat), provided x_max&nbsp;&lt;&nbsp;x_sat. The approximation
is done in such a way that the 1st function&#039;s derivative is zero at points
points (x_max,&nbsp;y_max) and (x_sat,&nbsp;y_sat).
Thus, the 1st function&#039;s derivative is continuous for all&nbsp;<em>x</em>.
The higher derivatives are, in contrast, discontinuous at these points.
</p>

<p>
The figure below shows the function&nbsp;<em>y</em> and its 1st derivative&nbsp;<em>dy/dx</em>
for the following input:
x_max&nbsp;=&nbsp;0.2,
x_sat&nbsp;=&nbsp;0.5,
y_max&nbsp;=&nbsp;1.4,
y_sat&nbsp;=&nbsp;1.2.
</p>

<blockquote>
<img src=\"modelica://PlanarMechanics/Resources/Images/Utilities/Functions/limitByStriple.png\">
</blockquote>
</html>", revisions="<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
    end limitByStriple;

    function limitBySform "Returns a S-shaped transition"
      extends Modelica.Icons.Function;

      input Real x_min "Abscissa for y_min";
      input Real x_max "Abscissa for y_max";
      input Real y_min "First value of y";
      input Real y_max "Second value of y";
      input Real x "Current abscissa value";
      output Real y "Current ordinate";
    protected
      Real x2;
    algorithm
      x2 := x - x_max/2 - x_min/2;
      x2 := x2*2/(x_max-x_min);
      if x2 > 1 then
        y := 1;
      elseif x2 < -1 then
        y := -1;
      else
        y := -0.5*x2^3 + 1.5*x2;
      end if;
      y := y*(y_max-y_min)/2;
      y := y + y_max/2 + y_min/2;

      annotation (
        smoothOrder=1,
        Documentation(
          info="<html>
<h4>Syntax</h4>
<blockquote><pre>
y = Functions.<strong>limitBySform</strong>(x_min, x_max, y_min, y_max, x);
</pre></blockquote>

<h4>Description</h4>
<p>
A smooth transition between points (x_min,&nbsp;y_min) and (x_max,&nbsp;y_max).
The transition is done in such a way that the 1st function&#039;s derivative
is continuous for all&nbsp;<em>x</em>.
The higher derivatives are, in contrast, discontinuous at input points.
</p>

<p>
The figure below shows the function&nbsp;<em>y</em> and its 1st derivative&nbsp;<em>dy/dx</em>
for the following input:
x_max&nbsp;=&nbsp;-0.4,
x_sat&nbsp;=&nbsp;0.6,
y_max&nbsp;=&nbsp;1.4,
y_sat&nbsp;=&nbsp;1.2.
</p>

<blockquote>
<img src=\"modelica://PlanarMechanics/Resources/Images/Utilities/Functions/limitBySform.png\">
</blockquote>
</html>", revisions="<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
    end limitBySform;

    model PartialDryFrictionWheelJoint3D "Dry-Friction based wheel joint"
      extends
        Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(
        final T=293.15);

        import Modelica.Mechanics.MultiBody.Frames;
        Modelica.SIunits.Force[3] worldForce;
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
        annotation (Placement(transformation(extent={{-56,-16},{-24,16}})));

        parameter Modelica.Mechanics.MultiBody.Frames.Orientation world_R = Modelica.Mechanics.MultiBody.Frames.nullRotation();
        parameter Modelica.SIunits.Position[3] world_r_0 = {0,0,0};
        Modelica.SIunits.Angle frame_a_phi;
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
          Placement(transformation(extent={{90,-8},{110,12}}), iconTransformation(
              extent={{90,-10},{110,10}})));
      outer Modelica.Mechanics.MultiBody.World world "planar world model";
      parameter StateSelect stateSelect=StateSelect.default
        "Priority to use acceleration as states" annotation(HideResult=true,Dialog(tab="Advanced"));

      parameter Modelica.SIunits.Length radius "Radius of the wheel";
      parameter Modelica.SIunits.Length r[2]
        "Driving direction of the wheel at angle phi = 0";

      parameter Modelica.SIunits.Velocity vAdhesion "Adhesion velocity";
      parameter Modelica.SIunits.Velocity vSlide "Sliding velocity";
      parameter Real mu_A "Friction coefficient at adhesion";
      parameter Real mu_S "Friction coefficient at sliding";
      final parameter Modelica.SIunits.Length l=sqrt(r*r) "Length of vector r";
      final parameter Real e[2] =  r/l "Normalized direction";
      Real e0[2] "Normalized direction w.r.t inertial system";
      Real R[2,2] "Rotation Matrix";
      Modelica.SIunits.Angle phi_roll(stateSelect=stateSelect, start=0)
        "Roll angle of the wheel"
        annotation (Dialog(group="Initialization", showStartAttribute=true));
      Modelica.SIunits.AngularVelocity w_roll(final stateSelect=stateSelect, start=0)
        "Roll velocity of wheel"
        annotation (Dialog(group="Initialization", showStartAttribute=true));
      Modelica.SIunits.Velocity v[2] "Velocity";
      Modelica.SIunits.Velocity v_lat(start=0) "Driving in lateral direction"
        annotation (Dialog(group="Initialization", showStartAttribute=true));
      Modelica.SIunits.Velocity v_long(start=0)
        "Velocity in longitudinal direction"
        annotation (Dialog(group="Initialization", showStartAttribute=true));
      Modelica.SIunits.Velocity v_slip_long(start=0)
        "Slip velocity in longitudinal direction"
        annotation (Dialog(group="Initialization", showStartAttribute=true));
      Modelica.SIunits.Velocity v_slip_lat(start=0)
        "Slip velocity in lateral direction"
        annotation (Dialog(group="Initialization", showStartAttribute=true));
      Modelica.SIunits.Velocity v_slip "Slip velocity";
      Modelica.SIunits.Force f "Longitudinal force";
      Modelica.SIunits.Force f_lat "Longitudinal force";
      Modelica.SIunits.Force f_long "Longitudinal force";

      parameter Boolean animate = true "= true, if animation shall be enabled"
         annotation(Dialog(group="Animation"));

      parameter Modelica.SIunits.Length diameter=0.1 "Diameter of the rims"
        annotation (Dialog(
          tab="Animation",
          group="if animation = true",
          enable=animate));
      parameter Modelica.SIunits.Length width=diameter*0.6 "Width of the wheel"
        annotation (Dialog(
          tab="Animation",
          group="if animation = true",
          enable=animate));

      Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape cylinder(
        shapeType="cylinder",
        color={63,63,63},
        length=width,
        width=radius*2,
        height=radius*2,
        lengthDirection={-e0[2],e0[1],0},
        widthDirection={0,0,1},
        r_shape=-0.03*{-e0[2],e0[1],0},
        r=Modelica.Mechanics.MultiBody.Frames.resolve1(world_R, {frame_a.r_0[1],
            frame_a.r_0[2],frame_a.r_0[3]}) + world_r_0,
        R=world_R) if world.enableAnimation and animate;
      Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape rim1(
        shapeType="cylinder",
        color={195,195,195},
        length=radius*2,
        width=diameter,
        height=diameter,
        lengthDirection={0,0,1},
        widthDirection={1,0,0},
        r_shape={0,0,-radius},
        r=Modelica.Mechanics.MultiBody.Frames.resolve1(world_R, {frame_a.r_0[1],
            frame_a.r_0[2],frame_a.r_0[3]}) + world_r_0,
        R=Modelica.Mechanics.MultiBody.Frames.absoluteRotation(world_R,
            Modelica.Mechanics.MultiBody.Frames.planarRotation(
            {-e0[2],e0[1],0},
            flange_a.phi,
            0))) if world.enableAnimation and animate;
      Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape rim2(
        shapeType="cylinder",
        color={195,195,195},
        length=radius*2,
        width=diameter,
        height=diameter,
        lengthDirection={0,0,1},
        widthDirection={1,0,0},
        r_shape={0,0,-radius},
        r=Modelica.Mechanics.MultiBody.Frames.resolve1(world_R, {frame_a.r_0[1],
            frame_a.r_0[2],frame_a.r_0[3]}) + world_r_0,
        R=Modelica.Mechanics.MultiBody.Frames.absoluteRotation(world_R,
            Modelica.Mechanics.MultiBody.Frames.planarRotation(
            {-e0[2],e0[1],0},
            flange_a.phi + Modelica.Constants.pi/2,
            0))) if world.enableAnimation and animate;
      Modelica.Blocks.Interfaces.RealInput N annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,108})));
    equation

      worldForce = Frames.resolve1(frame_a.R, frame_a.f);

      frame_a_phi = (Modelica.Mechanics.MultiBody.Frames.axesRotationsAngles(frame_a.R, {1,2,3}))[3];

      R = {{cos(frame_a_phi), -sin(frame_a_phi)}, {sin(frame_a_phi),cos(frame_a_phi)}};
      e0 = R*e;
      v = der({frame_a.r_0[1],frame_a.r_0[2]});
      phi_roll = flange_a.phi;
      w_roll = der(phi_roll);
      v_long = v*e0;
      v_lat = -v[1]*e0[2] + v[2]*e0[1];
      v_slip_lat = v_lat - 0;
      v_slip_long = v_long - radius*w_roll;
      v_slip = sqrt(v_slip_long^2 + v_slip_lat^2)+0.0001;
      -f_long*radius = flange_a.tau;
      frame_a.t = {0,0,0};
      f = N*noEvent(Praktikum11.HybridCar3D.limitByStriple(
            vAdhesion,
            vSlide,
            mu_A,
            mu_S,
            v_slip));
      f_long =f*v_slip_long/v_slip;
      f_lat  =f*v_slip_lat/v_slip;
      f_long = {worldForce[1], worldForce[2]}*e0;
      f_lat = {worldForce[2], -worldForce[1]}*e0;
      lossPower = f*v_slip;

      worldForce[3] = -N;
      annotation (Icon(graphics={
            Rectangle(
              extent={{-40,100},{40,-100}},
              lineColor={95,95,95},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={231,231,231}),
            Line(
              points={{-40,30},{40,30}},
              color={95,95,95}),
            Line(
              points={{-40,-30},{40,-30}},
              color={95,95,95}),
            Line(
              points={{-40,60},{40,60}},
              color={95,95,95}),
            Line(
              points={{-40,80},{40,80}},
              color={95,95,95}),
            Line(
              points={{-40,90},{40,90}},
              color={95,95,95}),
            Line(
              points={{-40,100},{40,100}},
              color={95,95,95}),
            Line(
              points={{-40,-80},{40,-80}},
              color={95,95,95}),
            Line(
              points={{-40,-90},{40,-90}},
              color={95,95,95}),
            Line(
              points={{-40,-100},{40,-100}},
              color={95,95,95}),
            Line(
              points={{-40,-60},{40,-60}},
              color={95,95,95}),
            Rectangle(
              extent={{100,10},{40,-10}},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={231,231,231}),
            Line(
              visible=useHeatPort,
              points={{-100,-100},{-100,-90},{0,-90}},
              color={191,0,0},
              pattern=LinePattern.Dot,
              smooth=Smooth.None),
            Text(
              extent={{-150,-110},{150,-140}},
              lineColor={0,0,0},
              textString="radius=%radius"),
            Text(
              extent={{-150,140},{150,100}},
              textString="%name",
              lineColor={0,0,255})}),
        Documentation(
          info="<html>
<p>The ideal wheel joint models the behavior of a wheel rolling on a x,y-plane whose contact patch has dry-friction characteristics. This is an approximation for stiff wheels without a tire.</p>
<p>The force depends with dry-friction characteristics on the slip velocity. The slip velocity is split into two components:</p>
<ul>
<li>the lateral velocity</li>
<li>the longitudinal velocity minus the rolling velocity times the radius.</li>
</ul>
<p>The radius of the wheel can be specified by the parameter <b>radius</b>. The driving direction (for phi=0) can be specified by the parameter <b>r</b>. The normal load is set by <b>N</b>.</p>
<p>The wheel contains a 2D connector <b>frame_a</b> for the steering on the plane. The rolling motion of the wheel can be actuated by the 1D  connector <b>flange_a</b>.</p>
<p>For examples of usage see the local <a href=\"modelica://PlanarMechanics.VehicleComponents.Examples\">Examples package</a>.</p>
</html>", revisions="<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<b>Developed 2010-2019 at the DLR Institute of System Dynamics and Control</b>
</p>
</html>"));
    end PartialDryFrictionWheelJoint3D;

    model DryFrictionWheelJoint3D
      "Driving direction of the wheel at angle phi = 0 is x-axis"

      parameter Modelica.SIunits.Length radius "Radius of the wheel";
      parameter Modelica.SIunits.Velocity vAdhesion "Adhesion velocity";
      parameter Modelica.SIunits.Velocity vSlide "Sliding velocity";
      parameter Real mu_A "Friction coefficient at adhesion";
      parameter Real mu_S "Friction coefficient at sliding";

        Modelica.SIunits.Distance penetration;
          parameter Real c = 1e5 "collision spring constant";
      parameter Real d = 1e3 "collision damping";

      Modelica.SIunits.Force N "Normal force";

      Praktikum11.HybridCar3D.PartialDryFrictionWheelJoint3D partialDryFrictionWheelJoint3D(
        radius=radius,
        r={1,0},
        vAdhesion=vAdhesion,
        vSlide=vSlide,
        mu_A=mu_A,
        mu_S=mu_S,
        animate=false)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute(useAxisFlange=true, n(
            displayUnit="1") = {0,1,0})
        annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_chassis
        annotation (Placement(transformation(extent={{-116,-16},{-84,16}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_wheel
        annotation (Placement(transformation(extent={{84,-16},{116,16}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange
        annotation (Placement(transformation(extent={{-40,90},{-20,110}}),
            iconTransformation(extent={{-40,90},{-20,110}})));
    equation

      N = -partialDryFrictionWheelJoint3D.N;

       penetration = frame_chassis.r_0[3]-radius;

      if penetration < 0.0 then
        N = penetration*c + der(penetration)*d;
      else
        N = 0.0;
      end if;

      connect(partialDryFrictionWheelJoint3D.flange_a, revolute.axis) annotation (
          Line(points={{10,0},{20,0},{20,-20},{0,-20},{0,-30}}, color={0,0,0}));
      connect(revolute.frame_b, frame_wheel) annotation (Line(
          points={{10,-40},{60,-40},{60,0},{100,0}},
          color={95,95,95},
          thickness=0.5));
      connect(partialDryFrictionWheelJoint3D.frame_a, frame_chassis) annotation (
          Line(
          points={{-4,0},{-100,0},{-100,0}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute.frame_a, frame_chassis) annotation (Line(
          points={{-10,-40},{-60,-40},{-60,0},{-100,0}},
          color={95,95,95},
          thickness=0.5));
      connect(flange, revolute.axis) annotation (Line(points={{-30,100},{20,100},{20,
              -20},{0,-20},{0,-30}},
                                color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{20,100},{100,-100}},
              lineColor={95,95,95},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={231,231,231}),
            Line(
              points={{20,30},{100,30}},
              color={95,95,95}),
            Line(
              points={{20,-30},{100,-30}},
              color={95,95,95}),
            Line(
              points={{20,78},{100,78}},
              color={95,95,95}),
            Rectangle(
              extent={{20,10},{-96,-10}},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={231,231,231}),
            Polygon(
              visible=useAxisFlange,
              points={{-40,28},{-20,28},{0,48},{-60,48},{-40,28}},
              lineColor={64,64,64},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Rectangle(
              visible=useAxisFlange,
              extent={{-40,98},{-20,48}},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={192,192,192}),
            Polygon(
              visible=useAxisFlange,
              points={{-20,28},{0,48},{0,-52},{-20,-32},{-20,28}},
              lineColor={64,64,64},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Polygon(
              visible=useAxisFlange,
              points={{-100,30},{-80,30},{-80,-30},{-100,-30},{-100,30}},
              lineColor={64,64,64},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}),                      Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end DryFrictionWheelJoint3D;

    model HybrdiCar3D
      Driver driver(fileName=
            "C:/Users/sturmdan/Documents/Dymola/sort1_steering.txt",                  k = 50) annotation (
        Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
      ElectricDrive electricDrive annotation (
        Placement(transformation(extent = {{-36, 4}, {-16, 24}})));
      CombustionEngineDrive combustionEngineDrive(omegaStart = 0) annotation (
        Placement(transformation(extent = {{-36, -28}, {-16, -8}})));
      Modelica.Blocks.Math.Gain gain(k = 0.5) annotation (
        Placement(transformation(extent = {{-52, -4}, {-44, 4}})));
      DriveTrain2D driveTrain2D annotation (
        Placement(transformation(extent = {{2, -10}, {22, 10}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport=false)
        annotation (Placement(transformation(extent={{-32,46},{-12,66}})));
      SteeringLimiter steeringLimiter
        annotation (Placement(transformation(extent={{18,46},{38,66}})));
      inner Modelica.Mechanics.MultiBody.World world(n(displayUnit="1") = {0,0,
          -1})
        annotation (Placement(transformation(extent={{-78,-86},{-58,-66}})));
      Body3D body3D(wheelRadius=0.31)
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Chassis3D chassis3D(
        wheelBase=2.578,
        trackWidth=1.549,
        wheelRadius=0.31,
        carMass=1376)
        annotation (Placement(transformation(extent={{32,-10},{52,10}})));
    equation
      connect(gain.u, driver.car_torque) annotation (
        Line(points = {{-52.8, 0}, {-63.6, 0}}, color = {0, 0, 127}));
      connect(gain.y, combustionEngineDrive.u) annotation (
        Line(points = {{-43.6, 0}, {-42, 0}, {-42, -18}, {-36.2, -18}}, color = {0, 0, 127}));
      connect(electricDrive.u, gain.y) annotation (
        Line(points = {{-36.4, 14}, {-42, 14}, {-42, 0}, {-43.6, 0}}, color = {0, 0, 127}));
      connect(electricDrive.flange_a, driveTrain2D.flange_a) annotation (
        Line(points = {{-16.2, 14}, {-8, 14}, {-8, 0}, {2, 0}}, color = {0, 0, 0}));
      connect(combustionEngineDrive.flange_a, driveTrain2D.flange_a) annotation (
        Line(points = {{-16, -18}, {-8, -18}, {-8, 0}, {2, 0}}, color = {0, 0, 0}));
      connect(torque.flange, steeringLimiter.flange_a)
        annotation (Line(points={{-12,56},{18,56}},
                                                  color={0,0,0}));
      connect(driver.steering_angle, torque.tau) annotation (Line(points={{-61.2,
              5.6},{-61.2,56},{-34,56}}, color={0,0,127}));
      connect(chassis3D.frame, body3D.frame) annotation (Line(points={{50.2,
              -0.4},{54.05,-0.4},{54.05,0.2},{62.2,0.2}}, color={0,0,0}));
      connect(chassis3D.HR, driveTrain2D.HR) annotation (Line(points={{32,-8.4},
              {26,-8.4},{26,-6},{22,-6}}, color={0,0,0}));
      connect(chassis3D.HL, driveTrain2D.HL) annotation (Line(points={{32,-5},{
              26,-5},{26,-2},{22,-2}}, color={0,0,0}));
      connect(chassis3D.VR, driveTrain2D.VR)
        annotation (Line(points={{32,0},{26,0},{26,2},{22,2}},
                                                 color={0,0,0}));
      connect(chassis3D.VL, driveTrain2D.VL)
        annotation (Line(points={{32,4},{26,4},{26,6},{22,6}},
                                                 color={0,0,0}));
      connect(body3D.speed, driver.car_speed) annotation (Line(points={{80.6,
              -4.6},{90,-4.6},{90,-38},{-92,-38},{-92,-3},{-78.4,-3}}, color={0,
              0,127}));
      connect(body3D.angle_Deg, driver.driving_angle_deg) annotation (Line(
            points={{78.2,8.2},{96,8.2},{96,-50},{-96,-50},{-96,8.8},{-76.8,8.8}},
            color={0,0,127}));
      connect(steeringLimiter.flange_b, chassis3D.Steering) annotation (Line(
            points={{38,56},{46,56},{46,10},{47.4,10}}, color={0,0,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
    end HybrdiCar3D;

    model WheelSuspensionRight
      Modelica.Mechanics.MultiBody.Interfaces.Frame frame
        annotation (Placement(transformation(extent={{-106,-46},{-94,-34}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame frame1
        annotation (Placement(transformation(extent={{92,-84},{104,-72}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r={0,0.35,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={12,-78})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r={0,0,
            0.3})     annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={72,-26})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute3(n(displayUnit="1")=
             {1,0,0}, phi(start=3.1415926535898))
                                    annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={50,-8})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r={0,0,
            0.35})     annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={72,16})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r={0,0.35,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={12,-8})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(n(displayUnit="1")=
             {1,0,0}, phi(start=0))               annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={48,-78})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute4(n(displayUnit="1")=
             {1,0,0}, phi(start=4.7123889803847, fixed=false)) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-24,-8})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r={0,0.15,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,-62})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute6(n(displayUnit="1")=
             {1,0,0}, phi(start=-2.3561944901923)) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,8})));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(
        n(displayUnit="1") = {0,1,0},
        s(start=0.494974747, fixed=true),
        v(start=0),
        a(start=0)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={8,42})));
      Modelica.Mechanics.MultiBody.Forces.Spring spring(c=10000)
                                                               annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={8,64})));
      Modelica.Mechanics.MultiBody.Forces.Damper damper(d=1000)
                                                               annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={8,18})));
      Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint revolute5(n(
            displayUnit="1") = {1,0,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={50,42})));
      Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint revolute2(n(
            displayUnit="1") = {1,0,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-18,-78})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r={0,-0.15,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,-18})));
      Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation(angle=90)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-70,-40})));
    equation
      connect(fixedTranslation1.frame_a, frame1) annotation (Line(
          points={{72,-36},{72,-78},{98,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute3.frame_a, fixedTranslation1.frame_b) annotation (Line(
          points={{60,-8},{72,-8},{72,-16}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation2.frame_a, fixedTranslation1.frame_b) annotation (
         Line(
          points={{72,6},{72,-16}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation3.frame_a, revolute3.frame_b) annotation (Line(
          points={{22,-8},{40,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute4.frame_a, fixedTranslation3.frame_b) annotation (Line(
          points={{-14,-8},{2,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute5.frame_a, fixedTranslation2.frame_b) annotation (Line(
          points={{60,42},{72,42},{72,26}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute5.frame_b, prismatic.frame_a) annotation (Line(
          points={{40,42},{18,42}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation5.frame_b, fixedTranslation4.frame_b) annotation (
         Line(
          points={{-50,-28},{-50,-52}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute4.frame_b, fixedTranslation5.frame_a) annotation (Line(
          points={{-34,-8},{-50,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(damper.frame_a, prismatic.frame_a) annotation (Line(
          points={{18,18},{28,18},{28,42},{18,42}},
          color={95,95,95},
          thickness=0.5));
      connect(spring.frame_a, prismatic.frame_a) annotation (Line(
          points={{18,64},{28,64},{28,42},{18,42}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute6.frame_b, fixedTranslation5.frame_a) annotation (Line(
          points={{-50,-2},{-50,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute6.frame_a, prismatic.frame_b) annotation (Line(
          points={{-50,18},{-50,42},{-2,42}},
          color={95,95,95},
          thickness=0.5));
      connect(damper.frame_b, prismatic.frame_b) annotation (Line(
          points={{-2,18},{-10,18},{-10,42},{-2,42}},
          color={95,95,95},
          thickness=0.5));
      connect(spring.frame_b, prismatic.frame_b) annotation (Line(
          points={{-2,64},{-10,64},{-10,42},{-2,42}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedRotation.frame_a, fixedTranslation4.frame_b) annotation (
          Line(
          points={{-60,-40},{-50,-40},{-50,-52}},
          color={95,95,95},
          thickness=0.5));
      connect(frame, fixedRotation.frame_b)
        annotation (Line(points={{-100,-40},{-80,-40}}, color={0,0,0}));
      connect(frame, frame) annotation (Line(points={{-100,-40},{-104,-40},{
              -104,-38},{-100,-38},{-100,-40}}, color={0,0,0}));
      connect(fixedTranslation.frame_a, revolute1.frame_b) annotation (Line(
          points={{22,-78},{38,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute1.frame_a, frame1) annotation (Line(
          points={{58,-78},{98,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation.frame_b, revolute2.frame_a) annotation (Line(
          points={{2,-78},{-8,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.frame_b, fixedTranslation4.frame_a) annotation (Line(
          points={{-28,-78},{-50,-78},{-50,-72}},
          color={95,95,95},
          thickness=0.5));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end WheelSuspensionRight;

    model WheelSuspensionLeft
      Modelica.Mechanics.MultiBody.Interfaces.Frame frame
        annotation (Placement(transformation(extent={{-106,-46},{-94,-34}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame frame1
        annotation (Placement(transformation(extent={{92,-84},{104,-72}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r={0,-0.35,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={12,-78})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r={0,0,
            0.3})     annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={72,-26})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute3(n(displayUnit="1")=
             {1,0,0}, phi(start=3.1415926535898))
                                    annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={50,-8})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r={0,0,
            0.35})     annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={72,16})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r={0,-0.35,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={12,-8})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(n(displayUnit="1")=
             {1,0,0}, phi(start=0))               annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={48,-78})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute4(n(displayUnit="1")=
             {1,0,0}, phi(start=1.5707963267949))              annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-24,-8})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r={0,0.15,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,-62})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute6(n(displayUnit="1")=
             {1,0,0}, phi(start=-2.3561944901923)) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,8})));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(
        n(displayUnit="1") = {0,1,0},
        s(start=0.494974747, fixed=true),
        v(start=0),
        a(start=0)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={8,42})));
      Modelica.Mechanics.MultiBody.Forces.Spring spring(c=10000)
                                                               annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={8,64})));
      Modelica.Mechanics.MultiBody.Forces.Damper damper(d=1000)
                                                               annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={8,18})));
      Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint revolute5(n(
            displayUnit="1") = {1,0,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={50,42})));
      Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint revolute2(n(
            displayUnit="1") = {1,0,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-18,-78})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r={0,-0.15,
            0})        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,-18})));
      Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation(angle=270)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-70,-40})));
    equation
      connect(fixedTranslation1.frame_a, frame1) annotation (Line(
          points={{72,-36},{72,-78},{98,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute3.frame_a, fixedTranslation1.frame_b) annotation (Line(
          points={{60,-8},{72,-8},{72,-16}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation2.frame_a, fixedTranslation1.frame_b) annotation (
         Line(
          points={{72,6},{72,-16}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation3.frame_a, revolute3.frame_b) annotation (Line(
          points={{22,-8},{40,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute4.frame_a, fixedTranslation3.frame_b) annotation (Line(
          points={{-14,-8},{2,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute5.frame_a, fixedTranslation2.frame_b) annotation (Line(
          points={{60,42},{72,42},{72,26}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute5.frame_b, prismatic.frame_a) annotation (Line(
          points={{40,42},{18,42}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation5.frame_b, fixedTranslation4.frame_b) annotation (
         Line(
          points={{-50,-28},{-50,-52}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute4.frame_b, fixedTranslation5.frame_a) annotation (Line(
          points={{-34,-8},{-50,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(damper.frame_a, prismatic.frame_a) annotation (Line(
          points={{18,18},{28,18},{28,42},{18,42}},
          color={95,95,95},
          thickness=0.5));
      connect(spring.frame_a, prismatic.frame_a) annotation (Line(
          points={{18,64},{28,64},{28,42},{18,42}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute6.frame_b, fixedTranslation5.frame_a) annotation (Line(
          points={{-50,-2},{-50,-8}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute6.frame_a, prismatic.frame_b) annotation (Line(
          points={{-50,18},{-50,42},{-2,42}},
          color={95,95,95},
          thickness=0.5));
      connect(damper.frame_b, prismatic.frame_b) annotation (Line(
          points={{-2,18},{-10,18},{-10,42},{-2,42}},
          color={95,95,95},
          thickness=0.5));
      connect(spring.frame_b, prismatic.frame_b) annotation (Line(
          points={{-2,64},{-10,64},{-10,42},{-2,42}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedRotation.frame_a, fixedTranslation4.frame_b) annotation (
          Line(
          points={{-60,-40},{-50,-40},{-50,-52}},
          color={95,95,95},
          thickness=0.5));
      connect(frame, fixedRotation.frame_b)
        annotation (Line(points={{-100,-40},{-80,-40}}, color={0,0,0}));
      connect(frame, frame) annotation (Line(points={{-100,-40},{-104,-40},{
              -104,-38},{-100,-38},{-100,-40}}, color={0,0,0}));
      connect(fixedTranslation.frame_a, revolute1.frame_b) annotation (Line(
          points={{22,-78},{38,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute1.frame_a, frame1) annotation (Line(
          points={{58,-78},{98,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation.frame_b, revolute2.frame_a) annotation (Line(
          points={{2,-78},{-8,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.frame_b, fixedTranslation4.frame_a) annotation (Line(
          points={{-28,-78},{-50,-78},{-50,-72}},
          color={95,95,95},
          thickness=0.5));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end WheelSuspensionLeft;
  end HybridCar3D;

  model WheelSuspensionTestRight
    inner Modelica.Mechanics.MultiBody.World world(n(displayUnit="1") = {0,0,-1})
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=0,
          origin={76,-14})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape1(
      animateSphere=false,
      r={0,0.1,0},
      r_CM={0,0,0},
      m=19,
      I_11=0.653,
      I_22=1.055,
      I_33=0.653,
      shapeType="cylinder",
      widthDirection(displayUnit="1") = {0,1,0},
      width=0.62,
      color={0,128,255})
                  annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-50,-14})));
    HybridCar3D.WheelSuspensionRight wheelSuspensionRight
      annotation (Placement(transformation(extent={{-6,-22},{14,-2}})));
  equation
    connect(bodyShape1.frame_a, wheelSuspensionRight.frame) annotation (Line(
        points={{-40,-14},{-24,-14},{-24,-16},{-6,-16}},
        color={95,95,95},
        thickness=0.5));
    connect(wheelSuspensionRight.frame1, world.frame_b) annotation (Line(points=
           {{13.8,-19.8},{39.9,-19.8},{39.9,-14},{66,-14}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end WheelSuspensionTestRight;

  model WheelSuspensionTestLeft
    inner Modelica.Mechanics.MultiBody.World world(n(displayUnit="1") = {0,0,-1})
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=0,
          origin={76,-14})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape1(
      animateSphere=false,
      r={0,0.1,0},
      r_CM={0,0,0},
      m=19,
      I_11=0.653,
      I_22=1.055,
      I_33=0.653,
      shapeType="cylinder",
      widthDirection(displayUnit="1") = {0,1,0},
      width=0.62,
      color={0,128,255})
                  annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-50,-14})));
    HybridCar3D.WheelSuspensionLeft wheelSuspensionLeft
      annotation (Placement(transformation(extent={{0,-24},{20,-4}})));
  equation
    connect(wheelSuspensionLeft.frame, bodyShape1.frame_a) annotation (Line(
          points={{0,-18},{-20,-18},{-20,-14},{-40,-14}}, color={0,0,0}));
    connect(wheelSuspensionLeft.frame1, world.frame_b) annotation (Line(points=
            {{19.8,-21.8},{42.9,-21.8},{42.9,-14},{66,-14}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end WheelSuspensionTestLeft;
  annotation (
    uses(Modelica(version = "3.2.3")));
end Praktikum11;
