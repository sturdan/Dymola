within ;
package Praktikum5
  model ElectricDrive
    parameter Real maxTorque = 150;

    Modelica.Blocks.Interfaces.RealInput u
      annotation (Placement(transformation(extent={{-124,-20},{-84,20}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque
      annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
      annotation (Placement(transformation(extent={{88,-10},{108,10}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1)
      annotation (Placement(transformation(extent={{42,-10},{62,10}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(T=0.1)
      annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=maxTorque)
      annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  equation
    connect(torque.flange, inertia.flange_a)
      annotation (Line(points={{14,0},{42,0}}, color={0,0,0}));
    connect(inertia.flange_b, flange_a)
      annotation (Line(points={{62,0},{98,0}}, color={0,0,0}));
    connect(u, firstOrder.u)
      annotation (Line(points={{-104,0},{-74,0}}, color={0,0,127}));
    connect(firstOrder.y, limiter.u)
      annotation (Line(points={{-51,0},{-42,0}}, color={0,0,127}));
    connect(limiter.y, torque.tau)
      annotation (Line(points={{-19,0},{-8,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ElectricDrive;

  model DriveTrain
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=6)
      annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  equation
    connect(idealGear.flange_b, flange_b)
      annotation (Line(points={{12,0},{100,0}}, color={0,0,0}));
    connect(idealGear.flange_a, flange_a)
      annotation (Line(points={{-8,0},{-100,0}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DriveTrain;

  model Chassis1D
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
      annotation (Placement(transformation(extent={{-84,-12},{-64,8}})));
    Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a1
      annotation (Placement(transformation(extent={{76,-12},{96,8}})));
    Modelica.Mechanics.Rotational.Components.IdealRollingWheel
      idealRollingWheel(radius=0.31)
      annotation (Placement(transformation(extent={{-28,-14},{-8,6}})));
  equation
    connect(idealRollingWheel.flangeT, flange_a1) annotation (Line(points={{-8,
            -4},{38,-4},{38,-2},{86,-2}}, color={0,127,0}));
    connect(idealRollingWheel.flangeR, flange_a) annotation (Line(points={{-28,
            -4},{-52,-4},{-52,-2},{-74,-2}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Chassis1D;

  model Body1D
    Modelica.Mechanics.Translational.Components.Mass mass(m=1300)
      annotation (Placement(transformation(extent={{-14,-6},{6,14}})));
    Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a
      annotation (Placement(transformation(extent={{-96,-6},{-76,14}})));
    Modelica.Mechanics.Translational.Sources.QuadraticSpeedDependentForce
      quadraticSpeedDependentForce(f_nominal=-35.478, v_nominal(displayUnit=
            "m/s") = 10)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=0,
          origin={72,4})));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{22,-80},{42,-60}})));
    Modelica.Blocks.Interfaces.RealOutput y
      annotation (Placement(transformation(extent={{72,-80},{92,-60}})));
  equation
    connect(quadraticSpeedDependentForce.flange, mass.flange_b)
      annotation (Line(points={{62,4},{6,4}}, color={0,127,0}));
    connect(flange_a, flange_a) annotation (Line(points={{-86,4},{-86,4}},
                      color={0,127,0}));
    connect(mass.flange_a, flange_a)
      annotation (Line(points={{-14,4},{-86,4}}, color={0,127,0}));
    connect(mass.flange_a, speedSensor.flange)
      annotation (Line(points={{-14,4},{-14,-70},{22,-70}}, color={0,127,0}));
    connect(speedSensor.v, y)
      annotation (Line(points={{43,-70},{82,-70}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Body1D;

  model Driver
    parameter String fileName="NoName" "File where matrix is stored"
    annotation(Dialog(
      group="Table data definition",
      loadSelector(filter="Text files (*.txt);;MATLAB MAT-files (*.mat)",
      caption="Open file in which table is present")));
    parameter Real k;

    Modelica.Blocks.Interfaces.RealInput car_speed
      annotation (Placement(transformation(extent={{-98,-44},{-70,-16}})));
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
      tableOnFile=true,
      tableName="Cycle",
      fileName=fileName,
      columns={2},
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
      annotation (Placement(transformation(extent={{-96,46},{-76,66}})));
    Modelica.Blocks.Math.Feedback feedback
      annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
    Modelica.Blocks.Math.Gain KmhMs(k=1/3.6) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-72,18})));
    Modelica.Blocks.Interfaces.RealOutput car_torque
      annotation (Placement(transformation(extent={{54,-10},{74,10}})));
    Modelica.Blocks.Math.Gain gain(k=k)
      annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
  equation
    connect(combiTimeTable.y[1], KmhMs.u) annotation (Line(points={{-75,56},{
            -65.5,56},{-65.5,30},{-72,30}},    color={0,0,127}));
    connect(car_speed, feedback.u2) annotation (Line(points={{-84,-30},{-62,-30},{
            -62,-8},{-50,-8}}, color={0,0,127}));
    connect(KmhMs.y, feedback.u1) annotation (Line(points={{-72,7},{-74,7},{-74,
            0},{-58,0}},
                      color={0,0,127}));
    connect(feedback.y, gain.u)
      annotation (Line(points={{-41,0},{-8,0}}, color={0,0,127}));
    connect(gain.y, car_torque)
      annotation (Line(points={{15,0},{64,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Driver;

  model HybrdiCar1D
    Driver driver(fileName="C:/Users/sturmdan/Documents/Dymola/sort1.txt", k=50)
      annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
    Body1D body1D
      annotation (Placement(transformation(extent={{56,-10},{76,10}})));
    Chassis1D chassis1D
      annotation (Placement(transformation(extent={{26,-10},{46,10}})));
    DriveTrain driveTrain
      annotation (Placement(transformation(extent={{-4,-10},{16,10}})));
    ElectricDrive electricDrive
      annotation (Placement(transformation(extent={{-36,-10},{-16,10}})));
  equation
    connect(chassis1D.flange_a1, body1D.flange_a) annotation (Line(points={{44.6,
            -0.2},{42.8,-0.2},{42.8,0.4},{57.4,0.4}},  color={0,127,0}));
    connect(driveTrain.flange_b, chassis1D.flange_a) annotation (Line(points={{
            16,0},{22,0},{22,-0.2},{28.6,-0.2}}, color={0,0,0}));
    connect(electricDrive.flange_a, driveTrain.flange_a)
      annotation (Line(points={{-16.2,0},{-4,0}}, color={0,0,0}));
    connect(driver.car_torque, electricDrive.u)
      annotation (Line(points={{-51.6,0},{-36.4,0}}, color={0,0,127}));
    connect(body1D.y, driver.car_speed) annotation (Line(points={{74.2,-7},{94,
            -7},{94,-46},{-88,-46},{-88,-3},{-66.4,-3}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=100, __Dymola_Algorithm="Dassl"));
  end HybrdiCar1D;
  annotation (uses(Modelica(version="3.2.3")));
end Praktikum5;
