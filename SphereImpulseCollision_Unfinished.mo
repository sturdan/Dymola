model SphereImpulseCollision_Unfinished
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
    annotation (Placement(transformation(extent={{-116,-16},{-84,16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b
    annotation (Placement(transformation(extent={{84,-16},{116,16}})));

    parameter Modelica.SIunits.Radius sphereARadius "Radius Kugel A";
    parameter Modelica.SIunits.Radius sphereBRadius "Radius Kugel B";

    Modelica.SIunits.Velocity sphereAVelocity[3] "Geschwindigkeit frame_a.";
    Modelica.SIunits.Velocity sphereBVelocity[3] "Geschwindigkeit frame_b.";

    Modelica.Blocks.Interfaces.RealOutput aVelocity "Y-Geschwindigkeit für Reinitialisierung von frame_a." annotation (
      Placement(transformation(extent={{-22,-78},{-2,-58}}),
        iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,-100})));
        
    Modelica.Blocks.Interfaces.RealOutput bVelocity "Y-Geschwindigkeit für Reinitialisierung von frame_b." annotation (
      Placement(transformation(extent={{-22,-78},{-2,-58}}),
        iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={80,-100})));
        
    Modelica.Blocks.Interfaces.BooleanOutput reinitTrigger "Trigger zum Reinitialisieren der Y-Geschwindigkeiten gdw. Kollision."
    annotation (Placement(transformation(extent={{-38,-102},{-18,-82}}),
        iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-100})));

  final parameter Modelica.SIunits.Velocity velocityThreshold = 1e-3 "Geschwindigkeiten unter 1e-3 m/s werden als 0 angenommen.";
  final parameter Modelica.SIunits.Distance collisionThreshold = 1e-8 "Kollisionstiefen unter 1e-8 m/s werden als keine Kollision angenommen.";

equation 

  /* [Abstand der Frames zueinander] < [minimaler Abstand ohne Kollision] - collisionThreshold */
  /* ToDo: reinitTrigger = ?; */

  /*Geschwindigkeit frame_a berechnen*/
  /* ToDo: sphereAVelocity = ?; */
  
  /*Geschwindigkeit frame_a berechnen*/
  /* ToDo: sphereBVelocity = ?; */

  if abs(sphereBVelocity[2]) < velocityThreshold then
    /* ToDo: aVelocity = ?; */
  else
    /* ToDo: aVelocity = ?; */
  end if;

  if abs(sphereAVelocity[2]) < velocityThreshold then
    /* ToDo: bVelocity = ?; */
  else
    /* ToDo: bVelocity = ?; */
  end if;

  /* Setzen der Momente und Kräfte auf frame_a und frame_b */
  /* ToDo ... */

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{0,50},{-100,-50}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Solid,
          fillColor={28,108,200}),
        Line(
          points={{0,80},{0,-80}},
          color={238,46,47},
          thickness=0.5),
        Ellipse(
          extent={{100,50},{0,-50}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Solid,
          fillColor={28,108,200})}),                             Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SphereImpulseCollision_Unfinished;