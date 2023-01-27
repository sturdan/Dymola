model BodyExternalReinit_Unfinished

  parameter Modelica.SIunits.Diameter sphereDiameter "Kugel Durchmesser";

  Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(
    animation=true,
    animateSphere=false,
    r={0,0,0},
    r_CM={0,0,0},
    m=1,
    shapeType="sphere",
    length=sphereDiameter,
    width=sphereDiameter,
    height=sphereDiameter) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,0})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame annotation (
     Placement(transformation(extent={{-116,-16},{-84,16}})));

  parameter Boolean doubleInput = false "Sollen zwei Kontakte unterstützt werden?";

  Modelica.Blocks.Interfaces.BooleanInput reinitTrigger2_external if doubleInput
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,40}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,80})));

  Modelica.Blocks.Interfaces.RealInput reinitVelocityY2_external if doubleInput annotation (
     Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,-40}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,40})));

  Modelica.Blocks.Interfaces.RealInput reinitVelocityY1
                                                       annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,-80}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,-40})));

  Modelica.Blocks.Interfaces.BooleanInput reinitTrigger1
                                                        annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,80}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={100,-80})));

  Boolean reinitTrigger "Trigger zum Reinitialisieren der Y-Geschwindigkeiten.";
  Modelica.SIunits.Velocity reinitVelocityY "Y-Geschwindigkeiten für die Reinitialisierung.";

protected
  Modelica.Blocks.Interfaces.BooleanInput reinitTrigger2;
  Modelica.Blocks.Interfaces.RealInput reinitVelocityY2;

equation

  /* ToDo: connect(?, ?); */
  /* ToDo: connect(?, ?); */

  if not doubleInput then /* wir nutzen nur einen Input */
    /* ToDo: reinitTrigger2 = ?; */
    /* ToDo: reinitVelocityY2 = ?; */
  end if;

  /* Setzen gdw. mindesten eines von reinitTrigger1 oder reinitTrigger2 gesetzt ist */
  /* ToDo: reinitTrigger = ?; */

  if reinitTrigger1 then
    /* ToDo: reinitVelocityY = ?; */
  else
    if reinitTrigger2 then
      /* ToDo: reinitVelocityY = ?; */
    else
      /* ToDo: reinitVelocityY = ?; */
    end if;
  end if;

  when reinitTrigger then /* die Geschwindigkeit in Y-Richtung mit neuem Wert initialisieren */
    /* ToDo: reinit(?, ?); */
  end when;

  connect(frame, bodyShape.frame_a) annotation (Line(
      points={{-100,0},{-10,0}},
      color={95,95,95},
      thickness=0.5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false),
        graphics={
        Text(
          extent={{-150,110},{150,70}},
          textString="%name",
          lineColor={0,0,255}),
        Text(
          extent={{-150,-100},{150,-70}},
          textString="r=%r"),
        Rectangle(
          extent={{-100,30},{52,-30}},
          lineColor={0,24,48},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255},
          radius=10),
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,24,48},
          fillPattern=FillPattern.Sphere,
          fillColor={0,127,255}),
        Text(
          extent={{-50,24},{55,-27}},
          textString="%m")}),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
end BodyExternalReinit_Unfinished;
