within ;
package Praktikum13
  model Newton_Pendel

    inner Modelica.Mechanics.MultiBody.World world(n(displayUnit="1") = {0,0,-1})
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r={0,0,-0.3})
                annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-70,14})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r={0,0,-0.3})
                annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-30,14})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r={0,0,-0.3})
                annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={10,14})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r={0,0,-0.3})
                annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={54,14})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r={0,0,-0.3})
                annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={94,14})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation6(r={0,0.1,
          0})      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-48,56})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation7(r={0,0.1,
          0})      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-10,56})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation8(r={0,0.1,
          0})      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={32,56})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation9(r={0,0.1,
          0})      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={74,56})));
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute(n(displayUnit="1") =
        {1,0,0}, phi(fixed=true, start=1.3962634015955))
                               annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-70,40})));
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(n(displayUnit="1")
         = {1,0,0}, phi(fixed=true, start=1.3962634015955))
                                 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-30,40})));
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute2(n(displayUnit="1")
         = {1,0,0}, phi(fixed=true))
                                 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={10,40})));
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute3(n(displayUnit="1")
         = {1,0,0}, phi(fixed=true))
                                 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={54,40})));
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute4(n(displayUnit="1")
         = {1,0,0}, phi(fixed=true))
                                 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={94,40})));
    BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished(sphereDiameter=
          0.1) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-70,-22})));
    BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished1(sphereDiameter
        =0.1, doubleInput=true) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-30,-22})));
    BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished2(sphereDiameter
        =0.1, doubleInput=true) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={10,-22})));
    BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished3(sphereDiameter
        =0.1, doubleInput=true) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={54,-22})));
    BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished4(sphereDiameter
        =0.1) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={94,-22})));
    SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished(
        sphereARadius=0.05, sphereBRadius=0.05)
      annotation (Placement(transformation(extent={{-62,-16},{-42,4}})));
    SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished1(
        sphereARadius=0.05, sphereBRadius=0.05)
      annotation (Placement(transformation(extent={{-18,-16},{2,4}})));
    SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished2(
        sphereARadius=0.05, sphereBRadius=0.05)
      annotation (Placement(transformation(extent={{22,-16},{42,4}})));
    SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished3(
        sphereARadius=0.05, sphereBRadius=0.05)
      annotation (Placement(transformation(extent={{66,-16},{86,4}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation10(r={0,0.4,
          0})      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-84,56})));
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r={0,0,0.5})
      annotation (Placement(transformation(extent={{-120,46},{-100,66}})));
  equation
    connect(fixedTranslation3.frame_a,revolute2. frame_b) annotation (Line(
        points={{10,24},{10,30}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute4.frame_b,fixedTranslation5. frame_a) annotation (Line(
        points={{94,30},{94,24}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute3.frame_b,fixedTranslation4. frame_a) annotation (Line(
        points={{54,30},{54,24}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute1.frame_b,fixedTranslation2. frame_a) annotation (Line(
        points={{-30,30},{-30,24}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute.frame_b,fixedTranslation1. frame_a) annotation (Line(
        points={{-70,30},{-70,24}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute.frame_a,fixedTranslation6. frame_a) annotation (Line(
        points={{-70,50},{-70,56},{-58,56}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation6.frame_b,fixedTranslation7. frame_a) annotation (
        Line(
        points={{-38,56},{-20,56}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation7.frame_b,fixedTranslation8. frame_a) annotation (
        Line(
        points={{0,56},{22,56}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation8.frame_b,fixedTranslation9. frame_a) annotation (
        Line(
        points={{42,56},{64,56}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation9.frame_b,revolute4. frame_a) annotation (Line(
        points={{84,56},{94,56},{94,50}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute3.frame_a,fixedTranslation9. frame_a) annotation (Line(
        points={{54,50},{54,56},{64,56}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute2.frame_a,fixedTranslation8. frame_a) annotation (Line(
        points={{10,50},{10,56},{22,56}},
        color={95,95,95},
        thickness=0.5));
    connect(revolute1.frame_a,fixedTranslation7. frame_a) annotation (Line(
        points={{-30,50},{-30,56},{-20,56}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation1.frame_b, bodyExternalReinit_Unfinished.frame)
      annotation (Line(
        points={{-70,4},{-70,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation2.frame_b, bodyExternalReinit_Unfinished1.frame)
      annotation (Line(
        points={{-30,4},{-30,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation3.frame_b, bodyExternalReinit_Unfinished2.frame)
      annotation (Line(
        points={{10,4},{10,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation4.frame_b, bodyExternalReinit_Unfinished3.frame)
      annotation (Line(
        points={{54,4},{54,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation5.frame_b, bodyExternalReinit_Unfinished4.frame)
      annotation (Line(
        points={{94,4},{94,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(bodyExternalReinit_Unfinished.frame,
      sphereImpulseCollision_Unfinished.frame_a) annotation (Line(
        points={{-70,-12},{-66,-12},{-66,-6},{-62,-6}},
        color={95,95,95},
        thickness=0.5));
    connect(bodyExternalReinit_Unfinished1.frame,
      sphereImpulseCollision_Unfinished.frame_b) annotation (Line(
        points={{-30,-12},{-36,-12},{-36,-6},{-42,-6}},
        color={95,95,95},
        thickness=0.5));
    connect(bodyExternalReinit_Unfinished1.frame,
      sphereImpulseCollision_Unfinished1.frame_a) annotation (Line(
        points={{-30,-12},{-26,-12},{-26,-6},{-18,-6}},
        color={95,95,95},
        thickness=0.5));
    connect(sphereImpulseCollision_Unfinished1.frame_b,
      bodyExternalReinit_Unfinished2.frame) annotation (Line(
        points={{2,-6},{6,-6},{6,-12},{10,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(sphereImpulseCollision_Unfinished2.frame_a,
      bodyExternalReinit_Unfinished2.frame) annotation (Line(
        points={{22,-6},{18,-6},{18,-12},{10,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(sphereImpulseCollision_Unfinished2.frame_b,
      bodyExternalReinit_Unfinished3.frame) annotation (Line(
        points={{42,-6},{50,-6},{50,-12},{54,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(sphereImpulseCollision_Unfinished3.frame_a,
      bodyExternalReinit_Unfinished3.frame) annotation (Line(
        points={{66,-6},{60,-6},{60,-12},{54,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(sphereImpulseCollision_Unfinished3.frame_b,
      bodyExternalReinit_Unfinished4.frame) annotation (Line(
        points={{86,-6},{90,-6},{90,-12},{94,-12}},
        color={95,95,95},
        thickness=0.5));
    connect(sphereImpulseCollision_Unfinished3.bVelocity,
      bodyExternalReinit_Unfinished4.reinitVelocityY1) annotation (Line(points=
            {{84,-16},{84,-42},{90,-42},{90,-32}}, color={0,0,127}));
    connect(sphereImpulseCollision_Unfinished3.reinitTrigger,
      bodyExternalReinit_Unfinished4.reinitTrigger1) annotation (Line(points={{
            76,-16},{76,-48},{86,-48},{86,-32}}, color={255,0,255}));
    connect(bodyExternalReinit_Unfinished3.reinitTrigger2_external,
      bodyExternalReinit_Unfinished4.reinitTrigger1) annotation (Line(points={{
            62,-32},{62,-48},{86,-48},{86,-32}}, color={255,0,255}));
    connect(bodyExternalReinit_Unfinished3.reinitVelocityY2_external,
      sphereImpulseCollision_Unfinished3.aVelocity) annotation (Line(points={{
            58,-32},{58,-42},{68,-42},{68,-16}}, color={0,0,127}));
    connect(bodyExternalReinit_Unfinished3.reinitVelocityY1,
      sphereImpulseCollision_Unfinished2.bVelocity) annotation (Line(points={{
            50,-32},{50,-42},{40,-42},{40,-16}}, color={0,0,127}));
    connect(bodyExternalReinit_Unfinished3.reinitTrigger1,
      sphereImpulseCollision_Unfinished2.reinitTrigger) annotation (Line(points
          ={{46,-32},{46,-48},{32,-48},{32,-16}}, color={255,0,255}));
    connect(bodyExternalReinit_Unfinished2.reinitTrigger2_external,
      sphereImpulseCollision_Unfinished2.reinitTrigger) annotation (Line(points
          ={{18,-32},{18,-48},{32,-48},{32,-16}}, color={255,0,255}));
    connect(bodyExternalReinit_Unfinished2.reinitVelocityY2_external,
      sphereImpulseCollision_Unfinished2.aVelocity) annotation (Line(points={{
            14,-32},{14,-42},{24,-42},{24,-16}}, color={0,0,127}));
    connect(bodyExternalReinit_Unfinished2.reinitTrigger1,
      sphereImpulseCollision_Unfinished1.reinitTrigger) annotation (Line(points
          ={{2,-32},{2,-48},{-8,-48},{-8,-16}}, color={255,0,255}));
    connect(sphereImpulseCollision_Unfinished1.bVelocity,
      bodyExternalReinit_Unfinished2.reinitVelocityY1) annotation (Line(points=
            {{0,-16},{0,-42},{6,-42},{6,-32}}, color={0,0,127}));
    connect(bodyExternalReinit_Unfinished1.reinitTrigger2_external,
      sphereImpulseCollision_Unfinished1.reinitTrigger) annotation (Line(points
          ={{-22,-32},{-22,-48},{-8,-48},{-8,-16}}, color={255,0,255}));
    connect(sphereImpulseCollision_Unfinished1.aVelocity,
      bodyExternalReinit_Unfinished1.reinitVelocityY2_external) annotation (
        Line(points={{-16,-16},{-16,-42},{-26,-42},{-26,-32}}, color={0,0,127}));
    connect(sphereImpulseCollision_Unfinished.bVelocity,
      bodyExternalReinit_Unfinished1.reinitVelocityY1) annotation (Line(points=
            {{-44,-16},{-44,-42},{-34,-42},{-34,-32}}, color={0,0,127}));
    connect(bodyExternalReinit_Unfinished1.reinitTrigger1,
      sphereImpulseCollision_Unfinished.reinitTrigger) annotation (Line(points=
            {{-38,-32},{-38,-48},{-52,-48},{-52,-16}}, color={255,0,255}));
    connect(bodyExternalReinit_Unfinished.reinitTrigger1,
      sphereImpulseCollision_Unfinished.reinitTrigger) annotation (Line(points=
            {{-62,-32},{-62,-48},{-52,-48},{-52,-16}}, color={255,0,255}));
    connect(bodyExternalReinit_Unfinished.reinitVelocityY1,
      sphereImpulseCollision_Unfinished.aVelocity) annotation (Line(points={{
            -66,-32},{-66,-42},{-60,-42},{-60,-16}}, color={0,0,127}));
    connect(fixed.frame_b, fixedTranslation10.frame_a) annotation (Line(
        points={{-100,56},{-94,56}},
        color={95,95,95},
        thickness=0.5));
    connect(fixedTranslation10.frame_b, fixedTranslation6.frame_a) annotation (
        Line(
        points={{-74,56},{-58,56}},
        color={95,95,95},
        thickness=0.5));
  end Newton_Pendel;

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
    reinitTrigger = sqrt((frame_a.r_0-frame_b.r_0)*(frame_a.r_0-frame_b.r_0)) < 0.1 - collisionThreshold;


    /*Geschwindigkeit frame_a berechnen*/
    sphereAVelocity = der(frame_a.r_0);

    /*Geschwindigkeit frame_b berechnen*/
    sphereBVelocity = der(frame_b.r_0);

    if abs(sphereBVelocity[2]) < velocityThreshold then
      aVelocity = 0;
    else
      aVelocity = sphereBVelocity[2];
    end if;

    if abs(sphereAVelocity[2]) < velocityThreshold then
      bVelocity = 0;
    else
      bVelocity = sphereAVelocity[2];
    end if;

    /* Setzen der Momente und Kräfte auf frame_a und frame_b */
    frame_a.f = {0,0,0};
    frame_b.f = {0,0,0};
    frame_a.t = {0,0,0};
    frame_b.t = {0,0,0};

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

    connect(reinitVelocityY2_external, reinitVelocityY2);
    connect(reinitTrigger2_external, reinitTrigger2);

    if not doubleInput then /* wir nutzen nur einen Input */
      reinitTrigger2 = false;
      reinitVelocityY2 = 0;
    end if;

    /* Setzen gdw. mindesten eines von reinitTrigger1 oder reinitTrigger2 gesetzt ist */
    reinitTrigger = reinitTrigger1 or reinitTrigger2;

    if reinitTrigger1 then
      reinitVelocityY = reinitVelocityY1;
    else
      if reinitTrigger2 then
        reinitVelocityY = reinitVelocityY2;
      else
        reinitVelocityY = 0;
      end if;
    end if;

    when reinitTrigger then /* die Geschwindigkeit in Y-Richtung mit neuem Wert initialisieren */
      reinit(bodyShape.v_0[2], reinitVelocityY);
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
  annotation (uses(Modelica(version="3.2.3")));
end Praktikum13;
