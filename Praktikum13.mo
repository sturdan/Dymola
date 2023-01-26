within ;
model Praktikum13
  inner Modelica.Mechanics.MultiBody.World world(n(displayUnit="1") = {0,0,-1})
    annotation (Placement(transformation(extent={{-96,-90},{-76,-70}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r={0,0,
        0.3}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-76,4})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r={0,0,
        0.3}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-36,4})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r={0,0,
        0.3}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={4,4})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r={0,0,
        0.3}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={48,4})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r={0,0,
        0.3}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={88,4})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation6(r={0,
        0.05,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-54,46})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation7(r={0,
        0.05,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-16,46})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation8(r={0,
        0.05,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={26,46})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation9(r={0,
        0.05,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={68,46})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(n(displayUnit="1") = {1,
      0,0}, phi(fixed=true)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-76,30})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(n(displayUnit="1") = {
      1,0,0}, phi(fixed=true)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-36,30})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute2(n(displayUnit="1") = {
      1,0,0}, phi(fixed=true)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={4,32})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute3(n(displayUnit="1") = {
      1,0,0}, phi(fixed=true)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={48,30})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute4(n(displayUnit="1") = {
      1,0,0}, phi(fixed=true)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={88,30})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r={0,0,1})
    annotation (Placement(transformation(extent={{-110,36},{-90,56}})));
  BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished(sphereDiameter=
        0.1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-76,-26})));
  BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished1(sphereDiameter=
        0.1, doubleInput=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-36,-26})));
  BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished2(sphereDiameter=
        0.1, doubleInput=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={4,-26})));
  BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished3(sphereDiameter=
        0.1, doubleInput=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={48,-26})));
  BodyExternalReinit_Unfinished bodyExternalReinit_Unfinished4(sphereDiameter=
        0.1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={88,-26})));
  SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished(
      sphereARadius=0.05, sphereBRadius=0.05)
    annotation (Placement(transformation(extent={{-66,-26},{-46,-6}})));
  SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished1(
      sphereARadius=0.05, sphereBRadius=0.05)
    annotation (Placement(transformation(extent={{-26,-26},{-6,-6}})));
  SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished2(
      sphereARadius=0.05, sphereBRadius=0.05)
    annotation (Placement(transformation(extent={{16,-26},{36,-6}})));
  SphereImpulseCollision_Unfinished sphereImpulseCollision_Unfinished3(
      sphereARadius=0.05, sphereBRadius=0.05)
    annotation (Placement(transformation(extent={{58,-26},{78,-6}})));
equation
  connect(fixedTranslation3.frame_a, revolute2.frame_b) annotation (Line(
      points={{4,14},{4,22}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute4.frame_b, fixedTranslation5.frame_a) annotation (Line(
      points={{88,20},{88,14}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute3.frame_b, fixedTranslation4.frame_a) annotation (Line(
      points={{48,20},{48,14}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_b, fixedTranslation2.frame_a) annotation (Line(
      points={{-36,20},{-36,14}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, fixedTranslation1.frame_a) annotation (Line(
      points={{-76,20},{-76,14}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_a, fixedTranslation6.frame_a) annotation (Line(
      points={{-76,40},{-76,46},{-64,46}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedTranslation6.frame_b, fixedTranslation7.frame_a) annotation (
      Line(
      points={{-44,46},{-26,46}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedTranslation7.frame_b, fixedTranslation8.frame_a) annotation (
      Line(
      points={{-6,46},{16,46}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedTranslation8.frame_b, fixedTranslation9.frame_a) annotation (
      Line(
      points={{36,46},{58,46}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedTranslation9.frame_b, revolute4.frame_a) annotation (Line(
      points={{78,46},{88,46},{88,40}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute3.frame_a, fixedTranslation9.frame_a) annotation (Line(
      points={{48,40},{48,46},{58,46}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute2.frame_a, fixedTranslation8.frame_a) annotation (Line(
      points={{4,42},{4,46},{16,46}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_a, fixedTranslation7.frame_a) annotation (Line(
      points={{-36,40},{-36,46},{-26,46}},
      color={95,95,95},
      thickness=0.5));
  connect(fixed.frame_b, fixedTranslation6.frame_a) annotation (Line(
      points={{-90,46},{-64,46}},
      color={95,95,95},
      thickness=0.5));
  connect(bodyExternalReinit_Unfinished1.frame, fixedTranslation2.frame_b)
    annotation (Line(
      points={{-36,-16},{-36,-6}},
      color={95,95,95},
      thickness=0.5));
  connect(bodyExternalReinit_Unfinished2.frame, fixedTranslation3.frame_b)
    annotation (Line(
      points={{4,-16},{4,-6}},
      color={95,95,95},
      thickness=0.5));
  connect(bodyExternalReinit_Unfinished3.frame, fixedTranslation4.frame_b)
    annotation (Line(
      points={{48,-16},{48,-6}},
      color={95,95,95},
      thickness=0.5));
  connect(bodyExternalReinit_Unfinished4.frame, fixedTranslation5.frame_b)
    annotation (Line(
      points={{88,-16},{88,-6}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished1.frame_a,
    bodyExternalReinit_Unfinished1.frame) annotation (Line(
      points={{-26,-16},{-36,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished.frame_b,
    bodyExternalReinit_Unfinished1.frame) annotation (Line(
      points={{-46,-16},{-36,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished.frame_a,
    bodyExternalReinit_Unfinished.frame) annotation (Line(
      points={{-66,-16},{-76,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedTranslation1.frame_b, bodyExternalReinit_Unfinished.frame)
    annotation (Line(
      points={{-76,-6},{-76,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished1.frame_b,
    bodyExternalReinit_Unfinished2.frame) annotation (Line(
      points={{-6,-16},{4,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished2.frame_a,
    bodyExternalReinit_Unfinished2.frame) annotation (Line(
      points={{16,-16},{4,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished2.frame_b,
    bodyExternalReinit_Unfinished3.frame) annotation (Line(
      points={{36,-16},{48,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(bodyExternalReinit_Unfinished3.frame,
    sphereImpulseCollision_Unfinished3.frame_a) annotation (Line(
      points={{48,-16},{58,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished3.frame_b,
    bodyExternalReinit_Unfinished4.frame) annotation (Line(
      points={{78,-16},{88,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(sphereImpulseCollision_Unfinished.reinitTrigger,
    bodyExternalReinit_Unfinished1.reinitTrigger1) annotation (Line(points={{
          -56,-26},{-56,-54},{-44,-54},{-44,-36}}, color={255,0,255}));
  connect(bodyExternalReinit_Unfinished.reinitTrigger1,
    bodyExternalReinit_Unfinished1.reinitTrigger1) annotation (Line(points={{
          -84,-36},{-82,-36},{-82,-54},{-44,-54},{-44,-36}}, color={255,0,255}));
  connect(sphereImpulseCollision_Unfinished1.reinitTrigger,
    bodyExternalReinit_Unfinished1.reinitTrigger2_external) annotation (Line(
        points={{-16,-26},{-16,-54},{-28,-54},{-28,-36}}, color={255,0,255}));
  connect(bodyExternalReinit_Unfinished2.reinitTrigger1,
    bodyExternalReinit_Unfinished1.reinitTrigger2_external) annotation (Line(
        points={{-4,-36},{-4,-54},{-28,-54},{-28,-36}}, color={255,0,255}));
  connect(bodyExternalReinit_Unfinished3.reinitTrigger1,
    bodyExternalReinit_Unfinished2.reinitTrigger2_external) annotation (Line(
        points={{40,-36},{40,-54},{12,-54},{12,-36}}, color={255,0,255}));
  connect(bodyExternalReinit_Unfinished3.reinitTrigger2_external,
    sphereImpulseCollision_Unfinished3.reinitTrigger) annotation (Line(points={
          {56,-36},{56,-54},{68,-54},{68,-26}}, color={255,0,255}));
  connect(sphereImpulseCollision_Unfinished2.reinitTrigger,
    bodyExternalReinit_Unfinished2.reinitTrigger2_external) annotation (Line(
        points={{26,-26},{26,-54},{12,-54},{12,-36}}, color={255,0,255}));
  connect(bodyExternalReinit_Unfinished4.reinitTrigger1,
    sphereImpulseCollision_Unfinished3.reinitTrigger) annotation (Line(points={
          {80,-36},{80,-54},{68,-54},{68,-26}}, color={255,0,255}));
  connect(sphereImpulseCollision_Unfinished.aVelocity,
    bodyExternalReinit_Unfinished.reinitVelocityY1) annotation (Line(points={{
          -64,-26},{-64,-46},{-80,-46},{-80,-36}}, color={0,0,127}));
  connect(sphereImpulseCollision_Unfinished.bVelocity,
    bodyExternalReinit_Unfinished1.reinitVelocityY1) annotation (Line(points={{
          -48,-26},{-48,-46},{-40,-46},{-40,-36}}, color={0,0,127}));
  connect(sphereImpulseCollision_Unfinished1.aVelocity,
    bodyExternalReinit_Unfinished1.reinitVelocityY2_external) annotation (Line(
        points={{-24,-26},{-24,-46},{-32,-46},{-32,-36}}, color={0,0,127}));
  connect(sphereImpulseCollision_Unfinished1.bVelocity,
    bodyExternalReinit_Unfinished2.reinitVelocityY1) annotation (Line(points={{
          -8,-26},{-8,-48},{0,-48},{0,-36}}, color={0,0,127}));
  connect(bodyExternalReinit_Unfinished2.reinitVelocityY2_external,
    sphereImpulseCollision_Unfinished2.aVelocity) annotation (Line(points={{8,
          -36},{8,-48},{18,-48},{18,-26}}, color={0,0,127}));
  connect(sphereImpulseCollision_Unfinished2.bVelocity,
    bodyExternalReinit_Unfinished3.reinitVelocityY1) annotation (Line(points={{
          34,-26},{34,-48},{44,-48},{44,-36}}, color={0,0,127}));
  connect(bodyExternalReinit_Unfinished3.reinitVelocityY2_external,
    sphereImpulseCollision_Unfinished3.aVelocity) annotation (Line(points={{52,
          -36},{52,-48},{60,-48},{60,-26}}, color={0,0,127}));
  connect(bodyExternalReinit_Unfinished4.reinitVelocityY1,
    sphereImpulseCollision_Unfinished3.bVelocity) annotation (Line(points={{84,
          -36},{84,-48},{76,-48},{76,-26}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.3")));
end Praktikum13;
