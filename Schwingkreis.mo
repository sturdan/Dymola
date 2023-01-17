within ;
model Schwingkreis
  Praktikum4.capacitor capacitor(C=0.01, u(start=1))
    annotation (Placement(transformation(extent={{-4,36},{16,56}})));
  Praktikum4.inductance inductance(
    L=0.01, u(start=1, fixed=true))
    annotation (Placement(transformation(extent={{-4,-36},{16,-16}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-48,-92},{-28,-72}})));
equation
  connect(inductance.pin_p, capacitor.pin_p) annotation (Line(points={{-4,-26},
          {-60,-26},{-60,46},{-4,46}},color={0,0,255}));
  connect(capacitor.pin_n, inductance.pin_n) annotation (Line(points={{16,46},{
          56,46},{56,-26},{16,-26}},
                                  color={0,0,255}));
  connect(inductance.pin_p, ground.p)
    annotation (Line(points={{-4,-26},{-38,-26},{-38,-72}},
                                                  color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.3")));
end Schwingkreis;
