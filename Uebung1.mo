within ;
package Uebung1
  model Schwingkreis
    Modelica.Electrical.Analog.Basic.Inductor inductor(i(start=0, fixed=true),
        L=0.01) annotation (Placement(transformation(extent={{-20,24},{0,44}})));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor(v(fixed=true, start=1),
        C=0.01) annotation (Placement(transformation(extent={{-20,-14},{0,6}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-44,-46},{-24,-26}})));
  equation
    connect(capacitor.p, ground.p) annotation (Line(points={{-20,-4},{-20,-26},
            {-34,-26}}, color={0,0,255}));
    connect(capacitor.n, inductor.n)
      annotation (Line(points={{0,-4},{0,34}}, color={0,0,255}));
    connect(capacitor.p, inductor.p)
      annotation (Line(points={{-20,-4},{-20,34}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Bitmap(extent={{-116,-116},{106,94}}, fileName=
                "modelica://Uebung1/../../Desktop/schwingkreis-schaltung-ca.png")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Schwingkreis;
  annotation (uses(Modelica(version="3.2.3")));
end Uebung1;
