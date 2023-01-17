within ;
package Uebung9
  package Interfaces
    connector Flange_a
      "(left) 1D translational flange (flange axis directed INTO cut plane, e. g. from left to right)"

      Modelica.SIunits.Position s "Absolute position of flange";
      flow Modelica.SIunits.Force f "Cut force directed into flange";
      annotation (
        defaultComponentName="flange_a",
        Documentation(info="<html>
<p>
This is a flange for 1D translational mechanical systems. In the cut plane of
the flange a unit vector n, called flange axis, is defined which is directed
INTO the cut plane, i. e. from left to right. All vectors in the cut plane are
resolved with respect to
this unit vector. E.g. force f characterizes a vector which is directed in
the direction of n with value equal to f. When this flange is connected to
other 1D translational flanges, this means that the axes vectors of the connected
flanges are identical.
</p>
<p>
The following variables are transported through this connector:
</p>
<pre>
  s: Absolute position of the flange in [m]. A positive translation
     means that the flange is translated along the flange axis.
  f: Cut-force in direction of the flange axis in [N].
</pre>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={0,127,0},
              fillColor={0,127,0},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Rectangle(
              extent={{-40,-40},{40,40}},
              lineColor={0,127,0},
              fillColor={0,127,0},
              fillPattern=FillPattern.Solid), Text(
              extent={{-160,110},{40,50}},
              lineColor={0,127,0},
              textString="%name")}));
    end Flange_a;

    connector Flange_b
      "(right) 1D translational flange (flange axis directed OUT OF cut plane)"

      Modelica.SIunits.Position s "Absolute position of flange";
      flow Modelica.SIunits.Force f "Cut force directed into flange";
      annotation (
        defaultComponentName="flange_b",
        Documentation(info="<html>
<p>
This is a flange for 1D translational mechanical systems. In the cut plane of
the flange a unit vector n, called flange axis, is defined which is directed
OUT OF the cut plane. All vectors in the cut plane are resolved with respect to
this unit vector. E.g. force f characterizes a vector which is directed in
the direction of n with value equal to f. When this flange is connected to
other 1D translational flanges, this means that the axes vectors of the connected
flanges are identical.
</p>
<p>
The following variables are transported through this connector:
</p>
<pre>
  s: Absolute position of the flange in [m]. A positive translation
     means that the flange is translated along the flange axis.
  f: Cut-force in direction of the flange axis in [N].
</pre>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={0,127,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Rectangle(
              extent={{-40,-40},{40,40}},
              lineColor={0,127,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-40,110},{160,50}},
              lineColor={0,127,0},
              textString="%name")}));
    end Flange_b;

    connector BadFlange_a
      "(left) 1D translational flange (flange axis directed INTO cut plane, e. g. from left to right)"

      Modelica.SIunits.Velocity v "Absolute velocity of flange";
      flow Modelica.SIunits.Force f "Cut force directed into flange";
      annotation (
        defaultComponentName="flange_a",
        Documentation(info="<html>
<p>
This is a flange for 1D translational mechanical systems. In the cut plane of
the flange a unit vector n, called flange axis, is defined which is directed
INTO the cut plane, i. e. from left to right. All vectors in the cut plane are
resolved with respect to
this unit vector. E.g. force f characterizes a vector which is directed in
the direction of n with value equal to f. When this flange is connected to
other 1D translational flanges, this means that the axes vectors of the connected
flanges are identical.
</p>
<p>
The following variables are transported through this connector:
</p>
<pre>
  s: Absolute position of the flange in [m]. A positive translation
     means that the flange is translated along the flange axis.
  f: Cut-force in direction of the flange axis in [N].
</pre>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={0,127,0},
              fillColor={0,127,0},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Rectangle(
              extent={{-40,-40},{40,40}},
              lineColor={0,127,0},
              fillColor={0,127,0},
              fillPattern=FillPattern.Solid), Text(
              extent={{-160,110},{40,50}},
              lineColor={0,127,0},
              textString="%name")}));
    end BadFlange_a;

    connector BadFlange_b
      "(right) 1D translational flange (flange axis directed OUT OF cut plane)"

      Modelica.SIunits.Velocity v "Absolute velocity of flange";
      flow Modelica.SIunits.Force f "Cut force directed into flange";
      annotation (
        defaultComponentName="flange_b",
        Documentation(info="<html>
<p>
This is a flange for 1D translational mechanical systems. In the cut plane of
the flange a unit vector n, called flange axis, is defined which is directed
OUT OF the cut plane. All vectors in the cut plane are resolved with respect to
this unit vector. E.g. force f characterizes a vector which is directed in
the direction of n with value equal to f. When this flange is connected to
other 1D translational flanges, this means that the axes vectors of the connected
flanges are identical.
</p>
<p>
The following variables are transported through this connector:
</p>
<pre>
  s: Absolute position of the flange in [m]. A positive translation
     means that the flange is translated along the flange axis.
  f: Cut-force in direction of the flange axis in [N].
</pre>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={0,127,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Rectangle(
              extent={{-40,-40},{40,40}},
              lineColor={0,127,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-40,110},{160,50}},
              lineColor={0,127,0},
              textString="%name")}));
    end BadFlange_b;
  end Interfaces;

  package Components
    model Fixed "Fixed flange"
      parameter Modelica.SIunits.Position s0=0 "Fixed offset position of housing";

      Interfaces.Flange_b flange annotation (
          Placement(transformation(extent={{-10,10},{10,-10}}, rotation=180)));
    equation
      flange.s = s0;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={
            Line(points={{-80,-40},{80,-40}}),
            Line(points={{80,-40},{40,-80}}),
            Line(points={{40,-40},{0,-80}}),
            Line(points={{0,-40},{-40,-80}}),
            Line(points={{-40,-40},{-80,-80}}),
            Line(points={{0,-40},{0,-10}}),
            Text(
              extent={{-150,-90},{150,-130}},
              textString="%name",
              lineColor={0,0,255})}), Documentation(info="<html>
<p>
The <i>flange</i> of a 1D translational mechanical system <i>fixed</i>
at an position s0 in the <i>housing</i>. May be used:
</p>
<ul>
<li> to connect a compliant element, such as a spring or a damper,
     between a sliding mass and the housing.</li>
<li> to fix a rigid element, such as a sliding mass, at a specific
     position.</li>
</ul>

</html>"));
    end Fixed;

    model Mass "Sliding mass with inertia"

      import SI=Modelica.SIunits;

      parameter SI.Mass m(min=0, start=1) "Mass of the sliding mass";
      parameter StateSelect stateSelect=StateSelect.default
        "Priority to use s and v as states" annotation (Dialog(tab="Advanced"));
      SI.Position s(start=0)
        "Absolute position of center of component (s = flange_a.s + L/2 = flange_b.s - L/2)";
      parameter SI.Length L(start=0)=0
        "Length of component, from left flange to right flange (= flange_b.s - flange_a.s)";
      SI.Velocity v(start=0, stateSelect=stateSelect)
        "Absolute velocity of component";
      SI.Acceleration a(start=0) "Absolute acceleration of component";

      Interfaces.Flange_a flange_a "Left flange of translational component"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.Flange_b flange_b "Right flange of translational component" annotation (
          Placement(transformation(extent={{90,-10},{110,10}})));

    equation
      flange_a.s = s - L/2;
      flange_b.s = s + L/2;
      v = der(s);
      a = der(v);
      m*a = flange_a.f + flange_b.f;
      annotation (
        Documentation(info="<html>
<p>
Sliding mass with <i>inertia, without friction</i> and two rigidly connected flanges.
</p>
<p>
The sliding mass has the length L, the position coordinate s is in the middle.
Sign convention: A positive force at flange flange_a moves the sliding mass in the positive direction.
A negative force at flange flange_a moves the sliding mass to the negative direction.
</p>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={
            Line(points={{-100,0},{-55,0}}, color={0,127,0}),
            Line(points={{55,0},{100,0}}, color={0,127,0}),
            Rectangle(
              extent={{-55,-30},{56,30}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={255,255,255}),
            Polygon(
              points={{50,-90},{20,-80},{20,-100},{50,-90}},
              lineColor={128,128,128},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,-90},{20,-90}}),
            Text(
              extent={{-150,85},{150,45}},
              textString="%name",
              lineColor={0,0,255}),
            Text(
              extent={{-150,-45},{150,-75}},
              lineColor={0,0,0},
              textString="m=%m")}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Line(points={{-100,0},{-55,0}}, color={0,
              127,0}),Line(points={{55,0},{100,0}}, color={0,127,0}),Rectangle(
                  extent={{-55,-30},{55,30}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={255,255,255}),Polygon(
                  points={{50,-90},{20,-80},{20,-100},{50,-90}},
                  lineColor={128,128,128},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),Line(points={{-60,-90},{20,-90}}),Line(points={{-100,-29},{-100,-61}}),
              Line(points={{100,-61},{100,-28}}),Line(points={{-98,
              -60},{98,-60}}),Polygon(
                  points={{-101,-60},{-96,-59},{-96,-61},{-101,-60}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),Polygon(
                  points={{100,-60},{95,-61},{95,-59},{100,-60}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-44,-41},{51,-57}},
                  textString="Length L",
                  lineColor={0,0,255}),Line(points={{0,30},{0,53}}, color={0,0,
              0}),Line(points={{-72,40},{1,40}}),Polygon(
                  points={{-7,42},{-7,38},{-1,40},{-7,42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-61,53},{-9,42}},
                  textString="Position s",
                  lineColor={0,0,255})}));
    end Mass;

    model Spring "Linear 1D translational spring"

      import SI=Modelica.SIunits;

      Interfaces.Flange_a flange_a
        "Left flange of compliant 1-dim. translational component"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.Flange_b flange_b "Right flange of compliant 1-dim. translational component"
                                                                   annotation (
          Placement(transformation(extent={{90,-10},{110,10}})));
      SI.Position s_rel(start=0) "Relative distance (= flange_b.s - flange_a.s)";
      SI.Force f "Force between flanges (positive in direction of flange axis R)";
      parameter SI.TranslationalSpringConstant c(final min=0, start=1)
        "Spring constant";
      parameter SI.Distance s_rel0=0 "Unstretched spring length";

    equation
      f = c*(s_rel - s_rel0);
      s_rel = flange_b.s - flange_a.s;
      flange_b.f = f;
      flange_a.f = -f;
      annotation (
        Documentation(info="<html>
<p>
A <i>linear 1D translational spring</i>. The component can be connected either
between two sliding masses, or between
a sliding mass and the housing (model Fixed), to describe
a coupling of the sliding mass with the housing via a spring.
</p>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={
            Line(points={{-60,-90},{20,-90}}),
            Polygon(
              points={{50,-90},{20,-80},{20,-100},{50,-90}},
              lineColor={128,128,128},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-150,90},{150,50}},
              textString="%name",
              lineColor={0,0,255}),
            Line(points={{-98,0},{-60,0},{-44,-30},{-16,30},{14,-30},{44,30},{
                  60,0},{100,0}}),
            Text(
              extent={{-150,-45},{150,-75}},
              lineColor={0,0,0},
              textString="c=%c")}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Line(points={{-100,0},{-100,65}}, color=
              {128,128,128}),Line(points={{100,0},{100,65}}, color={128,128,128}),
              Line(points={{-100,60},{100,60}}, color={128,128,128}),Polygon(
                  points={{90,63},{100,60},{90,57},{90,63}},
                  lineColor={128,128,128},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-56,66},{36,81}},
                  lineColor={0,0,255},
                  textString="s_rel"),Line(points={{-86,0},{-60,0},{-44,-30},{-16,
              30},{14,-30},{44,30},{60,0},{84,0}})}));
    end Spring;

    model MagicConstraint
      Interfaces.Flange_a flange_a annotation (Placement(transformation(extent=
                {{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{
                -90,10}})));
      Interfaces.Flange_b flange_b annotation (Placement(transformation(extent={{
                90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
      Real s1(start=2);
      Real s2(start=0);

    equation
      s1 = flange_a.s;
      s2 = flange_b.s;

      s1 = s2*s2;
      flange_a.f + flange_b.f = 0;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Ellipse(
              extent={{-80,80},{80,-80}},
              lineColor={28,108,200},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Ellipse(
              extent={{-60,60},{60,-60}},
              lineColor={28,108,200},
              fillColor={102,44,145},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Ellipse(
              extent={{-40,40},{40,-40}},
              lineColor={28,108,200},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Rectangle(
              extent={{-100,0},{100,-100}},
              lineColor={28,108,200},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-100,-20},{100,-60}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="Magic Constraint")}),                      Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end MagicConstraint;

    model BadFixed "Fixed flange"
      parameter Modelica.SIunits.Position s0=0 "Fixed offset position of housing";

      Interfaces.BadFlange_b flange_b
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation
      flange_b.v = der(s0);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={
            Line(points={{-80,-40},{80,-40}}),
            Line(points={{80,-40},{40,-80}}),
            Line(points={{40,-40},{0,-80}}),
            Line(points={{0,-40},{-40,-80}}),
            Line(points={{-40,-40},{-80,-80}}),
            Line(points={{0,-40},{0,-10}}),
            Text(
              extent={{-150,-90},{150,-130}},
              textString="%name",
              lineColor={0,0,255})}), Documentation(info="<html>
<p>
The <i>flange</i> of a 1D translational mechanical system <i>fixed</i>
at an position s0 in the <i>housing</i>. May be used:
</p>
<ul>
<li> to connect a compliant element, such as a spring or a damper,
     between a sliding mass and the housing.</li>
<li> to fix a rigid element, such as a sliding mass, at a specific
     position.</li>
</ul>

</html>"));
    end BadFixed;

    model BadMass "Sliding mass with inertia"

      import SI=Modelica.SIunits;

      parameter SI.Mass m(min=0, start=1) "Mass of the sliding mass";
      parameter StateSelect stateSelect=StateSelect.default
        "Priority to use s and v as states" annotation (Dialog(tab="Advanced"));
      SI.Position s(start=0)
        "Absolute position of center of component (s = flange_a.s + L/2 = flange_b.s - L/2)";
      parameter SI.Length L(start=0)=0
        "Length of component, from left flange to right flange (= flange_b.s - flange_a.s)";
      SI.Velocity v(start=0, stateSelect=stateSelect)
        "Absolute velocity of component";
      SI.Acceleration a(start=0) "Absolute acceleration of component";

      Interfaces.BadFlange_b flange_b
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Interfaces.BadFlange_a flange_a
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    equation
      flange_a.v = der(s - L/2);
      flange_b.v = der(s + L/2);
      v = der(s);
      a = der(v);
      m*a = flange_a.f + flange_b.f;
      annotation (
        Documentation(info="<html>
<p>
Sliding mass with <i>inertia, without friction</i> and two rigidly connected flanges.
</p>
<p>
The sliding mass has the length L, the position coordinate s is in the middle.
Sign convention: A positive force at flange flange_a moves the sliding mass in the positive direction.
A negative force at flange flange_a moves the sliding mass to the negative direction.
</p>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={
            Line(points={{-100,0},{-55,0}}, color={0,127,0}),
            Line(points={{55,0},{100,0}}, color={0,127,0}),
            Rectangle(
              extent={{-55,-30},{56,30}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={255,255,255}),
            Polygon(
              points={{50,-90},{20,-80},{20,-100},{50,-90}},
              lineColor={128,128,128},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,-90},{20,-90}}),
            Text(
              extent={{-150,85},{150,45}},
              textString="%name",
              lineColor={0,0,255}),
            Text(
              extent={{-150,-45},{150,-75}},
              lineColor={0,0,0},
              textString="m=%m")}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Line(points={{-100,0},{-55,0}}, color={0,
              127,0}),Line(points={{55,0},{100,0}}, color={0,127,0}),Rectangle(
                  extent={{-55,-30},{55,30}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={255,255,255}),Polygon(
                  points={{50,-90},{20,-80},{20,-100},{50,-90}},
                  lineColor={128,128,128},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),Line(points={{-60,-90},{20,-90}}),Line(points={{-100,-29},{-100,-61}}),
              Line(points={{100,-61},{100,-28}}),Line(points={{-98,
              -60},{98,-60}}),Polygon(
                  points={{-101,-60},{-96,-59},{-96,-61},{-101,-60}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),Polygon(
                  points={{100,-60},{95,-61},{95,-59},{100,-60}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-44,-41},{51,-57}},
                  textString="Length L",
                  lineColor={0,0,255}),Line(points={{0,30},{0,53}}, color={0,0,
              0}),Line(points={{-72,40},{1,40}}),Polygon(
                  points={{-7,42},{-7,38},{-1,40},{-7,42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-61,53},{-9,42}},
                  textString="Position s",
                  lineColor={0,0,255})}));
    end BadMass;

    model BadSpring "Linear 1D translational spring"

      import SI=Modelica.SIunits;

      SI.Position s_rel(start=0) "Relative distance (= flange_b.s - flange_a.s)";
      SI.Force f "Force between flanges (positive in direction of flange axis R)";
      parameter SI.TranslationalSpringConstant c(final min=0, start=1)
        "Spring constant";
      parameter SI.Distance s_rel0=0 "Unstretched spring length";

      Interfaces.BadFlange_a flange_a
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.BadFlange_b flange_b
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      f = c*(s_rel - s_rel0);
      der(s_rel) = flange_b.v - flange_a.v;
      flange_b.f = f;
      flange_a.f = -f;
      annotation (
        Documentation(info="<html>
<p>
A <i>linear 1D translational spring</i>. The component can be connected either
between two sliding masses, or between
a sliding mass and the housing (model Fixed), to describe
a coupling of the sliding mass with the housing via a spring.
</p>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={
            Line(points={{-60,-90},{20,-90}}),
            Polygon(
              points={{50,-90},{20,-80},{20,-100},{50,-90}},
              lineColor={128,128,128},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-150,90},{150,50}},
              textString="%name",
              lineColor={0,0,255}),
            Line(points={{-98,0},{-60,0},{-44,-30},{-16,30},{14,-30},{44,30},{
                  60,0},{100,0}}),
            Text(
              extent={{-150,-45},{150,-75}},
              lineColor={0,0,0},
              textString="c=%c")}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Line(points={{-100,0},{-100,65}}, color=
              {128,128,128}),Line(points={{100,0},{100,65}}, color={128,128,128}),
              Line(points={{-100,60},{100,60}}, color={128,128,128}),Polygon(
                  points={{90,63},{100,60},{90,57},{90,63}},
                  lineColor={128,128,128},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-56,66},{36,81}},
                  lineColor={0,0,255},
                  textString="s_rel"),Line(points={{-86,0},{-60,0},{-44,-30},{-16,
              30},{14,-30},{44,30},{60,0},{84,0}})}));
    end BadSpring;

    model BadMagicConstraint
      Real s1(start=2);
      Real s2(start=0);

      Interfaces.BadFlange_a flange_a
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.BadFlange_b flange_b
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      der(s1) = flange_a.v;
      der(s2) = flange_b.v;

      s1 = s2*s2;
      flange_a.f + flange_b.f = 0;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Ellipse(
              extent={{-80,80},{80,-80}},
              lineColor={28,108,200},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Ellipse(
              extent={{-60,60},{60,-60}},
              lineColor={28,108,200},
              fillColor={102,44,145},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Ellipse(
              extent={{-40,40},{40,-40}},
              lineColor={28,108,200},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Rectangle(
              extent={{-100,0},{100,-100}},
              lineColor={28,108,200},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-100,-20},{100,-60}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="Magic Constraint")}),                      Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end BadMagicConstraint;
  end Components;

  package Tests

    model MassSpringTest
      Components.Fixed fixed annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={-82,0})));
      Components.Fixed fixed1 annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={72,0})));
      Components.Spring spring(c=100)
        annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
      Components.Spring spring1(c=100)
        annotation (Placement(transformation(extent={{66,-10},{46,10}})));
      Components.Mass mass(m=1, s(fixed=true, start=0.0))
        annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
      Components.Mass mass1(m=1, v(start=1.0))
        annotation (Placement(transformation(extent={{36,-10},{16,10}})));
      Components.MagicConstraint magicConstraint
        annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
    equation
      connect(fixed.flange, spring.flange_a)
        annotation (Line(points={{-82,0},{-76,0}}, color={0,127,0}));
      connect(spring.flange_b, mass.flange_a) annotation (Line(points={{-56,0},
              {-52,0},{-52,0},{-46,0}}, color={0,127,0}));
      connect(mass.flange_b, magicConstraint.flange_a)
        annotation (Line(points={{-26,0},{-16,0}}, color={0,127,0}));
      connect(magicConstraint.flange_b, mass1.flange_b)
        annotation (Line(points={{4,0},{16,0}}, color={0,127,0}));
      connect(mass1.flange_a, spring1.flange_b)
        annotation (Line(points={{36,0},{46,0}}, color={0,127,0}));
      connect(spring1.flange_a, fixed1.flange)
        annotation (Line(points={{66,0},{72,0}}, color={0,127,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end MassSpringTest;

    model BadMassSpringTest
      Components.BadFixed badFixed1 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={78,0})));
      Components.BadMass badMass(m=1, s(start=0.0, fixed=true))
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Components.BadSpring badSpring(c=100)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      Components.BadMagicConstraint badMagicConstraint
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Components.BadMass badMass1(m=1, v(fixed=true, start=1.0))
        annotation (Placement(transformation(extent={{40,-10},{20,10}})));
      Components.BadSpring badSpring1(c=100)
        annotation (Placement(transformation(extent={{68,-10},{48,10}})));
      Components.BadFixed badFixed annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,0})));
    equation
      connect(badSpring.flange_b, badMass.flange_a)
        annotation (Line(points={{-50,0},{-40,0}}, color={0,127,0}));
      connect(badMass.flange_b, badMagicConstraint.flange_a)
        annotation (Line(points={{-20,0},{-10,0}}, color={0,127,0}));
      connect(badMagicConstraint.flange_b, badMass1.flange_b)
        annotation (Line(points={{10,0},{20,0}}, color={0,127,0}));
      connect(badMass1.flange_a, badSpring1.flange_b)
        annotation (Line(points={{40,0},{48,0}}, color={0,127,0}));
      connect(badSpring1.flange_a, badFixed1.flange_b)
        annotation (Line(points={{68,0},{78,0}}, color={0,127,0}));
      connect(badFixed.flange_b, badSpring.flange_a)
        annotation (Line(points={{-80,0},{-70,0}}, color={0,127,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(__Dymola_Algorithm="Dassl"));
    end BadMassSpringTest;
  end Tests;

  annotation (uses(Modelica(version="3.2.3")));
end Uebung9;
