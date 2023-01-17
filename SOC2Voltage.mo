within ;
block SOC2Voltage "<html>Voltage dependent SOC. 
  <br>Input: SOC<br>Output: corresponding voltage</html>"
  extends Modelica.Blocks.Interfaces.SISO;
  parameter Modelica.SIunits.Voltage Nominal_VDC "Nominal DC voltage";
  parameter Modelica.SIunits.Voltage Min_VDC "Minimal DC voltage at Min_SOC";
  parameter Real Min_SOC = 0.1 "Minimum state of charge";

equation
  assert(u>Min_SOC, "Sie haben die Batterie unterladen und somit zerstört!");
  assert(u<1.01, "Sie haben die Batterie um mindestens 1% ueberladen und somit zerstört!");

  y = Nominal_VDC - (Nominal_VDC - Min_VDC)*(1 - u)/(1 - Min_SOC);
  annotation (Icon(graphics={Text(
          extent={{-100,20},{0,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="SOC"), Text(
          extent={{0,20},{100,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="V")}));
end SOC2Voltage;
