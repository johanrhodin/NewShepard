package BlueOrigin "Going into space"
  extends Modelica.Icons.Package;

  package Examples "Example models"
    extends Modelica.Icons.ExamplesPackage;

    model NewShepard "Rocket launch"
      extends Modelica.Icons.Example;
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      //  Modelica.Mechanics.MultiBody.Parts.BodyShape crewModule(shapeType = "cone", r = {0, 2.5, 0}, m = 1000, lengthDirection = {0, 1, 0}, width = 2.2, length = 2.5, animateSphere = false, extra = 1) annotation(Placement(visible = true, transformation(origin = {40, 33.726}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
      Modelica.Mechanics.MultiBody.Visualizers.FixedShape groundVisualizer(length = 40, width = 40, height = .2, widthDirection = {0, 0, 1}, color = {46, 191, 84}, specularCoefficient = 0.7) "Show a ground for take off and landing" annotation(Placement(visible = true, transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BlueOrigin.Components.CrewModule crewModule(detachableStringDamper.c = 100000.0, detachableStringDamper.d = 10000.0, rampX.duration = .1, rampX.height = 2000.0, rampY.duration = .1, rampY.height = 12000.0, rampZ.duration = .1, rampZ.height = 100.0, releaseTime = releaseTime, crewCabin.extra = .99) annotation(Placement(visible = true, transformation(origin = {0, 30.629}, extent = {{9.371, -9.371}, {-9.371, 9.371}}, rotation = 630)));
      Modelica.Blocks.Sources.BooleanStep releaseMechanism(startValue = true, startTime = releaseTime) "Determines when to release the capsule" annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-5, 5}, {5, -5}}, rotation = 540)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {-20, 0, 0}, animation = false) annotation(Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Modelica.SIunits.Time releaseTime = 150.0 "Time for crew module release";
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder propulsionBody(r = {0, 18.288, 0}, density.displayUnit = "kg/m3", diameter = 2, innerDiameter = 1.95) "Contains thrusters, fuel" annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-15, -15}, {15, 15}}, rotation = -270)));
      Components.MainPropulsion mainPropulsion(releaseTime = releaseTime, PID.k = 3500, PID.yMax = 100000, PID.yMin = -10, booleanStep.startTime = 150, booleanStep.startValue = false, force.diameter = 0.5, force.color = {255, 9, 0}) annotation(Placement(visible = true, transformation(origin = {-0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
      /*, ramp.duration = 250.0, ramp.height = -9690.0, ramp.offset = 9730.0, ramp.startTime = 150.0*/
    equation
      connect(propulsionBody.frame_a, mainPropulsion.frame_b) annotation(Line(visible = true, points = {{0, 2.5}, {0, -2.5}}, color = {95, 95, 95}, origin = {0, -17.5}));
      connect(propulsionBody.frame_b, crewModule.moduleFrame) annotation(Line(visible = true, origin = {0.047, 18.123}, points = {{-0.047, -3.123}, {-0.047, 0.041}, {0.047, 0.041}, {0.047, 3.041}}));
      connect(groundVisualizer.frame_a, fixedTranslation.frame_b) annotation(Line(visible = true, origin = {15, -60}, points = {{5, 0}, {-5, -0}}));
      connect(world.frame_b, fixedTranslation.frame_a) annotation(Line(visible = true, origin = {-15, -60}, points = {{-5, 0}, {5, 0}}));
      connect(crewModule.attached, releaseMechanism.y) annotation(Line(visible = true, origin = {26.748, 20.419}, points = {{-15.503, 10.21}, {7.752, 10.21}, {7.752, -20.419}}, color = {255, 0, 255}));
      //  connect(propulsionModule.frame_b, crewModule.frame_a) annotation(Line(visible = true, origin = {40, 16.863}, points = {{0, -6.863}, {0, 6.863}}));
      annotation(Documentation(info = "<ul>
<li>A first model of Blue Origins New Shepard</li>
</ul>", revisions = ""), experiment(StartTime = 0.0, StopTime = 500, Interval = 0.01, __Wolfram_Algorithm = "dassl", Tolerance = 1e-6, __Wolfram_MaxStepSize = 0, __Wolfram_EventHysteresisEpsilon = 1e-10, __Wolfram_AccurateEventDetection = true, __Wolfram_NonlinearSolverTolerance = 1e-12, __Wolfram_MaxInternalStepCount = 0, __Wolfram_CheckMinMax = false, __Wolfram_StopAtSteadyState = false, __Wolfram_SynchronizeWithRealTime = false), Diagram(coordinateSystem(extent = {{-60, -80}, {70, 60}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})));
    end NewShepard;
  end Examples;

  package Components "Custom components"
    extends Modelica.Icons.Package;

    model DetachableString
      import SI = Modelica.SIunits;
      extends Modelica.Mechanics.Translational.Interfaces.PartialCompliant;
      parameter Modelica.SIunits.TranslationalSpringConstant c(final min = 0, start = 1) "spring constant ";
      parameter Modelica.SIunits.TranslationalDampingConstant d = 10 "damping constant";
      parameter Modelica.SIunits.Distance s_rel0 = 0 "unstretched spring length";
      Modelica.SIunits.Force f_d "Damping force";
      Modelica.Blocks.Interfaces.BooleanInput attached annotation(Placement(visible = true, transformation(origin = {-100.0, 68.0}, extent = {{-20.0, -20.0}, {20.0, 20.0}}, rotation = 0), iconTransformation(origin = {0.0, 120.0}, extent = {{-20.0, -20.0}, {20.0, 20.0}}, rotation = -90)));
    equation
      f_d = d * der(s_rel);
      f = if attached then c * (s_rel - s_rel0) + f_d else 0;
      annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-57.532, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}), Text(visible = true, origin = {-7.384, -127.369}, textColor = {0, 0, 255}, extent = {{-130, 49}, {132, 109}}, textString = "%name", fontName = "Arial"), Text(visible = true, origin = {81, 134.138}, extent = {{-141, -92}, {125, -51}}, textString = "c=%c", fontName = "Arial"), Ellipse(visible = true, fillPattern = FillPattern.Solid, extent = {{-8, -8}, {8, 8}}), Line(visible = true, origin = {-68.612, 10.922}, points = {{11.388, -10.922}, {-11.388, 10.922}}), Line(visible = true, origin = {-86.302, 0}, points = {{6.302, 0}, {-6.302, 0}}), Line(visible = true, origin = {-35, 63.923}, points = {{35, 36.077}, {35, 6.077}, {-35, 6.077}, {-35, -48.232}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 5), Text(visible = true, origin = {81, 108.195}, extent = {{-141, -92}, {125, -51}}, textString = "d=%d")}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics), Documentation(info = "", revisions = ""));
    end DetachableString;

    model CrewModule "Where the crew travels and the connection to the propulsion module"
      DetachableString detachableStringDamper(c = 10000, s_rel0 = 0.001) annotation(Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
      Modelica.Mechanics.MultiBody.Forces.LineForceWithMass loadSpring(animateLine = false, animateMass = false) annotation(Placement(visible = true, transformation(origin = {-0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
      Modelica.Blocks.Interfaces.BooleanInput attached annotation(Placement(visible = true, transformation(origin = {-0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -450)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a moduleFrame "Connects/disconnects to the propulsion module" annotation(Placement(visible = true, transformation(origin = {-81, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-101, 1}, extent = {{-16, -16}, {16, 16}}, rotation = -180)));
      Modelica.Mechanics.MultiBody.Parts.BodyShape crewCabin(shapeType = "cone", r = {0, 2.5, 0}, m = 1000, lengthDirection = {0, 1, 0}, width = 2.2, length = 2.5, animateSphere = false, extra = 1, color = {255, 49, 24}, r_0.start = {0, 17.85, 0}) "Where people travel" annotation(Placement(visible = true, transformation(origin = {60, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
      Modelica.Blocks.Sources.Ramp rampX(height = 100, duration = 10, startTime = releaseTime) annotation(Placement(visible = true, transformation(origin = {-30, -0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp rampY(height = -12000, duration = 2, startTime = releaseTime) annotation(Placement(visible = true, transformation(origin = {-90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp rampZ(height = 100, duration = 10, startTime = releaseTime) annotation(Placement(visible = true, transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Forces.Internal.BasicWorldForce basicWorldForce annotation(Placement(visible = true, transformation(origin = {-0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Modelica.SIunits.Time releaseTime "Time for crew module release";
      Modelica.Blocks.Math.Add add(k2 = 1, k1 = -1) annotation(Placement(visible = true, transformation(origin = {-57.354, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
      Modelica.Blocks.Sources.Ramp rampY1(height = 100, duration = .1, startTime = releaseTime + 10) annotation(Placement(visible = true, transformation(origin = {-90, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(rampY1.y, add.u1) annotation(Line(visible = true, origin = {-73.266, -33}, points = {{-5.734, -7}, {0.911, -7}, {0.911, 7}, {3.911, 7}}, color = {0, 0, 127}));
      connect(rampY.y, add.u2) annotation(Line(visible = true, origin = {-73.266, -12}, points = {{-5.734, 2}, {0.911, 2}, {0.911, -2}, {3.911, -2}}, color = {0, 0, 127}));
      connect(add.y, basicWorldForce.force[2]) "Total force in Y-direction" annotation(Line(visible = true, origin = {-22.089, -15}, points = {{-24.266, -5}, {7.089, -5}, {7.089, 5}, {10.089, 5}}, color = {0, 0, 127}));
      connect(basicWorldForce.frame_resolve, loadSpring.frame_b) annotation(Line(visible = true, origin = {18.405, -5.2}, points = {{-18.405, -14.8}, {-18.405, -24.8}, {6.595, -24.8}, {6.595, 25.2}, {-8.405, 25.2}}));
      connect(basicWorldForce.frame_b, crewCabin.frame_a) annotation(Line(visible = true, origin = {38.494, 5}, points = {{-28.494, -15}, {8.494, -15}, {8.494, 15}, {11.506, 15}}));
      connect(rampX.y, basicWorldForce.force[1]) "Force in X direction" annotation(Line(visible = true, origin = {-15.25, -5}, points = {{-3.75, 5}, {0.25, 5}, {0.25, -5}, {3.25, -5}}, color = {0, 0, 127}));
      connect(rampZ.y, basicWorldForce.force[3]) "Force in Z direction" annotation(Line(visible = true, origin = {-15.25, -25}, points = {{-3.75, -15}, {0.25, -15}, {0.25, 15}, {3.25, 15}}, color = {0, 0, 127}));
      connect(attached, detachableStringDamper.attached) annotation(Line(visible = true, origin = {0, 86}, points = {{0, 14}, {0, -14}}, color = {255, 0, 255}));
      connect(loadSpring.frame_a, moduleFrame) annotation(Line(visible = true, origin = {-45.5, 20}, points = {{35.5, 0}, {-35.5, 0}}));
      connect(detachableStringDamper.flange_a, loadSpring.flange_a) annotation(Line(visible = true, origin = {-9.6, 43.33}, points = {{-0.4, 16.67}, {-3.4, 16.67}, {-3.4, -10.005}, {3.6, -10.005}, {3.6, -13.33}}, color = {0, 127, 0}));
      connect(detachableStringDamper.flange_b, loadSpring.flange_b) annotation(Line(visible = true, origin = {9.69, 43.33}, points = {{0.31, 16.67}, {3.535, 16.67}, {3.535, -10.005}, {-3.69, -10.005}, {-3.69, -13.33}}, color = {0, 127, 0}));
      connect(loadSpring.frame_b, crewCabin.frame_a) annotation(Line(visible = true, origin = {30, 20}, points = {{-20, 0}, {20, 0}}));
      annotation(experiment(__Wolfram_SynchronizeWithRealTime = false), Documentation(info = "", revisions = ""), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Rectangle(visible = true, fillColor = {0, 0, 255}, extent = {{-100, -100}, {100, 100}}), Text(visible = true, origin = {0, -125}, rotation = -360, textColor = {0, 0, 255}, extent = {{-100, -100}, {100, 100}}, textString = "%name"), Rectangle(visible = true, origin = {-79.849, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-19.849, -100}, {19.849, 100}}), Polygon(visible = true, origin = {17.5, 0.836}, fillColor = {142, 165, 246}, fillPattern = FillPattern.HorizontalCylinder, points = {{-77.5, 99.164}, {77.5, 38.336}, {77.5, -36.664}, {-77.5, -100.836}}), Ellipse(visible = true, origin = {0, 34.094}, fillColor = {238, 246, 235}, fillPattern = FillPattern.Solid, extent = {{-41.379, -19.094}, {41.379, 19.094}}), Ellipse(visible = true, origin = {0, -39.094}, fillColor = {238, 246, 235}, fillPattern = FillPattern.Solid, extent = {{-41.379, -19.094}, {41.379, 19.094}})}), Diagram(coordinateSystem(extent = {{-100, -55}, {100, 80}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end CrewModule;

    model PropulsionSystem "Thruster and propulsion body, Very basic propulsion"
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder propulsionBody(r = {0, 18.288, 0}, density.displayUnit = "kg/m3", diameter = 2, innerDiameter = 1.95) "Contains thrusters, fuel" annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
      Modelica.Mechanics.MultiBody.Forces.WorldForce force(animation = true, N_to_m = 10000) annotation(Placement(visible = true, transformation(origin = {40, -30}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
      Modelica.Blocks.Sources.Ramp rampX(height = 0, duration = 10) annotation(Placement(visible = true, transformation(origin = {10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp rampY(height = 200000, duration = 10, offset = 300000.0) annotation(Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp rampZ(height = 0, duration = 10) annotation(Placement(visible = true, transformation(origin = {-30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(Placement(visible = true, transformation(origin = {67.101, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
      Modelica.Blocks.Sources.Ramp rampY1(height = 300000, duration = 10, offset = 0, startTime = 10) annotation(Placement(visible = true, transformation(origin = {100, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 720)));
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Visualizers.FixedShape groundVisualizer(length = 40, width = 40, height = .2, widthDirection = {0, 0, 1}, color = {46, 191, 84}, specularCoefficient = 0.7) "Show a ground for take off and landing" annotation(Placement(visible = true, transformation(origin = {130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {-20, 0, 0}, animation = false) annotation(Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(world.frame_b, fixedTranslation.frame_a) annotation(Line(visible = true, origin = {85, 0}, points = {{-5, 0}, {5, 0}}));
      connect(groundVisualizer.frame_a, fixedTranslation.frame_b) annotation(Line(visible = true, origin = {115, 0}, points = {{5, 0}, {-5, 0}}));
      connect(force.force[1], rampX.y) "x-direction" annotation(Line(visible = true, origin = {33.667, -47.333}, points = {{6.333, 5.333}, {6.333, -2.667}, {-12.667, -2.667}}, color = {0, 0, 127}));
      connect(rampZ.y, force.force[3]) annotation(Line(visible = true, origin = {20.333, -74}, points = {{-39.333, -16}, {19.667, -16}, {19.667, 32}}, color = {0, 0, 127}));
      connect(force.frame_b, propulsionBody.frame_a) annotation(Line(visible = true, origin = {40, -15}, points = {{0, -5}, {0, 5}}));
      connect(rampY.y, add.u1) annotation(Line(visible = true, origin = {42.05, -63.663}, points = {{-41.05, -6.337}, {-38.05, -6.337}, {-38.05, 6.675}, {40.05, 6.675}, {40.05, -0.337}, {37.05, -0.337}}, color = {0, 0, 127}));
      connect(rampY1.y, add.u2) annotation(Line(visible = true, origin = {83.076, -78}, points = {{5.924, -2}, {-0.975, -2}, {-0.975, 2}, {-3.975, 2}}, color = {0, 0, 127}));
      connect(add.y, force.force[2]) "Force in Y-direction" annotation(Line(visible = true, origin = {45.367, -60.667}, points = {{10.734, -9.333}, {-5.367, -9.333}, {-5.367, 18.667}}, color = {0, 0, 127}));
      annotation(experiment(StopTime = 1, __Wolfram_SynchronizeWithRealTime = false), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {0, -130}, extent = {{-100, -100}, {100, 100}}, textString = "%name"), Rectangle(visible = true, origin = {0, -0.413}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -64.079}, {100, 64.079}})}), Diagram(coordinateSystem(extent = {{-48.248, -105}, {148.5, 15}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end PropulsionSystem;

    model MainPropulsion "Main propulsion system"
      Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp rampY1(height = 300000, duration = 10, offset = 0, startTime = 10) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 720)));
      Modelica.Blocks.Sources.Ramp rampZ(height = 0, duration = 10) annotation(Placement(visible = true, transformation(origin = {-130, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp rampY(height = 200000, duration = 10, offset = 300000.0) annotation(Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
      Modelica.Blocks.Sources.Ramp rampX(height = 0, duration = 10) annotation(Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Forces.WorldForce force(animation = true, N_to_m = 10000) annotation(Placement(visible = true, transformation(origin = {-60, -10}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Blocks.Continuous.LimPID PID(yMax = 280000, yMin = 0, k = 1000) annotation(Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Switch switch1 annotation(Placement(visible = true, transformation(origin = {20, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 360)));
      Modelica.Blocks.Sources.Constant const(k = 0) "Reference height" annotation(Placement(visible = true, transformation(origin = {60, -80}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
      Modelica.Blocks.Math.Add3 addY(k2 = -1) annotation(Placement(visible = true, transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      parameter Modelica.SIunits.Time releaseTime "Time for crew module release";
      Modelica.Blocks.Sources.BooleanStep booleanStep(startValue = true, startTime = releaseTime) annotation(Placement(visible = true, transformation(origin = {100, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp landingDirections(startTime = 150, offset = 9730, height = -9690, duration = 250) annotation(Placement(visible = true, transformation(origin = {30, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
    equation
      connect(PID.u_s, landingDirections.y) annotation(Line(visible = true, origin = {44.5, 60}, points = {{3.5, 0}, {-3.5, 0}}, color = {0, 0, 127}));
      connect(booleanStep.y, switch1.u2) annotation(Line(visible = true, origin = {60.5, -60}, points = {{28.5, -0}, {-28.5, 0}}, color = {255, 0, 255}));
      connect(const.y, switch1.u3) annotation(Line(visible = true, origin = {37.75, -74}, points = {{11.25, -6}, {-2.75, -6}, {-2.75, 6}, {-5.75, 6}}, color = {0, 0, 127}));
      connect(switch1.y, addY.u3) annotation(Line(visible = true, origin = {-2.25, -44}, points = {{11.25, -16}, {-2.75, -16}, {-2.75, 16}, {-5.75, 16}}, color = {0, 0, 127}));
      connect(addY.y, force.force[2]) "Total force in Y-direction" annotation(Line(visible = true, origin = {-50.333, -14}, points = {{19.333, -6}, {-9.667, -6}, {-9.667, -8}}, color = {0, 0, 127}));
      connect(addY.u1, rampY.y) annotation(Line(visible = true, origin = {10.75, -6}, points = {{-18.75, -6}, {5.25, -6}, {5.25, 6}, {8.25, 6}}, color = {0, 0, 127}));
      connect(rampY1.y, addY.u2) annotation(Line(visible = true, origin = {20.5, -20}, points = {{28.5, 0}, {-28.5, -0}}, color = {0, 0, 127}));
      connect(PID.u_m, absolutePosition.r[2]) "Measured height" annotation(Line(visible = true, origin = {43.667, 42.667}, points = {{16.333, 5.333}, {16.333, -2.667}, {-32.667, -2.667}}, color = {0, 0, 127}));
      connect(PID.y, switch1.u1) annotation(Line(visible = true, origin = {106.25, -11}, points = {{-35.25, 71}, {-26.25, 71}, {-26.25, -41}, {-74.25, -41}}, color = {0, 0, 127}));
      connect(force.frame_b, frame_b) annotation(Line(visible = true, origin = {-60, 20}, points = {{0, -20}, {0, 20}}));
      connect(frame_b, absolutePosition.frame_a) annotation(Line(visible = true, origin = {-35, 40}, points = {{-25, 0}, {25, 0}}));
      connect(force.force[1], rampX.y) "x-direction" annotation(Line(visible = true, origin = {-66.333, -27.333}, points = {{6.333, 5.333}, {6.333, -2.667}, {-12.667, -2.667}}, color = {0, 0, 127}));
      connect(rampZ.y, force.force[3]) annotation(Line(visible = true, origin = {-79.667, -54}, points = {{-39.333, -16}, {19.667, -16}, {19.667, 32}}, color = {0, 0, 127}));
      annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {0, -125.126}, extent = {{-100, -100}, {100, 100}}, textString = "%name"), Polygon(visible = true, origin = {-10.532, 2.329}, fillColor = {255, 99, 49}, fillPattern = FillPattern.Solid, points = {{106.421, 5.388}, {30.077, 72.361}, {-17.327, 72.361}, {-72.724, 89.325}, {-21.737, 56.651}, {-85.357, 29.325}, {-15.357, 15.31}, {-80.717, -28.236}, {-11.539, -38.709}, {-78.512, -80.675}, {4.643, -90.675}, {54.643, -53.316}, {81.065, -38.433}, {106.421, -10.675}})}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end MainPropulsion;
  end Components;
end BlueOrigin;
