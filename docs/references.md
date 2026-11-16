# Bipedal wheeled robot reference notes

The supplied references provide build experience and a visual direction. We reviewed English auto-generated captions for all five videos, the expanded description of the first build video, and Mondo's website and visible imagery on 30 September 2026. This was a caption and website review, not a frame-by-frame engineering reconstruction.

Automatic captions contain inconsistent component names, units, and terminology. Treat the observations below as a guide to investigation; verify specifications against manufacturer documentation before adopting parts or equations.

## Engineering build references

| Reference | Relevant passages | What it establishes |
| --- | --- | --- |
| [Building a Robot so I don't get kicked out of Uni](https://www.youtube.com/watch?v=QZeO-wh_1HY) | 1:14–2:49, 5:30–8:10, 19:15–20:19 | Approximately half-metre prototype; six actuators; CAN termination and power startup problems; cascade balance and speed control improves on the earlier controller. |
| [My Robot almost got me Kicked out of Uni](https://www.youtube.com/watch?v=gqnW9qBCHnM) | 0:54–3:42, 4:08–5:59, 18:17–22:39, 23:45–27:05 | Approximately 0.7 m redesign; revised links and tires; attempted LQR; reported torque mismatch and actuator reliability problems; eventual return to PID with posture-dependent balancing. |
| [This is what a Master's degree in Robotics Engineering Looks like](https://www.youtube.com/watch?v=MzpK_Tev4as) | 1:10–3:20, 3:46–5:12, 9:48–10:52, 15:22–16:30 | Two joint actuators and one wheel actuator per leg; packaging and maintenance access; problems selecting the encoder variant; modular firmware; cascade controller details. |
| [The Entire Uni Found My Secret Robot Channel](https://www.youtube.com/watch?v=4pBfm9KIGo8) | 7:44–9:28, 16:54–18:57, 20:05–20:58 | Motor control modes and leg inverse kinematics; startup position issues; body pitch reference changes during crouching, with a suggestion that gains should also change. |

The first and third videos discuss the prototype; the second and fourth discuss its redesign with overlapping presentation and technical detail. Each pair is useful together. The creator says project files are on Patreon; their actual contents and availability have not been checked, and no source CAD or firmware has been obtained.

## Design references

[Mondo Robotics' camera robot trailer](https://www.youtube.com/watch?v=rBETwu7ssVw) is a product trailer rather than a technical build guide. Its captions describe following, exploration, filming, and recovery after a fall. Those are advertised capabilities, not validated requirements for our first version.

[Mondo Robotics' product page](https://www.mondorobotics.com/) lists 1.75 kg and 21.5 × 18 × 18 cm in length, width, and height. The reviewed page does not establish the measurement posture. Visible imagery suggests the desired rounded body, expressive front, wheels, and folded compact proportions. This is an exterior design interpretation; internal geometry, actuator count, and control implementation remain unknown.

## Lessons for our design

The following are proposed engineering responses to the reported issues.

| Observation | Proposed response |
| --- | --- |
| Tazer's platform is substantially larger than the compact size reference | Recalculate actuator and packaging needs for the compact design; scaling the exterior alone is insufficient. |
| Balancing only the body angle led to drift | Include speed control and test stationary position holding. |
| Crouching changed the required balance angle | Calculate posture-dependent centre of mass and equilibrium pitch; evaluate gains across heights. |
| Encoder variants and startup behaviour caused difficulty | Verify output position sensing and power-cycle behaviour from documentation; design homing or calibration explicitly. |
| Reported torque and actuator behaviour differed from expectations | Model uncertainty and record required torque calibration for future hardware tests. |
| CAN and supply startup issues consumed considerable debugging time | Review bus topology, protocol timing, power transients, and command timeout handling during electrical design. |
| Tire and link designs were revised after physical failures | Test friction sensitivity and load cases digitally, then retain physical validation tasks for tire grip and structural durability. |

These findings do not identify a proven component list for our robot. No specific actuator, battery voltage, MCU, or sensor has been selected.

## Official tool references

- [FreeCAD features](https://freecad.github.io/Website/features/): parametric solid modeling and engineering exports.
- [MuJoCo overview](https://mujoco.readthedocs.io/en/stable/overview.html): articulated dynamics, contact, actuator models, MJCF, and visualization.
- [MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html): Python bindings and viewer usage.
- [KiCad overview](https://www.kicad.org/about/kicad/): schematic capture and PCB layout on Linux.

FreeCAD and MuJoCo are proposed as the main design and simulation tools. KiCad enters when actuator and power requirements are established. Blender is available for exterior studies and presentation.
