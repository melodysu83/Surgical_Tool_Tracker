# Surgical_Tool_Tracker
This is a program for tracking surgical tools in real time based on robot kinematics prior.

Robot-assisted minimally invasive surgery com-
bines the skills and techniques of highly-trained surgeons

with the robustness and precision of machines. Several advan-
tages include precision beyond human dexterity alone, greater

kinematic degrees of freedom at the surgical tool tip, and
possibilities in remote surgical practices through teleoperation.
Nevertheless, obtaining accurate force feedback during those
surgical operations remains a heated topic. Though direct force
sensing using tool tip mounted force sensors is theoretically
possible, it is not amenable to required sterilization procedures.
Vision-based force estimation according to real-time analysis of
tissue deformation serves as a promising alternative. In this

application, along with numerous related research in robot-
assisted minimally invasive surgery, segmentation of surgical

instruments in endoscopic images is a prerequisite. Thus, a
surgical tool segmentation algorithm robust to partial occlusion
is proposed using DFT shape matching of robot kinematics
shape prior (u) fused with log likelihood mask (Q) in the
Opponent color space to generate final mask (U). Implemented
on the Raven II surgical robot system, a real-time performance
of up to 6 fps without GPU acceleration is achieved.
