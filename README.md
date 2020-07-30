# VerySimpleIK
A very simple IK system using FABRIK (forward and backward reaching inverse kinematics) with simple editor outlines visualization.

![alt text](https://github.com/arcsinxdx/VerySimpleIK/blob/master/ik1.gif)

# Quick Start
Just add the FABRIK Root component to the root joint, and assign the leaf and the target (end effector). You can also add pole and set iterations.
In the demo scenes, pole object is in yellow, and target object is in red. The editor visualizes the max distance the IK chain can reach, and marks the shortest line to target object with a green dashed line.

![alt text](https://github.com/arcsinxdx/VerySimpleIK/blob/master/slide-02.png)

If the target object is out of reach, the dashed line will become red.

![alt text](https://github.com/arcsinxdx/VerySimpleIK/blob/master/ik2.gif)

# How It Works
View full comments in source code [FABRIKRoot.cs](IKTest/FABRIKROOT.cs).
1. **Traverse Backwards:** Update optimised position from leaf joint to root joint.
2. **Traverse Forwards:** Update optimised position from root joint to leaf joint.
3. **MoveTowardsPole:** Bend the IK chain to the pole object, if the pole is present.
4. **ApplyChangesToJoints:** Put the joints into their desired position.
5. Repeat 1 to 4 until we have a good approximation (in this case, we repeat for a fixed number of iterations, assigned in the inspector).

# Terminology
**Joint:** a component that connects to other joint (bone).
**Root joint:** the beginning of an IK chain. The 0th joint in the chain. 
**Leaf joint:** the end of an IK chain. The nth joint in the chain.
**IK chain:** aseries of linearly linked N joint, starting from root joint, ending at leaf joint. Numerated in0,1,2,...,N.
**End effector (target):** a point where we wish the leaf joint can reach. 
**Pole:** a point where we want the IK chain to bend to.

# References
FABRIK (Inverse kinematics) - https://youtu.be/UNoX65PRehA
C# Inverse Kinematics in Unity - https://youtu.be/qqOAzn05fvk
