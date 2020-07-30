using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class FABRIKRoot : MonoBehaviour {

    [Header("Necessary Components")]
    [Tooltip("The leaf joint: Must be a child of this root joint")]
    public Transform leafJoint;
    [Tooltip("A desired point to reach")]
    public Transform target;

    [Header("Settings")]
    [Tooltip("Which point to bend to (Can be left blank)")] 
    public Transform pole;
    [Tooltip("How many times should traversing iterates")]
    public int iterations = 10;
    public bool visualizeInEditorRuntime = true;

    // The script will store all changes in these lists before applying to the joints (i.e. before calling ApplyChangesToJoints())
    // Items in these lists are all LEAF TO ROOT!
    protected List<Transform> joints;          // Size: N
    protected List<Vector3> jointPoints;       // Size: N
    protected List<Quaternion> jointRotations; // Size: N
    protected List<float> boneLengths;         // Size: N-1 (N joints have N-1 bones, for example: O----O----O----O has 4 joints, 3 bones)

    // Sum of all bones' lenghts
    protected float maxLength;

    // 1. If target is in reach: equals to target position
    // 2. If target is too far: equals the maximum point the leaf joint can reach along the direction towards target
    // Visualized in OnDrawGizmos in runtime
    protected Vector3 endEffectorPosition;

    // Initialization check
    protected bool initialized;

    private void Start() {
        Initialize();
    }

    private void LateUpdate() {

        // Editor check

        // Prevent unfinished initializing
        if (!initialized)
            return;

        // Check if end effector is too far away
        // If it's too far: project end effector on the sphere with radius = max length (visualized in OnDrawGizmos() in runtime)
        // If it's reachable: end effector = target
        // By doing so, we generalized all situations (end effector is ALWAYS reachable)
        if (Vector3.Distance(target.position, transform.position) > maxLength && maxLength > 0) {
            endEffectorPosition = transform.position + (target.position - transform.position).normalized * maxLength;
        } else {
            endEffectorPosition = target.position;
        }

        // Essense!!
        SolveIK();
    }

    // Main algorithm of FABRIK
    void SolveIK() {

        for (int i = 0; i < iterations; i++) {

            // 1. First traverse backwards (leaf to root), then traverse forwards (roof to leaf).
            // Reference: EgoMoose, FABRIK (Inverse kinematics) - youtu.be/UNoX65PRehA
            TraverseBackwards();
            TraverseForwards();

            // 2. Take account of the pole object (if it exists)
            // Reference: DitzelGames, C# Inverse Kinematics in Unity - youtu.be/qqOAzn05fvk
            if (pole != null)
                MoveTowardsPole();

            // 3. Finally, change the rotations and positions 
            // of the actual joints' positions and rotations (and consequently the bones' as well)
            ApplyChangesToJoints();
        }
    }

    void Initialize() {

        initialized = false;

        // Check if essential transforms have been set
        if (leafJoint == null) {
            if (Application.isPlaying)
                Debug.LogWarning(gameObject.name + ": FABRIK Root initialization failed. Leaf joint not assigned.");
            return;
        }

        if (target == null){
            if (Application.isPlaying)
                Debug.LogWarning(gameObject.name + ": FABRIK Root initialization failed. Target not assigned.");
            return;
        }

        if (iterations < 1) {
            if (Application.isPlaying)
                Debug.LogWarning(gameObject.name + ": FABRIK Root initialization failed. Iteration must be a positive number.");
            return;
        }

        // Initialize the lists
        joints = new List<Transform>();
        jointPoints = new List<Vector3>();
        jointRotations = new List<Quaternion>();
        boneLengths = new List<float>();

        // Set up iterator for traversing
        var current = leafJoint;

        // Traverse upwards
        while (current.parent != null && current != this.transform) {

            // Add joints
            joints.Add(current);
            jointPoints.Add(current.position);
            jointRotations.Add(current.rotation);

            // Add bone lengths: distance between current joint and its parent
            boneLengths.Add(Vector3.Distance(current.parent.position, current.position));

            // Update iterator
            current = current.parent;
        }

        // The leaf joint must be a child of the root joint
        if(current != this.transform) {
            if (Application.isPlaying)
                Debug.LogWarning(gameObject.name + ": FABRIK Root initialization failed. Leaf joint " + leafJoint.name + " is not a child of the root joint.");
            return;
        }

        // Only the root bone has not been traversed yet -> add it to the memory
        joints.Add(current);
        jointPoints.Add(current.position);
        jointRotations.Add(current.rotation);

        // Add all bone lengths together;
        maxLength = 0;
        foreach (float length in boneLengths)
            maxLength += length;

        // Mark initialization finished and display a summary
        initialized = true;
        if (Application.isPlaying)
            print(gameObject.name + ": FABRIK Root set up complete with " + joints.Count + " joints and " + boneLengths.Count + " bones. Sum length of all bones: " + maxLength + ".");
    }

    void TraverseBackwards() {

        // Traverse from leaf to root
        jointPoints[0] = target.position;

        for (int i = 1; i < jointPoints.Count; i++) {
            Vector3 direction = (jointPoints[i] - jointPoints[i - 1]).normalized;
            jointPoints[i] = jointPoints[i - 1] + direction * boneLengths[i - 1];
        }

        // Example: 4 joints -> O----O----O----O, this section will run like this:
        //               joints[3]  [2]  [1]  [0]
        //                    (Root)         (Leaf)
        /*
        jointPoints[0] = target.position;

        direction = (jointPoints[1] - jointPoints[0]).normalized;
        jointPoints[1] = jointPoints[0] + direction * boneLengths[0];

        direction = (jointPoints[2] - jointPoints[1]).normalized;
        positions[2] = positions[1] + direction * boneLengths[1];

        direction = (jointPoints[3] - jointPoints[2]).normalized;
        jointPoints[3] = positions[2] + direction * boneLengths[2];
        */
    }

    void TraverseForwards() {

        // Traverse from root to leaf
        jointPoints[jointPoints.Count - 1] = transform.position;

        for (int i = jointPoints.Count - 1; i > 0; i--) {
            Vector3 direction = (jointPoints[i - 1] - jointPoints[i]).normalized;
            jointPoints[i - 1] = jointPoints[i] + direction * boneLengths[i - 1];
        }

        // Example: 4 joints -> O----O----O----O, this section will run like this:
        //               joints[3]  [2]  [1]  [0]
        //                    (Root)         (Leaf)
        /*
        jointPoints[3] = transform.position;
        
        direction = (positions[2] - positions[3]).normalized;
        jointPoints[2] = jointPoints[3] + direction * boneLengths[2];

        direction = (positions[1] - positions[2]).normalized;
        jointPoints[1] = jointPoints[2] + direction * boneLengths[1];

        direction = (positions[0] - positions[1]).normalized;
        jointPoints[0] = jointPoints[1] + direction * boneLengths[0];
        */
    }

    void MoveTowardsPole() {

        // Traverse forward with N-2 joints
        // We are skipping root (hence i = jointPoints.Count - 2) and leaf (hence i > 0), for example: X----O----O----O----X
        for (int i = jointPoints.Count - 2; i > 0; i--) {

            // Create a plane containing point [i + 1] <- joint's parent
            Plane plane = new Plane(jointPoints[i - 1] - jointPoints[i + 1], jointPoints[i + 1]);

            // Project joint and pole on to the plane
            Vector3 jointProjectedOnPlane = plane.ClosestPointOnPlane(jointPoints[i]);
            Vector3 poleProjectedOnPlane = plane.ClosestPointOnPlane(pole.position);

            // Calculate the angle we need to rotate
            float angle = Vector3.SignedAngle(jointProjectedOnPlane - jointPoints[i + 1], poleProjectedOnPlane - jointPoints[i + 1], plane.normal);

            // Get the rotated vector
            Vector3 rotatedVector = Quaternion.AngleAxis(angle, plane.normal) * (jointPoints[i] - jointPoints[i + 1]);

            // Joint's new position = Joint's parent's position + rotated vector
            jointPoints[i] = jointPoints[i + 1] + rotatedVector;
        }

    }

    void ApplyChangesToJoints() {

        // PART I - Update the actual rotations and positions
        // Update rotations
        for (int i = joints.Count - 1; i > 0; i--) { // exclude the leaf bone (its rotation does not matter)

            Vector3 fromDirection = joints[i - 1].position - joints[i].position;
            Vector3 toDirection = jointPoints[i - 1] - joints[i].position;
            Vector3 rotateAxis = Vector3.Cross(fromDirection, toDirection);
            float angle = Vector3.SignedAngle(fromDirection, toDirection, rotateAxis);

            // We need to rotate the bone for angles around the rotateAxis
            joints[i].RotateAround(joints[i].position, rotateAxis, angle);
        }

        // Update positions
        for (int i = 0; i < joints.Count; i++) {
            joints[i].position = jointPoints[i];
        }

        // PART II - Update stored data
        // Update positions
        for (int i = 0; i < jointPoints.Count; i++) {
            jointPoints[i] = joints[i].position;
        }

        // Update rotations
        for (int i = 0; i < jointRotations.Count; i++) {
            jointRotations[i] = joints[i].rotation;
        }
    }

#if UNITY_EDITOR

    // Helps visualizing the whole structure
    private void OnDrawGizmos() {

        if (!visualizeInEditorRuntime)
            return;

        if (Application.isEditor && visualizeInEditorRuntime)
            Initialize();

        // Update end effector position if not playing
        if(!Application.isPlaying){
            if (Vector3.Distance(target.position, transform.position) > maxLength && maxLength > 0) {
                endEffectorPosition = transform.position + (target.position - transform.position).normalized * maxLength;
            } else {
                endEffectorPosition = target.position;
            }
        }

        if (maxLength > 0 && initialized) {

            // Draw the sphere around the root with max length as radius
            Handles.color = Color.green;
            Handles.RadiusHandle(Quaternion.identity, transform.position, maxLength);

            // Highlight the end effector position
            if (Vector3.Distance(target.position, transform.position) > maxLength && maxLength > 0) {
                Handles.color = Color.red;
                Handles.DrawDottedLine(transform.position, endEffectorPosition, 1f);
                Handles.DrawSolidDisc(endEffectorPosition, endEffectorPosition - transform.position, 0.25f);
            } else {
                Handles.color = Color.green;
                Handles.DrawDottedLine(transform.position, endEffectorPosition, 1f);
                Handles.DrawWireDisc(endEffectorPosition, endEffectorPosition - transform.position, 0.25f);
            }

            // Start from the leaf point
            var current = leafJoint;

            // Start drawing the line all the way up to current (root)
            while (current.parent != null && current != this.transform) {

                // Draw bones
                Handles.color = Color.yellow;
                DrawCubic(current.position, current.parent.position);

                // Draw axes on joints
                Handles.color = Color.blue;
                Handles.DrawLine(current.position, current.position + current.forward * 3);

                Handles.color = Color.red;
                Handles.DrawLine(current.position, current.position + current.right * 3);

                Handles.color = Color.green;
                Handles.DrawLine(current.position, current.position + current.up * 3);

                current = current.parent;
            }

        }

    }

    void DrawCubic(Vector3 start, Vector3 end) {

        // Remember old matrix
        Matrix4x4 oldMatrix = Gizmos.matrix;

        // Set up new matrix
        Vector3 newPosition = Vector3.Lerp(start, end, 0.5f);
        Quaternion newRotation = Quaternion.LookRotation(end - start);
        Vector3 newScale = new Vector3(0.5f, 0.5f, (end - start).magnitude);

        Handles.matrix = Matrix4x4.TRS(newPosition, newRotation, newScale);

        // Draw Cube
        Handles.DrawWireCube(Vector3.zero, Vector3.one);

        // Return old matrix
        Handles.matrix = oldMatrix;
    }

#endif

}
