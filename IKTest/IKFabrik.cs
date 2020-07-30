using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

//[ExecuteInEditMode]
public class IKFabrik : MonoBehaviour{

    public int iteration;
    public Transform leafPoint;
    public Transform target;
    public Transform pole;

    List<Transform> bones;      // leaf to root
    List<Vector3> positions;    // leaf to root
    List<Quaternion> rotations; // leaf to root
    List<float> lengths;        // leaf to root

    float totalLength;

    bool initialized;

    private void Start() {
        Init();
    }

    private void Update() {
        if (!initialized)
            return;

        if(!target){
            Debug.LogWarning("IK: target not assigned!");
            return;
        }

        if(Vector3.Distance(target.position, transform.position) > totalLength){
            Stretch();
        }else{
            for (int i = 0; i < iteration; i++)
                SolveGeneric();
        }
    }

    void Init(){
        initialized = false;

        // leaf point not assigned!
        if(leafPoint == null){
            return;
        }

        // Start from the leaf point
        var current = leafPoint;

        // Initialize the lists
        bones = new List<Transform>();
        positions = new List<Vector3>();
        rotations = new List<Quaternion>();
        lengths = new List<float>();

        // Traverse from leaf to root. Stop if the current point is a root (root, i.e. with no parent)
        while(current.parent != null){

            // Add current point to points
            bones.Add(current);
            positions.Add(current.position);
            rotations.Add(current.rotation);

            // Add length between current point and its parent
            lengths.Add(Vector3.Distance(current.position, current.parent.position));
            current = current.parent;
        }

        // Add the root point
        bones.Add(current);
        positions.Add(current.position);
        rotations.Add(current.rotation);

        // Sum all lenghts together
        totalLength = 0f;
        foreach (float length in lengths)
            totalLength += length;

        initialized = true;
    }

    void SolveGeneric(){
        TraverseBackward();
        TraverseForward();
        BendTowardsPole();
        UpdateObjects();
    }

    void TraverseBackward(){

        // Generic solution
        positions[0] = target.position;

        for (int i = 1; i < positions.Count; i++){
            Vector3 direction = (positions[i] - positions[i - 1]).normalized;
            positions[i] = positions[i - 1] + direction * lengths[i - 1];
        }

        // O----O----O----O
        /*
        direction = (positions[1] - positions[0]).normalized;
        positions[1] = positions[0] + direction * lengths[0];

        direction = (positions[2] - positions[1]).normalized;
        positions[2] = positions[1] + direction * lengths[1];

        direction = (positions[3] - positions[2]).normalized;
        positions[3] = positions[2] + direction * lengths[2];
        */

    }

    void TraverseForward(){

        // Generic solution
        positions[positions.Count - 1] = transform.position;

        for (int i = positions.Count - 1; i > 0; i--){
            Vector3 direction = (positions[i - 1] - positions[i]).normalized;
            positions[i - 1] = positions[i] + direction * lengths[i - 1];
        }

        // O----O----O----O
        /*

        positions[3] = transform.position;
        
        direction = (positions[2] - positions[3]).normalized;
        positions[2] = positions[3] + direction * lengths[2];

        direction = (positions[1] - positions[2]).normalized;
        positions[1] = positions[2] + direction * lengths[1];

        direction = (positions[0] - positions[1]).normalized;
        positions[0] = positions[1] + direction * lengths[0];
        */

    }

    void BendTowardsPole(){

        if (!pole)
            return;

        for (int i = positions.Count - 2; i > 1; i --){
            Plane plane = new Plane(positions[i - 1] - positions[i + 1], positions[i + 1]);
            Vector3 projectedPole = plane.ClosestPointOnPlane(pole.position);
            Vector3 projectedBone = plane.ClosestPointOnPlane(positions[i]);
            float angle = Vector3.SignedAngle(projectedBone - positions[i + 1], projectedBone - positions[i + 1], plane.normal);
            positions[i] = positions[i + 1] + Quaternion.AngleAxis(angle, plane.normal) * (positions[i] - positions[i + 1]);
        }

        /*
        Plane plane;
        Vector3 projectedPole;
        Vector3 projectedBone;
        float angle;

        plane = new Plane(positions[1] - positions[3], positions[3]);
        projectedPole = plane.ClosestPointOnPlane(pole.position);
        projectedBone = plane.ClosestPointOnPlane(positions[2]);
        angle = Vector3.SignedAngle(projectedBone - positions[3], projectedPole - positions[3], plane.normal);
        positions[2] = Quaternion.AngleAxis(angle, plane.normal) * (positions[2] - positions[3]) + positions[3];
        */
    }

    void Stretch(){
        Vector3 stretchDirection = (target.position - transform.position).normalized;

        // Traverse in inverse; the list is leaf->root
        for (int i = 0; i < bones.Count - 1; i++){
            bones[bones.Count - 2 - i].position = bones[bones.Count - 1 - i].position + stretchDirection * lengths[lengths.Count - 1 - i];
        }

        for (int i = 0; i < positions.Count; i++) {
            positions[i] = bones[i].position;
        }

        for (int i = bones.Count - 1; i > 0; i--) { // exclude the leaf bone (its rotation does not matter)

            Vector3 fromDirection = bones[i - 1].position - bones[i].position;
            Vector3 toDirection = positions[i - 1] - bones[i].position;
            Vector3 rotateAxis = Vector3.Cross(fromDirection, toDirection);
            float angle = Vector3.SignedAngle(fromDirection, toDirection, rotateAxis);

            // We need to rotate the bone for angles around the rotateAxis
            bones[i].RotateAround(bones[i].position, rotateAxis, angle);
        }

    }

    void UpdateObjects(){

        // Update the actual rotations and positions
        for (int i = bones.Count - 1; i > 0; i--) { // exclude the leaf bone (its rotation does not matter)

            Vector3 fromDirection = bones[i - 1].position - bones[i].position;
            Vector3 toDirection = positions[i - 1] - bones[i].position;
            Vector3 rotateAxis = Vector3.Cross(fromDirection, toDirection);
            float angle = Vector3.SignedAngle(fromDirection, toDirection, rotateAxis);

            // We need to rotate the bone for angles around the rotateAxis
            bones[i].RotateAround(bones[i].position, rotateAxis, angle);
        }

        for (int i = 0; i < bones.Count; i++) {
            bones[i].position = positions[i];
        }

        // Update stored data
        for (int i = 0; i < positions.Count; i++){
            positions[i] = bones[i].position;
        }

        for (int i = 0; i < rotations.Count; i++){
            rotations[i] = bones[i].rotation;
        }
    }

    private void OnDrawGizmos() {
#if UNITY_EDITOR
        // leaf point not assigned!
        if (leafPoint == null) {
            return;
        }

        // Start from the leaf point
        var current = leafPoint;

        // Start drawing the line all the way up to current (root)
        while (current.parent != null) {
            //Handles.DrawLine(current.position, current.parent.position);
            Handles.color = Color.yellow;
            DrawCubic(current.position, current.parent.position);

            Handles.color = Color.blue;
            Handles.DrawLine(current.position, current.position + current.forward * 5);

            Handles.color = Color.red;
            Handles.DrawLine(current.position, current.position + current.right * 5);

            Handles.color = Color.green;
            Handles.DrawLine(current.position, current.position + current.up * 5);

            current = current.parent;
        }
#endif
    }

    void DrawCubic(Vector3 start, Vector3 end) {
#if UNITY_EDITOR
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
#endif
    }
}
