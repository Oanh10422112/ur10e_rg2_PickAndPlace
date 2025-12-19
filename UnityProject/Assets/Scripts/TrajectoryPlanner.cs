using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

// =========================================================================
// 1. Serialisable Custom Class to hold the GameObject and its ID
// =========================================================================
[System.Serializable]
public class Obstacle
{
    // The Unity GameObject to be published
    public GameObject GameObject;
    // The unique string ID for the CollisionObjectMsg (e.g., "table", "printer", "cup")
    public string CollisionId; 
}

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    private string m_TopicName = "/collision_object";
    [SerializeField]
    string m_RosServiceName = "ur10e_rg2_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_UR10e;
    public GameObject UR10e { get => m_UR10e; set => m_UR10e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }
    
    [SerializeField]
    GameObject m_GripperBase;
    
    [Header("Safety Bar Settings")]
    [SerializeField] float m_Amplitude = 1.0f; 
    [SerializeField] Vector3 m_MovementAxis = Vector3.forward;

    // =========================================================================
    // 2. NEW: List for all obstacles (Table, Printer, etc.)
    // Removed old fields: m_Table and m_Printer
    // =========================================================================
    [SerializeField]
    public List<Obstacle> m_ObstaclesToPublish = new List<Obstacle>();


    // Assures that the gripper is always positioned beside the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(0, 180, 90);
    readonly Vector3 m_PickPoseOffset = Vector3.left * 0.47f;
    readonly Quaternion m_PlaceOrientation = Quaternion.Euler(0, 90, 180);
    readonly Vector3 m_PlacePoseOffset = Vector3.up * 0.28f;

    // Articulation Bodies for the Robot arm
    ArticulationBody[] m_JointArticulationBodies;
    // Articulation Bodies for the Gripper
    ArticulationBody m_LeftInnerKnuckle;
    ArticulationBody m_RightInnerKnuckle;
    ArticulationBody m_LeftOuterKnuckle;
    ArticulationBody m_RightOuterKnuckle;
    ArticulationBody m_leftInnerFinger;
    ArticulationBody m_rightInnerFinger;


    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///       Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///       Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);
        // Register the collision object publisher
        m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_UR10e.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Identify gripper joints
        string gripperBasePath = "base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/onrobot_rg2_base_link";
        m_LeftInnerKnuckle = m_UR10e.transform.Find(gripperBasePath + "/left_inner_knuckle").GetComponent<ArticulationBody>();
        m_RightInnerKnuckle = m_UR10e.transform.Find(gripperBasePath + "/right_inner_knuckle").GetComponent<ArticulationBody>();
        m_LeftOuterKnuckle = m_UR10e.transform.Find(gripperBasePath + "/left_outer_knuckle").GetComponent<ArticulationBody>();
        m_RightOuterKnuckle = m_UR10e.transform.Find(gripperBasePath + "/right_outer_knuckle").GetComponent<ArticulationBody>();
        m_leftInnerFinger = m_UR10e.transform.Find(gripperBasePath + "/left_outer_knuckle/left_inner_finger").GetComponent<ArticulationBody>();
        m_rightInnerFinger = m_UR10e.transform.Find(gripperBasePath + "/right_outer_knuckle/right_inner_finger").GetComponent<ArticulationBody>();

        if (!m_LeftInnerKnuckle || !m_RightInnerKnuckle || !m_LeftOuterKnuckle || !m_RightOuterKnuckle || !m_leftInnerFinger || !m_rightInnerFinger)
        {
            Debug.LogError("Some gripper articulation bodies are missing. Please check the hierarchy.");
        }
    }
    
    void CloseGripper()
    {
        float closeValue = 24f;

        SetGripperPosition(closeValue);
        
    // PARENTING LOGIC: Make the target stick to the gripper
    if (m_Target != null && m_GripperBase != null)
    {
        // Attach to the gripper base so it moves with the arm
        m_Target.transform.SetParent(m_LeftOuterKnuckle.transform.parent); 
        
        // Optional: Disable physics so it doesn't jitter while being carried
        if (m_Target.TryGetComponent<Rigidbody>(out Rigidbody rb))
        {
            rb.isKinematic = true;
            rb.detectCollisions = false;
        }
    }
    }

    void OpenGripper()
    {
        float openValue = 10f;

        SetGripperPosition(openValue);
        
        // UNPARENTING LOGIC: Release the target
    if (m_Target != null)
    {
        m_Target.transform.SetParent(null); // Detach from gripper
        
        // Optional: Re-enable physics so it stays where it was placed
        if (m_Target.TryGetComponent<Rigidbody>(out Rigidbody rb))
        {
            rb.isKinematic = false;
            rb.detectCollisions = true;
        }
    }
    }

    /// <summary>
    ///       Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>Ur10eMoveitJoints</returns>
    Ur10eMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new Ur10eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///       Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///       the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///       Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///       execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        // =========================================================================
        // 4. NEW: Loop through all obstacles and publish their mesh data
        // Replaced PublishTableMesh() and PublishPrinterMesh() calls
        // =========================================================================
        foreach (Obstacle obs in m_ObstaclesToPublish)
        {
            if (obs.GameObject != null)
            {
                PublishObstacleMesh(obs.GameObject, obs.CollisionId);
            }
        }
        
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PlacePoseOffset).To<FLU>(),
            orientation = m_PlaceOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///       Execute the returned trajectories from the MoverService. (omitted for brevity)
    /// </summary>
IEnumerator ExecuteTrajectories(MoverServiceResponse response)
{
    if (response.trajectories != null)
    {
        for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
        {
            foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
            {
                var jointPositions = t.positions;
                var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                {
                    var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                    joint1XDrive.target = result[joint];
                    m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                }
                yield return new WaitForSeconds(k_JointAssignmentWait);
            }

            // --- GRIPPER CONTROL LOGIC ---

            // 1. Close and Attach when finishing the Grasp trajectory
            if (poseIndex == (int)Poses.Grasp)
            {
                CloseGripper(); 
            }

            // 2. Open and Detach ONLY when finishing the Place trajectory
            if (poseIndex == (int)Poses.Place)
            {
                OpenGripper();
            }

            yield return new WaitForSeconds(k_PoseAssignmentWait);
        }

        // REMOVE the OpenGripper() from here, as it's now handled inside the loop
    }
}

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
    
    // =========================================================================
    // 3. NEW: Generic function to replace PublishTableMesh() and PublishPrinterMesh()
    // The old PublishTableMesh() and PublishPrinterMesh() functions are removed.
    // =========================================================================
    /// <summary>
    /// Publishes the mesh of a given GameObject as a MoveIt! Collision Object.
    /// </summary>
    /// <param name="obstacleGameObject">The Unity GameObject representing the obstacle.</param>
    /// <param name="collisionObjectId">The unique ID for the collision object (e.g., "table", "printer").</param>
    void PublishObstacleMesh(GameObject obstacleGameObject, string collisionObjectId)
    {
        // Get the MeshFilter component
        MeshFilter meshFilter = obstacleGameObject.GetComponent<MeshFilter>();
        if (meshFilter == null)
        {
            Debug.LogError($"MeshFilter component not found on {collisionObjectId} GameObject.");
            return;
        }

        // Retrieve the mesh, vertices, and triangles data
        Mesh mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // Validate mesh data
        if (vertices == null || vertices.Length == 0 || triangles == null || triangles.Length == 0)
        {
            Debug.LogError($"Mesh data for {collisionObjectId} is invalid or empty.");
            return;
        }

        // Prepare vertices and triangles for ROS messages
        List<PointMsg> points = new List<PointMsg>();
        foreach (Vector3 vertex in vertices)
        {
            // Convert vertex to world space
            Vector3 worldVertex = obstacleGameObject.transform.TransformPoint(vertex);

            // =========================================================================
            // CALCULATION: Stretch the "moving_cube" into a static bar
            // =========================================================================
            if (collisionObjectId == "moving_cube")
            {
                // Determine if this vertex is on the 'positive' or 'negative' side of the axis
                float projection = Vector3.Dot(vertex, m_MovementAxis);
                float stretch = projection > 0 ? m_Amplitude : -m_Amplitude;
                
                // Offset the world position to create the "Bar"
                worldVertex += obstacleGameObject.transform.TransformDirection(m_MovementAxis) * stretch;
            }
            // Convert vertex to world space, then to FLU orientation and add to points list
            var fluVertex = obstacleGameObject.transform.TransformPoint(vertex).To<FLU>();
            points.Add(new PointMsg(fluVertex.x, fluVertex.y, fluVertex.z));
        }

        List<MeshTriangleMsg> triangleMsgs = new List<MeshTriangleMsg>();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            triangleMsgs.Add(new MeshTriangleMsg
            {
                vertex_indices = new uint[]
                {
                    (uint)triangles[i],
                    (uint)triangles[i + 2], // Swap these two indices for winding order correction
                    (uint)triangles[i + 1]
                }
            });
        }


        // Create MeshMsg
        MeshMsg meshMsg = new MeshMsg
        {
            vertices = points.ToArray(),
            triangles = triangleMsgs.ToArray()
        };

        // PoseMsg: The mesh is published relative to the base_link frame at (0, 0, 0)
        PoseMsg pose = new PoseMsg
        {
            position = new PointMsg(0.0, 0.0, 0.0),
            orientation = new QuaternionMsg(0.0, 0.0, 0.0, 1.0)
        };

        // Create CollisionObjectMsg
        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "base_link"
            },
            id = collisionObjectId, // <--- Dynamic ID
            operation = CollisionObjectMsg.ADD,
            mesh_poses = new PoseMsg[] { pose },
            meshes = new MeshMsg[] { meshMsg }
        };
        
        // 1. Create a quick "Delete" message for this specific ID
	var removeObject = new CollisionObjectMsg {
    	id = collisionObjectId,
    	operation = CollisionObjectMsg.REMOVE 
	};

	// 2. Publish the delete, then immediately publish the new stretched ADD
	m_Ros.Publish(m_TopicName, removeObject); 
	m_Ros.Publish(m_TopicName, collisionObject);

        // Publish the CollisionObjectMsg
        m_Ros.Publish("/collision_object", collisionObject);
    }

    void SetGripperPosition(float position)
    {
        ArticulationDrive drive = m_LeftInnerKnuckle.xDrive;
        drive.target = -position;
        m_LeftInnerKnuckle.xDrive = drive;

        drive = m_RightInnerKnuckle.xDrive;
        drive.target = -position;
        m_RightInnerKnuckle.xDrive = drive;

        drive = m_LeftOuterKnuckle.xDrive;
        drive.target = position;
        m_LeftOuterKnuckle.xDrive = drive;

        drive = m_RightOuterKnuckle.xDrive;
        drive.target = -position;
        m_RightOuterKnuckle.xDrive = drive;

        drive = m_leftInnerFinger.xDrive;
        drive.target = position;
        m_leftInnerFinger.xDrive = drive;

        drive = m_rightInnerFinger.xDrive;
        drive.target = position;
        m_rightInnerFinger.xDrive = drive;
    }
}
