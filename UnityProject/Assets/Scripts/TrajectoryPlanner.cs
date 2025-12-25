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

[System.Serializable]
public class Obstacle
{
    public GameObject GameObject;
    public string CollisionId; 
}

public class TrajectoryPlanner : MonoBehaviour
{
    // --- Constants ---
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // --- ROS Settings ---
    [Header("ROS Settings")]
    [SerializeField] string m_TopicName = "/collision_object";
    [SerializeField] string m_RosServiceName = "ur10e_rg2_moveit";

    // --- References ---
    [Header("Robot References")]
    [SerializeField] public GameObject m_UR10e;
    [SerializeField] public GameObject m_Target;
    [SerializeField] public GameObject m_TargetPlacement;
    
    // Manual Joint List
    [Header("Joints (Shoulder -> Wrist3)")]
    public ArticulationBody[] m_JointArticulationBodies;

    // --- Obstacles ---
    [Header("Obstacles")]
    [SerializeField] public GameObject m_Table;
    [SerializeField] public GameObject m_Printer;
    [SerializeField] public GameObject m_MovingObstacle; // The Snake Cube
    
    [Header("Other Static Obstacles")]
    public List<Obstacle> m_StaticObstacles = new List<Obstacle>();
    
    // --- Snake Settings ---
    [Header("Snake Settings")]
    [SerializeField] float m_ScanDuration = 4.0f;
    [SerializeField] float m_ScanInterval = 0.5f;
    private List<string> m_PublishedBarIds = new List<string>();

    // --- Gripper Joints ---
    ArticulationBody m_LeftInnerKnuckle;
    ArticulationBody m_RightInnerKnuckle;
    ArticulationBody m_LeftOuterKnuckle;
    ArticulationBody m_RightOuterKnuckle;
    ArticulationBody m_leftInnerFinger;
    ArticulationBody m_rightInnerFinger;

    // --- The Working Offsets ---
    readonly Quaternion m_PickOrientation = Quaternion.Euler(0, 180, 90);
    readonly Vector3 m_PickPoseOffset = Vector3.left * 0.47f;
    readonly Quaternion m_PlaceOrientation = Quaternion.Euler(0, 90, 180);
    readonly Vector3 m_PlacePoseOffset = Vector3.up * 0.28f;

    ROSConnection m_Ros;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);
        m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);
        m_PublishedBarIds = new List<string>();

        // Find Gripper Parts
        string gripperBasePath = "base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/onrobot_rg2_base_link";
        Transform baseT = m_UR10e.transform;
        // Note: Using Transform.Find helper
        m_LeftInnerKnuckle = baseT.Find(gripperBasePath + "/left_inner_knuckle")?.GetComponent<ArticulationBody>();
        m_RightInnerKnuckle = baseT.Find(gripperBasePath + "/right_inner_knuckle")?.GetComponent<ArticulationBody>();
        m_LeftOuterKnuckle = baseT.Find(gripperBasePath + "/left_outer_knuckle")?.GetComponent<ArticulationBody>();
        m_RightOuterKnuckle = baseT.Find(gripperBasePath + "/right_outer_knuckle")?.GetComponent<ArticulationBody>();
        m_leftInnerFinger = baseT.Find(gripperBasePath + "/left_outer_knuckle/left_inner_finger")?.GetComponent<ArticulationBody>();
        m_rightInnerFinger = baseT.Find(gripperBasePath + "/right_outer_knuckle/right_inner_finger")?.GetComponent<ArticulationBody>();
    }

    public void PublishJoints()
    {
        // 1. Publish Static Obstacles (Table & Printer)
        // We use the "Baked Transform" method so they are perfect in ROS.
        PublishBakedMesh(m_Table, "table");
        PublishBakedMesh(m_Printer, "3d_printer");
        
        foreach (Obstacle obs in m_StaticObstacles)
        {
            if (obs.GameObject != null)
            {
                PublishBakedMesh(obs.GameObject, obs.CollisionId);
            }
        }

        // 2. Start Snake Scan
        StartCoroutine(ScanAndGenerateObstacleBars());
    }

    IEnumerator ScanAndGenerateObstacleBars()
    {
        Debug.Log("Starting Scan...");
        m_PublishedBarIds.Clear();

        if (m_MovingObstacle != null)
        {
            Vector3 startPos = m_MovingObstacle.transform.position;
            float elapsedTime = 0f;

            while (elapsedTime < m_ScanDuration)
            {
                string barId = $"path_bar_{elapsedTime:F1}";
                m_PublishedBarIds.Add(barId);
                
                // Publish the snake bar using the baked method too!
                PublishBakedMesh(m_MovingObstacle, barId);
                
                yield return new WaitForSeconds(m_ScanInterval);
                elapsedTime += m_ScanInterval;
                
                // Optional: Move obstacle here if simulating
            }
            m_MovingObstacle.transform.position = startPos;
        }

        Debug.Log("Scan Complete. Requesting Movement.");
        SendMoverServiceRequest();
    }

    void SendMoverServiceRequest()
    {
        MoverServiceRequest request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PlacePoseOffset).To<FLU>(),
            orientation = m_PlaceOrientation.To<FLU>()
        };

        Debug.Log($"[ROS] Requesting Plan. Pick Offset: {m_PickPoseOffset}");
        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories != null && response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned. Executing...");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoveIt.");
        }
    }

    // --- TIME-SYNCED EXECUTION (Crucial for smooth movement) ---
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // Iterate through every point in the plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    
                    // Convert ROS (Rad) to Unity (Deg)
                    // var result = jointPositions.Select((r, i) => (float)r * Mathf.Rad2Deg * m_JointDirection[i]).ToArray();
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        SetJointTargetStep(m_JointArticulationBodies[joint], result[joint]);
                    }

                    // 0.1f = 10 FPS (Choppy)
                    // 0.02f = 50 FPS (Smoother)
                    yield return new WaitForSeconds(0.02f); 
                }

                // Small settle time between stages
                yield return new WaitForSeconds(0.25f);

                // Gripper Actions
                if (poseIndex == (int)Poses.Grasp) CloseGripper();
                if (poseIndex == (int)Poses.Place) OpenGripper();
                
                yield return new WaitForSeconds(0.5f);
            }
        }
    }
    
    // --- HELPER FUNCTIONS ---
    void PublishBakedMesh(GameObject obj, string id)
    {
        MeshFilter meshFilter = obj.GetComponent<MeshFilter>();
        if (meshFilter == null) return;

        Mesh mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        List<PointMsg> points = new List<PointMsg>();

        foreach (Vector3 vertex in vertices)
        {
            var fluVertex = obj.transform.TransformPoint(vertex).To<FLU>();
            points.Add(new PointMsg(fluVertex.x, fluVertex.y, fluVertex.z));
        }

        // Triangles need to be flipped for ROS (Culling order)
        List<MeshTriangleMsg> triangleMsgs = new List<MeshTriangleMsg>();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            triangleMsgs.Add(new MeshTriangleMsg
            {
                vertex_indices = new uint[] { (uint)triangles[i], (uint)triangles[i + 2], (uint)triangles[i + 1] }
            });
        }

        MeshMsg meshMsg = new MeshMsg { vertices = points.ToArray(), triangles = triangleMsgs.ToArray() };
        
        // Identity Pose
        PoseMsg pose = new PoseMsg
        {
            position = new PointMsg(0,0,0),
            orientation = new QuaternionMsg(0,0,0,1)
        };

        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg { frame_id = "base_link" },
            id = id,
            operation = CollisionObjectMsg.ADD,
            mesh_poses = new PoseMsg[] { pose },
            meshes = new MeshMsg[] { meshMsg }
        };

        m_Ros.Publish(m_TopicName, collisionObject);
    }

    // Handles X/Y/Z Drive automatically
    void SetJointTargetStep(ArticulationBody body, float targetAngle)
    {
        if (body.twistLock == ArticulationDofLock.FreeMotion || body.twistLock == ArticulationDofLock.LimitedMotion)
        {
            var d = body.xDrive;
            d.target = targetAngle;
            body.xDrive = d;
        }
        else if (body.swingYLock == ArticulationDofLock.FreeMotion || body.swingYLock == ArticulationDofLock.LimitedMotion)
        {
            var d = body.yDrive;
            d.target = targetAngle;
            body.yDrive = d;
        }
        else if (body.swingZLock == ArticulationDofLock.FreeMotion || body.swingZLock == ArticulationDofLock.LimitedMotion)
        {
            var d = body.zDrive;
            d.target = targetAngle;
            body.zDrive = d;
        }
    }

    Ur10eMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new Ur10eMoveitJointsMsg();
        for (var i = 0; i < k_NumRobotJoints; i++)
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        return joints;
    }

    void CloseGripper() { 
    SetGripperPosition(24f); 
    if(m_Target) { 
    	Transform handBase = m_JointArticulationBodies[5].transform;
    	m_Target.transform.SetParent(handBase);
    	var rb = m_Target.GetComponent<Rigidbody>(); 
    	if(rb) rb.isKinematic = true; 
    	} 
    }
    void OpenGripper() { 
    SetGripperPosition(10f); 
    if(m_Target) { 
    	m_Target.transform.SetParent(null); 
    	var rb = m_Target.GetComponent<Rigidbody>(); 
    	if(rb) rb.isKinematic = false; 
    	} 
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

    enum Poses { PreGrasp, Grasp, PickUp, Place }
}
