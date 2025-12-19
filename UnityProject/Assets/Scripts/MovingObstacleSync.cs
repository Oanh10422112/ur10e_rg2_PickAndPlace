using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Moveit;
using RosMessageTypes.Geometry;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class MovingObstacleSync : MonoBehaviour
{
    [Header("ROS Configuration")]
    [Tooltip("Every obstacle MUST have a unique ID (e.g., 'cube_1', 'cube_2')")]
    [SerializeField] string m_CollisionId = "moving_obstacle";
    
    [SerializeField] float m_UpdateFrequency = 0.05f; // 20Hz for smooth motion
    
    private ROSConnection m_Ros;
    private float m_LastUpdateTime;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        // Register the standard MoveIt collision topic
        m_Ros.RegisterPublisher<CollisionObjectMsg>("/collision_object");
    }

    void Update()
    {
        // Limit the publish rate to save bandwidth/CPU
        if (Time.time - m_LastUpdateTime > m_UpdateFrequency)
        {
            PublishToMoveIt();
            m_LastUpdateTime = Time.time;
        }
    }

    void PublishToMoveIt()
    {
        CollisionObjectMsg msg = new CollisionObjectMsg();
        msg.header.frame_id = "world"; // Ensure this matches your ROS world frame
        msg.id = m_CollisionId;

        // Use Box primitive for performance
        SolidPrimitiveMsg box = new SolidPrimitiveMsg();
        box.type = SolidPrimitiveMsg.BOX;
        
        // Match the Unity scale (Unity X,Y,Z -> ROS Y,Z,X for FLU conversion)
        Vector3 scale = transform.lossyScale;
        float padding = 0;
        box.dimensions = new double[] { scale.z + padding, scale.x + padding, scale.y + padding };

        // Convert Unity world position/rotation to ROS FLU coordinates
        msg.primitives = new SolidPrimitiveMsg[] { box };
        msg.primitive_poses = new PoseMsg[] { 
            new PoseMsg(transform.position.To<FLU>(), transform.rotation.To<FLU>()) 
        };
        
        msg.operation = CollisionObjectMsg.ADD;

        m_Ros.Publish("/collision_object", msg);
    }
}
