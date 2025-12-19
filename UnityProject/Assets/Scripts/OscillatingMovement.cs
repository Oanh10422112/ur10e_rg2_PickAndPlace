using UnityEngine;

/// <summary>
/// Simple script to make a GameObject oscillate between two points along the Z-axis (Forward/Backward).
/// Attach this to an Obstacle GameObject to simulate motion.
/// </summary>
public class OscillatingMovement : MonoBehaviour
{
    [Header("Movement Parameters")]
    // The maximum distance the object will move from its starting position
    [SerializeField]
    private float m_Amplitude = 1.0f; 

    // The speed of the oscillation (in Hz)
    [SerializeField]
    private float m_Frequency = 0.5f; 

    [Header("Direction")]
    [Tooltip("The axis along which the oscillation occurs (e.g., Z for forward/backward).")]
    [SerializeField]
    private Vector3 m_MovementAxis = Vector3.forward;

    // The initial position of the object in world space
    private Vector3 m_StartPosition;
    
    // The accumulated time for the sine wave calculation
    private float m_Time = 0f;

    void Start()
    {
        // Store the starting position when the scene loads
        m_StartPosition = transform.position;

        // Normalize the movement axis to ensure consistent speed regardless of its length
        m_MovementAxis = m_MovementAxis.normalized;
    }

    void Update()
    {
        // Increment time based on the delta time since the last frame
        m_Time += Time.deltaTime;

        // Calculate the sine wave value based on time and frequency
        // The sine function returns a value between -1 and 1
        float offset = Mathf.Sin(m_Time * m_Frequency * 2f * Mathf.PI) * m_Amplitude;
        
        // Calculate the new position
        // New Position = Start Position + (Movement Direction * Offset)
        transform.position = m_StartPosition + (m_MovementAxis * offset);
        
        // The TrajectoryPlanner.cs script will read this new world position
        // and publish it to MoveIt! via the FixedUpdate loop.
    }
}
