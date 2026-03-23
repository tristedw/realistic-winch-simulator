using UnityEngine;
using UnityEngine.InputSystem;

public class WinchController : MonoBehaviour
{
    [Header("Winch Settings")]
    public float rotationSpeed = 100f; // degrees per second
    public Vector3 rotationAxis = Vector3.up; // axis to rotate around
    public Rigidbody winchRigidbody; // Rigidbody of the winch

    [Header("Input Settings")]
    public InputActionAsset inputActions;
    public string actionMapName = "Winch";
    public string moveActionName = "Move";
    private InputAction _moveAction;

    void Awake()
    {
        SetupActions();
    }

    void SetupActions()
    {
        if (inputActions == null)
        {
            Debug.LogWarning("[WinchController] No InputActionAsset assigned.");
            return;
        }

        var map = inputActions.FindActionMap(actionMapName, throwIfNotFound: false);
        if (map == null)
        {
            Debug.LogError($"[WinchController] Action map '{actionMapName}' not found in asset {inputActions.name}");
            return;
        }

        _moveAction = map.FindAction(moveActionName, throwIfNotFound: false);

        if (_moveAction == null)
            Debug.LogError($"[WinchController] Move action '{moveActionName}' not found in map '{actionMapName}'. " +
                           $"Available actions: {string.Join(", ", System.Array.ConvertAll(map.actions.ToArray(), a => a.name))}", this);
    }

    void FixedUpdate()
    {
        // Winch control logic
        // only rotate winch when move action is active which is a 1d axis negative and positive 
        // and only rotate on specified axis
        // also make sure that it rotates around the center of the winch and not around the world axis
        if (_moveAction == null) return;

        float input = _moveAction.ReadValue<float>();
        if (Mathf.Abs(input) < 0.01f) return;

        float angle = input * rotationSpeed * Time.fixedDeltaTime;

        // Convert local axis to world axis
        Vector3 worldAxis = winchRigidbody.transform.TransformDirection(rotationAxis);

        Quaternion delta = Quaternion.AngleAxis(angle, worldAxis);

        winchRigidbody.MoveRotation(winchRigidbody.rotation * delta);
    }
}
