using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(CableSimulator))]
public class CableGrabHandler : MonoBehaviour
{
    [Header("Grab Settings")]
    public float grabRadius = 0.35f;
    public float dragSmoothing = 18f;
    public bool instantDrag = false;

    [Header("Grabbable Node Range")]
    [Tooltip("First node index that can be grabbed (0 = the start end).")]
    public int grabableNodeMin = 0;
    public int grabableNodeMax = 999;

    public InputActionAsset inputActions;
    public string actionMapName = "Cable";
    public string pointActionName = "Point";
    public string clickActionName = "Click";

    [Header("Debug")]
    public bool logGrabAttempts = true;
    public bool logDragEveryFrame = false;
    public bool logGrabResult = true;
    public bool logRelease = true;
    public bool logPointerReads = false;

    private CableSimulator _sim;
    private Camera _cam;

    private InputAction _pointAction;
    private InputAction _clickAction;

    private Rigidbody _heldRb;
    private int _heldIndex = -1;
    private float _holdDepth;
    private bool _wasKinematic;
    private bool _prevPressed;

    void Awake()
    {
        _sim = GetComponent<CableSimulator>();
        _cam = Camera.main;

        if (_sim == null)
            Debug.LogError("[CableGrab] CableSimulator not found on this GameObject! " +
                           "CableGrabHandler will not work.");

        SetupActions();
    }

    void OnEnable()
    {
        _pointAction?.Enable();
        _clickAction?.Enable();

        if (_clickAction != null)
        {
            _clickAction.started += OnGrabStarted;
            _clickAction.canceled += OnGrabCanceled;
        }
    }

    void OnDisable()
    {
        if (_clickAction != null)
        {
            _clickAction.started -= OnGrabStarted;
            _clickAction.canceled -= OnGrabCanceled;
        }

        _pointAction?.Disable();
        _clickAction?.Disable();

        Release();
    }

    void Update()
    {
        if (_cam == null)
        {
            _cam = Camera.main;
        }

        DragHeld();
    }

    void SetupActions()
    {
        if (inputActions == null)
        {
            Debug.LogWarning("[CableGrab] No InputActionAsset assigned.");
            return;
        }

        var map = inputActions.FindActionMap(actionMapName, throwIfNotFound: false);
        if (map == null)
        {
            Debug.LogError($"[CableGrab] Action map '{actionMapName}' not found in asset {inputActions.name}");
            return;
        }

        _pointAction = map.FindAction(pointActionName, throwIfNotFound: false);
        _clickAction = map.FindAction(clickActionName, throwIfNotFound: false);

        if (_pointAction == null)
            Debug.LogError($"[CableGrab] Point action '{pointActionName}' not found in map '{actionMapName}'. " +
                           $"Available actions: {string.Join(", ", System.Array.ConvertAll(map.actions.ToArray(), a => a.name))}", this);

        if (_clickAction == null)
            Debug.LogError($"[CableGrab] Click action '{clickActionName}' not found in map '{actionMapName}'. " +
                           $"Available actions: {string.Join(", ", System.Array.ConvertAll(map.actions.ToArray(), a => a.name))}", this);

        if (_pointAction == null || _clickAction == null)
        {
            Debug.LogWarning("[CableGrab] One or more actions missing.");
        }
    }

    void OnGrabStarted(InputAction.CallbackContext ctx)
    {
        Vector2 pos = ReadPointerPosition();
        TryGrab(pos);
    }

    void OnGrabCanceled(InputAction.CallbackContext ctx)
    {
        Release();
    }

    Vector2 ReadPointerPosition()
    {
        Vector2 pos;

        pos = _pointAction != null ? _pointAction.ReadValue<Vector2>() : Vector2.zero;

        return pos;
    }

    void TryGrab(Vector2 screenPos)
    {
        if (_cam == null)
        {
            Debug.LogWarning("[CableGrab] TryGrab called but camera is null.", this);
            return;
        }

        if (_sim == null)
        {
            Debug.LogError("[CableGrab] TryGrab called but CableSimulator is null.", this);
            return;
        }

        Ray ray = _cam.ScreenPointToRay(screenPos);

        int nodeMax = grabableNodeMax >= _sim.NodeCount - 1
                           ? _sim.NodeCount
                           : grabableNodeMax + 1;
        int nodeMin = Mathf.Max(0, grabableNodeMin);
        int eligible = nodeMax - nodeMin;

        if (logGrabAttempts)
            Debug.Log($"[CableGrab] TryGrab screenPos={screenPos} | " +
                      $"ray origin={ray.origin:F2} dir={ray.direction:F2} | " +
                      $"checking nodes [{nodeMin}..{nodeMax - 1}] ({eligible} nodes) | " +
                      $"grabRadius={grabRadius}", this);

        if (eligible <= 0)
        {
            Debug.LogWarning($"[CableGrab] No eligible nodes to check. " +
                             $"NodeCount={_sim.NodeCount}, min={nodeMin}, max={nodeMax - 1}. " +
                             $"Check grabableNodeMin/Max.");
            return;
        }

        float bestDist = grabRadius;
        int bestIdx = -1;

        for (int i = nodeMin; i < nodeMax; i++)
        {
            Rigidbody rb = _sim.GetNode(i);
            if (rb == null)
            {
                Debug.LogWarning($"[CableGrab] Node {i} Rigidbody is null — skipping.");
                continue;
            }

            Vector3 nodePos = rb.position;
            Vector3 toNode = nodePos - ray.origin;
            float along = Vector3.Dot(toNode, ray.direction);

            if (along < 0f)
            {
                if (logGrabAttempts)
                    Debug.Log($"[CableGrab] Node {i}: behind camera (along={along:F2}) skipped.");
                continue;
            }

            Vector3 closest = ray.origin + ray.direction * along;
            float dist = Vector3.Distance(closest, nodePos);
            bool isCandidate = dist < bestDist;

            if (logGrabAttempts)
                Debug.Log($"[CableGrab] Node {i}: worldPos={nodePos:F2} | " +
                          $"rayDist={dist:F3} | threshold={bestDist:F3} | " +
                          $"{(isCandidate ? "can" : "too far")}");

            if (isCandidate)
            {
                bestDist = dist;
                bestIdx = i;
            }
        }

        if (bestIdx < 0)
        {
            if (logGrabResult)
                Debug.Log($"[CableGrab] TryGrab MISSED no node within grabRadius ({grabRadius}).");
            return;
        }

        //lock node
        _heldIndex = bestIdx;
        _heldRb = _sim.GetNode(bestIdx);
        _holdDepth = Vector3.Dot(
                            _heldRb.position - _cam.transform.position,
                            _cam.transform.forward);
        _wasKinematic = _heldRb.isKinematic;
        _heldRb.isKinematic = true;

        if (logGrabResult)
            Debug.Log($"[CableGrab] GRABBED node {bestIdx} at worldPos={_heldRb.position:F2} | " +
                      $"holdDepth={_holdDepth:F2} | wasKinematic={_wasKinematic}");
    }

    void DragHeld()
    {
        if (_heldRb == null || _cam == null) return;

        Vector2 screenPos = ReadPointerPosition();
        Ray ray = _cam.ScreenPointToRay(screenPos);
        float denom = Vector3.Dot(ray.direction, _cam.transform.forward);

        if (Mathf.Abs(denom) < 1e-4f)
        {
            if (logDragEveryFrame)
                Debug.LogWarning("[CableGrab] DragHeld: ray nearly parallel to camera plane.");
            return;
        }

        float t = (_holdDepth - Vector3.Dot(
                              ray.origin - _cam.transform.position,
                              _cam.transform.forward)) / denom;
        Vector3 target = ray.origin + ray.direction * t;

        if (logDragEveryFrame)
            Debug.Log($"[CableGrab] Dragging node {_heldIndex} target={target:F2} | " +
                      $"current={_heldRb.position:F2} | delta={Vector3.Distance(_heldRb.position, target):F3}");

        if (instantDrag)
            _heldRb.MovePosition(target);
        else
            _heldRb.MovePosition(Vector3.Lerp(
                _heldRb.position, target, Time.deltaTime * dragSmoothing));
    }

    void Release()
    {
        if (_heldRb == null) return;

        if (logRelease)
            Debug.Log($"[CableGrab] Released node {_heldIndex} at worldPos={_heldRb.position:F2} | " +
                      $"restoring isKinematic={_wasKinematic}");

        _heldRb.isKinematic = _wasKinematic;
        _heldRb.linearVelocity = Vector3.zero;
        _heldRb = null;
        _heldIndex = -1;
    }

    void OnDrawGizmosSelected()
    {
        if (_sim == null) return;

        int nodeMax = grabableNodeMax >= _sim.NodeCount - 1
                         ? _sim.NodeCount
                         : grabableNodeMax + 1;
        int nodeMin = Mathf.Max(0, grabableNodeMin);

        for (int i = nodeMin; i < nodeMax; i++)
        {
            var rb = _sim.GetNode(i);
            if (rb == null) continue;

            // Held node = green, grabbable = yellow
            bool isHeld = (i == _heldIndex);
            Gizmos.color = isHeld
                ? new Color(0.2f, 1f, 0.2f, 0.6f)
                : new Color(1f, 0.85f, 0f, 0.25f);
            Gizmos.DrawWireSphere(rb.position, grabRadius);

            // Draw node index label offset slightly
#if UNITY_EDITOR
            UnityEditor.Handles.Label(rb.position + Vector3.up * (grabRadius + 0.04f),
                                      $"N{i}" + (isHeld ? " [HELD]" : ""));
#endif
        }

        // If currently dragging, draw a line from held node to camera ray
        if (_heldRb != null && _cam != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(_heldRb.position, _cam.transform.position);
        }
    }
}