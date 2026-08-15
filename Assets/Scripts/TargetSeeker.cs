using CustomNavMesh;
using TMPro;
using UnityEngine;

public class TargetSeeker : MonoBehaviour
{
    public CustomNavMeshAgent[] agents;
    public GameObject target;
    public Vector3 lastTargetPos;


    public void Start()
    {
        //SetNewPosition();
        SendSquad();
    }

    public void LateUpdate()
    {
        if (lastTargetPos != target.transform.position)
        {
            //SetNewPosition();
            SendSquad();
        }
    }

    void SetNewPosition()
    {
        lastTargetPos = target.transform.position;
        for (int i = 0; i < agents.Length; i++)
        {
            agents[i].SetDestination(lastTargetPos);
        }
    }

    void SendSquad()
    {
        lastTargetPos = target.transform.position;
        NavMeshJobManager.Instance.MoveGroupWithFlowField(agents, lastTargetPos);
    }
}
