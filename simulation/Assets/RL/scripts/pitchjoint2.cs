using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class pitchjoint2 : MonoBehaviour
{
    public GameObject pitch;
    private ArticulationBody articulation,articulation_P;
    public float primaryAxisRotation;
    // Start is called before the first frame update
    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
        articulation_P = pitch.GetComponent<ArticulationBody>();
    }

    // Update is called once per frame
    void Update()
    {
        primaryAxisRotation = pitch.GetComponent<pitchjoint>().primaryAxisRotation;
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        drive.damping= pitch.GetComponent<pitchjoint>().damping;
        drive.stiffness = pitch.GetComponent<pitchjoint>().stiff;
        drive.forceLimit = pitch.GetComponent<pitchjoint>().maxforce;
        articulation.xDrive = drive;
        articulation.jointPosition = new ArticulationReducedSpace(articulation_P.jointPosition[0],0f,0f);
        // joints[0].jointAcceleration = new ArticulationReducedSpace(0f, 0f, 0f);
        articulation.jointForce = new ArticulationReducedSpace(articulation_P.jointForce[0],0f,0f);
        articulation.jointVelocity = new ArticulationReducedSpace(articulation_P.jointVelocity[0],0f,0f);
        
    }
}
