using UnityEngine;
using System.Collections;

public class CameraRotationController : MonoBehaviour {
	public float dragSpeed = 2;
	public float theta=0, phi=0, thetaStart, phiStart;
	public float dist = 40.0f;
	private Vector3 dragOrigin, dragStart;	
	public GameObject cubes;
    public Vector3 angleFix = new Vector3(0, 0, 0);
	// Use this for initialization
	void Start () {
		cubes = GameObject.Find("Cubes");
		phi = Mathf.Acos(0.0f);
		theta = Mathf.Acos(0.7f);
	}
	
	// Update is called once per frame
	void Update () {
        
        transform.position = new Vector3(dist * Mathf.Sin(phi) * Mathf.Cos(theta), dist * Mathf.Cos(phi), dist * Mathf.Sin(phi) * Mathf.Sin(theta));
		transform.LookAt(Vector3.zero);                  
        transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x + angleFix.x, transform.rotation.eulerAngles.y + angleFix.y, transform.rotation.eulerAngles.z+ angleFix.z);        
		if(Input.GetAxis("Mouse ScrollWheel") > 0)
		{
			dist *= 0.9f;
		}
		else if(Input.GetAxis("Mouse ScrollWheel") < 0)
		{
			dist *= 1.1f;
		}
		if(Input.GetMouseButtonDown(0))
		{
			dragOrigin = Input.mousePosition;
			//dragStart = transform.position;
			phiStart = phi;
			thetaStart = theta;
			return;
		}        
		if(!Input.GetMouseButton(0)) return;
		// Vector3 pos = Camera.main.ScreenToViewportPoint(Input.mousePosition - dragOrigin);			
		Vector3 dp = Input.mousePosition - dragOrigin;
        phi = (phiStart + dp.y / 100) % (Mathf.PI * 2);
        if (phi > Mathf.PI) { angleFix.z = -180; }
        if (phi < Mathf.PI && phi > 0) { angleFix.z = 0; }
        if (phi < 0) { angleFix.z = 180; }
        if (phi < -Mathf.PI) { angleFix.z = 0; }
        theta = (thetaStart + dp.x/100) % (Mathf.PI * 2);
                
        Debug.Log(string.Format("phi : {0} | theta : {1}",phi,theta));        
        // Vector3 prev = new Vector3(transform.rotation.x, transform.rotation.y, transform.rotation.z);
        // transform.rotation = Quaternion.Euler(move/10);

    }
}
