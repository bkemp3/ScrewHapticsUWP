using System.Collections;
using System.Collections.Generic;
using UnityEngine;
// using System.Math;
public class axial_rot : MonoBehaviour
{
    private BleTest bletest;
    Vector3 quaternion = new Vector3(0, 0, 0);
    float revolutions = 0.0f;
    float pitch = 2.4e-3f;
    float t = 0.0f;
    // Start is called before the first frame update
    void Start()
    {
        bletest = GameObject.FindObjectOfType<BleTest>();   
    }

    // Update is called once per frame
    void Update()
    {
        // revolutions = bletest.get_revolutions();
        t = revolutions * pitch;
        // Get modulus of revolutions
        //float rot_y = 2*(float)System.Math.PI*(revolutions % 1);
         float rot_y = 360 * (revolutions % 1);
        quaternion = new Vector3(0, rot_y, 0);
        // Debug.Log("rot_y: " + rot_y);
        rotation();
        // translation();
        revolutions = revolutions + 0.01f;
    }

    void translation()
    {
        // TODO: Check if you need local or global position for this to work
        gameObject.transform.localPosition = gameObject.transform.localPosition - new Vector3(0, t, 0);
    }

    void rotation()
    {
        // Unity accepts x,y,z,w
        // Unity and the IMU are both left handed coordinate systems
        //Debug.Log("quaternion: " + quaternion);
        gameObject.transform.eulerAngles = quaternion;
    }

    Quaternion IMUToUnity(Quaternion input) {
    return new Quaternion(
        -input.y,   // -(  right = -left  )
        input.z,   // -(     up =  up     )
        input.x,   // -(forward =  forward)
         input.w
    );
    }
}
