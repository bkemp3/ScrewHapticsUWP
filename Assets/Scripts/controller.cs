using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class controller : MonoBehaviour
{
    private BleTest bletest;
    Quaternion quaternion = new Quaternion(0, 0, 0, 0);
    // Start is called before the first frame update
    void Start()
    {
        bletest = GameObject.FindObjectOfType<BleTest>();   
    }

    // Update is called once per frame
    void Update()
    {
        quaternion = bletest.get_quaternion();
        quaternion = IMUToUnity(quaternion);
        // Debug.Log("quaternion: " + quaternion);
        rotation();
    }

    void rotation()
    {
        // Unity accepts x,y,z,w
        // Quaternion spin1 = Quaternion.Euler(new Vector3(0, 90, 0));
        // Quaternion spin = Quaternion.Euler(new Vector3(0, 0, 90));
        // Quaternion spin3 = Quaternion.Euler(new Vector3(180, 0, 0));
        // Quaternion spin4 = Quaternion.Euler(new Vector3(0, 0, 90));
        // Quaternion spin5 = Quaternion.Euler(new Vector3(0, 90, 0));
        // Unity and the IMU are both left handed coordinate systems
        gameObject.transform.rotation = quaternion;
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
