using System.Runtime.InteropServices;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

public class ArucoScript : MonoBehaviour
{
    public MeshRenderer displayMesh;
    private WebCamTexture _webcam;
    private Texture2D _cameraTexture;
    public GameObject markerObjectPrefab;
    public Camera cam_world;
    private Color32[] rawImg;
    public int webcamDeviceNumber;
    public GameObject Display;

    public int aruco_dict_id;
    public int dict_size;
    public float marker_size;
    public int cam_width;
    public int cam_height;
    public Vector2 fparams;
    public Vector2 cparams;

    public static int marker_count;
    public static int[] ids;
    public static float[] corners;
    public static double[] rvecs;
    public static double[] tvecs;
    public static Dictionary<int, GameObject> pose_dict;

    [DllImport("aruco_unity_dll", EntryPoint = "init")]
    private static extern int init(int dict_id, float marker_size);

    [DllImport("aruco_unity_dll", EntryPoint = "detect_markers")]
    private static extern int lib_detect_markers(IntPtr unity_img, ref int marker_count, ref IntPtr out_ids, ref IntPtr out_corners, ref IntPtr out_rvecs, ref IntPtr out_tvecs);

    void Start()
    {
        WebCamDevice[] devices = WebCamTexture.devices;
        _webcam = new WebCamTexture(devices[webcamDeviceNumber].name, cam_width, cam_height);
        _webcam.Play();
        displayMesh.material.mainTexture = _webcam;

        float vfov = 2.0f * Mathf.Atan(0.5f * cam_height / fparams.y) * Mathf.Rad2Deg;
        float aspect = (cam_width * 1.0f) / cam_height;
        cam_world.fieldOfView = vfov;
        //cam_world.aspect = aspect;

        rawImg = new Color32[cam_height * cam_width];

        Debug.Log("Camera size:" + _webcam.width + "x" + _webcam.height);
        Debug.Log("Camera fov:" + vfov + ", Aspect:" + aspect);

        int read_ok = init(aruco_dict_id, marker_size);
        Debug.Log("Camera Return status:" + read_ok);
        if (read_ok != 0)
        {
            Debug.Log("Camera parameters imported");
        }
        else
        {
            Debug.Log("Failed reading camera parameters");
            Application.Quit();
        }

        pose_dict = new Dictionary<int, GameObject>();
        updateSize(vfov, aspect);
    }

    void Update()
    {
        if (_webcam.isPlaying)
        {
            _webcam.GetPixels32(rawImg);
            detect_markers(rawImg);
            update_pose();
            activate_objs();
        }

        if (Input.GetKey("escape"))
        {
            Application.Quit();
        }
    }

    private void updateSize(float vfov, float aspect)
    {
        float distance = Display.transform.localPosition.z;
        float frustrum_height = 2.0f * distance * Mathf.Tan(vfov * 0.5f * Mathf.Deg2Rad);
        float frustrum_width = frustrum_height * aspect;

        Display.transform.localScale = new Vector2(frustrum_width, frustrum_height);
    }

    private static void detect_markers(Color32[] _image)
    {
        GCHandle img_handle = GCHandle.Alloc(_image, GCHandleType.Pinned);

        marker_count = 0;
        IntPtr out_ids = IntPtr.Zero;
        IntPtr out_corners = IntPtr.Zero;
        IntPtr out_rvecs = IntPtr.Zero;
        IntPtr out_tvecs = IntPtr.Zero;

        lib_detect_markers(img_handle.AddrOfPinnedObject(), ref marker_count, ref out_ids, ref out_corners, ref out_rvecs, ref out_tvecs);
        img_handle.Free();
        //Debug.Log("Marker count: " + marker_count);
        if (marker_count > 0)
        {
            //Copy over data from plugin side to c# managed arrays
            ids = new int[marker_count];
            Marshal.Copy(out_ids, ids, 0, marker_count);

            corners = new float[marker_count * 8];
            Marshal.Copy(out_corners, corners, 0, marker_count * 8);

            rvecs = new double[marker_count * 3];
            Marshal.Copy(out_rvecs, rvecs, 0, marker_count * 3);

            tvecs = new double[marker_count * 3];
            Marshal.Copy(out_tvecs, tvecs, 0, marker_count * 3);
        }
        else
        {
            ids = null;
            corners = null;
            rvecs = null;
            tvecs = null;
        }
    }

    private void update_pose()
    {
        if (marker_count == 0) return;

        Vector3 rvec = new Vector3();
        for (int i = 0; i < marker_count; i++)
        {
            Vector3 pos = new Vector3();
            Quaternion rot;

            pos.Set((float)tvecs[i * 3], (float)tvecs[i * 3 + 1], (float)tvecs[i * 3 + 2]);
            rvec.Set((float)rvecs[i * 3], (float)rvecs[i * 3 + 1], (float)rvecs[i * 3 + 2]);

            // Transform to unity co-ordinate system 
            pos.y = -pos.y;
            rvec.x = -rvec.x;
            rvec.z = -rvec.z;

            float theta = rvec.magnitude;
            rvec.Normalize();
            rot = Quaternion.AngleAxis(theta * Mathf.Rad2Deg, rvec);

            // Correct pose for non-centre cx and cy
            // Seems correct but not sure
            Vector3 imageCenter = new Vector3(0.5f, 0.5f, pos.z); // in viewport coordinates
            // cy in opencv format
            //Vector3 opticalCenter = new Vector3(cparams.x / cam_width, cparams.y / cam_height, pos.z);
            Vector3 opticalCenter = new Vector3(cparams.x / (1.0f * cam_width), 1 - (cparams.y / (1.0f * cam_height)), pos.z);
            Vector3 offset = new Vector3();
            offset = cam_world.ViewportToWorldPoint(imageCenter) - cam_world.ViewportToWorldPoint(opticalCenter);
            pos += offset;

            if (!pose_dict.ContainsKey(ids[i]))
                pose_dict[ids[i]] = makeMarkerObject();
            pose_dict[ids[i]].transform.localPosition = pos;
            pose_dict[ids[i]].transform.localRotation = rot;
        }
    }

    private GameObject makeMarkerObject()
    {
        GameObject quad = GameObject.Instantiate(markerObjectPrefab);
        quad.transform.localScale = Vector3.Scale(quad.transform.localScale, new Vector3(marker_size, marker_size, marker_size));
        quad.transform.parent = cam_world.transform;
        return quad;
    }

    private void activate_objs()
    {
        for (int i = 0; i < dict_size; i++)
        {
            if (marker_count > 0 && ids.Contains(i))
                pose_dict[i].SetActive(true);
            else
            {
                if (pose_dict.ContainsKey(i))
                {
                    pose_dict[i].SetActive(false);
                }
            }
        }
    }
}