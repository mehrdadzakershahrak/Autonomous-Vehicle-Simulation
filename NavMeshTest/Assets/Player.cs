using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Player : MonoBehaviour
{

    public Renderer rend;
    public GameObject target;
    NavMeshAgent agent;
    Mesh mesh;

    // Use this for initialization
    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        rend = GetComponent<Renderer>();
        Debug.Log("Hello World!");

        //NavMesh.pathfindingIterationsPerFrame = 100000000;
        var t = NavMesh.CalculateTriangulation();
        Debug.Log("# of vertices:" + t.vertices.Length);
        Debug.Log("# of triangles:" + t.indices.Length / 3);
        //mesh = new Mesh();

        //mesh.vertices = t.vertices;
        //mesh.triangles = t.indices;
        ////mesh.normals = t.normals;
        //Vector3[] normals = new Vector3[t.vertices.Length];
        //for (int i = 0; i < t.vertices.Length; i++)
        //{
        //    normals[i] = Vector3.up;
        //}
        //mesh.RecalculateNormals();
        //mesh.RecalculateBounds();
    }

    private void OnDrawGizmos()
    {
        var t = NavMesh.CalculateTriangulation();
        var meshs = new Mesh[t.indices.Length / 3];
        for (int j = 0; j < t.indices.Length / 3; j++)
        {
            int[] v = new int[] { t.indices[j * 3 + 0], t.indices[j * 3 + 1], t.indices[j * 3 + 2] };
            meshs[j] = new Mesh();
            meshs[j].vertices = new Vector3[] { t.vertices[v[0]], t.vertices[v[1]], t.vertices[v[2]] };
            meshs[j].triangles = new int[] { 0, 1, 2 };
            //meshs[j].uv = new Vector2[] { new Vector2(0, 0), new Vector2(0, 1), new Vector2(1, 1) };
            meshs[j].normals = new Vector3[] { Vector3.up, Vector3.up, Vector3.up };
            meshs[j].RecalculateNormals();
            meshs[j].RecalculateBounds();
            if (j % 6 == 0)
            {
                Gizmos.color = Color.cyan;
            } else if (j % 6 == 1)
            {
                Gizmos.color = Color.blue;
            }
            else if (j % 6 == 2)
            {
                Gizmos.color = Color.yellow;
            } else if (j % 6 == 3)
            {
                Gizmos.color = Color.white;
            } else if (j % 6 == 4)
            {
                Gizmos.color = Color.black;
            } else
            {
                Gizmos.color = Color.magenta;
            }      
            Gizmos.DrawMesh(meshs[j]);
        }
    }

    // Update is called once per frame
    void Update()
    {
        agent.destination = target.transform.position;
    }

}
    