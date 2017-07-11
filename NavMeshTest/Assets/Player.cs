using System;
using System.IO;
//using System.Runtime.Serialization;
//using System.Runtime.Serialization.Json;
//using System.Xml.Serialization;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
//using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.AI;
using Newtonsoft.Json;

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
        

        Debug.Log("Converting the triangulated data to Json format...");
        //var serializer = new DataContractJsonSerializer(typeof(TriangulationToJson));
        //MemoryStream ms = new MemoryStream();
        //var vlist = new List<Vertex>();
        //for (int vid = 0 ; vid < t.vertices.Length ; vid++)
        //{
        //    vlist.Add(new Vertex(vid, t.vertices[vid].x, t.vertices[vid].y, t.vertices[vid].z));
        //}
	var triangulation = new TriangulationToJson();
	for (int vid = 0 ; vid < t.vertices.Length ; vid++) {
	    triangulation.vertices.Add(vid, new List<float>(new float[] {t.vertices[vid].x, t.vertices[vid].y, t.vertices[vid].z}));
	}
	for (int tid = 0 ; tid < t.indices.Length / 3 ; tid++)
	{
	    triangulation.triangles.Add(tid, new List<int>(new int[] {t.indices[tid * 3 + 0], t.indices[tid * 3 + 1], t.indices[tid * 3 + 2]}));
	}

	string json = JsonConvert.SerializeObject(triangulation);

        //var tlist = new List<Triangle>();
        //for (int tid = 0 ; tid < t.indices.Length / 3 ; tid++)
        //{
        //    tlist.Add(new Triangle(tid, t.indices[tid * 3 + 0], t.indices[tid * 3 + 1], t.indices[tid * 3 + 2]));
        //}
        //var triangulationToJson = new TriangulationToJson(vlist, tlist);
        //serializer.WriteObject(ms, triangulationToJson);
        //ms.Position = 0;
        //var clone = serializer.ReadObject(ms) as TriangulationToJson;

        Debug.Log("Writing the Json data to a file...");
        using (FileStream fs = new FileStream("triangulation.json", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None))
        using (StreamWriter sw = new StreamWriter(fs, Encoding.UTF8))
        {
            //sw.WriteLine(clone);
	    sw.WriteLine(json);
        }
        Debug.Log("Writing Done.");
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

/*
[DataContract]
public class Vertex
{
    [DataMember]
    public float x;
    [DataMember]
    public float y;
    [DataMember]
    public float z;
    [DataMember]
    public int vid;

    public Vertex(int vid, float x, float y, float z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.vid = vid;
    }

    public override string ToString()
    {
        return string.Format("'{0}': [{1}, {2}, {3}]", this.vid, this.x, this.y, this.z);
    }
}

[DataContract]
public class Vertices
{
    [DataMember]
    public List<Vertex> vertexList;

    public Vertices(List<Vertex> vertex_list)
    {
        this.vertexList = new List<Vertex>();
        foreach (var vertex in vertex_list)
        {
            this.vertexList.Add(vertex);
        }
    }

    public override string ToString()
    {
        var result = "{" + string.Join(", ", (object[])this.vertexList.ToArray()) + "}";
        return result;
        //return "[" + string.Join(", ", this.vertexList.ToArray()) + "]";
    }
}

[DataContract]
public class Triangle
{
    [DataMember]
    public int tid;
    [DataMember]
    public int p1;
    [DataMember]
    public int p2;
    [DataMember]
    public int p3;

    public Triangle(int tid, int p1, int p2, int p3)
    {
        this.tid = tid;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
    }

    public override string ToString()
    {
        return string.Format("'{0}': [{1}, {2}, {3}]", this.tid, this.p1, this.p2, this.p3);
    }
}

[DataContract]
public class Triangles
{
    [DataMember]
    public List<Triangle> triangleList;

    public Triangles(List<Triangle> triangle_list)
    {
        this.triangleList = new List<Triangle>();
        foreach (var triangle in triangle_list)
        {
            this.triangleList.Add(triangle);
        }
    }

    public override string ToString()
    {
        var result = "{" + string.Join(", ", (object[])this.triangleList.ToArray()) + "}";
        return result;
    }
}

[DataContract]
public class TriangulationToJson
{
    [DataMember]
    public Vertices vertices;
    [DataMember]
    public Triangles triangles;

    public TriangulationToJson(List<Vertex> vlist, List<Triangle> tlist)
    {
        this.vertices = new Vertices(vlist);
        this.triangles = new Triangles(tlist);
    }

    public override string ToString()
    {
        var res1 = "'vertices': " + string.Join(", ", this.vertices);
        var res2 = "'triangles': " + string.Join(", ", this.triangles);
        return "{" + res1 + ", " + res2 + "}";
    }
}
*/

public class TriangulationToJson
{
    public Dictionary<int, List<float>> vertices;
    public Dictionary<int, List<int>> triangles;
    public TriangulationToJson()
    {
        this.vertices = new Dictionary<int, List<float>>();
        this.triangles = new Dictionary<int, List<int>>();
    }
    //public override string ToString()
    //{
    //    StringBuilder sb = new StringBuilder();
    //    sb.Append("{'vertices': {");
    //    foreach (var pair in this.vertices)
    //    {
    //        //sb.AppendFormat("'{0}' : [{1}], ", pair.Key, pair.Value);
    //        sb.AppendFormat("'{0}' : ", pair.Key);
    //        sb.Append("[" + string.Join(", ", pair.Value.ToArray()) + "], ");
    //    }
    //    sb.Append("}, 'triangles': {");
    //    foreach (var pair in this.triangles)
    //    {
    //        //sb.AppendFormat("'{0}' : [{1}], ", pair.Key, pair.Value);
    //        sb.AppendFormat("'{0}' : ", pair.Key);
    //        sb.Append("[" + string.Join(", ", pair.Value.ToArray()) + "], ");
    //    }
    //    sb.Append("}}");
    //    return sb.ToString();
    //}
}
