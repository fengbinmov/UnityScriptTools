using System.Collections.Generic;
using UnityEngine;

public static class MovHelper
{
    public delegate T Converter<S,T>(S s);

    public static int Int(float v) => Mathf.RoundToInt(v);

    public static Vector2Int Int(this Vector2 vector2) { return new Vector2Int(Mathf.RoundToInt(vector2.x), Mathf.RoundToInt(vector2.y)); }
    public static Vector3Int Int(this Vector3 vector3) => new Vector3Int(Mathf.RoundToInt(vector3.x), Mathf.RoundToInt(vector3.y), Mathf.RoundToInt(vector3.z));

    public static Vector2 V2(this Vector3 vector3) => vector3;
    public static Vector2Int V2(this Vector3Int vector3) { return new Vector2Int(vector3.x, vector3.y); }

    public static Vector3 V3(this Vector2 vector, float z = 0) => new Vector3(vector.x, vector.y, z);
    public static Vector3 V3(this Vector2Int vector2, float z = 0) => new Vector3(vector2.x, vector2.y, z);

    public static T[] ArrConvert<S,T>(this ICollection<S> list, Converter<S,T> toFunct) {

        T[] arr = new T[list.Count];
        int i = 0;
        foreach (var item in list)
        {
            arr[i++] = toFunct(item);
        }
        return arr;
    }

    public static bool ContainsAbout(this List<Vector3> list, Vector3 vector, float preci = 0.01f)
    {
        return list.IndexOfAbout(vector, preci) > -1;
    }

    public static bool EqualAbout(this Vector2 a, Vector2 b, float preci = 0.01f)
    {
        return (a - b).magnitude < preci;
    }

    public static int IndexOfAbout(this IEnumerable<Vector3> list, Vector3 vector, float preci = 0.01f)
    {
        int i = -1;
        foreach (var item in list)
        {
            i++;
            if ((item - vector).magnitude < preci) return i;
        }
        return i;
    }
    public static int IndexOfAbout(this IEnumerable<Vector2> list, Vector2 vector, float preci = 0.01f)
    {
        int i = -1;
        foreach (var item in list)
        {
            i++;
            if ((item - vector).magnitude < preci) return i;
        }
        return i;
    }

    public static int ChildCountAll(this Transform trans, bool includeInactive = false)
    {
        return ChildCount(trans, includeInactive);
    }

    private static int ChildCount(Transform tran, bool includeInactive = false)
    {
        int count = tran.childCount;
        for (int i = 0; i < count; i++)
        {
            ChildCount(tran.GetChild(i), includeInactive);

            count += (includeInactive ? 1 : (tran.GetChild(i).gameObject.activeSelf ? 1 : 0));
        }
        return count;
    }

    public static bool TryGetComponent<T>(this GameObject obj,out T temp) {

        temp = obj.GetComponent<T>();
        return temp != null;
    }
    public static void Destroy<T>(this GameObject obj) where T : Component
    {
        T temp = null;
        if (obj.TryGetComponent<T>(out temp))
        {
            if (Application.isPlaying)
            {
                GameObject.Destroy(temp);
            }
            else
            {
                GameObject.DestroyImmediate(temp);
            }
        }
    }

    /// <summary>
    /// 获取组件，如未挂载则自动挂载
    /// </summary>
    public static T Component<T>(this GameObject obj) where T : Component
    {
        if (obj.GetComponent<T>())
        {
            return obj.GetComponent<T>();
        }
        else
        {
            return obj.AddComponent<T>();
        }
    }

    public static void Destroy(this GameObject obj)
    {
        obj.name = string.Empty;
        obj.SetActive(false);

        if (Application.isPlaying)
        {
            GameObject.Destroy(obj);
        }
        else
        {
            GameObject.DestroyImmediate(obj);
        }
    }

    public static string TryGetFileInfo(string filePath, out string directory, out string fileName, out string suffix)
    {

        fileName = "";
        directory = "";

        string[] ms = filePath.Replace('\\', '/').Split('/');
        for (int i = 0; i < ms.Length - 1; i++)
        {
            directory += ms[i] + "/";
        }
        string nameEnd = ms[ms.Length - 1];
        string[] ms2 = nameEnd.Split('.');

        for (int i = 0; i < ms2.Length - 1; i++)
        {
            fileName += ms2[i] + (i == ms2.Length - 2 ? "" : ".");
        }
        suffix = "." + ms2[ms2.Length - 1];

        return directory + " | " + fileName + " | " + suffix;
    }

    public static Vector2 MouseInRectPosition(this RectTransform rect)
    {
        return new Vector2(Input.mousePosition.x / Screen.width * rect.sizeDelta.x, Input.mousePosition.y / Screen.height * rect.sizeDelta.y);
    }

    public static int GetNearIndex(this List<Vector2Int> list, Vector2 vector)
    {

        float d = float.MaxValue;
        int index = -1;
        for (int i = 0; i < list.Count; i++)
        {
            float t = Vector2.Distance(list[i], vector);
            if (t < d)
            {
                d = t;
                index = i;
            }
        }
        return index;
    }

    public static float PathLength(this List<Vector2> path)
    {
        float d = 0;
        for (int i = 0; i < path.Count - 1; i++)
        {
            d += Vector2.Distance(path[i], path[i + 1]);
        }
        return d;
    }
}