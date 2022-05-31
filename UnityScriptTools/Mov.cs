using MOV.Path;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class Mov
{
    public class Angle {

        //角度标准化到[-180,180]区间内
        public static float NormalAngle(float angle)
        {
            return angle > 180 ? (angle % 360f - 360f) : (angle < -180f ? (angle % 360f + 360f) : angle);
        }
    }

    public static class Vector {

        public static Vector2 NormLeft90(Vector2 normal)
        {
            return new Vector2(-normal.y, normal.x);
        }
        public static Vector2 NormRight90(Vector2 normal)
        {
            return new Vector2(normal.y, -normal.x);
        }

        /// <summary>
        /// limt.x < 0 && limt.y >=0,则从与y == 0 开始，否则则从limt.x开始
        /// </summary>
        /// <param name="limt">限制范围</param>
        /// <param name="t">弧度</param>
        /// <returns></returns>
        public static float SinLimt(Vector2 limt, float t)
        {

            if (limt.x < 0 && limt.y >= 0)
            {
                float c = limt.y - limt.x;
                float f = (-limt.x * 2) / c - 1;
                return (Mathf.Sin(t + Mathf.Asin(f)) + 1f) / 2f * c + limt.x;
            }
            else
            {
                return (Mathf.Sin(t - 1.590f) + 1f) / 2f * (limt.y - limt.x) + limt.x;
            }
        }

        /// <summary>
        /// 输入方向逆时针旋转 a 弧度 
        /// </summary>
        /// <param name="v">方向</param>
        /// <param name="a">弧度</param>
        /// <returns>逆时针旋转后的方向,保留原大小</returns>
        public static Vector2 Rotate2(Vector2 v, float a)
        {
            Vector2 value = v;
            value.x = v.x * Mathf.Cos(a) - v.y * Mathf.Sin(a);
            value.y = v.y * Mathf.Cos(a) + v.x * Mathf.Sin(a);
            return value;
        }

        /// <summary>
        /// 输入矢量逆时针旋转 a 弧度
        /// </summary>
        /// <param name="v">方向</param>
        /// <param name="a">弧度</param>
        /// <returns>逆时针旋转后的方向,保留原大小</returns>
        public static Vector2 Rotate(Vector2 v, float a)
        {
            Vector2 n = v.normalized;
            a += Mathf.Atan2(n.y, n.x);
            return new Vector2(Mathf.Cos(a), Mathf.Sin(a)) * v.magnitude;
        }

        public static bool IsRight(Vector2 a, Vector2 b, Vector2 c)
        {

            Vector2 bc = (c - b).normalized;
            Vector2 ab = (b - a).normalized;
            Vector2 aTb = new Vector2(-ab.y, ab.x);

            return Vector2.Dot(bc, aTb) < 0;
        }
        public static float IsRightV(Vector2 a, Vector2 b, Vector2 c)
        {

            Vector2 bc = (c - b).normalized;
            Vector2 ab = (b - a).normalized;
            Vector2 aTb = new Vector2(-ab.y, ab.x);

            return Vector2.Dot(bc, aTb);
        }

    }

    public static class Geometry {


        public static float PointToLineDistance(Vector2 line, Vector2 point)
        {
            return (point - Vector2.Dot(point, line) * line).magnitude;
        }

        /// <summary>
        /// 点到直线的交点
        /// </summary>
        /// <param name="line">直线方向</param>
        /// <param name="p">点的世界位置</param>
        /// <param name="pos">直线的位置</param>
        /// <returns></returns>
        public static Vector2 PointToLinePos(Vector2 lineIntZeroDir, Vector2 point, Vector2 LinePos)
        {
            return Vector2.Dot(point - LinePos, lineIntZeroDir) * lineIntZeroDir + LinePos;
        }
        public static Vector2 PointToSegmentPos(Vector2 line, float len, Vector2 p, Vector2 pos)
        {
            Vector2 t = PointToLinePos(line, p, pos) - pos;
            float len0 = t.magnitude * (Vector2.Dot(t.normalized, line) > 0 ? 1 : -1);
            bool result = (len0 - len <= 0) && (len0 >= 0);

            if (result)
            {
                return t + pos;
            }
            else
            {
                if (len0 >= 0)
                {
                    return pos + line * len;
                }
                else
                {
                    return pos;
                }
            }
        }
        public static Vector2 PointToSegmentPos(Vector2 line, float len, float radius, Vector2 p, Vector2 pos)
        {
            Vector2 t = PointToLinePos(line, p, pos) - pos;
            float len0 = t.magnitude * (Vector2.Dot(t.normalized, line) > 0 ? 1 : -1);
            bool result = (len0 - len <= 0) && (len0 >= 0);

            if (result)
            {
                return (p - t - pos).normalized * radius + t + pos;
            }
            else
            {
                if (len0 >= 0)
                {
                    Vector2 d = pos + line * len;
                    Vector2 dir = (p - d).normalized * radius;
                    return dir + d;
                }
                else
                {
                    Vector2 dir = (p - pos).normalized * radius;
                    return dir + pos;
                }
            }
        }
        public static float PointToSegmentDis(Vector2 line, float len, Vector2 p, Vector2 pos)
        {
            return Vector2.Distance(PointToSegmentPos(line, len, p, pos), p);
        }

        public static bool SegmentIntersection(Vector2 seg0l, Vector2 seg0r, Vector2 seg1l, Vector2 seg1r)
        {
            Vector2 p = seg0l;
            Vector2 r = seg0r - seg0l;
            Vector2 q = seg1l;
            Vector2 s = seg1r - seg1l;
            Vector2 pq = q - p;
            float rxs = r.x * s.y - r.y * s.x;
            float pqxr = pq.x * r.y - pq.y * r.x;
            if (Mathf.Abs(rxs) < 0.0001f)
            {
                if (Mathf.Abs(pqxr) < 0.0001f)
                {
                    return true;
                }
                return false;
            }
            float pqxs = pq.x * s.y - pq.y * s.x;
            float t = pqxs / rxs;
            float u = pqxr / rxs;
            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }

        //忽略处理 忽略相交等特殊情况
        public static bool PointInContour(Vector2 p, List<Vector2> ps)
        {

            int count = 0;
            Vector2 r = new Vector2(100, 1);
            for (int i = 0; i < ps.Count; i++)
            {
                Vector2 a = ps[i];
                Vector2 b = ps[(i + 1) % ps.Count];
                Debug.DrawLine(a, b);
                if (SegmentIntersection(p, p + r, a, b))
                {
                    count++;
                }
            }
            return count % 2 == 1;
        }
        public static float PointInContourDis(Vector2 p, List<Vector2> ps)
        {
            float result = float.MaxValue;
            int count = 0;
            Vector2 r = new Vector2(100, 1);
            for (int i = 0; i < ps.Count; i++)
            {
                Vector2 a = ps[i];
                Vector2 b = ps[(i + 1) % ps.Count];
                Debug.DrawLine(a, b);
                if (SegmentIntersection(p, p + r, a, b))
                {
                    count++;
                }
                float d = Vector2.Distance(p, a);
                if (d < result)
                {
                    result = d;
                }
            }

            return result * (count % 2 == 1 ? 1 : -1);
        }

        /// <summary>
        /// 从 Right 方向开始，根据 rad 逆时针获取单位圆的边缘位置
        /// </summary>
        /// <param name="rad">弧度</param>
        /// <param name="circleRatio">圆 宽高比</param>
        /// <returns></returns>
        public static Vector3 GetCirclePosRad(float rad, Vector2 circleRatio)
        {
            return new Vector3(Mathf.Cos(rad) * circleRatio.x, Mathf.Sin(rad) * circleRatio.y, 0);
        }
        public static Vector3 GetCirclePosRadUp(float rad, Vector2 circleRatio)
        {
            return new Vector3(Mathf.Cos(rad) * circleRatio.x, Mathf.Max(0, Mathf.Sin(rad)) * circleRatio.y, 0);
        }

        public static float LineStepDistance(Vector2Int a, Vector2Int b)
        {
            return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y);
        }

        public static void SmoothCurve(AnimationCurve curve)
        {

            for (int i = 0; i < curve.keys.Length; i++)
            {
                curve.SmoothTangents(i, 0);
            }
        }

        public static float PathDistance(List<Vector2> path)
        {

            float distance = 0;
            for (int i = 0; i < path.Count - 1; i++)
            {
                distance += Vector2.Distance(path[i], path[i + 1]);
            }
            return distance;
        }
        /// <summary>
        /// 在最大长度内的路径,
        /// </summary>
        public static List<Vector2> GetInRangePath(List<Vector2> path, float maxDistance)
        {
            float distance = 0;
            int i = 0;
            for (; i < path.Count - 1; i++)
            {
                float space = Vector2.Distance(path[i], path[i + 1]);
                if (distance + space > maxDistance)
                {
                    break;
                }
                else
                {
                    distance += space;
                }
            }
            for (int n = path.Count - 1; n > i; n--)
            {
                path.RemoveAt(n);
            }
            if (path.Count < 2)
            {
                return null;
            }
            else
            {
                return path;
            }
        }

        public static float[] GetCurvePointArray(float values_, int l_, int c_, int r_)
        {
            int count = r_ - l_ + 1;
            float[] values = new float[count];

            r_ -= l_;
            c_ -= l_;
            l_ = 0;

            float lc = c_;
            float rc = r_ - c_;

            for (int i = 0; i <= r_; i++)
            {
                float x = 0f;
                if (i <= c_)
                {
                    x = Mathf.Lerp(0.1f, 0.5f, (float)(i + 1f) / (lc + 1f));
                }
                else
                {
                    x = Mathf.Lerp(0.5f, 0.9f, (float)(i - c_ + 1f) / (rc + 1f));
                }

                values[i] = values_ * ((-Mathf.Cos(x * 2f * Mathf.PI) + 1f) / 2f);
            }
            return values;
        }
        public static float[] SmoothPointArray(float[] values_, int l_, int c_, int r_)
        {
            l_ = Mathf.Max(0, l_);
            r_ = Mathf.Min(values_.Length - 1, r_);

            float[] result = new float[values_.Length];
            Array.Copy(values_, result, values_.Length);

            int s = l_;
            int count = r_ - l_ + 1;
            float[] values = new float[count];
            for (int i = l_, j = 0; i <= r_; i++, j++)
            {
                values[j] = result[i];
            }
            r_ -= l_;
            c_ -= l_;
            l_ = 0;

            float lc = c_;
            float rc = r_ - c_;

            //Debug.Log(l_ + " " + c_+" "+r_+" "+lc+" "+rc);
            for (int i = 0; i <= r_; i++)
            {
                float x = 0f;
                if (i <= c_)
                {
                    x = Mathf.Lerp(0, 0.5f, (float)(i) / lc);
                }
                else
                {
                    x = Mathf.Lerp(0.5f, 1f, (float)(i - c_) / rc);
                }
                //Debug.Log(i + " " + x);

                float l = i == 0 ? values[i] : (values[i] + values[i - 1]) / 2f;
                float r = i == r_ ? values[i] : (values[i] + values[i + 1]) / 2f;
                float e = (l + r) / 2f;
                float add = e - result[i + s];
                result[i + s] = result[i + s] + add * ((Mathf.Cos(x * 2f * Mathf.PI) + 1f) / 2f);
            }
            return result;
        }

        public static void SmoothPointArray(float[] values_)
        {

            int count = values_.Length - 1;
            float[] values = new float[count + 1];
            for (int i = 0; i <= count; i++)
            {
                values[i] = values_[i];

            }
            for (int i = 0; i < values.Length; i++)
            {
                float l = i == 0 ? values[i] : (values[i] + values[i - 1]) / 2f;
                float r = i == count ? values[i] : (values[i] + values[i + 1]) / 2f;

                values_[i] = Mathf.Min(1f, Mathf.Max((l + r) / 2f, 0.2f));
            }
        }

        public static List<Vector2> SmoothContour(List<Vector2> points, int smCount = 2)
        {

            List<Vector2> result = new List<Vector2>(points);
            Vector2[] offs = new Vector2[points.Count];
            for (int sm = 0; sm < smCount; sm++)
            {
                if (result.Count < 5) { continue; }

                for (int j = 0; j < result.Count; j++)
                {
                    int l = (j - 1) < 0 ? result.Count + (j - 1) : j - 1;
                    int r = (j + 1) % result.Count;
                    Vector2 v = (result[l] + result[r]) / 2f;
                    result[j] = Vector2.Lerp(result[j], v, 0.5f);
                    offs[j] = result[j] - v;
                }

                //防止过小距离出现
                for (int n = 1; n < result.Count - 1; n++)
                {
                    int index0 = n - 1;
                    int index1 = n;
                    int index2 = n + 1;

                    Vector2 dir0 = result[index1] - result[index0];
                    Vector2 dir1 = result[index1] - result[index2];

                    float dist0 = dir0.magnitude;
                    float dist1 = dir1.magnitude;

                    if (dist0 * 2 < dist1)
                    {
                        result[index1] = dir1.normalized * dist1 * 0.5f + result[index2];
                    }
                    else if (dist1 * 2 < dist0)
                    {
                        result[index1] = dir0.normalized * dist0 * 0.5f + result[index0];
                    }
                }
            }
            return result;
        }

        public static Vector2 Bezier(Vector2 a, Vector2 a1, Vector2 b, Vector2 b1, float t)
        {

            Vector2 p = Mathf.Pow(1 - t, 3) * a;
            p += 3 * t * Mathf.Pow(1 - t, 2) * a1;
            p += 3 * t * t * (1 - t) * b1;
            p += Mathf.Pow(t, 3) * b;

            return p;
        }

        public static Vector2[] CheckLoop(Vector2[] temp, float preci = 0.001f)
        {
            if ((temp[0] - temp[temp.Length - 1]).magnitude > preci)
            {
                Vector2[] temp2 = new Vector2[temp.Length + 1];
                Array.Copy(temp, temp2, temp.Length);
                temp2[temp2.Length - 1] = temp[0];
                return temp2;
            }
            return temp;
        }

        public static float GetAngle(Vector2 a)
        {
            return Vector2.SignedAngle(a.normalized, Vector2.right) * -1;
        }


        public static Bounds Bounds(Transform target, Bounds bounds)
        {

            float rad = -target.eulerAngles.z * Mathf.Deg2Rad;
            Vector2 ur = Vector.Rotate(bounds.max - target.position, rad);
            Vector2 dl = Vector.Rotate(bounds.min - target.position, rad);
            Vector2 center = Vector.Rotate(bounds.center - target.position, rad);
            Vector2 size = ur - dl;
            return new Bounds(center, size);
        }

        public static float ClampAngle(Vector2 from, Vector2 target)
        {
            float angle = Vector2.SignedAngle(from, target);
            return angle >= 0 ? angle : 360 + angle;
        }

    }

    public static class DebugGizmos {

        public static void DebugDrawSphere(Vector2 pos, float radius, int bian = 16)
        {
            bian = Mathf.Max(3, bian);
            for (float i = 0; i < bian; i++)
            {
                Vector2 a = Vector.Rotate(Vector2.up, (i / bian) * Mathf.PI * 2f) * radius + pos;
                Vector2 b = Vector.Rotate(Vector2.up, ((i + 1) % bian / bian) * Mathf.PI * 2f) * radius + pos;
                Debug.DrawLine(a, b);
            }
        }

    }
    
    public static class Gizmes {

        public static void DrawArrow(Vector2 a, Vector2 b)
        {
            Vector2 dir = (a - b).normalized;
            float l = (a - b).magnitude * 0.2f;
            Vector2 u = Vector.Rotate(dir, 30 * Mathf.Deg2Rad) * l;
            Vector2 d = Vector.Rotate(dir, -30 * Mathf.Deg2Rad) * l;

            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, b + u);
            Gizmos.DrawLine(b, b + d);
        }

        public static void DrawArrow(Vector3 a, Vector3 b)
        {
            Vector2 dir = (a - b).normalized;
            float l = (a - b).magnitude * 0.2f;
            Vector3 u = b.V2() + Vector.Rotate(dir, 30 * Mathf.Deg2Rad) * l;
            Vector3 d = b.V2() + Vector.Rotate(dir, -30 * Mathf.Deg2Rad) * l;

            u.z = Mathf.Lerp(b.z, a.z, 0.2f);
            d.z = Mathf.Lerp(b.z, a.z, 0.2f);


            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, u);
            Gizmos.DrawLine(b, d);
        }

        public static void DrawRect(Vector2 pos, Vector2 size)
        {

            Vector2 half = size / 2f;
            Vector2 a = pos - half;
            Vector2 b = pos + new Vector2(half.x, -half.y);
            Vector2 c = pos + half;
            Vector2 d = pos + new Vector2(-half.x, half.y);

            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, d);
            Gizmos.DrawLine(d, a);
        }
        public static void DrawCirce(Vector2 pos, float radius, int bian = 16)
        {
            bian = Mathf.Max(3, bian);
            for (float i = 0; i < bian; i++)
            {
                Vector2 a = Vector.Rotate(Vector2.up, (i / bian) * Mathf.PI * 2f) * radius + pos;
                Vector2 b = Vector.Rotate(Vector2.up, ((i + 1) % bian / bian) * Mathf.PI * 2f) * radius + pos;
                Gizmos.DrawLine(a, b);
            }
        }
        public static void DrawPath(IEnumerable<Vector2> path, bool loop = false)
        {
            var arr = path.ToArray();

            for (int i = 0; i < arr.Length - 1; i++)
            {
                Gizmos.DrawLine(arr[i], arr[i + 1]);
            }
            if (loop && arr.Length > 1)
            {
                Gizmos.DrawLine(arr[0], arr[arr.Length - 1]);
            }
        }
        public static void DrawArrowPath(IEnumerable<Vector2> path, bool loop = false)
        {
            var arr = path.ToArray();

            for (int i = 0; i < arr.Length - 1; i++)
            {
                DrawArrow(arr[i], arr[i + 1]);
            }
            if (loop && arr.Length > 1)
            {
                DrawArrow(arr[0], arr[arr.Length - 1]);
            }
        }

        public static void DrawCircleLimt(Vector2 anchor, Vector2 right, Vector2 limt, float angleOff, float len, Vector2 anchor2)
        {
            Vector2 dir = right * len;

            Vector2 a, b, c;
            a = anchor;
            b = a + Vector.Rotate(dir, -(limt.x + angleOff) * Mathf.Deg2Rad);
            c = a + Vector.Rotate(dir, -(limt.y + angleOff) * Mathf.Deg2Rad);

            Color green = new Color(0, 1, 1, .3f);
            Color blue = new Color(0, 0, 1, .3f);
            Color white = new Color(1, 1, 1, .3f);
            Color black = new Color(0, 0, 0, .3f);
            Gizmos.color = green;
            Gizmos.DrawLine(a, b);
            Gizmos.color = blue;
            Gizmos.DrawLine(a, c);

            Vector2 s = b - a;
            float rad = (limt.y - limt.x) * Mathf.Deg2Rad;
            for (float i = 0; i < 1f; i += 0.1f)
            {
                Gizmos.color = Color.Lerp(green, black, i);
                Gizmos.DrawLine(a + Vector.Rotate(s, -i * rad), a + Vector.Rotate(s, -(i + 0.1f) * rad));
            }
            if (anchor != anchor2)
            {
                Gizmos.color = white;
                Gizmos.DrawLine(a, a + (a - anchor2).normalized * len * 1.05f);
            }
        }

        public static void DrawXArrow(Vector2 a, Vector2 b)
        {
            Vector2 dir = (a - b).normalized;
            float l = (a - b).magnitude * 0.2f;
            Vector2 u = Vector.Rotate(dir, 30 * Mathf.Deg2Rad) * l;
            Vector2 d = Vector.Rotate(dir, -30 * Mathf.Deg2Rad) * l;

            DrawXLine(a, b);
            DrawXLine(b, b + u);
            DrawXLine(b, b + d);
        }

        public static void DrawXLine(Vector2 a, Vector2 b)
        {

            int c = (int)(Vector2.Distance(a, b) / 0.01f);

            for (int i = 0; i < c; i += 2)
            {
                float t = (float)i / c;
                float t2 = (float)(i + 1) / c;

                Gizmos.DrawLine(Vector2.Lerp(a, b, t), Vector2.Lerp(a, b, t2));
            }
        }

    }

    public static class Curve
    {
        public static Vector2[] GetSmoothPath(Vector2[] path, int smoothLevel = 5)
        {
            if (path.Length < 2) return path;

            SmoothPath smoothPath = new SmoothPath();
            int SmoothAmount = path.Length * smoothLevel;


            SmoothPath.Waypoint[] waypoints = new SmoothPath.Waypoint[path.Length];
            for (int i = 0; i < waypoints.Length; i++) waypoints[i] = new SmoothPath.Waypoint() { position = path[i] };
            smoothPath.m_Looped = path[0] == path[path.Length - 1];
            smoothPath.m_Resolution = smoothLevel;
            smoothPath.m_Waypoints = waypoints;
            smoothPath.InvalidateDistanceCache();

            Vector2[] result = new Vector2[SmoothAmount];
            int count = SmoothAmount - 1;
            for (int i = 0; i < count; i++)
            {
                float v = (float)i / SmoothAmount;
                result[i] = smoothPath.EvaluatePositionAtUnit(v, PathBase.PositionUnits.Normalized);
            }
            result[count] = smoothPath.EvaluatePositionAtUnit(1, PathBase.PositionUnits.Normalized);

            return result;
        }
    }
}
