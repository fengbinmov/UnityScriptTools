using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace MOV.Path
{
    [Serializable]
    public abstract class PathBase
    {
        public const float Epsilon = 0.0001F;

        public Transform transform;

        /// <summary>Path samples per waypoint</summary>
        [Tooltip("Path samples per waypoint.  This is used for calculating path distances.")]
        [Range(1, 100)]
        public int m_Resolution = 10;

        /// <summary>The minimum value for the path position</summary>
        public abstract float MinPos { get; }

        /// <summary>The maximum value for the path position</summary>
        public abstract float MaxPos { get; }

        /// <summary>True if the path ends are joined to form a continuous loop</summary>
        public abstract bool Looped { get; }

        /// <summary>Get a standardized path position, taking spins into account if looped</summary>
        /// <param name="pos">Position along the path</param>
        /// <returns>Standardized position, between MinPos and MaxPos</returns>
        public virtual float StandardizePos(float pos)
        {
            if (Looped && MaxPos > 0)
            {
                pos = pos % MaxPos;
                if (pos < 0)
                    pos += MaxPos;
                return pos;
            }
            return Mathf.Clamp(pos, 0, MaxPos);
        }

        /// <summary>Get a worldspace position of a point along the path</summary>
        /// <param name="pos">Postion along the path.  Need not be standardized.</param>
        /// <returns>World-space position of the point along at path at pos</returns>
        public abstract Vector3 EvaluatePosition(float pos);

        /// <summary>Get the tangent of the curve at a point along the path.</summary>
        /// <param name="pos">Postion along the path.  Need not be standardized.</param>
        /// <returns>World-space direction of the path tangent.
        /// Length of the vector represents the tangent strength</returns>
        public abstract Vector3 EvaluateTangent(float pos);

        /// <summary>Get the orientation the curve at a point along the path.</summary>
        /// <param name="pos">Postion along the path.  Need not be standardized.</param>
        /// <returns>World-space orientation of the path</returns>
        public abstract Quaternion EvaluateOrientation(float pos);

        /// <summary>Find the closest point on the path to a given worldspace target point.</summary>
        /// <remarks>Performance could be improved by checking the bounding polygon of each segment,
        /// and only entering the best segment(s)</remarks>
        /// <param name="p">Worldspace target that we want to approach</param>
        /// <param name="startSegment">In what segment of the path to start the search.
        /// A Segment is a section of path between 2 waypoints.</param>
        /// <param name="searchRadius">How many segments on either side of the startSegment
        /// to search.  -1 means no limit, i.e. search the entire path</param>
        /// <param name="stepsPerSegment">We search a segment by dividing it into this many
        /// straight pieces.  The higher the number, the more accurate the result, but performance
        /// is proportionally slower for higher numbers</param>
        /// <returns>The position along the path that is closest to the target point.
        /// The value is in Path Units, not Distance units.</returns>
        public virtual float FindClosestPoint(
            Vector3 p, int startSegment, int searchRadius, int stepsPerSegment)
        {
            float start = MinPos;
            float end = MaxPos;
            if (searchRadius >= 0)
            {
                int r = Mathf.FloorToInt(Mathf.Min(searchRadius, (end - start) / 2f));
                start = startSegment - r;
                end = startSegment + r + 1;
                if (!Looped)
                {
                    start = Mathf.Max(start, MinPos);
                    end = Mathf.Min(end, MaxPos);
                }
            }
            stepsPerSegment = Mathf.RoundToInt(Mathf.Clamp(stepsPerSegment, 1f, 100f));
            float stepSize = 1f / stepsPerSegment;
            float bestPos = startSegment;
            float bestDistance = float.MaxValue;
            int iterations = (stepsPerSegment == 1) ? 1 : 3;
            for (int i = 0; i < iterations; ++i)
            {
                Vector3 v0 = EvaluatePosition(start);
                for (float f = start + stepSize; f <= end; f += stepSize)
                {
                    Vector3 v = EvaluatePosition(f);
                    float t = ClosestPointOnSegment(p, v0, v);
                    float d = Vector3.SqrMagnitude(p - Vector3.Lerp(v0, v, t));
                    if (d < bestDistance)
                    {
                        bestDistance = d;
                        bestPos = f - (1 - t) * stepSize;
                    }
                    v0 = v;
                }
                start = bestPos - stepSize;
                end = bestPos + stepSize;
                stepSize /= stepsPerSegment;
            }
            return bestPos;
        }

        /// <summary>How to interpret the Path Position</summary>
        public enum PositionUnits
        {
            /// <summary>Use PathPosition units, where 0 is first waypoint, 1 is second waypoint, etc</summary>
            PathUnits,
            /// <summary>Use Distance Along Path.  Path will be sampled according to its Resolution
            /// setting, and a distance lookup table will be cached internally</summary>
            Distance,
            /// <summary>Normalized units, where 0 is the start of the path, and 1 is the end.
            /// Path will be sampled according to its Resolution
            /// setting, and a distance lookup table will be cached internally</summary>
            Normalized
        }

        /// <summary>Get the minimum value, for the given unit type</summary>
        /// <param name="units">The unit type</param>
        /// <returns>The minimum allowable value for this path</returns>
        public float MinUnit(PositionUnits units)
        {
            if (units == PositionUnits.Normalized)
                return 0;
            return units == PositionUnits.Distance ? 0 : MinPos;
        }

        /// <summary>Get the maximum value, for the given unit type</summary>
        /// <param name="units">The unit type</param>
        /// <returns>The maximum allowable value for this path</returns>
        public float MaxUnit(PositionUnits units)
        {
            if (units == PositionUnits.Normalized)
                return 1;
            return units == PositionUnits.Distance ? PathLength : MaxPos;
        }

        /// <summary>Standardize the unit, so that it lies between MinUmit and MaxUnit</summary>
        /// <param name="pos">The value to be standardized</param>
        /// <param name="units">The unit type</param>
        /// <returns>The standardized value of pos, between MinUnit and MaxUnit</returns>
        public virtual float StandardizeUnit(float pos, PositionUnits units)
        {
            if (units == PositionUnits.PathUnits)
                return StandardizePos(pos);
            if (units == PositionUnits.Distance)
                return StandardizePathDistance(pos);
            float len = PathLength;
            if (len < Epsilon)
                return 0;
            return StandardizePathDistance(pos * len) / len;
        }

        /// <summary>Get a worldspace position of a point along the path</summary>
        /// <param name="pos">Postion along the path.  Need not be normalized.</param>
        /// <param name="units">The unit to use when interpreting the value of pos.</param>
        /// <returns>World-space position of the point along at path at pos</returns>
        public Vector3 EvaluatePositionAtUnit(float pos, PositionUnits units)
        {
            return EvaluatePosition(ToNativePathUnits(pos, units));
        }

        /// <summary>Get the tangent of the curve at a point along the path.</summary>
        /// <param name="pos">Postion along the path.  Need not be normalized.</param>
        /// <param name="units">The unit to use when interpreting the value of pos.</param>
        /// <returns>World-space direction of the path tangent.
        /// Length of the vector represents the tangent strength</returns>
        public Vector3 EvaluateTangentAtUnit(float pos, PositionUnits units)
        {
            return EvaluateTangent(ToNativePathUnits(pos, units));
        }

        /// <summary>Get the orientation the curve at a point along the path.</summary>
        /// <param name="pos">Postion along the path.  Need not be normalized.</param>
        /// <param name="units">The unit to use when interpreting the value of pos.</param>
        /// <returns>World-space orientation of the path</returns>
        public Quaternion EvaluateOrientationAtUnit(float pos, PositionUnits units)
        {
            return EvaluateOrientation(ToNativePathUnits(pos, units));
        }

        /// <summary>When calculating the distance cache, sample the path this many
        /// times between points</summary>
        public abstract int DistanceCacheSampleStepsPerSegment { get; }

        /// <summary>Call this if the path changes in such a way as to affect distances
        /// or other cached path elements</summary>
        public virtual void InvalidateDistanceCache()
        {
            m_DistanceToPos = null;
            m_PosToDistance = null;
            m_CachedSampleSteps = 0;
            m_PathLength = 0;
        }

        /// <summary>See whether the distance cache is valid.  If it's not valid,
        /// then any call to GetPathLength() or ToNativePathUnits() will
        /// trigger a potentially costly regeneration of the path distance cache</summary>
        /// <returns>Whether the cache is valid</returns>
        public bool DistanceCacheIsValid()
        {
            return (MaxPos == MinPos)
                || (m_DistanceToPos != null && m_PosToDistance != null
                    && m_CachedSampleSteps == DistanceCacheSampleStepsPerSegment
                    && m_CachedSampleSteps > 0);
        }

        /// <summary>Get the length of the path in distance units.
        /// If the distance cache is not valid, then calling this will
        /// trigger a potentially costly regeneration of the path distance cache</summary>
        /// <returns>The length of the path in distance units, when sampled at this rate</returns>
        public float PathLength
        {
            get
            {
                if (DistanceCacheSampleStepsPerSegment < 1)
                    return 0;
                if (!DistanceCacheIsValid())
                    ResamplePath(DistanceCacheSampleStepsPerSegment);
                return m_PathLength;
            }
        }

        /// <summary>Standardize a distance along the path based on the path length.
        /// If the distance cache is not valid, then calling this will
        /// trigger a potentially costly regeneration of the path distance cache</summary>
        /// <param name="distance">The distance to standardize</param>
        /// <returns>The standardized distance, ranging from 0 to path length</returns>
        public float StandardizePathDistance(float distance)
        {
            float length = PathLength;
            if (length < Vector3.kEpsilon)
                return 0;
            if (Looped)
            {
                distance = distance % length;
                if (distance < 0)
                    distance += length;
            }
            return Mathf.Clamp(distance, 0, length);
        }

        /// <summary>Get the path position to native path units.
        /// If the distance cache is not valid, then calling this will
        /// trigger a potentially costly regeneration of the path distance cache</summary>
        /// <param name="pos">The value to convert from</param>
        /// <param name="units">The units in which pos is expressed</param>
        /// <returns>The path position, in native units</returns>
        public float ToNativePathUnits(float pos, PositionUnits units)
        {
            if (units == PositionUnits.PathUnits)
                return pos;
            if (DistanceCacheSampleStepsPerSegment < 1 || PathLength < Epsilon)
                return MinPos;
            if (units == PositionUnits.Normalized)
                pos *= PathLength;
            pos = StandardizePathDistance(pos);
            float d = pos / m_cachedDistanceStepSize;
            int i = Mathf.FloorToInt(d);
            if (i >= m_DistanceToPos.Length - 1)
                return MaxPos;
            float t = d - (float)i;
            return MinPos + Mathf.Lerp(m_DistanceToPos[i], m_DistanceToPos[i + 1], t);
        }

        /// <summary>Convert a path position from native path units to the desired units.
        /// If the distance cache is not valid, then calling this will
        /// trigger a potentially costly regeneration of the path distance cache</summary>
        /// <param name="pos">The value to convert from, in native units</param>
        /// <param name="units">The units to convert to</param>
        /// <returns>The path position, in the requested units</returns>
        public float FromPathNativeUnits(float pos, PositionUnits units)
        {
            if (units == PositionUnits.PathUnits)
                return pos;
            float length = PathLength;
            if (DistanceCacheSampleStepsPerSegment < 1 || length < Epsilon)
                return 0;
            pos = StandardizePos(pos);
            float d = pos / m_cachedPosStepSize;
            int i = Mathf.FloorToInt(d);
            if (i >= m_PosToDistance.Length - 1)
                pos = m_PathLength;
            else
            {
                float t = d - (float)i;
                pos = Mathf.Lerp(m_PosToDistance[i], m_PosToDistance[i + 1], t);
            }
            if (units == PositionUnits.Normalized)
                pos /= length;
            return pos;
        }

        private float[] m_DistanceToPos;
        private float[] m_PosToDistance;
        private int m_CachedSampleSteps;
        private float m_PathLength;
        private float m_cachedPosStepSize;
        private float m_cachedDistanceStepSize;

        private void ResamplePath(int stepsPerSegment)
        {
            InvalidateDistanceCache();

            float minPos = MinPos;
            float maxPos = MaxPos;
            float stepSize = 1f / Mathf.Max(1, stepsPerSegment);

            // Sample the positions
            int numKeys = Mathf.RoundToInt((maxPos - minPos) / stepSize) + 1;
            m_PosToDistance = new float[numKeys];
            m_CachedSampleSteps = stepsPerSegment;
            m_cachedPosStepSize = stepSize;

            Vector3 p0 = EvaluatePosition(0);
            m_PosToDistance[0] = 0;
            float pos = minPos;
            for (int i = 1; i < numKeys; ++i)
            {
                pos += stepSize;
                Vector3 p = EvaluatePosition(pos);
                float d = Vector3.Distance(p0, p);
                m_PathLength += d;
                p0 = p;
                m_PosToDistance[i] = m_PathLength;
            }

            // Resample the distances
            m_DistanceToPos = new float[numKeys];
            m_DistanceToPos[0] = 0;
            if (numKeys > 1)
            {
                stepSize = m_PathLength / (numKeys - 1);
                m_cachedDistanceStepSize = stepSize;
                float distance = 0;
                int posIndex = 1;
                for (int i = 1; i < numKeys; ++i)
                {
                    distance += stepSize;
                    float d = m_PosToDistance[posIndex];
                    while (d < distance && posIndex < numKeys - 1)
                        d = m_PosToDistance[++posIndex];
                    float d0 = m_PosToDistance[posIndex - 1];
                    float delta = d - d0;
                    float t = (distance - d0) / delta;
                    m_DistanceToPos[i] = m_cachedPosStepSize * (t + posIndex - 1);
                }
            }
        }

        /// <summary>
        /// Get the closest point on a line segment.
        /// </summary>
        /// <param name="p">A point in space</param>
        /// <param name="s0">Start of line segment</param>
        /// <param name="s1">End of line segment</param>
        /// <returns>The interpolation parameter representing the point on the segment, with 0==s0, and 1==s1</returns>
        protected float ClosestPointOnSegment(Vector3 p, Vector3 s0, Vector3 s1)
        {
            Vector3 s = s1 - s0;
            float len2 = Vector3.SqrMagnitude(s);
            if (len2 < Epsilon)
                return 0; // degenrate segment
            return Mathf.Clamp01(Vector3.Dot(p - s0, s) / len2);
        }

        protected void ComputeSmoothControlPoints(ref Vector4[] knot, ref Vector4[] ctrl1, ref Vector4[] ctrl2)
        {
            int numPoints = knot.Length;
            if (numPoints <= 2)
            {
                if (numPoints == 2)
                {
                    ctrl1[0] = Vector4.Lerp(knot[0], knot[1], 0.33333f);
                    ctrl2[0] = Vector4.Lerp(knot[0], knot[1], 0.66666f);
                }
                else if (numPoints == 1)
                    ctrl1[0] = ctrl2[0] = knot[0];
                return;
            }

            var a = new float[numPoints];
            var b = new float[numPoints];
            var c = new float[numPoints];
            var r = new float[numPoints];
            for (int axis = 0; axis < 4; ++axis)
            {
                int n = numPoints - 1;

                // Linear into the first segment
                a[0] = 0;
                b[0] = 2;
                c[0] = 1;
                r[0] = knot[0][axis] + 2 * knot[1][axis];

                // Internal segments
                for (int i = 1; i < n - 1; ++i)
                {
                    a[i] = 1;
                    b[i] = 4;
                    c[i] = 1;
                    r[i] = 4 * knot[i][axis] + 2 * knot[i + 1][axis];
                }

                // Linear out of the last segment
                a[n - 1] = 2;
                b[n - 1] = 7;
                c[n - 1] = 0;
                r[n - 1] = 8 * knot[n - 1][axis] + knot[n][axis];

                // Solve with Thomas algorithm
                for (int i = 1; i < n; ++i)
                {
                    float m = a[i] / b[i - 1];
                    b[i] = b[i] - m * c[i - 1];
                    r[i] = r[i] - m * r[i - 1];
                }

                // Compute ctrl1
                ctrl1[n - 1][axis] = r[n - 1] / b[n - 1];
                for (int i = n - 2; i >= 0; --i)
                    ctrl1[i][axis] = (r[i] - c[i] * ctrl1[i + 1][axis]) / b[i];

                // Compute ctrl2 from ctrl1
                for (int i = 0; i < n; i++)
                    ctrl2[i][axis] = 2 * knot[i + 1][axis] - ctrl1[i + 1][axis];
                ctrl2[n - 1][axis] = 0.5f * (knot[n][axis] + ctrl1[n - 1][axis]);
            }
        }

        protected void ComputeSmoothControlPointsLooped(ref Vector4[] knot, ref Vector4[] ctrl1, ref Vector4[] ctrl2)
        {
            int numPoints = knot.Length;
            if (numPoints < 2)
            {
                if (numPoints == 1)
                    ctrl1[0] = ctrl2[0] = knot[0];
                return;
            }

            int margin = Mathf.Min(4, numPoints - 1);
            Vector4[] knotLooped = new Vector4[numPoints + 2 * margin];
            Vector4[] ctrl1Looped = new Vector4[numPoints + 2 * margin];
            Vector4[] ctrl2Looped = new Vector4[numPoints + 2 * margin];
            for (int i = 0; i < margin; ++i)
            {
                knotLooped[i] = knot[numPoints - (margin - i)];
                knotLooped[numPoints + margin + i] = knot[i];
            }
            for (int i = 0; i < numPoints; ++i)
                knotLooped[i + margin] = knot[i];
            ComputeSmoothControlPoints(ref knotLooped, ref ctrl1Looped, ref ctrl2Looped);
            for (int i = 0; i < numPoints; ++i)
            {
                ctrl1[i] = ctrl1Looped[i + margin];
                ctrl2[i] = ctrl2Looped[i + margin];
            }
        }

        /// <summary>Is the vector within Epsilon of zero length?</summary>
        /// <param name="v"></param>
        /// <returns>True if the square magnitude of the vector is within Epsilon of zero</returns>
        protected bool AlmostZero(Vector3 v)
        {
            return v.sqrMagnitude < (Epsilon * Epsilon);
        }

        protected Vector3 Bezier3(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            t = Mathf.Clamp01(t);
            float d = 1f - t;
            return d * d * d * p0 + 3f * d * d * t * p1
                + 3f * d * t * t * p2 + t * t * t * p3;
        }

        protected float Bezier1(float t, float p0, float p1, float p2, float p3)
        {
            t = Mathf.Clamp01(t);
            float d = 1f - t;
            return d * d * d * p0 + 3f * d * d * t * p1
                + 3f * d * t * t * p2 + t * t * t * p3;
        }

        protected Vector3 BezierTangent3(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            t = Mathf.Clamp01(t);
            return (-3f * p0 + 9f * p1 - 9f * p2 + 3f * p3) * (t * t)
                + (6f * p0 - 12f * p1 + 6f * p2) * t
                - 3f * p0 + 3f * p1;
        }

    }

    [Serializable]
    public class SmoothPath : PathBase
    {

        /// <summary>If checked, then the path ends are joined to form a continuous loop</summary>
        [Tooltip("If checked, then the path ends are joined to form a continuous loop.")]
        public bool m_Looped;

        /// <summary>A waypoint along the path</summary>
        [Serializable]
        public struct Waypoint
        {
            /// <summary>Position in path-local space</summary>
            [Tooltip("Position in path-local space")]
            public Vector3 position;

            /// <summary>Defines the roll of the path at this waypoint.  
            /// The other orientation axes are inferred from the tangent and world up.</summary>
            [Tooltip("Defines the roll of the path at this waypoint.  The other orientation axes are inferred from the tangent and world up.")]
            public float roll;

            /// <summary>Representation as Vector4</summary>
            internal Vector4 AsVector4
            {
                get { return new Vector4(position.x, position.y, position.z, roll); }
            }

            internal static Waypoint FromVector4(Vector4 v)
            {
                Waypoint wp = new Waypoint();
                wp.position = new Vector3(v[0], v[1], v[2]);
                wp.roll = v[3];
                return wp;
            }
        }

        /// <summary>The waypoints that define the path.
        /// They will be interpolated using a bezier curve</summary>
        [Tooltip("The waypoints that define the path.  They will be interpolated using a bezier curve.")]
        public Waypoint[] m_Waypoints = new Waypoint[0];

        /// <summary>The minimum value for the path position</summary>
        public override float MinPos { get { return 0; } }

        /// <summary>The maximum value for the path position</summary>
        public override float MaxPos
        {
            get
            {
                int count = m_Waypoints.Length - 1;
                if (count < 1)
                    return 0;
                return m_Looped ? count + 1 : count;
            }
        }
        /// <summary>True if the path ends are joined to form a continuous loop</summary>
        public override bool Looped { get { return m_Looped; } }

        /// <summary>When calculating the distance cache, sample the path this many 
        /// times between points</summary>
        public override int DistanceCacheSampleStepsPerSegment { get { return m_Resolution; } }

        private void Reset()
        {
            m_Looped = false;
            m_Waypoints = new Waypoint[2]
            {
                new Waypoint { position = new Vector3(0, 0, -5) },
                new Waypoint { position = new Vector3(0, 0, 5) }
            };

            InvalidateDistanceCache();
        }

        /// <summary>Call this if the path changes in such a way as to affect distances
        /// or other cached path elements</summary>
        public override void InvalidateDistanceCache()
        {
            base.InvalidateDistanceCache();
            m_ControlPoints1 = null;
            m_ControlPoints2 = null;
        }

        Waypoint[] m_ControlPoints1;
        Waypoint[] m_ControlPoints2;
        bool m_IsLoopedCache;

        void UpdateControlPoints()
        {
            int numPoints = (m_Waypoints == null) ? 0 : m_Waypoints.Length;
            if (numPoints > 1
                && (Looped != m_IsLoopedCache
                    || m_ControlPoints1 == null || m_ControlPoints1.Length != numPoints
                    || m_ControlPoints2 == null || m_ControlPoints2.Length != numPoints))
            {
                Vector4[] p1 = new Vector4[numPoints];
                Vector4[] p2 = new Vector4[numPoints];
                Vector4[] K = new Vector4[numPoints];
                for (int i = 0; i < numPoints; ++i)
                    K[i] = m_Waypoints[i].AsVector4;
                if (Looped)
                    ComputeSmoothControlPointsLooped(ref K, ref p1, ref p2);
                else
                    ComputeSmoothControlPoints(ref K, ref p1, ref p2);

                m_ControlPoints1 = new Waypoint[numPoints];
                m_ControlPoints2 = new Waypoint[numPoints];
                for (int i = 0; i < numPoints; ++i)
                {
                    m_ControlPoints1[i] = Waypoint.FromVector4(p1[i]);
                    m_ControlPoints2[i] = Waypoint.FromVector4(p2[i]);
                }
                m_IsLoopedCache = Looped;
            }
        }

        /// <summary>Returns standardized position</summary>
        float GetBoundingIndices(float pos, out int indexA, out int indexB)
        {
            pos = StandardizePos(pos);
            int numWaypoints = m_Waypoints.Length;
            if (numWaypoints < 2)
                indexA = indexB = 0;
            else
            {
                indexA = Mathf.FloorToInt(pos);
                if (indexA >= numWaypoints)
                {
                    // Only true if looped
                    pos -= MaxPos;
                    indexA = 0;
                }
                indexB = indexA + 1;
                if (indexB == numWaypoints)
                {
                    if (Looped)
                        indexB = 0;
                    else
                    {
                        --indexB;
                        --indexA;
                    }
                }
            }
            return pos;
        }

        /// <summary>Get a worldspace position of a point along the path</summary>
        /// <param name="pos">Position along the path.  Need not be normalized.</param>
        /// <returns>World-space position of the point along at path at pos</returns>
        public override Vector3 EvaluatePosition(float pos)
        {
            Vector3 result = Vector3.zero;
            if (m_Waypoints.Length > 0)
            {
                UpdateControlPoints();
                int indexA, indexB;
                pos = GetBoundingIndices(pos, out indexA, out indexB);
                if (indexA == indexB)
                    result = m_Waypoints[indexA].position;
                else
                    result = Bezier3(pos - indexA,
                        m_Waypoints[indexA].position, m_ControlPoints1[indexA].position,
                        m_ControlPoints2[indexA].position, m_Waypoints[indexB].position);
            }

            return transform != null ? transform.TransformPoint(result) : result;
        }

        /// <summary>Get the tangent of the curve at a point along the path.</summary>
        /// <param name="pos">Position along the path.  Need not be normalized.</param>
        /// <returns>World-space direction of the path tangent.
        /// Length of the vector represents the tangent strength</returns>
        public override Vector3 EvaluateTangent(float pos)
        {
            Vector3 result = transform != null ? transform.rotation * Vector3.forward : Vector3.forward;
            if (m_Waypoints.Length > 1)
            {
                UpdateControlPoints();
                int indexA, indexB;
                pos = GetBoundingIndices(pos, out indexA, out indexB);
                if (!Looped && indexA == m_Waypoints.Length - 1)
                    --indexA;
                result = BezierTangent3(pos - indexA,
                    m_Waypoints[indexA].position, m_ControlPoints1[indexA].position,
                    m_ControlPoints2[indexA].position, m_Waypoints[indexB].position);
            }
            return transform != null ? transform.TransformDirection(result) : result;
        }

        /// <summary>Get the orientation the curve at a point along the path.</summary>
        /// <param name="pos">Position along the path.  Need not be normalized.</param>
        /// <returns>World-space orientation of the path, as defined by tangent, up, and roll.</returns>
        public override Quaternion EvaluateOrientation(float pos)
        {
            Quaternion transformRot = transform != null ? transform.rotation : Quaternion.identity;
            Vector3 transformUp = transformRot * Vector3.up;
            Quaternion result = transformRot;
            if (m_Waypoints.Length > 0)
            {
                float roll = 0;
                int indexA, indexB;
                pos = GetBoundingIndices(pos, out indexA, out indexB);
                if (indexA == indexB)
                    roll = m_Waypoints[indexA].roll;
                else
                {
                    UpdateControlPoints();
                    roll = Bezier1(pos - indexA,
                        m_Waypoints[indexA].roll, m_ControlPoints1[indexA].roll,
                        m_ControlPoints2[indexA].roll, m_Waypoints[indexB].roll);
                }

                Vector3 fwd = EvaluateTangent(pos);
                if (!AlmostZero(fwd))
                {
                    Quaternion q = Quaternion.LookRotation(fwd, transformUp);
                    result = q * RollAroundForward(roll);
                }
            }
            return result;
        }

        // same as Quaternion.AngleAxis(roll, Vector3.forward), just simplified
        Quaternion RollAroundForward(float angle)
        {
            float halfAngle = angle * 0.5F * Mathf.Deg2Rad;
            return new Quaternion(
                0,
                0,
                Mathf.Sin(halfAngle),
                Mathf.Cos(halfAngle));
        }

    }

}