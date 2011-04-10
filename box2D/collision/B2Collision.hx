/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package box2D.collision;

import flash.Vector;

import box2D.collision.shapes.B2CircleShape;
import box2D.collision.shapes.B2PolygonShape;
import box2D.common.B2Settings;
import box2D.common.math.B2Vec2;
import box2D.common.math.B2Mat22;
import box2D.common.math.B2Transform;

/**
* @private
*/
class B2Collision {
	
	// Null feature
	public static var B2_nullFeature:Int = 0x000000ff;//UCHAR_MAX;
	
	// Sutherland-Hodgman clipping.
	static public function ClipSegmentToLine(vOut:Vector<ClipVertex>, vIn:Vector<ClipVertex>, normal:B2Vec2, offset:Float):Int
	{
		var cv:ClipVertex;
		
		// Start with no output points
		var numOut:Int = 0;
		
		cv = vIn[0];
		var vIn0:B2Vec2 = cv.v;
		cv = vIn[1];
		var vIn1:B2Vec2 = cv.v;
		
		// Calculate the distance of end points to the line
		var distance0:Float = normal.x * vIn0.x + normal.y * vIn0.y - offset;
		var distance1:Float = normal.x * vIn1.x + normal.y * vIn1.y - offset;
		
		// If the points are behind the plane
		if (distance0 <= 0.0) vOut[numOut++].Set(vIn[0]);
		if (distance1 <= 0.0) vOut[numOut++].Set(vIn[1]);
		
		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0)
		{
			// Find intersection point of edge and plane
			var interp:Float = distance0 / (distance0 - distance1);
			// expanded for performance 
			// vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
			cv = vOut[numOut];
			var tVec:B2Vec2 = cv.v;
			tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
			tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
			cv = vOut[numOut];
			var cv2: ClipVertex;
			if (distance0 > 0.0)
			{
				cv2 = vIn[0];
				cv.id = cv2.id;
			}
			else
			{
				cv2 = vIn[1];
				cv.id = cv2.id;
			}
			++numOut;
		}
		
		return numOut;
	}
	
	
	// Find the separation between poly1 and poly2 for a give edge normal on poly1.
	public static function EdgeSeparation(	poly1:B2PolygonShape, xf1:B2Transform, edge1:Int, 
											poly2:B2PolygonShape, xf2:B2Transform):Float
	{
		var count1:Int = poly1.m_vertexCount;
		var vertices1:Vector<B2Vec2> = poly1.m_vertices;
		var normals1:Vector<B2Vec2> = poly1.m_normals;
		
		var count2:Int = poly2.m_vertexCount;
		var vertices2:Vector<B2Vec2> = poly2.m_vertices;
		
		//b2Assert(0 <= edge1 && edge1 < count1);
		
		var tMat:B2Mat22;
		var tVec:B2Vec2;
		
		// Convert normal from poly1's frame into poly2's frame.
		//B2Vec2 normal1World = B2Mul(xf1.R, normals1[edge1]);
		tMat = xf1.R;
		tVec = normals1[edge1];
		var normal1WorldX:Float = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var normal1WorldY:Float = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//B2Vec2 normal1 = B2MulT(xf2.R, normal1World);
		tMat = xf2.R;
		var normal1X:Float = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
		var normal1Y:Float = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);
		
		// Find support vertex on poly2 for -normal.
		var index:Int = 0;
		var minDot:Float = box2D.common.math.B2Math.MAX_VALUE;
		for (i in 0...count2)
		{
			//float32 dot = B2Dot(poly2->m_vertices[i], normal1);
			tVec = vertices2[i];
			var dot:Float = tVec.x * normal1X + tVec.y * normal1Y;
			if (dot < minDot)
			{
				minDot = dot;
				index = i;
			}
		}
		
		//B2Vec2 v1 = B2Mul(xf1, vertices1[edge1]);
		tVec = vertices1[edge1];
		tMat = xf1.R;
		var v1X:Float = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var v1Y:Float = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//B2Vec2 v2 = B2Mul(xf2, vertices2[index]);
		tVec = vertices2[index];
		tMat = xf2.R;
		var v2X:Float = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var v2Y:Float = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//var separation:Number = B2Math.B2Dot( B2Math.SubtractVV( v2, v1 ) , normal);
		v2X -= v1X;
		v2Y -= v1Y;
		//float32 separation = B2Dot(v2 - v1, normal1World);
		var separation:Float = v2X * normal1WorldX + v2Y * normal1WorldY;
		return separation;
	}
	
	
	
	
	// Find the max separation between poly1 and poly2 using edge normals
	// from poly1.
	static public function FindMaxSeparation(edgeIndex:Vector<Int>, 
											poly1:B2PolygonShape, xf1:B2Transform, 
											poly2:B2PolygonShape, xf2:B2Transform):Float
	{
		var count1:Int = poly1.m_vertexCount;
		var normals1:Vector<B2Vec2> = poly1.m_normals;
		
		var tVec:B2Vec2;
		var tMat:B2Mat22;
		
		// Vector pointing from the centroid of poly1 to the centroid of poly2.
		//B2Vec2 d = B2Mul(xf2, poly2->m_centroid) - B2Mul(xf1, poly1->m_centroid);
		tMat = xf2.R;
		tVec = poly2.m_centroid;
		var dX:Float = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var dY:Float = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf1.R;
		tVec = poly1.m_centroid;
		dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//B2Vec2 dLocal1 = B2MulT(xf1.R, d);
		var dLocal1X:Float = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
		var dLocal1Y:Float = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);
		
		// Get support vertex as a hint for our search
		var edge:Int = 0;
		var maxDot:Float = -box2D.common.math.B2Math.MAX_VALUE;
		for (i in 0...count1)
		{
			//var dot:Number = B2Math.B2Dot(normals1[i], dLocal1);
			tVec = normals1[i];
			var dot:Float = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
			if (dot > maxDot)
			{
				maxDot = dot;
				edge = i;
			}
		}
		
		// Get the separation for the edge normal.
		var s:Float = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
		
		// Check the separation for the previous edge normal.
		var prevEdge:Int = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		var sPrev:Float = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
		
		// Check the separation for the next edge normal.
		var nextEdge:Int = edge + 1 < count1 ? edge + 1 : 0;
		var sNext:Float = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
		
		// Find the best edge and the search direction.
		var bestEdge:Int;
		var bestSeparation:Float;
		var increment:Int;
		if (sPrev > s && sPrev > sNext)
		{
			increment = -1;
			bestEdge = prevEdge;
			bestSeparation = sPrev;
		}
		else if (sNext > s)
		{
			increment = 1;
			bestEdge = nextEdge;
			bestSeparation = sNext;
		}
		else
		{
			// pointer out
			edgeIndex[0] = edge;
			return s;
		}
		
		// Perform a local search for the best edge normal.
		while (true)
		{
			
			if (increment == -1)
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			else
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
			
			s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
			
			if (s > bestSeparation)
			{
				bestEdge = edge;
				bestSeparation = s;
			}
			else
			{
				break;
			}
		}
		
		// pointer out
		edgeIndex[0] = bestEdge;
		return bestSeparation;
	}
	
	
	
	static public function FindIncidentEdge(c:Vector<ClipVertex>, 
											poly1:B2PolygonShape, xf1:B2Transform, edge1:Int, 
											poly2:B2PolygonShape, xf2:B2Transform) : Void
	{
		var count1:Int = poly1.m_vertexCount;
		var normals1:Vector<B2Vec2> = poly1.m_normals;
		
		var count2:Int = poly2.m_vertexCount;
		var vertices2:Vector<B2Vec2> = poly2.m_vertices;
		var normals2:Vector<B2Vec2> = poly2.m_normals;
		
		//b2Assert(0 <= edge1 && edge1 < count1);
		
		var tMat:B2Mat22;
		var tVec:B2Vec2;
		
		// Get the normal of the reference edge in poly2's frame.
		//B2Vec2 normal1 = B2MulT(xf2.R, B2Mul(xf1.R, normals1[edge1]));
		tMat = xf1.R;
		tVec = normals1[edge1];
		var normal1X:Float = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var normal1Y:Float = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf2.R;
		var tX:Float = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y);
		normal1Y = 		(tMat.col2.x * normal1X + tMat.col2.y * normal1Y);
		normal1X = tX;
		
		// Find the incident edge on poly2.
		var index:Int = 0;
		var minDot:Float = box2D.common.math.B2Math.MAX_VALUE;
		for (i in 0...count2)
		{
			//var dot:Number = B2Dot(normal1, normals2[i]);
			tVec = normals2[i];
			var dot:Float = (normal1X * tVec.x + normal1Y * tVec.y);
			if (dot < minDot)
			{
				minDot = dot;
				index = i;
			}
		}
		
		var tClip:ClipVertex;
		// Build the clip vertices for the incident edge.
		var i1:Int = index;
		var i2:Int = i1 + 1 < count2 ? i1 + 1 : 0;
		
		tClip = c[0];
		//c[0].v = B2Mul(xf2, vertices2[i1]);
		tVec = vertices2[i1];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i1;
		tClip.id.features.incidentVertex = 0;
		
		tClip = c[1];
		//c[1].v = B2Mul(xf2, vertices2[i2]);
		tVec = vertices2[i2];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i2;
		tClip.id.features.incidentVertex = 1;
	}
	
	
	private static function MakeClipPointVector():Vector<ClipVertex>
	{
		var r:Vector<ClipVertex> = new Vector<ClipVertex>(2);
		r[0] = new ClipVertex();
		r[1] = new ClipVertex();
		return r;
	}
	private static var s_incidentEdge:Vector<ClipVertex> = MakeClipPointVector();
	private static var s_clipPoints1:Vector<ClipVertex> = MakeClipPointVector();
	private static var s_clipPoints2:Vector<ClipVertex> = MakeClipPointVector();
	private static var s_edgeAO:Vector<Int> = new Vector<Int>(1);
	private static var s_edgeBO:Vector<Int> = new Vector<Int>(1);
	static var s_localTangent:B2Vec2 = new B2Vec2();
	static var s_localNormal:B2Vec2 = new B2Vec2();
	static var s_planePoint:B2Vec2 = new B2Vec2();
	static var s_normal:B2Vec2 = new B2Vec2();
	static var s_tangent:B2Vec2 = new B2Vec2();
	static var s_tangent2:B2Vec2 = new B2Vec2();
	static var s_v11:B2Vec2 = new B2Vec2();
	static var s_v12:B2Vec2 = new B2Vec2();
	// Find edge normal of max separation on A - return if separating axis is found
	// Find edge normal of max separation on B - return if separation axis is found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip
	static var B2CollidePolyTempVec:B2Vec2 = new B2Vec2();
	// The normal points from 1 to 2
	public static function CollidePolygons(manifold:B2Manifold, 
											polyA:B2PolygonShape, xfA:B2Transform,
											polyB:B2PolygonShape, xfB:B2Transform) : Void
	{
		var cv: ClipVertex;
		
		manifold.m_pointCount = 0;
		var totalRadius:Float = polyA.m_radius + polyB.m_radius;

		var edgeA:Int = 0;
		s_edgeAO[0] = edgeA;
		var separationA:Float = FindMaxSeparation(s_edgeAO, polyA, xfA, polyB, xfB);
		edgeA = s_edgeAO[0];
		if (separationA > totalRadius)
			return;

		var edgeB:Int = 0;
		s_edgeBO[0] = edgeB;
		var separationB:Float = FindMaxSeparation(s_edgeBO, polyB, xfB, polyA, xfA);
		edgeB = s_edgeBO[0];
		if (separationB > totalRadius)
			return;

		var poly1:B2PolygonShape;	// reference poly
		var poly2:B2PolygonShape;	// incident poly
		var xf1:B2Transform;
		var xf2:B2Transform;
		var edge1:Int;		// reference edge
		var flip:Int;
		var k_relativeTol:Float = 0.98;
		var k_absoluteTol:Float = 0.001;
		var tMat:B2Mat22;

		if (separationB > k_relativeTol * separationA + k_absoluteTol)
		{
			poly1 = polyB;
			poly2 = polyA;
			xf1 = xfB;
			xf2 = xfA;
			edge1 = edgeB;
			manifold.m_type = B2Manifold.e_faceB;
			flip = 1;
		}
		else
		{
			poly1 = polyA;
			poly2 = polyB;
			xf1 = xfA;
			xf2 = xfB;
			edge1 = edgeA;
			manifold.m_type = B2Manifold.e_faceA;
			flip = 0;
		}

		var incidentEdge:Vector<ClipVertex> = s_incidentEdge; 
		FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		var count1:Int = poly1.m_vertexCount;
		var vertices1:Vector<B2Vec2> = poly1.m_vertices;

		var local_v11:B2Vec2 = vertices1[edge1];
		var local_v12:B2Vec2;
		if (edge1 + 1 < count1) {
			local_v12 = vertices1[edge1+1];
		} else {
			local_v12 = vertices1[0];
		}

		var localTangent:B2Vec2 = s_localTangent;
		localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
		localTangent.Normalize();
		
		var localNormal:B2Vec2 = s_localNormal;
		localNormal.x = localTangent.y;
		localNormal.y = -localTangent.x;
		
		var planePoint:B2Vec2 = s_planePoint;
		planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));
		
		var tangent:B2Vec2 = s_tangent;
		//tangent = B2Math.B2MulMV(xf1.R, localTangent);
		tMat = xf1.R;
		tangent.x = (tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y);
		tangent.y = (tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y);
		var tangent2:B2Vec2 = s_tangent2;
		tangent2.x = - tangent.x;
		tangent2.y = - tangent.y;
		var normal:B2Vec2 = s_normal;
		normal.x = tangent.y;
		normal.y = -tangent.x;

		//v11 = B2Math.MulX(xf1, local_v11);
		//v12 = B2Math.MulX(xf1, local_v12);
		var v11:B2Vec2 = s_v11;
		var v12:B2Vec2 = s_v12;
		v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
		v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
		v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
		v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);

		// Face offset
		var frontOffset:Float = normal.x * v11.x + normal.y * v11.y;
		// Side offsets, extended by polytope skin thickness
		var sideOffset1:Float = -tangent.x * v11.x - tangent.y * v11.y + totalRadius;
		var sideOffset2:Float = tangent.x * v12.x + tangent.y * v12.y + totalRadius;

		// Clip incident edge against extruded edge1 side edges.
		var clipPoints1:Vector<ClipVertex> = s_clipPoints1;
		var clipPoints2:Vector<ClipVertex> = s_clipPoints2;
		var np:Int;

		// Clip to box side 1
		//np = ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1);
		np = ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);

		if (np < 2)
			return;

		// Clip to negative box side 1
		np = ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2);

		if (np < 2)
			return;

		// Now clipPoints2 contains the clipped points.
		manifold.m_localPlaneNormal.SetV(localNormal);
		manifold.m_localPoint.SetV(planePoint);
		
		var pointCount:Int = 0;
		for (i in 0...B2Settings.b2_maxManifoldPoints)
		{
			cv = clipPoints2[i];
			var separation:Float = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
			if (separation <= totalRadius)
			{
				var cp:B2ManifoldPoint = manifold.m_points[ pointCount ];
				//cp.m_localPoint = B2Math.B2MulXT(xf2, cv.v);
				tMat = xf2.R;
				var tX:Float = cv.v.x - xf2.position.x;
				var tY:Float = cv.v.y - xf2.position.y;
				cp.m_localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y );
				cp.m_localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y );
				cp.m_id.Set(cv.id);
				cp.m_id.features.flip = flip;
				++pointCount;
			}
		}
		
		manifold.m_pointCount = pointCount;
	}
	
	
	
	public static function CollideCircles(
		manifold:B2Manifold, 
		circle1:B2CircleShape, xf1:B2Transform, 
		circle2:B2CircleShape, xf2:B2Transform) : Void
	{
		manifold.m_pointCount = 0;
		
		var tMat:B2Mat22;
		var tVec:B2Vec2;
		
		//B2Vec2 p1 = B2Mul(xf1, circle1->m_p);
		tMat = xf1.R; tVec = circle1.m_p;
		var p1X:Float = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var p1Y:Float = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//B2Vec2 p2 = B2Mul(xf2, circle2->m_p);
		tMat = xf2.R; tVec = circle2.m_p;
		var p2X:Float = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var p2Y:Float = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//B2Vec2 d = p2 - p1;
		var dX:Float = p2X - p1X;
		var dY:Float = p2Y - p1Y;
		//var distSqr:Number = B2Math.B2Dot(d, d);
		var distSqr:Float = dX * dX + dY * dY;
		var radius:Float = circle1.m_radius + circle2.m_radius;
		if (distSqr > radius * radius)
		{
			return;
		}
		manifold.m_type = B2Manifold.e_circles;
		manifold.m_localPoint.SetV(circle1.m_p);
		manifold.m_localPlaneNormal.SetZero();
		manifold.m_pointCount = 1;
		manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
		manifold.m_points[0].m_id.key = 0;
	}
	
	
	
	public static function CollidePolygonAndCircle(
		manifold:B2Manifold, 
		polygon:B2PolygonShape, xf1:B2Transform,
		circle:B2CircleShape, xf2:B2Transform) : Void
	{
		manifold.m_pointCount = 0;
		var tPoint:B2ManifoldPoint;
		
		var dX:Float;
		var dY:Float;
		var positionX:Float;
		var positionY:Float;
		
		var tVec:B2Vec2;
		var tMat:B2Mat22;
		
		// Compute circle position in the frame of the polygon.
		//B2Vec2 c = B2Mul(xf2, circle->m_localPosition);
		tMat = xf2.R;
		tVec = circle.m_p;
		var cX:Float = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var cY:Float = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//B2Vec2 cLocal = B2MulT(xf1, c);
		dX = cX - xf1.position.x;
		dY = cY - xf1.position.y;
		tMat = xf1.R;
		var cLocalX:Float = (dX * tMat.col1.x + dY * tMat.col1.y);
		var cLocalY:Float = (dX * tMat.col2.x + dY * tMat.col2.y);
		
		var dist:Float;
		
		// Find the min separating edge.
		var normalIndex:Int = 0;
		var separation:Float = -box2D.common.math.B2Math.MAX_VALUE;
		var radius:Float = polygon.m_radius + circle.m_radius;
		var vertexCount:Int = polygon.m_vertexCount;
		var vertices:Vector<B2Vec2> = polygon.m_vertices;
		var normals:Vector<B2Vec2> = polygon.m_normals;

		for (i in 0...vertexCount)
		{
			//float32 s = B2Dot(normals[i], cLocal - vertices[i]);
			tVec = vertices[i];
			dX = cLocalX-tVec.x;
			dY = cLocalY-tVec.y;
			tVec = normals[i];
			var s:Float = tVec.x * dX + tVec.y * dY;
			
			if (s > radius)
			{
				// Early out.
				return;
			}
			
			if (s > separation)
			{
				separation = s;
				normalIndex = i;
			}
		}
		// Vertices that subtend the incident face
		var vertIndex1:Int = normalIndex;
		var vertIndex2:Int = ((vertIndex1 + 1) < vertexCount)?vertIndex1 + 1:0;
		var v1:B2Vec2 = vertices[vertIndex1];
		var v2:B2Vec2 = vertices[vertIndex2];
		
		// If the center is inside the polygon ...
		if (separation < box2D.common.math.B2Math.MIN_VALUE)
		{
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
			manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
			manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
			return;
		}
		
		// Project the circle center onto the edge segment.
		var u1:Float = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
		var u2:Float = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
		if (u1 <= 0.0)
		{
			if ((cLocalX-v1.x)*(cLocalX-v1.x)+(cLocalY-v1.y)*(cLocalY-v1.y) > radius * radius)
				return;
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v1.x;
			manifold.m_localPlaneNormal.y = cLocalY - v1.y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.SetV(v1);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		else if (u2 <= 0)
		{
			if ((cLocalX-v2.x)*(cLocalX-v2.x)+(cLocalY-v2.y)*(cLocalY-v2.y) > radius * radius)
				return;
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v2.x;
			manifold.m_localPlaneNormal.y = cLocalY - v2.y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.SetV(v2);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		else
		{
			var faceCenterX:Float = 0.5 * (v1.x + v2.x);
			var faceCenterY:Float = 0.5 * (v1.y + v2.y);
			separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
			if (separation > radius)
				return;
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
			manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.Set(faceCenterX,faceCenterY);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
	}




	public static function TestOverlap(a:B2AABB, b:B2AABB):Bool
	{
		var t1:B2Vec2 = b.lowerBound;
		var t2:B2Vec2 = a.upperBound;
		//d1 = B2Math.SubtractVV(b.lowerBound, a.upperBound);
		var d1X:Float = t1.x - t2.x;
		var d1Y:Float = t1.y - t2.y;
		//d2 = B2Math.SubtractVV(a.lowerBound, b.upperBound);
		t1 = a.lowerBound;
		t2 = b.upperBound;
		var d2X:Float = t1.x - t2.x;
		var d2Y:Float = t1.y - t2.y;
		
		if (d1X > 0.0 || d1Y > 0.0)
			return false;
		
		if (d2X > 0.0 || d2Y > 0.0)
			return false;
		
		return true;
	}
	
	
	

}

