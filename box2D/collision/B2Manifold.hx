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

import box2D.common.B2Settings;
import box2D.common.math.B2Vec2;

/**
 * A manifold for two touching convex shapes.
 * box2D supports multiple types of contact:
 * - clip point versus plane with radius
 * - point versus point with radius (circles)
 * The local point usage depends on the manifold type:
 * -e_circles: the local center of circleA
 * -e_faceA: the center of faceA
 * -e_faceB: the center of faceB
 * Similarly the local normal usage:
 * -e_circles: not used
 * -e_faceA: the normal on polygonA
 * -e_faceB: the normal on polygonB
 * We store contacts in this way so that position correction can
 * account for movement, which is critical for continuous physics.
 * All contact scenarios must be expressed in one of these types.
 * This structure is stored across time steps, so we keep it small.
 */
class B2Manifold
 {
	
	public function new(){
		m_points = new Vector<B2ManifoldPoint>(B2Settings.b2_maxManifoldPoints);
		for (i in 0...B2Settings.b2_maxManifoldPoints){
			m_points[i] = new B2ManifoldPoint();
		}
		m_localPlaneNormal = new B2Vec2();
		m_localPoint = new B2Vec2();
	}
	public function Reset() : Void{
		for (i in 0...B2Settings.b2_maxManifoldPoints){
			m_points[i].Reset();
		}
		m_localPlaneNormal.SetZero();
		m_localPoint.SetZero();
		m_type = 0;
		m_pointCount = 0;
	}
	public function Set(m:B2Manifold) : Void{
		m_pointCount = m.m_pointCount;
		for (i in 0...B2Settings.b2_maxManifoldPoints){
			m_points[i].Set(m.m_points[i]);
		}
		m_localPlaneNormal.SetV(m.m_localPlaneNormal);
		m_localPoint.SetV(m.m_localPoint);
		m_type = m.m_type;
	}
	public function Copy():B2Manifold
	{
		var copy:B2Manifold = new B2Manifold();
		copy.Set(this);
		return copy;
	}
	/** The points of contact */	
	public var m_points:Vector<B2ManifoldPoint>;	
	/** Not used for Type e_points*/	
	public var m_localPlaneNormal:B2Vec2;	
	/** Usage depends on manifold type */	
	public var m_localPoint:B2Vec2;	
	public var m_type:Int;
	/** The number of manifold points */	
	public var m_pointCount:Int ;
	
	//enum Type
	public static var e_circles:Int = 0x0001;
	public static var e_faceA:Int = 0x0002;
	public static var e_faceB:Int = 0x0004;
}


