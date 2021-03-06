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

package box2D.dynamics.contacts;

import box2D.collision.B2Collision;
import box2D.collision.shapes.B2CircleShape;

/**
* @private
*/
class B2CircleContact extends B2Contact {
	public function new() { super(); }
	
	public static function Create(allocator:Dynamic):B2Contact{
		return new B2CircleContact();
	}
	public static function Destroy(contact:B2Contact, allocator:Dynamic) : Void{
		//
	}

	override function Reset(?fixtureA:B2Fixture = null, ?fixtureB:B2Fixture = null):Void{
		super.Reset(fixtureA, fixtureB);
		//B2Settings.b2Assert(m_shape1.m_type == B2Shape.e_circleShape);
		//B2Settings.b2Assert(m_shape2.m_type == B2Shape.e_circleShape);
	}
	//~B2CircleContact() {}
	
	override function Evaluate() : Void{
		var bA:B2Body = m_fixtureA.GetBody();
		var bB:B2Body = m_fixtureB.GetBody();
		
		B2Collision.CollideCircles(m_manifold, 
					cast( m_fixtureA.GetShape(), B2CircleShape), bA.m_xf, 
					cast( m_fixtureB.GetShape(), B2CircleShape), bB.m_xf);
	}
}

