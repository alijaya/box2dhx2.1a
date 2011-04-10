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

package box2D.dynamics;

import box2D.collision.B2AABB;
import box2D.collision.B2RayCastOutput;
import box2D.collision.B2RayCastInput;
import box2D.collision.IBroadPhase;
import box2D.collision.shapes.B2Shape;
import box2D.collision.shapes.B2MassData;
import box2D.common.math.B2Math;
import box2D.common.math.B2Vec2;
import box2D.common.math.B2Transform;
import box2D.dynamics.contacts.B2Contact;
import box2D.dynamics.contacts.B2ContactEdge;

/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via B2Body::CreateFixture.
 * @warning you cannot reuse fixtures.
 */
class B2Fixture
{
	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	
	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public function GetType():Int
	{
		return m_shape.GetType();
	}
	
	/**
	 * Get the child shape. You can modify the child shape, however you should not change the
	 * number of vertices because this will crash some collision caching mechanisms.
	 */
	public function GetShape():B2Shape
	{
		return m_shape;
	}
	
	/**
	 * Set if this fixture is a sensor.
	 */
	public function SetSensor(sensor:Bool):Void
	{
		if ( m_isSensor == sensor)
			return;
			
		m_isSensor = sensor;
		
		if (m_body == null)
			return;
			
		var edge:B2ContactEdge = m_body.GetContactList();
		while (edge != null)
		{
			var contact:B2Contact = edge.contact;
			var fixtureA:B2Fixture = contact.GetFixtureA();
			var fixtureB:B2Fixture = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
			edge = edge.next;
		}
		
	}
	
	/**
	 * Is this fixture a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 */
	public function IsSensor():Bool
	{
		return m_isSensor;
	}
	
	/**
	 * Set the contact filtering data. This will not update contacts until the next time
	 * step when either parent body is active and awake.
	 */
	public function SetFilterData(filter:B2FilterData):Void
	{
		m_filter = filter.Copy();
		
		if (m_body != null)
			return;
			
		var edge:B2ContactEdge = m_body.GetContactList();
		while (edge != null)
		{
			var contact:B2Contact = edge.contact;
			var fixtureA:B2Fixture = contact.GetFixtureA();
			var fixtureB:B2Fixture = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.FlagForFiltering();
			edge = edge.next;
		}
	}
	
	/**
	 * Get the contact filtering data.
	 */
	public function GetFilterData(): B2FilterData
	{
		return m_filter.Copy();
	}
	
	/**
	 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
	 * @return the parent body.
	 */
	public function GetBody():B2Body
	{
		return m_body;
	}
	
	/**
	 * Get the next fixture in the parent body's fixture list.
	 * @return the next shape.
	 */
	public function GetNext():B2Fixture
	{
		return m_next;
	}
	
	/**
	 * Get the user data that was assigned in the fixture definition. Use this to
	 * store your application specific data.
	 */
	public function GetUserData():Dynamic
	{
		return m_userData;
	}
	
	/**
	 * Set the user data. Use this to store your application specific data.
	 */
	public function SetUserData(data:Dynamic):Void
	{
		m_userData = data;
	}
	
	/**
	 * Test a point for containment in this fixture.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 */
	public function TestPoint(p:B2Vec2):Bool
	{
		return m_shape.TestPoint(m_body.GetTransform(), p);
	}
	
	/**
	 * Perform a ray cast against this shape.
	 * @param output the ray-cast results.
	 * @param input the ray-cast input parameters.
	 */
	public function RayCast(output:B2RayCastOutput, input:B2RayCastInput):Bool
	{
		return m_shape.RayCast(output, input, m_body.GetTransform());
	}
	
	/**
	 * Get the mass data for this fixture. The mass data is based on the density and
	 * the shape. The rotational inertia is about the shape's origin. This operation may be expensive
	 * @param massData - this is a reference to a valid massData, if it is null a new B2MassData is allocated and then returned
	 * @note if the input is null then you must get the return value.
	 */
	public function GetMassData(?massData:B2MassData = null):B2MassData
	{
		if ( massData == null )
		{
			massData = new B2MassData();
		}
		m_shape.ComputeMass(massData, m_density);
		return massData;
	}
	
	/**
	 * Set the density of this fixture. This will _not_ automatically adjust the mass
	 * of the body. You must call B2Body::ResetMassData to update the body's mass.
	 * @param	density
	 */
	public function SetDensity(density:Float):Void {
		//B2Settings.b2Assert(B2Math.B2IsValid(density) && density >= 0.0);
		m_density = density;
	}
	
	/**
	 * Get the density of this fixture.
	 * @return density
	 */
	public function GetDensity():Float {
		return m_density;
	}
	
	/**
	 * Get the coefficient of friction.
	 */
	public function GetFriction():Float
	{
		return m_friction;
	}
	
	/**
	 * Set the coefficient of friction.
	 */
	public function SetFriction(friction:Float):Void
	{
		m_friction = friction;
	}
	
	/**
	 * Get the coefficient of restitution.
	 */
	public function GetRestitution():Float
	{
		return m_restitution;
	}
	
	/**
	 * Get the coefficient of restitution.
	 */
	public function SetRestitution(restitution:Float):Void
	{
		m_restitution = restitution;
	}
	
	/**
	 * Get the fixture's AABB. This AABB may be enlarge and/or stale.
	 * If you need a more accurate AABB, compute it using the shape and
	 * the body transform.
	 * @return
	 */
	public function GetAABB():B2AABB {
		return m_aabb;
	}
	
	/**
	 * @private
	 */
	public function new()
	{
		m_filter = new B2FilterData();
		
		m_aabb = new B2AABB();
		m_userData = null;
		m_body = null;
		m_next = null;
		//m_proxyId = B2BroadPhase.e_nullProxy;
		m_shape = null;
		m_density = 0.0;
		
		m_friction = 0.0;
		m_restitution = 0.0;
	}
	
	/**
	 * the destructor cannot access the allocator (no destructor arguments allowed by C++).
	 *  We need separation create/destroy functions from the constructor/destructor because
	 */
	public function Create( body:B2Body, xf:B2Transform, def:B2FixtureDef):Void
	{
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		
		m_body = body;
		m_next = null;
		
		m_filter = def.filter.Copy();
		
		m_isSensor = def.isSensor;
		
		m_shape = def.shape.Copy();
		
		m_density = def.density;
	}
	
	/**
	 * the destructor cannot access the allocator (no destructor arguments allowed by C++).
	 *  We need separation create/destroy functions from the constructor/destructor because
	 */
	public function Destroy():Void
	{
		// The proxy must be destroyed before calling this.
		//b2Assert(m_proxyId == B2BroadPhase::e_nullProxy);
		
		// Free the child shape
		m_shape = null;
	}
	
	/**
	 * This supports body activation/deactivation.
	 */ 
	public function CreateProxy(broadPhase:IBroadPhase, xf:B2Transform):Void {
		//b2Assert(m_proxyId == B2BroadPhase::e_nullProxy);
		
		// Create proxy in the broad-phase.
		m_shape.ComputeAABB(m_aabb, xf);
		m_proxy = broadPhase.CreateProxy(m_aabb, this);
	}
	
	/**
	 * This supports body activation/deactivation.
	 */
	public function DestroyProxy(broadPhase:IBroadPhase):Void {
		if (m_proxy == null)
		{
			return;
		}
		
		// Destroy proxy in the broad-phase.
		broadPhase.DestroyProxy(m_proxy);
		m_proxy = null;
	}
	
	public function Synchronize(broadPhase:IBroadPhase, transform1:B2Transform, transform2:B2Transform):Void
	{
		if (!m_proxy)
			return;
			
		// Compute an AABB that ocvers the swept shape (may miss some rotation effect)
		var aabb1:B2AABB = new B2AABB();
		var aabB2:B2AABB = new B2AABB();
		m_shape.ComputeAABB(aabb1, transform1);
		m_shape.ComputeAABB(aabB2, transform2);
		
		m_aabb.SetCombine(aabb1, aabB2);
		var displacement:B2Vec2 = B2Math.SubtractVV(transform2.position, transform1.position);
		broadPhase.MoveProxy(m_proxy, m_aabb, displacement);
	}
	
	public var m_massData:B2MassData;
	
	public var m_aabb:B2AABB;
	public var m_density:Float;
	public var m_next:B2Fixture;
	public var m_body:B2Body;
	public var m_shape:B2Shape;
	
	public var m_friction:Float;
	public var m_restitution:Float;
	
	public var m_proxy:Dynamic;
	public var m_filter:B2FilterData ;
	
	public var m_isSensor:Bool;
	
	public var m_userData:Dynamic;
}



