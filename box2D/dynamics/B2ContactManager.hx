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

import box2D.collision.B2ContactPoint;
import box2D.collision.B2DynamicTreeBroadPhase;
import box2D.collision.IBroadPhase;
import box2D.dynamics.contacts.B2ContactFactory;
import box2D.dynamics.contacts.B2Contact;
import box2D.dynamics.contacts.B2ContactEdge;

// Delegate of B2World.
/**
* @private
*/
class B2ContactManager 
 {
	
	public function new() {
		m_world = null;
		m_contactCount = 0;
		m_contactFilter = B2ContactFilter.b2_defaultFilter;
		m_contactListener = B2ContactListener.b2_defaultListener;
		m_contactFactory = new B2ContactFactory(m_allocator);
		m_broadPhase = new B2DynamicTreeBroadPhase();
	}

	// This is a callback from the broadphase when two AABB proxies begin
	// to overlap. We create a B2Contact to manage the narrow phase.
	public function AddPair(proxyUserDataA:Dynamic, proxyUserDataB:Dynamic):Void {
		var fixtureA:B2Fixture = cast proxyUserDataA;
		var fixtureB:B2Fixture = cast proxyUserDataB;
		
		var bodyA:B2Body = fixtureA.GetBody();
		var bodyB:B2Body = fixtureB.GetBody();
		
		// Are the fixtures on the same body?
		if (bodyA == bodyB)
			return;
		
		// Does a contact already exist?
		var edge:B2ContactEdge = bodyB.GetContactList();
		while (edge != null)
		{
			if (edge.other == bodyA)
			{
				var fA:B2Fixture = edge.contact.GetFixtureA();
				var fB:B2Fixture = edge.contact.GetFixtureB();
				if (fA == fixtureA && fB == fixtureB)
					return;
				if (fA == fixtureB && fB == fixtureA)
					return;
			}
			edge = edge.next;
		}
		
		//Does a joint override collision? Is at least one body dynamic?
		if (bodyB.ShouldCollide(bodyA) == false)
		{
			return;
		}
		
		// Check user filtering
		if (m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
		{
			return;
		}
		
		// Call the factory.
		var c:B2Contact = m_contactFactory.Create(fixtureA, fixtureB);
		
		// Contact creation may swap shapes.
		fixtureA = c.GetFixtureA();
		fixtureB = c.GetFixtureB();
		bodyA = fixtureA.m_body;
		bodyB = fixtureB.m_body;
		
		// Insert into the world.
		c.m_prev = null;
		c.m_next = m_world.m_contactList;
		if (m_world.m_contactList != null)
		{
			m_world.m_contactList.m_prev = c;
		}
		m_world.m_contactList = c;
		
		
		// Connect to island graph.
		
		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;
		
		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null)
		{
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;
		
		// Connect to body 2
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;
		
		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null)
		{
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;
		
		++m_world.m_contactCount;
		return;
		
	}

	public function FindNewContacts():Void
	{
		m_broadPhase.UpdatePairs(AddPair);
	}
	
	static var s_evalCP:B2ContactPoint = new B2ContactPoint();
	public function Destroy(c:B2Contact) : Void
	{
		
		var fixtureA:B2Fixture = c.GetFixtureA();
		var fixtureB:B2Fixture = c.GetFixtureB();
		var bodyA:B2Body = fixtureA.GetBody();
		var bodyB:B2Body = fixtureB.GetBody();
		
		if (c.IsTouching())
		{
			m_contactListener.EndContact(c);
		}
		
		// Remove from the world.
		if (c.m_prev != null)
		{
			c.m_prev.m_next = c.m_next;
		}
		
		if (c.m_next != null)
		{
			c.m_next.m_prev = c.m_prev;
		}
		
		if (c == m_world.m_contactList)
		{
			m_world.m_contactList = c.m_next;
		}
		
		// Remove from body A
		if (c.m_nodeA.prev != null)
		{
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}
		
		if (c.m_nodeA.next != null)
		{
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}
		
		if (c.m_nodeA == bodyA.m_contactList)
		{
			bodyA.m_contactList = c.m_nodeA.next;
		}
		
		// Remove from body 2
		if (c.m_nodeB.prev != null)
		{
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}
		
		if (c.m_nodeB.next != null)
		{
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}
		
		if (c.m_nodeB == bodyB.m_contactList)
		{
			bodyB.m_contactList = c.m_nodeB.next;
		}
		
		// Call the factory.
		m_contactFactory.Destroy(c);
		--m_contactCount;
	}
	

	// This is the top level collision call for the time step. Here
	// all the narrow phase collision is processed for the world
	// contact list.
	public function Collide() : Void
	{
		// Update awake contacts.
		var c:B2Contact = m_world.m_contactList;
		while (c != null)
		{
			var fixtureA:B2Fixture = c.GetFixtureA();
			var fixtureB:B2Fixture = c.GetFixtureB();
			var bodyA:B2Body = fixtureA.GetBody();
			var bodyB:B2Body = fixtureB.GetBody();
			if (bodyA.IsAwake() == false && bodyB.IsAwake() == false)
			{
				c = c.GetNext();
				continue;
			}
			
			var cNuke:B2Contact;
			// Is this contact flagged for filtering?
			if ((c.m_flags & B2Contact.e_filterFlag) != 0)
			{
				// Should these bodies collide?
				if (bodyB.ShouldCollide(bodyA) == false)
				{
					cNuke = c;
					c = cNuke.GetNext();
					Destroy(cNuke);
					continue;
				}
				
				// Check user filtering.
				if (m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
				{
					cNuke = c;
					c = cNuke.GetNext();
					Destroy(cNuke);
					continue;
				}
				
				// Clear the filtering flag
				c.m_flags &= ~B2Contact.e_filterFlag;
			}
			
			var proxyA:Dynamic = fixtureA.m_proxy;
			var proxyB:Dynamic = fixtureB.m_proxy;
			
			var overlap:Bool = m_broadPhase.TestOverlap(proxyA, proxyB);
			
			// Here we destroy contacts that cease to overlap in the broadphase
			if ( overlap == false)
			{
				cNuke = c;
				c = cNuke.GetNext();
				Destroy(cNuke);
				continue;
			}
			
			c.Update(m_contactListener);
			c = c.GetNext();
		}
	}

	
	public var m_world:B2World;
	public var m_broadPhase:IBroadPhase;
	
	public var m_contactList:B2Contact;
	public var m_contactCount:Int;
	public var m_contactFilter:B2ContactFilter;
	public var m_contactListener:B2ContactListener;
	public var m_contactFactory:B2ContactFactory;
	public var m_allocator:Dynamic;
}

