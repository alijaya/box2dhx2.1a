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

import flash.Vector;

import box2D.collision.shapes.B2Shape;

//typedef B2Contact* B2ContactCreateFcn(B2Shape* shape1, B2Shape* shape2, B2BlockAllocator* allocator);
//typedef void B2ContactDestroyFcn(B2Contact* contact, B2BlockAllocator* allocator);



/**
 * This class manages creation and destruction of B2Contact objects.
 * @private
 */
class B2ContactFactory
{
	
	public function new(allocator:Dynamic)
	{
		m_allocator = allocator;
		InitializeRegisters();
	}
	
	function AddType(createFcn:Dynamic, destroyFcn:Dynamic, type1:Int, type2:Int) : Void
	{
		//B2Settings.b2Assert(B2Shape.e_unknownShape < type1 && type1 < B2Shape.e_shapeTypeCount);
		//B2Settings.b2Assert(B2Shape.e_unknownShape < type2 && type2 < B2Shape.e_shapeTypeCount);
		
		m_registers[type1][type2].createFcn = createFcn;
		m_registers[type1][type2].destroyFcn = destroyFcn;
		m_registers[type1][type2].primary = true;
		
		if (type1 != type2)
		{
			m_registers[type2][type1].createFcn = createFcn;
			m_registers[type2][type1].destroyFcn = destroyFcn;
			m_registers[type2][type1].primary = false;
		}
	}
	function InitializeRegisters() : Void{
		m_registers = new Vector<Vector<B2ContactRegister>>(B2Shape.e_shapeTypeCount);
		for (i in 0...B2Shape.e_shapeTypeCount){
			m_registers[i] = new Vector<B2ContactRegister>(B2Shape.e_shapeTypeCount);
			for (j in 0...B2Shape.e_shapeTypeCount){
				m_registers[i][j] = new B2ContactRegister();
			}
		}
		
		AddType(B2CircleContact.Create, B2CircleContact.Destroy, B2Shape.e_circleShape, B2Shape.e_circleShape);
		AddType(B2PolyAndCircleContact.Create, B2PolyAndCircleContact.Destroy, B2Shape.e_polygonShape, B2Shape.e_circleShape);
		AddType(B2PolygonContact.Create, B2PolygonContact.Destroy, B2Shape.e_polygonShape, B2Shape.e_polygonShape);
		
		AddType(B2EdgeAndCircleContact.Create, B2EdgeAndCircleContact.Destroy, B2Shape.e_edgeShape, B2Shape.e_circleShape);
		AddType(B2PolyAndEdgeContact.Create, B2PolyAndEdgeContact.Destroy, B2Shape.e_polygonShape, B2Shape.e_edgeShape);
	}
	public function Create(fixtureA:B2Fixture, fixtureB:B2Fixture):B2Contact{
		var type1:Int = fixtureA.GetType();
		var type2:Int = fixtureB.GetType();
		
		//B2Settings.b2Assert(B2Shape.e_unknownShape < type1 && type1 < B2Shape.e_shapeTypeCount);
		//B2Settings.b2Assert(B2Shape.e_unknownShape < type2 && type2 < B2Shape.e_shapeTypeCount);
		
		var reg:B2ContactRegister = m_registers[type1][type2];
		
		var c:B2Contact;
		
		if (reg.pool != null)
		{
			// Pop a contact off the pool
			c = reg.pool;
			reg.pool = c.m_next;
			reg.poolCount--;
			c.Reset(fixtureA, fixtureB);
			return c;
		}
		
		var createFcn:Dynamic = reg.createFcn;
		if (createFcn != null)
		{
			if (reg.primary)
			{
				c = createFcn(m_allocator);
				c.Reset(fixtureA, fixtureB);
				return c;
			}
			else
			{
				c = createFcn(m_allocator);
				c.Reset(fixtureB, fixtureA);
				return c;
			}
		}
		else
		{
			return null;
		}
	}
	public function Destroy(contact:B2Contact) : Void{
		if (contact.m_manifold.m_pointCount > 0)
		{
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}
		
		var type1:Int = contact.m_fixtureA.GetType();
		var type2:Int = contact.m_fixtureB.GetType();
		
		//B2Settings.b2Assert(B2Shape.e_unknownShape < type1 && type1 < B2Shape.e_shapeTypeCount);
		//B2Settings.b2Assert(B2Shape.e_unknownShape < type2 && type2 < B2Shape.e_shapeTypeCount);
		
		var reg:B2ContactRegister = m_registers[type1][type2];
		
		if (true)
		{
			reg.poolCount++;
			contact.m_next = reg.pool;
			reg.pool = contact;
		}
		
		var destroyFcn:Dynamic = reg.destroyFcn;
		destroyFcn(contact, m_allocator);
	}

	
	private var m_registers:Vector<Vector<B2ContactRegister>>;
	var m_allocator:Dynamic;
}


