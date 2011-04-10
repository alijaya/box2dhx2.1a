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

package box2D.dynamics.joints;

import box2D.common.B2Settings;
import box2D.common.math.B2Vec2;

/**
* The base joint class. Joints are used to constraint two bodies together in
* various fashions. Some joints also feature limits and motors.
* @see B2JointDef
*/
class B2Joint
 {
	/**
	* Get the type of the concrete joint.
	*/
	
	/**
	* Get the type of the concrete joint.
	*/
	public function GetType():Int{
		return m_type;
	}
	
	/**
	* Get the anchor point on bodyA in world coordinates.
	*/
	public function GetAnchorA():B2Vec2{return null;}
	/**
	* Get the anchor point on bodyB in world coordinates.
	*/
	public function GetAnchorB():B2Vec2{return null;}
	
	/**
	* Get the reaction force on body2 at the joint anchor in Newtons.
	*/
	public function GetReactionForce(inv_dt:Float):B2Vec2 {return null;}
	/**
	* Get the reaction torque on body2 in N*m.
	*/
	public function GetReactionTorque(inv_dt:Float):Float {return 0.0;}
	
	/**
	* Get the first body attached to this joint.
	*/
	public function GetBodyA():B2Body
	{
		return m_bodyA;
	}
	
	/**
	* Get the second body attached to this joint.
	*/
	public function GetBodyB():B2Body
	{
		return m_bodyB;
	}

	/**
	* Get the next joint the world joint list.
	*/
	public function GetNext():B2Joint{
		return m_next;
	}

	/**
	* Get the user data pointer.
	*/
	public function GetUserData():Dynamic{
		return m_userData;
	}

	/**
	* Set the user data pointer.
	*/
	public function SetUserData(data:Dynamic):Void{
		m_userData = data;
	}

	/**
	 * Short-cut function to determine if either body is inactive.
	 * @return
	 */
	public function IsActive():Bool {
		return m_bodyA.IsActive() && m_bodyB.IsActive();
	}
	
	//--------------- Internals Below -------------------

	public static function Create(def:B2JointDef, allocator:Dynamic):B2Joint{
		var joint:B2Joint = null;
		
		switch (def.type)
		{
		case e_distanceJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2DistanceJoint));
				joint = new B2DistanceJoint(cast( def, B2DistanceJointDef));
			}
		
		case e_mouseJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2MouseJoint));
				joint = new B2MouseJoint(cast( def, B2MouseJointDef));
			}
		
		case e_prismaticJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2PrismaticJoint));
				joint = new B2PrismaticJoint(cast( def, B2PrismaticJointDef));
			}
		
		case e_revoluteJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2RevoluteJoint));
				joint = new B2RevoluteJoint(cast( def, B2RevoluteJointDef));
			}
		
		case e_pulleyJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2PulleyJoint));
				joint = new B2PulleyJoint(cast( def, B2PulleyJointDef));
			}
		
		case e_gearJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2GearJoint));
				joint = new B2GearJoint(cast( def, B2GearJointDef));
			}
		
		case e_lineJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2LineJoint));
				joint = new B2LineJoint(cast( def, B2LineJointDef));
			}
			
		case e_weldJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2WeldJoint));
				joint = new B2WeldJoint(cast( def, B2WeldJointDef));
			}
			
		case e_frictionJoint:
			{
				//void* mem = allocator->Allocate(sizeof(B2FrictionJoint));
				joint = new B2FrictionJoint(cast( def, B2FrictionJointDef));
			}
			
		default:
			//B2Settings.b2Assert(false);
		}
		
		return joint;
	}
	
	public static function Destroy(joint:B2Joint, allocator:Dynamic) : Void{
		/*joint->~B2Joint();
		switch (joint.m_type)
		{
		case e_distanceJoint:
			allocator->Free(joint, sizeof(B2DistanceJoint));
			break;
		
		case e_mouseJoint:
			allocator->Free(joint, sizeof(B2MouseJoint));
			break;
		
		case e_prismaticJoint:
			allocator->Free(joint, sizeof(B2PrismaticJoint));
			break;
		
		case e_revoluteJoint:
			allocator->Free(joint, sizeof(B2RevoluteJoint));
			break;
		
		case e_pulleyJoint:
			allocator->Free(joint, sizeof(B2PulleyJoint));
			break;
		
		case e_gearJoint:
			allocator->Free(joint, sizeof(B2GearJoint));
			break;
		
		case e_lineJoint:
			allocator->Free(joint, sizeof(B2LineJoint));
			break;
			
		case e_weldJoint:
			allocator->Free(joint, sizeof(B2WeldJoint));
			break;
			
		case e_frictionJoint:
			allocator->Free(joint, sizeof(B2FrictionJoint));
			break;
		
		default:
			b2Assert(false);
			break;
		}*/
	}

	/** @private */
	public function new(def:B2JointDef) {
		m_localCenterA = new B2Vec2();
		m_localCenterB = new B2Vec2();
		m_edgeA = new B2JointEdge();
		m_edgeB = new B2JointEdge();
		B2Settings.b2Assert(def.bodyA != def.bodyB);
		m_type = def.type;
		m_prev = null;
		m_next = null;
		m_bodyA = def.bodyA;
		m_bodyB = def.bodyB;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		m_userData = def.userData;
	}
	//~B2Joint() {}

	public function InitVelocityConstraints(step:B2TimeStep) : Void{}
	public function SolveVelocityConstraints(step:B2TimeStep) : Void { }
	public function FinalizeVelocityConstraints() : Void{}

	// This returns true if the position errors are within tolerance.
	public function SolvePositionConstraints(baumgarte:Float):Bool { return false; }

	public var m_type:Int;
	public var m_prev:B2Joint;
	public var m_next:B2Joint;
	public var m_edgeA:B2JointEdge ;
	public var m_edgeB:B2JointEdge ;
	public var m_bodyA:B2Body;
	public var m_bodyB:B2Body;

	public var m_islandFlag:Bool;
	public var m_collideConnected:Bool;

	public var m_userData:Dynamic;
	
	// Cache here per time step to reduce cache misses.
	public var m_localCenterA:B2Vec2 ;
	public var m_localCenterB:B2Vec2 ;
	public var m_invMassA:Float;
	public var m_invMassB:Float;
	public var m_invIA:Float;
	public var m_invIB:Float;
	
	// ENUMS
	
	// enum B2JointType
	public static var e_unknownJoint:Int = 0;
	public static var e_revoluteJoint:Int = 1;
	public static var e_prismaticJoint:Int = 2;
	public static var e_distanceJoint:Int = 3;
	public static var e_pulleyJoint:Int = 4;
	public static var e_mouseJoint:Int = 5;
	public static var e_gearJoint:Int = 6;
	public static var e_lineJoint:Int = 7;
	public static var e_weldJoint:Int = 8;
	public static var e_frictionJoint:Int = 9;

	// enum B2LimitState
	public static var e_inactiveLimit:Int = 0;
	public static var e_atLowerLimit:Int = 1;
	public static var e_atUpperLimit:Int = 2;
	public static var e_equalLimits:Int = 3;
	
}



