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

import box2D.common.math.B2Math;
import box2D.common.math.B2Vec2;
import box2D.common.math.B2Mat22;

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/**
 * Friction joint. This is used for top-down friction.
 * It provides 2D translational friction and angular friction.
 * @see B2FrictionJointDef
 */
class B2FrictionJoint extends B2Joint {
	/** @inheritDoc */
	
	/** @inheritDoc */
	public override function GetAnchorA():B2Vec2{
		return m_bodyA.GetWorldPoint(m_localAnchorA);
	}
	/** @inheritDoc */
	public override function GetAnchorB():B2Vec2{
		return m_bodyB.GetWorldPoint(m_localAnchorB);
	}
	
	/** @inheritDoc */
	public override function GetReactionForce(inv_dt:Float):B2Vec2
	{
		return new B2Vec2(inv_dt * m_linearImpulse.x, inv_dt * m_linearImpulse.y);
	}

	/** @inheritDoc */
	public override function GetReactionTorque(inv_dt:Float):Float
	{
		//B2_NOT_USED(inv_dt);
		return inv_dt * m_angularImpulse;
	}
	
	public function SetMaxForce(force:Float):Void
	{
		m_maxForce = force;
	}
	
	public function GetMaxForce():Float
	{
		return m_maxForce;
	}
	
	public function SetMaxTorque(torque:Float):Void
	{
		m_maxTorque = torque;
	}
	
	public function GetMaxTorque():Float
	{
		return m_maxTorque;
	}
	
	//--------------- Internals Below -------------------

	/** @private */
	public function new(def:B2FrictionJointDef){
		m_localAnchorA = new B2Vec2();
		m_localAnchorB = new B2Vec2();
		m_linearMass = new B2Mat22();
		m_linearImpulse = new B2Vec2();
		super(def);
		
		m_localAnchorA.SetV(def.localAnchorA);
		m_localAnchorB.SetV(def.localAnchorB);
		
		m_linearMass.SetZero();
		m_angularMass = 0.0;
		
		m_linearImpulse.SetZero();
		m_angularImpulse = 0.0;
		
		m_maxForce = def.maxForce;
		m_maxTorque = def.maxTorque;
	}

	override function InitVelocityConstraints(step:B2TimeStep) : Void {
		var tMat:B2Mat22;
		var tX:Float;
		
		var bA:B2Body = m_bodyA;
		var bB:B2Body= m_bodyB;

		// Compute the effective mass matrix.
		//B2Vec2 rA = B2Mul(bA->m_xf.R, m_localAnchorA - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		var rAX:Float = m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY:Float = m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		//B2Vec2 rB = B2Mul(bB->m_xf.R, m_localAnchorB - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		var rBX:Float = m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY:Float = m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		var mA:Float = bA.m_invMass;
		var mB:Float = bB.m_invMass;
		var iA:Float = bA.m_invI;
		var iB:Float = bB.m_invI;

		var K:B2Mat22 = new B2Mat22();
		K.col1.x = mA + mB;	K.col2.x = 0.0;
		K.col1.y = 0.0;		K.col2.y = mA + mB;

		K.col1.x+=  iA * rAY * rAY;	K.col2.x+= -iA * rAX * rAY;
		K.col1.y+= -iA * rAX * rAY;	K.col2.y+=  iA * rAX * rAX;

		K.col1.x+=  iB * rBY * rBY;	K.col2.x+= -iB * rBX * rBY;
		K.col1.y+= -iB * rBX * rBY;	K.col2.y+=  iB * rBX * rBX;

		K.GetInverse(m_linearMass);

		m_angularMass = iA + iB;
		if (m_angularMass > 0.0)
		{
			m_angularMass = 1.0 / m_angularMass;
		}

		if (step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			m_linearImpulse.x *= step.dtRatio;
			m_linearImpulse.y *= step.dtRatio;
			m_angularImpulse *= step.dtRatio;

			var P:B2Vec2 = m_linearImpulse;

			bA.m_linearVelocity.x -= mA * P.x;
			bA.m_linearVelocity.y -= mA * P.y;
			bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + m_angularImpulse);

			bB.m_linearVelocity.x += mB * P.x;
			bB.m_linearVelocity.y += mB * P.y;
			bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + m_angularImpulse);
		}
		else
		{
			m_linearImpulse.SetZero();
			m_angularImpulse = 0.0;
		}

	}
	
	
	
	override function SolveVelocityConstraints(step:B2TimeStep): Void{
		//B2_NOT_USED(step);
		var tMat:B2Mat22;
		var tX:Float;

		var bA:B2Body = m_bodyA;
		var bB:B2Body= m_bodyB;

		var vA:B2Vec2 = bA.m_linearVelocity;
		var wA:Float = bA.m_angularVelocity;
		var vB:B2Vec2 = bB.m_linearVelocity;
		var wB:Float = bB.m_angularVelocity;

		var mA:Float = bA.m_invMass;
		var mB:Float = bB.m_invMass;
		var iA:Float = bA.m_invI;
		var iB:Float = bB.m_invI;

		//B2Vec2 rA = B2Mul(bA->m_xf.R, m_localAnchorA - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		var rAX:Float = m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY:Float = m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		//B2Vec2 rB = B2Mul(bB->m_xf.R, m_localAnchorB - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		var rBX:Float = m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY:Float = m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;
		
		var maxImpulse:Float;

		// Solve angular friction
		{
			var Cdot:Float = wB - wA;
			var impulse:Float = -m_angularMass * Cdot;

			var oldImpulse:Float = m_angularImpulse;
			maxImpulse = step.dt * m_maxTorque;
			m_angularImpulse = B2Math.Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve linear friction
		{
			//B2Vec2 Cdot = vB + B2Cross(wB, rB) - vA - B2Cross(wA, rA);
			var CdotX:Float = vB.x - wB * rBY - vA.x + wA * rAY;
			var CdotY:Float = vB.y + wB * rBX - vA.y - wA * rAX;

			var impulseV:B2Vec2 = B2Math.MulMV(m_linearMass, new B2Vec2(-CdotX, -CdotY));
			var oldImpulseV:B2Vec2 = m_linearImpulse.Copy();
			
			m_linearImpulse.Add(impulseV);

			maxImpulse = step.dt * m_maxForce;

			if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				m_linearImpulse.Normalize();
				m_linearImpulse.Multiply(maxImpulse);
			}

			impulseV = B2Math.SubtractVV(m_linearImpulse, oldImpulseV);

			vA.x -= mA * impulseV.x;
			vA.y -= mA * impulseV.y;
			wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);

			vB.x += mB * impulseV.x;
			vB.y += mB * impulseV.y;
			wB += iB * (rBX * impulseV.y - rBY * impulseV.x);
		}

		// References has made some sets unnecessary
		//bA->m_linearVelocity = vA;
		bA.m_angularVelocity = wA;
		//bB->m_linearVelocity = vB;
		bB.m_angularVelocity = wB;

	}
	
	override function SolvePositionConstraints(baumgarte:Float):Bool
	{
		//B2_NOT_USED(baumgarte);
		
		return true;
		
	}

	public var m_localAnchorA:B2Vec2 ;
	public var m_localAnchorB:B2Vec2 ;
	
	public var m_linearMass:B2Mat22 ;
	public var m_angularMass:Float;
	
	public var m_linearImpulse:B2Vec2 ;
	public var m_angularImpulse:Float;
	
	public var m_maxForce:Float;
	public var m_maxTorque:Float;
}

