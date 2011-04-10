package tests;

import box2D.collision.shapes.B2PolygonShape;
import box2D.collision.shapes.B2CircleShape;
import box2D.common.math.B2Math;
import box2D.common.math.B2Vec2;
import box2D.common.math.B2Transform;
import box2D.dynamics.B2Body;
import box2D.dynamics.B2BodyDef;
import box2D.dynamics.B2Fixture;
import box2D.dynamics.B2FixtureDef;
import box2D.dynamics.joints.B2RevoluteJoint;
import box2D.dynamics.joints.B2PrismaticJoint;
import box2D.dynamics.joints.B2GearJoint;
import box2D.dynamics.joints.B2RevoluteJointDef;
import box2D.dynamics.joints.B2PrismaticJointDef;
import box2D.dynamics.joints.B2GearJointDef;
import box2D.dynamics.joints.B2PulleyJointDef;
import box2D.dynamics.joints.B2LineJointDef;
import box2D.dynamics.joints.B2FrictionJointDef;
import box2D.dynamics.joints.B2WeldJointDef;

class TestCrankGearsPulley extends Test
{
	private var joint1:B2RevoluteJoint;
	private var joint2:B2PrismaticJoint;
	
	public var gJoint1:B2RevoluteJoint;
	public var gJoint2:B2RevoluteJoint;
	public var gJoint3:B2PrismaticJoint;
	public var gJoint4:B2GearJoint;
	public var gJoint5:B2GearJoint;
	
	public function new(main:Main)
	{
		super(main);

		main.aboutText.text = "Joints";
		
		var ground:B2Body = world.GetGroundBody();
		
		var body:B2Body;
		var circleBody:B2Body;
		var sd:B2PolygonShape;
		var bd:B2BodyDef;
		var fixtureDef:B2FixtureDef = new B2FixtureDef();
		
		//
		// CRANK
		//
		
		// Define crank.
		sd = new B2PolygonShape();
		sd.SetAsBox(7.5 / scale, 30.0 / scale);
		fixtureDef.shape = sd;
		fixtureDef.density = 1.0;
		
		var rjd:B2RevoluteJointDef = new B2RevoluteJointDef();
		
		var prevBody:B2Body = ground;
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position.Set(100.0/scale, (360.0-105.0)/scale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		rjd.Initialize(prevBody, body, new B2Vec2(100.0/scale, (360.0-75.0)/scale));
		rjd.motorSpeed = 1.0 * -Math.PI;
		rjd.maxMotorTorque = 5000.0;
		rjd.enableMotor = true;
		joint1 = cast world.CreateJoint(rjd);
		
		prevBody = body;
		
		// Define follower.
		sd = new B2PolygonShape();
		sd.SetAsBox(7.5 / scale, 60.0 / scale);
		fixtureDef.shape = sd;
		bd.position.Set(100.0/scale, (360.0-195.0)/scale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		rjd.Initialize(prevBody, body, new B2Vec2(100.0/scale, (360.0-135.0)/scale));
		rjd.enableMotor = false;
		world.CreateJoint(rjd);
		
		prevBody = body;
		
		// Define piston
		sd = new B2PolygonShape();
		sd.SetAsBox(22.5 / scale, 22.5 / scale);
		fixtureDef.shape = sd;
		bd.position.Set(100.0/scale, (360.0-255.0)/scale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		rjd.Initialize(prevBody, body, new B2Vec2(100.0/scale, (360.0-255.0)/scale));
		world.CreateJoint(rjd);
		
		var pjd:B2PrismaticJointDef = new B2PrismaticJointDef();
		pjd.Initialize(ground, body, new B2Vec2(100.0/scale, (360.0-255.0)/scale), new B2Vec2(0.0, 1.0));
		
		pjd.maxMotorForce = 500.0;
		pjd.enableMotor = true;
		
		joint2 = cast world.CreateJoint(pjd);
		
		// Create a payload
		sd = new B2PolygonShape();
		sd.SetAsBox(22.5 / scale, 22.5 / scale);
		fixtureDef.shape = sd;
		fixtureDef.density = 2.0;
		bd.position.Set(100.0/scale, (360.0-345.0)/scale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		
		
		// 
		// GEARS
		//
		var circle1:B2CircleShape = new B2CircleShape(25 / scale);
		fixtureDef.shape = circle1;
		fixtureDef.density = 5.0;
		
		var bd1:B2BodyDef = new B2BodyDef();
		bd1.type = B2Body.b2_dynamicBody;
		bd1.position.Set(200 / scale, 360/2 / scale);
		var body1:B2Body = world.CreateBody(bd1);
		body1.CreateFixture(fixtureDef);
		
		var jd1:B2RevoluteJointDef = new B2RevoluteJointDef();
		jd1.Initialize(ground, body1, bd1.position);
		gJoint1 = cast world.CreateJoint(jd1);
		
		var circle2:B2CircleShape = new B2CircleShape(50 / scale);
		fixtureDef.shape = circle2;
		fixtureDef.density = 5.0;
		
		var bd2:B2BodyDef = new B2BodyDef();
		bd2.type = B2Body.b2_dynamicBody;
		bd2.position.Set(275 / scale, 360/2 / scale);
		var body2:B2Body = world.CreateBody(bd2);
		body2.CreateFixture(fixtureDef);
		
		var jd2:B2RevoluteJointDef = new B2RevoluteJointDef();
		jd2.Initialize(ground, body2, bd2.position);
		gJoint2 = cast world.CreateJoint(jd2);
		
		var box:B2PolygonShape = new B2PolygonShape();
		box.SetAsBox(10 / scale, 100 / scale);
		fixtureDef.shape = box;
		fixtureDef.density = 5.0;
		
		var bd3:B2BodyDef = new B2BodyDef();
		bd3.type = B2Body.b2_dynamicBody;
		bd3.position.Set(335 / scale, 360/2 / scale);
		var body3:B2Body = world.CreateBody(bd3);
		body3.CreateFixture(fixtureDef);
		
		var jd3:B2PrismaticJointDef = new B2PrismaticJointDef();
		jd3.Initialize(ground, body3, bd3.position, new B2Vec2(0,1));
		jd3.lowerTranslation = -25.0 / scale;
		jd3.upperTranslation = 100.0 / scale;
		jd3.enableLimit = true;
		
		gJoint3 = cast world.CreateJoint(jd3);
		
		var jd4:B2GearJointDef = new B2GearJointDef();
		jd4.bodyA = body1;
		jd4.bodyB = body2;
		jd4.joint1 = gJoint1;
		jd4.joint2 = gJoint2;
		jd4.ratio = circle2.GetRadius() / circle1.GetRadius();
		gJoint4 = cast world.CreateJoint(jd4);
		
		var jd5:B2GearJointDef = new B2GearJointDef();
		jd5.bodyA = body2;
		jd5.bodyB = body3;
		jd5.joint1 = gJoint2;
		jd5.joint2 = gJoint3;
		jd5.ratio = -1.0 / circle2.GetRadius();
		gJoint5 = cast world.CreateJoint(jd5);
		
		
		
		//
		// PULLEY
		//
		sd = new B2PolygonShape();
		sd.SetAsBox(50 / scale, 20 / scale);
		fixtureDef.shape = sd;
		fixtureDef.density = 5.0;
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		
		bd.position.Set(480 / scale, 200 / scale);
		body2 = world.CreateBody(bd);
		body2.CreateFixture(fixtureDef);
		
		var pulleyDef:B2PulleyJointDef = new B2PulleyJointDef();
		
		var anchor1:B2Vec2 = new B2Vec2(335 / scale, 180 / scale);
		var anchor2:B2Vec2 = new B2Vec2(480 / scale, 180 / scale);
		var groundAnchor1:B2Vec2 = new B2Vec2(335 / scale, 50 / scale);
		var groundAnchor2:B2Vec2 = new B2Vec2(480 / scale, 50 / scale);
		pulleyDef.Initialize(body3, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0);
		
		pulleyDef.maxLengthA = 200 / scale;
		pulleyDef.maxLengthB = 150 / scale;
		
		//joint1 = world.CreateJoint(pulleyDef) as B2PulleyJoint;
		world.CreateJoint(pulleyDef);
		
		
		// Add a circle to weigh down the pulley
		var circ:B2CircleShape = new B2CircleShape(40 / scale);
		fixtureDef.shape = circ;
		fixtureDef.friction = 0.3;
		fixtureDef.restitution = 0.3;
		fixtureDef.density = 5.0;
		bd.position.Set(485 / scale, 100 / scale);
		body1 = circleBody = world.CreateBody(bd);
		body1.CreateFixture(fixtureDef);
		
		//
		// LINE JOINT
		//
		sd = new B2PolygonShape();
		sd.SetAsBox(7.5 / scale, 30.0 / scale);
		fixtureDef.shape = sd;
		fixtureDef.density = 1.0;
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position.Set(500 / scale, 500/2 / scale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		var ljd:B2LineJointDef = new B2LineJointDef();
		ljd.Initialize(ground, body, body.GetPosition(), new B2Vec2(0.4, 0.6));
		
		ljd.lowerTranslation = -1;
		ljd.upperTranslation = 1;
		ljd.enableLimit = true;
		
		ljd.maxMotorForce = 1;
		ljd.motorSpeed = 0;
		ljd.enableMotor = true;
		
		world.CreateJoint(ljd);
		
		//
		// FRICTION JOINT
		//
		var fjd:B2FrictionJointDef = new B2FrictionJointDef();
		fjd.Initialize(circleBody, world.GetGroundBody(), circleBody.GetPosition());
		fjd.collideConnected = true;
		fjd.maxForce = 200;
		world.CreateJoint(fjd);
		
		//
		// WELD JOINT
		//
		// Not enabled as Weld joints are not encouraged compared with merging two bodies
		if(false)
		{
			var wjd:B2WeldJointDef = new B2WeldJointDef();
			wjd.Initialize(circleBody, body, circleBody.GetPosition());
			world.CreateJoint(wjd);
		}
	}
}
