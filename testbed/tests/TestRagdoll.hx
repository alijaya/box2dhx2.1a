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
import box2D.dynamics.joints.B2RevoluteJointDef;

class TestRagdoll extends Test
{
	
	
	public function new(main:Main)
	{
		super(main);

		main.aboutText.text = "Ragdolls";
		
		var circ:B2CircleShape; 
		var box:B2PolygonShape;
		var head:B2Body;
		var bd:B2BodyDef = new B2BodyDef();
		var jd:B2RevoluteJointDef = new B2RevoluteJointDef();
		var fixtureDef:B2FixtureDef = new B2FixtureDef();
		
		// Add 5 ragdolls along the top
		for (i in 0...2){
			var startX:Float = 70 + Math.random() * 20 + 480 * i;
			var startY:Float = 20 + Math.random() * 50;
			
			// BODIES
			// Set these to dynamic bodies
			bd.type = B2Body.b2_dynamicBody;
			
			// Head
			circ = new B2CircleShape( 12.5 / scale );
			fixtureDef.shape = circ;
			fixtureDef.density = 1.0;
			fixtureDef.friction = 0.4;
			fixtureDef.restitution = 0.3;
			bd.position.Set(startX / scale, startY / scale);
			head = world.CreateBody(bd);
			head.CreateFixture(fixtureDef);
			//if (i == 0){
				head.ApplyImpulse(new B2Vec2(Math.random() * 100 - 50, Math.random() * 100 - 50), head.GetWorldCenter());
			//}
			
			// Torso1
			box = new B2PolygonShape();
			box.SetAsBox(15 / scale, 10 / scale);
			fixtureDef.shape = box;
			fixtureDef.density = 1.0;
			fixtureDef.friction = 0.4;
			fixtureDef.restitution = 0.1;
			bd.position.Set(startX / scale, (startY + 28) / scale);
			var torso1:B2Body = world.CreateBody(bd);
			torso1.CreateFixture(fixtureDef);
			// Torso2
			box = new B2PolygonShape();
			box.SetAsBox(15 / scale, 10 / scale);
			fixtureDef.shape = box;
			bd.position.Set(startX / scale, (startY + 43) / scale);
			var torso2:B2Body = world.CreateBody(bd);
			torso2.CreateFixture(fixtureDef);
			// Torso3
			box.SetAsBox(15 / scale, 10 / scale);
			fixtureDef.shape = box;
			bd.position.Set(startX / scale, (startY + 58) / scale);
			var torso3:B2Body = world.CreateBody(bd);
			torso3.CreateFixture(fixtureDef);
			
			// UpperArm
			fixtureDef.density = 1.0;
			fixtureDef.friction = 0.4;
			fixtureDef.restitution = 0.1;
			// L
			box = new B2PolygonShape();
			box.SetAsBox(18 / scale, 6.5 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX - 30) / scale, (startY + 20) / scale);
			var upperArmL:B2Body = world.CreateBody(bd);
			upperArmL.CreateFixture(fixtureDef);
			// R
			box = new B2PolygonShape();
			box.SetAsBox(18 / scale, 6.5 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX + 30) / scale, (startY + 20) / scale);
			var upperArmR:B2Body = world.CreateBody(bd);
			upperArmR.CreateFixture(fixtureDef);
			
			// LowerArm
			fixtureDef.density = 1.0;
			fixtureDef.friction = 0.4;
			fixtureDef.restitution = 0.1;
			// L
			box = new B2PolygonShape();
			box.SetAsBox(17 / scale, 6 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX - 57) / scale, (startY + 20) / scale);
			var lowerArmL:B2Body = world.CreateBody(bd);
			lowerArmL.CreateFixture(fixtureDef);
			// R
			box = new B2PolygonShape();
			box.SetAsBox(17 / scale, 6 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX + 57) / scale, (startY + 20) / scale);
			var lowerArmR:B2Body = world.CreateBody(bd);
			lowerArmR.CreateFixture(fixtureDef);
			
			// UpperLeg
			fixtureDef.density = 1.0;
			fixtureDef.friction = 0.4;
			fixtureDef.restitution = 0.1;
			// L
			box = new B2PolygonShape();
			box.SetAsBox(7.5 / scale, 22 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX - 8) / scale, (startY + 85) / scale);
			var upperLegL:B2Body = world.CreateBody(bd);
			upperLegL.CreateFixture(fixtureDef);
			// R
			box = new B2PolygonShape();
			box.SetAsBox(7.5 / scale, 22 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX + 8) / scale, (startY + 85) / scale);
			var upperLegR:B2Body = world.CreateBody(bd);
			upperLegR.CreateFixture(fixtureDef);
			
			// LowerLeg
			fixtureDef.density = 1.0;
			fixtureDef.friction = 0.4;
			fixtureDef.restitution = 0.1;
			// L
			box = new B2PolygonShape();
			box.SetAsBox(6 / scale, 20 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX - 8) / scale, (startY + 120) / scale);
			var lowerLegL:B2Body = world.CreateBody(bd);
			lowerLegL.CreateFixture(fixtureDef);
			// R
			box = new B2PolygonShape();
			box.SetAsBox(6 / scale, 20 / scale);
			fixtureDef.shape = box;
			bd.position.Set((startX + 8) / scale, (startY + 120) / scale);
			var lowerLegR:B2Body = world.CreateBody(bd);
			lowerLegR.CreateFixture(fixtureDef);
			
			
			// JOINTS
			jd.enableLimit = true;
			
			// Head to shoulders
			jd.lowerAngle = -40 / (180/Math.PI);
			jd.upperAngle = 40 / (180/Math.PI);
			jd.Initialize(torso1, head, new B2Vec2(startX / scale, (startY + 15) / scale));
			world.CreateJoint(jd);
			
			// Upper arm to shoulders
			// L
			jd.lowerAngle = -85 / (180/Math.PI);
			jd.upperAngle = 130 / (180/Math.PI);
			jd.Initialize(torso1, upperArmL, new B2Vec2((startX - 18) / scale, (startY + 20) / scale));
			world.CreateJoint(jd);
			// R
			jd.lowerAngle = -130 / (180/Math.PI);
			jd.upperAngle = 85 / (180/Math.PI);
			jd.Initialize(torso1, upperArmR, new B2Vec2((startX + 18) / scale, (startY + 20) / scale));
			world.CreateJoint(jd);
			
			// Lower arm to upper arm
			// L
			jd.lowerAngle = -130 / (180/Math.PI);
			jd.upperAngle = 10 / (180/Math.PI);
			jd.Initialize(upperArmL, lowerArmL, new B2Vec2((startX - 45) / scale, (startY + 20) / scale));
			world.CreateJoint(jd);
			// R
			jd.lowerAngle = -10 / (180/Math.PI);
			jd.upperAngle = 130 / (180/Math.PI);
			jd.Initialize(upperArmR, lowerArmR, new B2Vec2((startX + 45) / scale, (startY + 20) / scale));
			world.CreateJoint(jd);
			
			// Shoulders/stomach
			jd.lowerAngle = -15 / (180/Math.PI);
			jd.upperAngle = 15 / (180/Math.PI);
			jd.Initialize(torso1, torso2, new B2Vec2(startX / scale, (startY + 35) / scale));
			world.CreateJoint(jd);
			// Stomach/hips
			jd.Initialize(torso2, torso3, new B2Vec2(startX / scale, (startY + 50) / scale));
			world.CreateJoint(jd);
			
			// Torso to upper leg
			// L
			jd.lowerAngle = -25 / (180/Math.PI);
			jd.upperAngle = 45 / (180/Math.PI);
			jd.Initialize(torso3, upperLegL, new B2Vec2((startX - 8) / scale, (startY + 72) / scale));
			world.CreateJoint(jd);
			// R
			jd.lowerAngle = -45 / (180/Math.PI);
			jd.upperAngle = 25 / (180/Math.PI);
			jd.Initialize(torso3, upperLegR, new B2Vec2((startX + 8) / scale, (startY + 72) / scale));
			world.CreateJoint(jd);
			
			// Upper leg to lower leg
			// L
			jd.lowerAngle = -25 / (180/Math.PI);
			jd.upperAngle = 115 / (180/Math.PI);
			jd.Initialize(upperLegL, lowerLegL, new B2Vec2((startX - 8) / scale, (startY + 105) / scale));
			world.CreateJoint(jd);
			// R
			jd.lowerAngle = -115 / (180/Math.PI);
			jd.upperAngle = 25 / (180/Math.PI);
			jd.Initialize(upperLegR, lowerLegR, new B2Vec2((startX + 8) / scale, (startY + 105) / scale));
			world.CreateJoint(jd);
			
		}
		
		
		// Add stairs on the left, these are static bodies so set the type accordingly
		bd.type = B2Body.b2_staticBody;
		fixtureDef.density = 0.0;
		fixtureDef.friction = 0.4;
		fixtureDef.restitution = 0.3;
		for (j in 1...11) {
			box = new B2PolygonShape();
			box.SetAsBox((10*j) / scale, 10 / scale);
			fixtureDef.shape = box;
			bd.position.Set((10*j) / scale, (150 + 20*j) / scale);
			head = world.CreateBody(bd);
			head.CreateFixture(fixtureDef);
		}
		
		// Add stairs on the right
		for (k in 1...11){
			box = new B2PolygonShape();
			box.SetAsBox((10 * k) / scale, 10 / scale);
			fixtureDef.shape = box;
			bd.position.Set((640-10*k) / scale, (150 + 20*k) / scale);
			head = world.CreateBody(bd);
			head.CreateFixture(fixtureDef);
		}
		
		box = new B2PolygonShape();
		box.SetAsBox(30 / scale, 40 / scale);
		fixtureDef.shape = box;
		bd.position.Set(320 / scale, 320 / scale);
		head = world.CreateBody(bd);
		head.CreateFixture(fixtureDef);
	}
}
