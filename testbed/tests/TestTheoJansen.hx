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
import box2D.dynamics.joints.B2RevoluteJointDef;
import box2D.dynamics.joints.B2DistanceJointDef;

class TestTheoJansen extends Test
{
	private var tScale:Float;
	
	private var offset:B2Vec2;
	private var chassis:B2Body;
	private var wheel:B2Body;
	private var motorJoint:B2RevoluteJoint;
	private var motorOn:Bool;
	private var motorSpeed:Float;

	public function new(main:Main)
	{
		motorOn = true;
		offset = new B2Vec2();
		super(main);

		main.aboutText.text = "Theo Jansen Walker";
		
		tScale = scale * 2;
		
		offset.Set(120.0/scale, 250/scale);
		motorSpeed = -2.0;
		motorOn = true;
		var pivot:B2Vec2 = new B2Vec2(0.0, -24.0/tScale);
		
		var pd:B2PolygonShape;
		var cd:B2CircleShape;
		var fd:B2FixtureDef;
		var bd:B2BodyDef;
		var body:B2Body;
		
		for (i in 0...40)
		{
			cd = new B2CircleShape(7.5/tScale);
			
			bd = new B2BodyDef();
			bd.type = B2Body.b2_dynamicBody;
			bd.position.Set((Math.random() * 620 + 10)/scale, 350/scale);
			
			body = world.CreateBody(bd);
			body.CreateFixture2(cd, 1.0);
		}
		
		pd = new B2PolygonShape();
		pd.SetAsBox(75 / tScale, 30 / tScale);
		fd = new B2FixtureDef();
		fd.shape = pd;
		fd.density = 1.0;
		fd.filter.groupIndex = -1;
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position = B2Math.AddVV(pivot, offset);
		chassis = world.CreateBody(bd);
		chassis.CreateFixture(fd);
		
		cd = new B2CircleShape(48 / tScale);
		fd = new B2FixtureDef();
		fd.shape = cd;
		fd.density = 1.0;
		fd.filter.groupIndex = -1;
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position = B2Math.AddVV(pivot, offset);
		wheel = world.CreateBody(bd);
		wheel.CreateFixture(fd);
		
		var jd:B2RevoluteJointDef = new B2RevoluteJointDef();
		var po:B2Vec2 = pivot.Copy();
		po.Add(offset);
		jd.Initialize(wheel, chassis, po);
		jd.collideConnected = false;
		jd.motorSpeed = motorSpeed;
		jd.maxMotorTorque = 400.0;
		jd.enableMotor = motorOn;
		motorJoint = cast world.CreateJoint(jd);
		
		var wheelAnchor:B2Vec2;
		
		wheelAnchor = new B2Vec2(0.0, 24.0/tScale);
		wheelAnchor.Add(pivot);
		
		createLeg(-1.0, wheelAnchor);
		createLeg(1.0, wheelAnchor);
		
		wheel.SetPositionAndAngle(wheel.GetPosition(), 120.0 * Math.PI / 180.0);
		createLeg(-1.0, wheelAnchor);
		createLeg(1.0, wheelAnchor);
		
		wheel.SetPositionAndAngle(wheel.GetPosition(), -120.0 * Math.PI / 180.0);
		createLeg(-1.0, wheelAnchor);
		createLeg(1.0, wheelAnchor);
	}

	private function createLeg(s:Float, wheelAnchor:B2Vec2)
	{
		
		var p1:B2Vec2 = new B2Vec2(162 * s/tScale, 183/tScale);
		var p2:B2Vec2 = new B2Vec2(216 * s/tScale, 36 /tScale);
		var p3:B2Vec2 = new B2Vec2(129 * s/tScale, 57 /tScale);
		var p4:B2Vec2 = new B2Vec2( 93 * s/tScale, -24  /tScale);
		var p5:B2Vec2 = new B2Vec2(180 * s/tScale, -45  /tScale);
		var p6:B2Vec2 = new B2Vec2( 75 * s/tScale, -111 /tScale);
		
		//B2PolygonDef sd1, sd2;
		var sd1:B2PolygonShape = new B2PolygonShape();
		var sd2:B2PolygonShape = new B2PolygonShape();
		var fd1:B2FixtureDef = new B2FixtureDef();
		var fd2:B2FixtureDef = new B2FixtureDef();
		fd1.shape = sd1;
		fd2.shape = sd2;
		fd1.filter.groupIndex = -1;
		fd2.filter.groupIndex = -1;
		fd1.density = 1.0;
		fd2.density = 1.0;
		
		if (s > 0.0)
		{
			sd1.SetAsArray([p3, p2, p1]);
			sd2.SetAsArray([
				B2Math.SubtractVV(p6, p4),
				B2Math.SubtractVV(p5, p4),
				new B2Vec2()
				]);
		}
		else
		{
			sd1.SetAsArray([p2, p3, p1]);
			sd2.SetAsArray([
				B2Math.SubtractVV(p5, p4),
				B2Math.SubtractVV(p6, p4),
				new B2Vec2()
				]);
		}
		
		//B2BodyDef bd1, bd2;
		var bd1:B2BodyDef = new B2BodyDef();
		bd1.type = B2Body.b2_dynamicBody;
		var bd2:B2BodyDef = new B2BodyDef();
		bd2.type = B2Body.b2_dynamicBody;
		bd1.position.SetV(offset);
		bd2.position = B2Math.AddVV(p4, offset);
		
		bd1.angularDamping = 10.0;
		bd2.angularDamping = 10.0;
		
		var body1:B2Body = world.CreateBody(bd1);
		var body2:B2Body = world.CreateBody(bd2);
		
		body1.CreateFixture(fd1);
		body2.CreateFixture(fd2);
		
		var djd:B2DistanceJointDef = new B2DistanceJointDef();
		
		// Using a soft distance constraint can reduce some jitter.
		// It also makes the structure seem a bit more fluid by
		// acting like a suspension system.
		djd.dampingRatio = 0.5;
		djd.frequencyHz = 10.0;
		
		djd.Initialize(body1, body2, B2Math.AddVV(p2, offset), B2Math.AddVV(p5, offset));
		world.CreateJoint(djd);
		
		djd.Initialize(body1, body2, B2Math.AddVV(p3, offset), B2Math.AddVV(p4, offset));
		world.CreateJoint(djd);
		
		djd.Initialize(body1, wheel, B2Math.AddVV(p3, offset), B2Math.AddVV(wheelAnchor, offset));
		world.CreateJoint(djd);
		
		djd.Initialize(body2, wheel, B2Math.AddVV(p6, offset), B2Math.AddVV(wheelAnchor, offset));
		world.CreateJoint(djd);
		
		var rjd:B2RevoluteJointDef = new B2RevoluteJointDef();
		
		rjd.Initialize(body2, chassis, B2Math.AddVV(p4, offset));
		world.CreateJoint(rjd);
		
	}

	public override function update()
	{
		
		//case 'a':
		if (c.isPressed("A")){ // A
			chassis.SetAwake(true);
			motorJoint.SetMotorSpeed(-motorSpeed);
		}
		//case 's':
		if (c.isPressed("S")){ // S
			chassis.SetAwake(true);
			motorJoint.SetMotorSpeed(0.0);
		}
		//case 'd':
		if (c.isPressed("D")){ // D
			chassis.SetAwake(true);
			motorJoint.SetMotorSpeed(motorSpeed);
		}
		//case 'm':
		if (c.isPressed("M")){ // M
			chassis.SetAwake(true);
			motorJoint.EnableMotor(!motorJoint.IsMotorEnabled());
		}
		
		// Finally update super class
		super.update();
	}
	
}
