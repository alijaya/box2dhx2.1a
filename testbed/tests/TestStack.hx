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

class TestStack extends Test
{
	
	
	public function new(main:Main)
	{
		super(main);

		main.aboutText.text = "Stacked Boxes";
		
		var fd = new B2FixtureDef();
		var sd = new B2PolygonShape();
		var bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		var b:B2Body;
		fd.density = 1.0;
		fd.friction = 0.5;
		fd.restitution = 0.1;
		fd.shape = sd;
		
		for (i in 0...10){
			sd.SetAsBox((10) / scale, (10) / scale);
			//bd.position.Set((640/2+100+Math.random()*0.02 - 0.01) / m_physScale, (360-5-i*25) / m_physScale);
			bd.position.Set((640/2+100) / scale, (360-5-i*25) / scale);
			b = world.CreateBody(bd);
			b.CreateFixture(fd);
		}
		for (i in 0...10){
			sd.SetAsBox((10) / scale, (10) / scale);
			bd.position.Set((640/2-0+Math.random()*0.02 - 0.01) / scale, (360-5-i*25) / scale);
			b = world.CreateBody(bd);
			b.CreateFixture(fd);
		}
		for (i in 0...10){
			sd.SetAsBox((10) / scale, (10) / scale);
			bd.position.Set((640/2+200+Math.random()*0.02 - 0.01) / scale, (360-5-i*25) / scale);
			b = world.CreateBody(bd);
			b.CreateFixture(fd);
		}
		// Create ramp
		var vxs:Array<B2Vec2> = [new B2Vec2(0, 0),
			new B2Vec2(0, -100 / scale),
			new B2Vec2(200 /scale, 0)];
		sd.SetAsArray(vxs, vxs.length);
		fd.density = 0;
		bd.type = B2Body.b2_staticBody;
		bd.userData = "ramp";
		bd.position.Set(0, 360 / scale);
		b = world.CreateBody(bd);
		b.CreateFixture(fd);
		
		// Create ball
		var cd = new B2CircleShape();
		cd.SetRadius(40/scale);
		fd.density = 2;
		fd.restitution = 0.2;
		fd.friction = 0.5;
		fd.shape = cd;
		bd.type = B2Body.b2_dynamicBody;
		bd.userData = "ball";
		bd.position.Set(50/scale, 100 / scale);
		b = world.CreateBody(bd);
		b.CreateFixture(fd);
	}
}
