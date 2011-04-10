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

class TestCompound extends Test
{
	
	
	public function new(main:Main)
	{
		super(main);

		main.aboutText.text = "Compound Shapes";
		
		var bd:B2BodyDef;
		var body:B2Body;
		var x:Float;
		
		var cd1:B2CircleShape = new B2CircleShape();
		cd1.SetRadius(15.0/scale);
		cd1.SetLocalPosition(new B2Vec2( -15.0 / scale, 15.0 / scale));
		
		var cd2:B2CircleShape = new B2CircleShape();
		cd2.SetRadius(15.0/scale);
		cd2.SetLocalPosition(new B2Vec2(15.0 / scale, 15.0 / scale));
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		
		for (i in 0...5)
		{
			x = 320.0 + B2Math.RandomRange(-3.0, 3.0);
			bd.position.Set((x + 150.0)/scale, (31.5 - 75.0 * i + 300.0)/scale);
			bd.angle = B2Math.RandomRange(-Math.PI, Math.PI);
			body = world.CreateBody(bd);
			body.CreateFixture2(cd1, 2.0);
			body.CreateFixture2(cd2, 0.0);
		}
		
		var pd1:B2PolygonShape = new B2PolygonShape();
		pd1.SetAsBox(7.5/scale, 15.0/scale);
		
		var pd2:B2PolygonShape = new B2PolygonShape();
		pd2.SetAsOrientedBox(7.5/scale, 15.0/scale, new B2Vec2(0.0, -15.0/scale), 0.5 * Math.PI);
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		
		for (i in 0...5)
		{
			x = 320.0 + B2Math.RandomRange(-3.0, 3.0);
			bd.position.Set((x - 150.0)/scale, (31.5 - 75.0 * i + 300)/scale);
			bd.angle = B2Math.RandomRange(-Math.PI, Math.PI);
			body = world.CreateBody(bd);
			body.CreateFixture2(pd1, 2.0);
			body.CreateFixture2(pd2, 2.0);
		}
		
		var xf1:B2Transform = new B2Transform();
		xf1.R.Set(0.3524 * Math.PI);
		xf1.position = B2Math.MulMV(xf1.R, new B2Vec2(1.0, 0.0));
		
		var sd1:B2PolygonShape = new B2PolygonShape();
		sd1.SetAsArray([
			B2Math.MulX(xf1, new B2Vec2(-30.0/scale, 0.0)),
			B2Math.MulX(xf1, new B2Vec2(30.0/scale, 0.0)),
			B2Math.MulX(xf1, new B2Vec2(0.0, 15.0 / scale)),
			]);
		
		var xf2:B2Transform = new B2Transform();
		xf2.R.Set(-0.3524 * Math.PI);
		xf2.position = B2Math.MulMV(xf2.R, new B2Vec2(-30.0/scale, 0.0));
		
		var sd2:B2PolygonShape = new B2PolygonShape();
		sd2.SetAsArray([
			B2Math.MulX(xf2, new B2Vec2(-30.0/scale, 0.0)),
			B2Math.MulX(xf2, new B2Vec2(30.0/scale, 0.0)),
			B2Math.MulX(xf2, new B2Vec2(0.0, 15.0 / scale)),
			]);
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.fixedRotation = true;
		
		for (i in 0...5)
		{
			x = 320.0 + B2Math.RandomRange(-3.0, 3.0);
			bd.position.Set(x/scale, (-61.5 - 55.0 * i + 300)/scale);
			bd.angle = 0.0;
			body = world.CreateBody(bd);
			body.CreateFixture2(sd1, 2.0);
			body.CreateFixture2(sd2, 2.0);
		}
		
		var sd_bottom:B2PolygonShape = new B2PolygonShape();
		sd_bottom.SetAsBox( 45.0/scale, 4.5/scale );
		
		var sd_left:B2PolygonShape = new B2PolygonShape();
		sd_left.SetAsOrientedBox(4.5/scale, 81.0/scale, new B2Vec2(-43.5/scale, -70.5/scale), -0.2);
		
		var sd_right:B2PolygonShape = new B2PolygonShape();
		sd_right.SetAsOrientedBox(4.5/scale, 81.0/scale, new B2Vec2(43.5/scale, -70.5/scale), 0.2);
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position.Set( 320.0/scale, 300.0/scale );
		body = world.CreateBody(bd);
		body.CreateFixture2(sd_bottom, 4.0);
		body.CreateFixture2(sd_left, 4.0);
		body.CreateFixture2(sd_right, 4.0);
	}
}
