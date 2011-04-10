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

class TestCCD extends Test
{
	
	
	public function new(main:Main)
	{
		super(main);

		main.aboutText.text = "Continuous Collision Detection (Off)";
		
		var bd:B2BodyDef;
		var body:B2Body;
		var fixtureDef = new B2FixtureDef();
		fixtureDef.density = 4.0; 
		fixtureDef.restitution = 1.4;
		
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.bullet = true;
		bd.position.Set( 150.0/scale, 100.0/scale );
		body = world.CreateBody(bd);
		var sd_bottom = new B2PolygonShape();
		sd_bottom.SetAsBox( 45.0 / scale, 4.5 / scale );
		fixtureDef.shape = sd_bottom;
		body.CreateFixture( fixtureDef );
		
		var sd_left = new B2PolygonShape();
		sd_left.SetAsOrientedBox(4.5/scale, 81.0/scale, new B2Vec2(-43.5/scale, -70.5/scale), -0.2);
		fixtureDef.shape = sd_left;
		body.CreateFixture( fixtureDef );
		
		var sd_right = new B2PolygonShape();
		sd_right.SetAsOrientedBox(4.5/scale, 81.0/scale, new B2Vec2(43.5/scale, -70.5/scale), 0.2);
		fixtureDef.shape = sd_right;
		body.CreateFixture( fixtureDef );
		
		for (i in 0...5){
			var cd = new B2CircleShape((Math.random() * 10 + 5) / scale);
			fixtureDef.shape = cd;
			fixtureDef.friction = 0.3;
			fixtureDef.density = 1.0;
			fixtureDef.restitution = 1.1;
			bd = new B2BodyDef();
			bd.type = B2Body.b2_dynamicBody;
			bd.bullet = true;
			bd.position.Set( (Math.random()*300 + 250)/scale, (Math.random()*320 + 20)/scale );
			body = world.CreateBody(bd);
			body.CreateFixture(fixtureDef);
		}
	}
}
