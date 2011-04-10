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

class TestBridge extends Test
{
	
	
	public function new(main:Main)
	{
		super(main);

		main.aboutText.text = "Bridge";
		
		var ground:B2Body = world.GetGroundBody();
		var anchor:B2Vec2 = new B2Vec2();
		var body:B2Body;
		
		// Bridge
		var sd:B2PolygonShape = new B2PolygonShape();
		var fixtureDef:B2FixtureDef = new B2FixtureDef();
		sd.SetAsBox(24 / scale, 5 / scale);
		fixtureDef.shape = sd;
		fixtureDef.density = 20.0;
		fixtureDef.friction = 0.2;
		
		var bd:B2BodyDef = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		
		var jd:B2RevoluteJointDef = new B2RevoluteJointDef();
		var numPlanks = 10;
		jd.lowerAngle = -15 / (180/Math.PI);
		jd.upperAngle = 15 / (180/Math.PI);
		jd.enableLimit = true;
		
		var prevBody:B2Body = ground;
		for (i in 0...numPlanks)
		{
			bd.position.Set((100 + 22 + 44 * i) / scale, 250 / scale);
			body = world.CreateBody(bd);
			body.CreateFixture(fixtureDef);
			
			anchor.Set((100 + 44 * i) / scale, 250 / scale);
			jd.Initialize(prevBody, body, anchor);
			world.CreateJoint(jd);
			
			prevBody = body;
		}
		
		anchor.Set((100 + 44 * numPlanks) / scale, 250 / scale);
		jd.Initialize(prevBody, ground, anchor);
		world.CreateJoint(jd);
		
		
		// Spawn in a bunch of crap
		for (i in 0...5){
			var bodyDef:B2BodyDef = new B2BodyDef();
			bodyDef.type = B2Body.b2_dynamicBody;
			var boxShape:B2PolygonShape = new B2PolygonShape();
			fixtureDef.shape = boxShape;
			fixtureDef.density = 1.0;
			// Override the default friction.
			fixtureDef.friction = 0.3;
			fixtureDef.restitution = 0.1;
			boxShape.SetAsBox((Math.random() * 5 + 10) / scale, (Math.random() * 5 + 10) / scale);
			bodyDef.position.Set((Math.random() * 400 + 120) / scale, (Math.random() * 150 + 50) / scale);
			bodyDef.angle = Math.random() * Math.PI;
			body = world.CreateBody(bodyDef);
			body.CreateFixture(fixtureDef);
			
		}
		for (i in 0...5){
			var bodyDefC:B2BodyDef = new B2BodyDef();
			bodyDefC.type = B2Body.b2_dynamicBody;
			var circShape:B2CircleShape = new B2CircleShape((Math.random() * 5 + 10) / scale);
			fixtureDef.shape = circShape;
			fixtureDef.density = 1.0;
			// Override the default friction.
			fixtureDef.friction = 0.3;
			fixtureDef.restitution = 0.1;
			bodyDefC.position.Set((Math.random() * 400 + 120) / scale, (Math.random() * 150 + 50) / scale);
			bodyDefC.angle = Math.random() * Math.PI;
			body = world.CreateBody(bodyDefC);
			body.CreateFixture(fixtureDef);
			
		}
		for (i in 0...15){
			var bodyDefP:B2BodyDef = new B2BodyDef();
			bodyDefP.type = B2Body.b2_dynamicBody;
			var polyShape:B2PolygonShape = new B2PolygonShape();
			var vertices:Array<B2Vec2> = [];
			var vertexCount:Int;
			if (Math.random() > 0.66){
				vertexCount = 4;
				for ( j in 0...vertexCount )
				{
					vertices[j] = new B2Vec2();
				}
				vertices[0].Set((-10 -Math.random()*10) / scale, ( 10 +Math.random()*10) / scale);
				vertices[1].Set(( -5 -Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
				vertices[2].Set((  5 +Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
				vertices[3].Set(( 10 +Math.random()*10) / scale, ( 10 +Math.random()*10) / scale);
			}
			else if (Math.random() > 0.5){
				vertexCount = 5;
				for ( j in 0...vertexCount )
				{
					vertices[j] = new B2Vec2();
				}
				vertices[0].Set(0, (10 +Math.random()*10) / scale);
				vertices[2].Set((-5 -Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
				vertices[3].Set(( 5 +Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
				vertices[1].Set((vertices[0].x + vertices[2].x), (vertices[0].y + vertices[2].y));
				vertices[1].Multiply(Math.random()/2+0.8);
				vertices[4].Set((vertices[3].x + vertices[0].x), (vertices[3].y + vertices[0].y));
				vertices[4].Multiply(Math.random()/2+0.8);
			}
			else{
				vertexCount = 3;
				for ( j in 0...vertexCount )
				{
					vertices[j] = new B2Vec2();
				}
				vertices[0].Set(0, (10 +Math.random()*10) / scale);
				vertices[1].Set((-5 -Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
				vertices[2].Set(( 5 +Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
			}
			polyShape.SetAsArray( vertices, vertexCount );
			fixtureDef.shape = polyShape;
			fixtureDef.density = 1.0;
			fixtureDef.friction = 0.3;
			fixtureDef.restitution = 0.1;
			bodyDefP.position.Set((Math.random() * 400 + 120) / scale, (Math.random() * 150 + 50) / scale);
			bodyDefP.angle = Math.random() * Math.PI;
			body = world.CreateBody(bodyDefP);
			body.CreateFixture(fixtureDef);
		}
	}
}
