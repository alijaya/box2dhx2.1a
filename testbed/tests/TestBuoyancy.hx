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
import box2D.dynamics.controllers.B2Controller;
import box2D.dynamics.controllers.B2BuoyancyController;

class TestBuoyancy extends Test
{

	private var bodies:Array<B2Body>;
	private var controller:B2Controller;
	
	public function new(main:Main)
	{
		bodies = [];
		super(main);

		main.aboutText.text = "Buoyancy";

		var bc:B2BuoyancyController = new B2BuoyancyController();
		controller = bc;
		
		bc.normal.Set(0,-1);
		bc.offset = -200 / scale;
		bc.density = 2.0;
		bc.linearDrag = 5;
		bc.angularDrag = 2;
		
		var ground:B2Body = world.GetGroundBody();
		var anchor:B2Vec2 = new B2Vec2();
		var body:B2Body;
		var fd:B2FixtureDef;
		var bodyDef:B2BodyDef = null;
		var boxDef:B2PolygonShape = null;
		var circDef:B2CircleShape = null;
		var polyDef:B2PolygonShape = null;
		
		// Spawn in a bunch of crap
		for (i in 0...5){
			bodyDef = new B2BodyDef();
			bodyDef.type = B2Body.b2_dynamicBody;
			//bodyDef.isBullet = true;
			boxDef = new B2PolygonShape();
			fd = new B2FixtureDef();
			fd.shape = boxDef;
			fd.density = 1.0;
			// Override the default friction.
			fd.friction = 0.3;
			fd.restitution = 0.1;
			boxDef.SetAsBox((Math.random() * 5 + 10) / scale, (Math.random() * 5 + 10) / scale);
			bodyDef.position.Set((Math.random() * 400 + 120) / scale, (Math.random() * 150 + 50) / scale);
			bodyDef.angle = Math.random() * Math.PI;
			body = world.CreateBody(bodyDef);
			body.CreateFixture(fd);
			bodies.push(body);
			
		}
		for (i in 0...5){
			var bodyDefC:B2BodyDef = new B2BodyDef();
			bodyDefC.type = B2Body.b2_dynamicBody;
			//bodyDefC.isBullet = true;
			circDef = new B2CircleShape((Math.random() * 5 + 10) / scale);
			fd = new B2FixtureDef();
			fd.shape = circDef;
			fd.density = 1.0;
			// Override the default friction.
			fd.friction = 0.3;
			fd.restitution = 0.1;
			bodyDefC.position.Set((Math.random() * 400 + 120) / scale, (Math.random() * 150 + 50) / scale);
			bodyDefC.angle = Math.random() * Math.PI;
			body = world.CreateBody(bodyDefC);
			body.CreateFixture(fd);
			bodies.push(body);
		}
		for (i in 0...15){
			var bodyDefP:B2BodyDef = new B2BodyDef();
			bodyDefP.type = B2Body.b2_dynamicBody;
			//bodyDefP.isBullet = true;
			polyDef = new B2PolygonShape();
			if (Math.random() > 0.66) {
				polyDef.SetAsArray([
					new B2Vec2((-10 -Math.random()*10) / scale, ( 10 +Math.random()*10) / scale),
					new B2Vec2(( -5 -Math.random()*10) / scale, (-10 -Math.random()*10) / scale),
					new B2Vec2((  5 +Math.random()*10) / scale, (-10 -Math.random()*10) / scale),
					new B2Vec2(( 10 +Math.random() * 10) / scale, ( 10 +Math.random() * 10) / scale)
					]);
			}
			else if (Math.random() > 0.5) 
			{
				var array:Array<B2Vec2> = [];
				array[0] = new B2Vec2(0, (10 +Math.random()*10) / scale);
				array[2] = new B2Vec2((-5 -Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
				array[3] = new B2Vec2(( 5 +Math.random()*10) / scale, (-10 -Math.random()*10) / scale);
				array[1] = new B2Vec2((array[0].x + array[2].x), (array[0].y + array[2].y));
				array[1].Multiply(Math.random()/2+0.8);
				array[4] = new B2Vec2((array[3].x + array[0].x), (array[3].y + array[0].y));
				array[4].Multiply(Math.random() / 2 + 0.8);
				polyDef.SetAsArray(array);
			}
			else 
			{
				polyDef.SetAsArray([
					new B2Vec2(0, (10 +Math.random()*10) / scale),
					new B2Vec2((-5 -Math.random()*10) / scale, (-10 -Math.random()*10) / scale),
					new B2Vec2(( 5 +Math.random() * 10) / scale, ( -10 -Math.random() * 10) / scale)
				]);
			}
			fd = new B2FixtureDef();
			fd.shape = polyDef;
			fd.density = 1.0;
			fd.friction = 0.3;
			fd.restitution = 0.1;
			bodyDefP.position.Set((Math.random() * 400 + 120) / scale, (Math.random() * 150 + 50) / scale);
			bodyDefP.angle = Math.random() * Math.PI;
			body = world.CreateBody(bodyDefP);
			body.CreateFixture(fd);
			bodies.push(body);
		}
		
		//Add some exciting bath toys
		boxDef.SetAsBox(40 / scale, 10 / scale);
		fd = new B2FixtureDef();
		fd.shape = boxDef;
		fd.density = 3.0;
		bodyDef.position.Set(50 / scale, 300 / scale);
		bodyDef.angle = 0;
		body = world.CreateBody(bodyDef);
		body.CreateFixture(fd);
		bodies.push(body);
		
		bodyDef.position.Set(300/ scale, 300 / scale);
		body = world.CreateBody(bodyDef);
		circDef = new B2CircleShape(7 / scale);
		fd = new B2FixtureDef();
		fd.shape = circDef;
		fd.density =2;
		circDef.SetLocalPosition(new B2Vec2(30 / scale, 0 / scale));
		body.CreateFixture(fd);
		circDef.SetLocalPosition(new B2Vec2(-30 / scale, 0 / scale));
		body.CreateFixture(fd);
		circDef.SetLocalPosition(new B2Vec2(0 / scale, 30 / scale));
		body.CreateFixture(fd);
		circDef.SetLocalPosition(new B2Vec2(0 / scale, -30 / scale));
		body.CreateFixture(fd);
		
		fd = new B2FixtureDef();
		fd.shape = boxDef;
		fd.density = 2.0;
		boxDef.SetAsBox(30 / scale, 2 / scale);
		body.CreateFixture(fd);
		fd.density = 2.0;
		boxDef.SetAsBox(2 / scale, 30 / scale);
		body.CreateFixture(fd);
		bodies.push(body);
		
		for (body in bodies)
			controller.AddBody(body);
		world.AddController(controller);
		
	}

	public override function update()
	{
		super.update();
		//Draw water line
		sprite.graphics.lineStyle(1,0x0000ff,1);
		sprite.graphics.moveTo(5,200);
		sprite.graphics.lineTo(635,200);
		//It's not water without transparency...
		sprite.graphics.lineStyle();
		sprite.graphics.beginFill(0x0000ff,0.2);
		sprite.graphics.moveTo(5,200);
		sprite.graphics.lineTo(635,200);
		sprite.graphics.lineTo(635,355);
		sprite.graphics.lineTo(5,355);
		sprite.graphics.endFill();

	}
}
