package tests;

import box2D.collision.B2RayCastInput;
import box2D.collision.B2RayCastOutput;
import box2D.collision.shapes.B2PolygonShape;
import box2D.collision.shapes.B2CircleShape;
import box2D.common.math.B2Math;
import box2D.common.math.B2Vec2;
import box2D.common.math.B2Transform;
import box2D.dynamics.B2Body;
import box2D.dynamics.B2BodyDef;
import box2D.dynamics.B2Fixture;
import box2D.dynamics.B2FixtureDef;

class TestRaycast extends Test
{
	var laser : B2Body;
	
	public function new(main:Main)
	{
		super(main);

		main.aboutText.text = "Raycast";
		
		world.SetGravity(new B2Vec2(0,0));
		
		var ground = world.GetGroundBody();
		
		var box = new B2PolygonShape();
		box.SetAsBox(30 / scale, 4 / scale);
		var fd = new B2FixtureDef();
		fd.shape = box;
		fd.density = 4;
		fd.friction = 0.4;
		fd.restitution = 0.3;
		fd.userData="laser";
		var bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position.Set(320 / scale, 150 / scale);
		bd.position.Set(40 / scale, 150 / scale);
		laser = world.CreateBody(bd);
		laser.CreateFixture(fd);
		laser.SetAngle(0.5);
		laser.SetAngle(Math.PI);
		
		var circle = new B2CircleShape(30 / scale);
		fd.shape = circle;
		fd.density = 4;
		fd.friction = 0.4;
		fd.restitution = 0.3;
		fd.userData="circle";
		bd.position.Set(100 / scale, 100 / scale);
		var body = world.CreateBody(bd);
		body.CreateFixture(fd);
	}

	public override function update()
	{
		super.update();
		
		var p1 = laser.GetWorldPoint(new B2Vec2(30.1 / scale, 0));
		var p2 = laser.GetWorldPoint(new B2Vec2(130.1 / scale, 0));
		
		var f = world.RayCastOne(p1, p2);
		var lambda:Float = 1;
		if (f != null)
		{
			var input = new B2RayCastInput(p1, p2);
			var output = new B2RayCastOutput();
			f.RayCast(output, input);
			lambda = output.fraction;
		}
		sprite.graphics.lineStyle(1,0xff0000,1);
		sprite.graphics.moveTo(p1.x * scale, p1.y * scale);
		sprite.graphics.lineTo( 	(p2.x * lambda + (1 - lambda) * p1.x) * scale,
						(p2.y * lambda + (1 - lambda) * p1.y) * scale);
	}
}
