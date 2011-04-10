package tests;

import flash.display.Sprite;

import box2D.collision.shapes.B2PolygonShape;
import box2D.dynamics.joints.B2MouseJoint;
import box2D.dynamics.joints.B2MouseJointDef;
import box2D.dynamics.B2Body;
import box2D.dynamics.B2BodyDef;
import box2D.dynamics.B2Fixture;
import box2D.dynamics.B2World;
import box2D.dynamics.B2DebugDraw;
import box2D.common.math.B2Vec2;

import ctrl.Ctrl;

class Test 
{
	var sprite : Sprite;
	var world : B2World;
	var timeStep : Float;
	var scale : Float;
	var velocityI : Int;
	var positionI : Int;
	var mouseJoint : B2MouseJoint;
	var c : Ctrl;
	
	public function new(main:Main)
	{
		sprite = main.sprite;

		c = Ctrl.instance;

		timeStep = 1/30;
		scale = 30;
		velocityI = 10;
		positionI = 10;

		world = new B2World(new B2Vec2(0, 10), true);
		world.SetWarmStarting(true);
		world.SetContinuousPhysics(false);

		var draw = new B2DebugDraw();
		draw.SetSprite(sprite);
		draw.SetDrawScale(30.0);
		draw.SetFillAlpha(0.3);
		draw.SetLineThickness(1.0);
		draw.SetFlags(B2DebugDraw.e_shapeBit | B2DebugDraw.e_jointBit);
		world.SetDebugDraw(draw);
	
		// Create border of boxes
		var wall = new B2PolygonShape();
		var wallBd = new B2BodyDef();
		var wallB:B2Body;
		
		// Left
		wallBd.position.Set( -95 / scale, 360 / scale / 2);
		wall.SetAsBox(100/scale, 400/scale/2);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
		// Right
		wallBd.position.Set((640 + 95) / scale, 360 / scale / 2);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
		// Top
		wallBd.position.Set(640 / scale / 2, -95 / scale);
		wall.SetAsBox(680/scale/2, 100/scale);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
		// Bottom
		wallBd.position.Set(640 / scale / 2, (360 + 95) / scale);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
	}

	public function update()
	{
		mouseDestroy();
		mouseDrag();

		world.Step(timeStep, velocityI, positionI);
		world.ClearForces();

		world.DrawDebugData();
	}

	function mouseDestroy()
	{
		if(!c.mouseDown && c.isPressed("D"))
		{
			var b = getBodyAtMouse(true);
			if(b!=null) world.DestroyBody(b);
		}
	}

	function mouseDrag()
	{
		if(c.mouseDown && mouseJoint == null)
		{
			var b = getBodyAtMouse();
			if(b!=null)
			{
				var d = new B2MouseJointDef();
				d.bodyA = world.GetGroundBody();
				d.bodyB = b;
				d.target.Set(c.mouseX/scale, c.mouseY/scale);
				d.collideConnected = true;
				d.maxForce = 300 * b.GetMass();
				mouseJoint = cast world.CreateJoint(d);
				b.SetAwake(true);
			}
		}

		if(!c.mouseDown && mouseJoint != null)
		{
			world.DestroyJoint(mouseJoint);
			mouseJoint = null;
		}

		if(mouseJoint != null)
		{
			mouseJoint.SetTarget(new B2Vec2(c.mouseX/scale, c.mouseY/scale));
		}
	}

	function getBodyAtMouse(?includeStatic:Bool = false) : B2Body
	{
		var b:B2Body = null;
		world.QueryPoint(function(fixture:B2Fixture) : Bool
		{
			if(fixture.GetBody().GetType() == B2Body.b2_staticBody && !includeStatic)
			{
				return true;
			} else
			{
				b = fixture.GetBody();
				return false;
			}
		}, new B2Vec2(c.mouseX/scale, c.mouseY/scale));
		return b;
	}
	
}
