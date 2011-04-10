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
import box2D.dynamics.B2ContactImpulse;
import box2D.dynamics.B2ContactListener;
import box2D.dynamics.contacts.B2Contact;

class TestBreakable extends Test
{
	public var body1:B2Body;
	public var velocity:B2Vec2;
	public var angularVelocity:Float;
	public var shape1:B2PolygonShape;
	public var shape2:B2PolygonShape;
	public var piece1:B2Fixture;
	public var piece2:B2Fixture;
	public var sbroke:Bool;
	public var sbreak:Bool;
	
	public function new(main:Main)
	{
		velocity = new B2Vec2();
		shape1 = new B2PolygonShape();
		shape2 = new B2PolygonShape();
		super(main);
		
		main.aboutText.text = "Breakable";
		
		world.SetContactListener(new ContactListenerBreakable(this));
		
		var ground = world.GetGroundBody();
		
		var bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position.Set(5.0, 5.0);
		bd.angle = 0.25 * Math.PI;
		body1 = world.CreateBody(bd);
		
		shape1.SetAsOrientedBox(0.5, 0.5, new B2Vec2( -0.5, 0.0));
		piece1 = body1.CreateFixture2(shape1, 1.0);
		
		shape2.SetAsOrientedBox(0.5, 0.5, new B2Vec2( 0.5, 0.0));
		piece2 = body1.CreateFixture2(shape2, 1.0);
		
		sbreak = false;
		sbroke = false;
	}


	public function doBreak()
	{
		// Apply cached velocity for more realistic break
		body1.SetLinearVelocity(velocity);
		body1.SetAngularVelocity(angularVelocity);
		
		// Split body into two pieces
		var self = this;
		body1.Split(function(fixture:B2Fixture):Bool {
			return fixture != self.piece1;
		});
	}

	override public function update()
	{
		super.update();
		if (sbreak)
		{
			doBreak();
			sbroke = true;
			sbreak = false;
		}
		
		// Cache velocities to improve movement on breakage
		if (sbroke == false)
		{
			velocity = body1.GetLinearVelocity();
			angularVelocity = body1.GetAngularVelocity();
		}
	}
	
}

class ContactListenerBreakable extends B2ContactListener
{
	private var test:TestBreakable;
	public function new(test:TestBreakable)
	{
		super();
		this.test = test;
	}
	
	override public function PostSolve(contact:B2Contact, impulse:B2ContactImpulse)
	{
		if (test.sbroke)
		{
			// The body already broke
			return;
		}
		
		// Should the body break?
		var count:Int = contact.GetManifold().m_pointCount;
		
		var maxImpulse:Float = 0.0;
		for (i in 0...count)
		{
			maxImpulse = B2Math.Max(maxImpulse, impulse.normalImpulses[i]);
		}
		
		if (maxImpulse > 50)
		{
			test.sbreak = true;
		}
	}
}
