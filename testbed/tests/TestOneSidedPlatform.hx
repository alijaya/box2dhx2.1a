package tests;

import box2D.collision.B2Manifold;
import box2D.collision.shapes.B2PolygonShape;
import box2D.collision.shapes.B2CircleShape;
import box2D.common.math.B2Math;
import box2D.common.math.B2Vec2;
import box2D.common.math.B2Transform;
import box2D.dynamics.B2Body;
import box2D.dynamics.B2BodyDef;
import box2D.dynamics.B2Fixture;
import box2D.dynamics.B2FixtureDef;
import box2D.dynamics.B2ContactListener;
import box2D.dynamics.contacts.B2Contact;

class TestOneSidedPlatform extends Test
{
	
	
	static private var e_unknown:Int = 0;
	static private var e_above:Int = 1;
	static private var e_below:Int = 2;
	
	public var radius:Float;
	public var top:Float;
	public var bottom:Float;
	public var state:Int;
	public var platform:B2Fixture;
	public var character:B2Fixture;
		
	public function new(main:Main)
	{
		super(main);
		
		main.aboutText.text = "One Sided Platform";
		
		var bd:B2BodyDef;
		var body:B2Body;
		
		// Platform
		bd = new B2BodyDef();
		bd.position.Set(10.0, 10.0);
		body = world.CreateBody(bd);
		
		var polygon = B2PolygonShape.AsBox(3.0, 0.5);
		platform = body.CreateFixture2(polygon);
		
		bottom = bd.position.y + 0.5;
		top = bd.position.y - 0.5;
		
		// Actor
		bd = new B2BodyDef();
		bd.type = B2Body.b2_dynamicBody;
		bd.position.Set(10.0, 12.0);
		body = world.CreateBody(bd);
		
		radius = 0.5;
		var circle = new B2CircleShape(radius);
		character = body.CreateFixture2(circle, 1.0);
		
		state = e_unknown;
		
		world.SetContactListener(new ContactListenerOneSidePlatform(this));
	}
}

class ContactListenerOneSidePlatform extends B2ContactListener
{
	private var test:TestOneSidedPlatform;
	public function new(test:TestOneSidedPlatform)
	{
		super();
		this.test = test;
	}
	override public function PreSolve(contact:B2Contact, oldManifold:B2Manifold):Void 
	{
		var fixtureA = contact.GetFixtureA();
		var fixtureB = contact.GetFixtureB();
		if (fixtureA != test.platform && fixtureA != test.character)
			return;
		if (fixtureB != test.platform && fixtureB != test.character)
			return;
			
		var position = test.character.GetBody().GetPosition();
		if (position.y > test.top)
			contact.SetEnabled(false);
	}
}
