package ;

import flash.Lib;
import flash.display.Sprite;
import flash.text.TextField;
import flash.text.TextFormat;
import flash.text.TextFormatAlign;

import haxe.Timer;

import tests.Test;
import tests.TestCompound;
import tests.TestStack;
import tests.TestRaycast;
import tests.TestOneSidedPlatform;
import tests.TestCCD;
import tests.TestBreakable;
import tests.TestCrankGearsPulley;
import tests.TestBridge;
import tests.TestRagdoll;
import tests.TestTheoJansen;
import tests.TestBuoyancy;

import ctrl.Ctrl;

import com.remixtechnology.SWFProfiler;

class Main 
{
	var curTest:Test;
	public var sprite:Sprite;
	public var aboutText:TextField;
	var instructions_text:TextField;
	
	var curId:Int;
	
	var c:Ctrl;

	var tests:Array<Class<Test>>;

	var timer : Timer;
	
	public function new()
	{
		SWFProfiler.init();
		c = Ctrl.instance;
		
		curId = 0;
		
		tests = cast [
			TestRagdoll,			// Ragdoll
			TestCompound,			// Compound Shapes
			TestCrankGearsPulley,	// Crank/Gears/Pulley
			TestBridge,				// Bridge
			TestStack,				// Stack
			TestCCD,				// CCD
			TestTheoJansen,			// Theo Jansen
			//TestEdge
			TestBuoyancy,			// Buoyancy
			TestOneSidedPlatform,	// One Sided Platform
			TestBreakable,			// Breakable
			TestRaycast,			// Raycast
		];
		
		sprite = new Sprite();
		Lib.current.addChild(sprite);
		
		instructions_text = new TextField();
		var instructions_text_format:TextFormat = new TextFormat("Arial", 16, 0xffffff, false, false, false);
		instructions_text_format.align = TextFormatAlign.RIGHT;
		instructions_text.defaultTextFormat = instructions_text_format;
		instructions_text.x = 140;
		instructions_text.y = 4.5;
		instructions_text.width = 495;
		instructions_text.height = 61;
		instructions_text.text = "Box2DFlashAS3 2.1a in haXe\n'Left'/'Right' arrows to go to previous/next example. \n'R' to reset.";
		Lib.current.addChild(instructions_text);
		
		aboutText = new TextField();
		var aboutTextFormat:TextFormat = new TextFormat("Arial", 16, 0x00CCFF, true, false, false);
		aboutTextFormat.align = TextFormatAlign.RIGHT;
		aboutText.defaultTextFormat = aboutTextFormat;
		aboutText.x = 334;
		aboutText.y = 71;
		aboutText.width = 300;
		aboutText.height = 30;
		Lib.current.addChild(aboutText);
		
		instructions_text.mouseEnabled = false;
		aboutText.mouseEnabled = false;
		
		timer = new Timer(30);
		timer.run = update;
	}
	
	function update():Void{
		sprite.graphics.clear();
		
		// toggle between tests
		if (c.isPressed("RIGHT")){ // Right Arrow
			curId++;
			curTest = null;
		}
		else if (c.isPressed("LEFT")){ // Left Arrow
			curId--;
			curTest = null;
		}
		else if (c.isPressed("R")){ // R
			curTest = null;
		}
		
  		var testCount:Int = tests.length;
		curId = (curId + testCount) % testCount;
		
		// if null, set new test
		if (curTest == null){
			curTest = Type.createInstance(tests[curId], [this]);
		}
		
		// update current test
		curTest.update();
		
		// Update input (last)
		c.update();
	}
	
	static function main()
	{
		new Main();
	}
}
