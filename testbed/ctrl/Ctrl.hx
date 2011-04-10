package ctrl;

import flash.events.Event;
import flash.events.MouseEvent;
import flash.events.KeyboardEvent;
import flash.ui.Keyboard;
import flash.Lib;

class Ctrl 
{
	public static var inited:Bool = false;
	
	private static var _instance : Ctrl;
	public static var instance(get_instance, null) : Ctrl;
	public static function get_instance() : Ctrl
	{
		if (_instance == null) _instance = new Ctrl();
		return _instance;
	}
	
	public var lookups(default, null) : Hash<Int>;
	
	// 0 (00) : up
	// 1 (01) : released
	// 2 (10) : down
	// 3 (11) : pressed
	public var keys(default, null) : Array<Int>;
	
	public var lastPressed(default, null) : Null<Int>;
	public var lastReleased(default, null) : Null<Int>;
	
	public var capsLock(get_capsLock, null) : Bool;
	public var numLock(get_numLock, null) : Bool;
	
	public var mouseLeave(default, null) : Bool;
	
	public var mouseDown(default, null) : Bool;
	public var mousePressed(default, null) : Bool;
	public var mouseUp(default, null) : Bool;
	public var mouseReleased(default, null) : Bool;
	
	public var mouseX(get_mouseX, null) : Float;
	public var mouseY(get_mouseY, null) : Float;
	public var mouseWheel(default, null) : Int;

	public var bufferLength : Int;
	public var buffer(default, null) : Array<Int>;
	
	
	private function new()
	{
		bufferLength = 0;
		buffer = [];
		lookups = new Hash<Int>();
		keys = [];
		
		mouseWheel = 0;
		
		for(n in 0...256)
		{
			keys.push(0);
		}
		
		
		// Special Key
		addKey("BACKSPACE", 8);
		addKey("TAB",       9);
		addKey("ENTER",     13);
		addKey("SHIFT",     16);
		addKey("CONTROL",   17);
		addKey("CAPS_LOCK", 20);
		addKey("ESCAPE",    27);
		
		addKey("SPACE", 32);
		
		// Navigation Key
		addKey("PAGE_UP",   33);
		addKey("PAGE_DOWN", 34);
		addKey("END",       35);
		addKey("HOME",      36);
		
		// Direction Key
		addKey("LEFT",  37);
		addKey("UP",    38);
		addKey("RIGHT", 39);
		addKey("DOWN",  40);
		
		addKey("INSERT", 45);
		addKey("DELETE", 46);
		
		// Number Key
		for(n in 0...10)
		{
			addKey(""+n, 48+n);
		}
		
		// Letter Key
		for(n in 0...26)
		{
			addKey(String.fromCharCode(65+n), 65+n);
		}
		
		// Numpad Key
		for(n in 0...10)
		{
			addKey("NUMPAD_"+n, 96+n);
		}
		
		addKey("NUMPAD_MULTIPLY", 106);
		addKey("NUMPAD_ADD",      107);
		addKey("NUMPAD_ENTER",    108);
		addKey("NUMPAD_SUBTRACT", 109);
		addKey("NUMPAD_DECIMAL",  110);
		addKey("NUMPAD_DIVIDE",   111);
		
		// Function Key
		for(n in 0...15)
		{
			addKey("F"+(n+1), 112+n);
		}
		
		// Punctuation Key
		addKey("SEMICOLON", 186);
		addKey(";",         186);
		addKey("EQUAL",     187);
		addKey("=",         187);
		addKey("COMMA",     188);
		addKey(",",         188);
		addKey("MINUS",     189);
		addKey("-",         189);
		addKey("PERIOD",    190);
		addKey(".",         190);
		addKey("SLASH",     191);
		addKey("/",         191);
		addKey("BACKQUOTE", 192);
		addKey("`",         192);
		
		addKey("LEFTBRACKET",  219);
		addKey("[",            219);
		addKey("BACKSLASH",    220);
		addKey("\\",           220);
		addKey("RIGHTBRACKET", 221);
		addKey("]",            221);
		addKey("QUOTE",        222);
		addKey("'",            222);
		
		if(Lib.current.stage == null)
		{
			Lib.current.addEventListener(Event.ADDED_TO_STAGE, onStage);
		} else
		{
			init();
		}
	}
	
	public function update()
	{
		lastPressed = null;
		lastReleased = null;
		
		mousePressed = false;
		mouseReleased = false;
		
		mouseWheel = 0;
		
		for(n in 0...keys.length)
		{
			keys[n] = keys[n] & 2; // mask the states with 10
		}
	}
	
	function onStage(_)
	{
		Lib.current.removeEventListener(Event.ADDED_TO_STAGE, onStage);
		
		init();
	}

	function init()
	{
		Lib.current.stage.addEventListener(KeyboardEvent.KEY_DOWN, keyDownHandler);
		Lib.current.stage.addEventListener(KeyboardEvent.KEY_UP, keyUpHandler);
		
		Lib.current.stage.addEventListener(MouseEvent.MOUSE_DOWN, mouseDownHandler);
		Lib.current.stage.addEventListener(MouseEvent.MOUSE_UP, mouseUpHandler);
		Lib.current.stage.addEventListener(MouseEvent.MOUSE_WHEEL, mouseWheelHandler);
		Lib.current.stage.addEventListener(Event.MOUSE_LEAVE, mouseLeaveHandler);
		
		inited = true;
	}
	
	public function addKey(keyName:String, keyCode:Int)
	{
		lookups.set(keyName.toUpperCase(), keyCode);
	}
	
	function keyDownHandler(e:KeyboardEvent)
	{
		if(keys[e.keyCode] >> 1 == 0)
		{
			lastPressed = e.keyCode;
			keys[e.keyCode] = 3;
			buffer.unshift(lastPressed);
			boundBuffer();
		}
	}
	
	function keyUpHandler(e:KeyboardEvent)
	{
		if(keys[e.keyCode] >> 1 == 1)
		{
			lastReleased = e.keyCode;
			keys[e.keyCode] = 1;
		}
	}
	
	function mouseDownHandler(e:MouseEvent)
	{
		mouseDown = true;
		mousePressed = true;
		mouseUp = false;
		mouseReleased = false;
	}
	
	function mouseUpHandler(e:MouseEvent)
	{
		mouseDown = false;
		mousePressed = false;
		mouseUp = true;
		mouseReleased = true;
	}
	
	function mouseWheelHandler(e:MouseEvent)
	{
		mouseWheel = e.delta;
	}
	
	function mouseLeaveHandler(e:Event)
	{
		mouseLeave = true;
		Lib.current.stage.addEventListener(MouseEvent.MOUSE_MOVE, mouseMoveHandler);
	}
	
	function mouseMoveHandler(e:MouseEvent)
	{
		Lib.current.stage.removeEventListener(MouseEvent.MOUSE_MOVE, mouseMoveHandler);
		mouseLeave = false;
	}

	function boundBuffer()
	{
		for(n in 0...buffer.length-bufferLength)
		{
			buffer.pop();
		}
	}
	
	public function isDown(key:String) : Bool
	{
		return (keys[getKeyCode(key)] >> 1 == 1);
	}
	
	public function isPressed(key:String) : Bool
	{
		return (keys[getKeyCode(key)] == 3);
	}
	
	public function isUp(key:String) : Bool
	{
		return (keys[getKeyCode(key)] >> 1 == 0);
	}
	
	public function isReleased(key:String) : Bool
	{
		return (keys[getKeyCode(key)] == 1);
	}
	
	public function get_capsLock() : Bool
	{
		return Keyboard.capsLock;
	}
	
	public function get_numLock() : Bool
	{
		return Keyboard.numLock;
	}
	
	public function get_mouseX() : Float
	{
		return Lib.current.mouseX;
	}
	
	public function get_mouseY() : Float
	{
		return Lib.current.mouseY;
	}
	
	public function getKeyCode(key:String) : Int
	{
		key = key.toUpperCase();
		var code = lookups.get(key);
		if(code == null) throw "Error : the key \""+key+"\" doesn't exist";
		return code;
	}

	public function isNumeric(keyCode:Int) : Bool
	{
		return (keyCode >= 48 && keyCode <= 57) || (keyCode >= 96 && keyCode <= 105);
	}

	public function isAlpha(keyCode:Int) : Bool
	{
		return (keyCode >=65 && keyCode <= 90);
	}

	public function isSpecial(keyCode:Int) : Bool
	{
		return (keyCode >= 186 && keyCode <= 192) || (keyCode >= 219 && keyCode <= 222);
	}

	public function isWhiteSpace(keyCode:Int) : Bool
	{
		return keyCode == 9 || keyCode == 13 || keyCode == 32;
	}
	
	
	

	public function isDisplayChar(keyCode:Int) : Bool
	{
		return isNumeric(keyCode) || isAlpha(keyCode) || isSpecial(keyCode) || isWhiteSpace(keyCode);
	}

	public function getChar() : String
	{
		var kc = lastPressed;
		if(kc == null) return null;
		return if(isDown("SHIFT"))
		{
			if(isNumeric(kc))
			{
				switch(kc)
				{
					case 48 : ")";
					case 49 : "!";
					case 50 : "@";
					case 51 : "#";
					case 52 : "$";
					case 53 : "%";
					case 54 : "^";
					case 55 : "&";
					case 56 : "*";
					case 57 : "(";
				}
			} else if(isAlpha(kc))
			{
				if(capsLock)
				{
					String.fromCharCode(kc+32);
				} else
				{
					String.fromCharCode(kc);
				}
			} else if(isSpecial(kc))
			{
				switch(kc)
				{
					case 186 : ":";
					case 187 : "+";
					case 188 : "<";
					case 189 : "_";
					case 190 : ">";
					case 191 : "?";
					case 192 : "~";

					case 219 : "{";
					case 220 : "|";
					case 221 : "}";
					case 222 : "\"";
				}
			} else if(isWhiteSpace(kc))
			{
				switch(kc)
				{
					case 9 : "\t";
					case 13 : "\n";
					case 32 : " ";
				}
			} else
			{
				null;
			}
		} else
		{
			if(isNumeric(kc))
			{
				""+(kc-48);
			} else if(isAlpha(kc))
			{
				if(capsLock)
				{
					String.fromCharCode(kc);
				} else
				{
					String.fromCharCode(kc+32);
				}
			} else if(isSpecial(kc))
			{
				switch(kc)
				{
					case 186 : ";";
					case 187 : "=";
					case 188 : ",";
					case 189 : "-";
					case 190 : ".";
					case 191 : "/";
					case 192 : "`";

					case 219 : "[";
					case 220 : "\\";
					case 221 : "]";
					case 222 : "'";
				}
			} else if(isWhiteSpace(kc))
			{
				switch(kc)
				{
					case 9 : "\t";
					case 13 : "\n";
					case 32 : " ";
				}
			} else
			{
				null;
			}
		}
	}
	
	
}
