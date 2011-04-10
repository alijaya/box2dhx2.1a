/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package box2D.collision;

import flash.Vector;
import flash.utils.Dictionary;

/**
* @private
*/
class B2Proxy {
	public function new() { 
		lowerBounds = new Vector<Int>(2);
		upperBounds = new Vector<Int>(2);
		pairs = new Dictionary();
		userData = null;
	}
	
	public function IsValid():Bool { return overlapCount != B2BroadPhase.b2_invalid; }

	public var lowerBounds:Vector<Int>;
	public var upperBounds:Vector<Int>;
	public var overlapCount:Int;
	public var timeStamp:Int;
	
	// Maps from the other B2Proxy to their mutual B2Pair.
	public var pairs:Dictionary ;
	
	public var next:B2Proxy;
	
	public var userData:Dynamic ;
}
	
	
