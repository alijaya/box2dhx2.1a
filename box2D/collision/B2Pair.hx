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

// The pair manager is used by the broad-phase to quickly add/remove/find pairs
// of overlapping proxies. It is based closely on code provided by Pierre Terdiman.
// http://www.codercorner.com/IncrementalSAP.txt

package box2D.collision;

import box2D.common.B2Settings;

/**
 * A Pair represents a pair of overlapping B2Proxy in the broadphse.
 * @private
 */
class B2Pair
{
	

	public function new() { 
		userData = null;
	}
	
	

	public function SetBuffered() : Void	{ status |= e_pairBuffered; }
	public function ClearBuffered() : Void	{ status &= ~e_pairBuffered; }
	public function IsBuffered():Bool	{ return (status & e_pairBuffered) == e_pairBuffered; }

	public function SetRemoved() : Void		{ status |= e_pairRemoved; }
	public function ClearRemoved() : Void	{ status &= ~e_pairRemoved; }
	public function IsRemoved():Bool		{ return (status & e_pairRemoved) == e_pairRemoved; }
	
	public function SetFinal() : Void		{ status |= e_pairFinal; }
	public function IsFinal():Bool		{ return (status & e_pairFinal) == e_pairFinal; }

	public var userData:Dynamic ;
	public var proxy1:B2Proxy;
	public var proxy2:B2Proxy;
	public var next:B2Pair;
	public var status:Int;
	
	// STATIC
	public static var B2_nullProxy:Int = B2Settings.USHRT_MAX;
	
	// enum
	public static var e_pairBuffered:Int = 0x0001;
	public static var e_pairRemoved:Int = 0x0002;
	public static var e_pairFinal:Int = 0x0004;

}


