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

package box2D.common.math;

/**
* @private
*/
class B2Math {
	
	public static inline var MIN_VALUE = 5e-324;
	public static inline var MAX_VALUE = 1.79e+308;
	
	/**
	* This function is used to ensure that a floating point number is
	* not a NaN or infinity.
	*/
	public function new() { }
	

	/**
	* This function is used to ensure that a floating point number is
	* not a NaN or infinity.
	*/
	public static function IsValid(x:Float) : Bool
	{
		return Math.isFinite(x);
	}
	
	/*static public function B2InvSqrt(x:Number):Number{
		union
		{
			float32 x;
			int32 i;
		} convert;
		
		convert.x = x;
		float32 xhalf = 0.5f * x;
		convert.i = 0x5f3759df - (convert.i >> 1);
		x = convert.x;
		x = x * (1.5f - xhalf * x * x);
		return x;
	}*/

	public static function Dot(a:B2Vec2, b:B2Vec2):Float
	{
		return a.x * b.x + a.y * b.y;
	}

	public static function CrossVV(a:B2Vec2, b:B2Vec2):Float
	{
		return a.x * b.y - a.y * b.x;
	}

	public static function CrossVF(a:B2Vec2, s:Float):B2Vec2
	{
		var v:B2Vec2 = new B2Vec2(s * a.y, -s * a.x);
		return v;
	}

	public static function CrossFV(s:Float, a:B2Vec2):B2Vec2
	{
		var v:B2Vec2 = new B2Vec2(-s * a.y, s * a.x);
		return v;
	}

	public static function MulMV(A:B2Mat22, v:B2Vec2):B2Vec2
	{
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		var u:B2Vec2 = new B2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
		return u;
	}

	public static function MulTMV(A:B2Mat22, v:B2Vec2):B2Vec2
	{
		// (tVec.x * tMat.col1.x + tVec.y * tMat.col1.y)
		// (tVec.x * tMat.col2.x + tVec.y * tMat.col2.y)
		var u:B2Vec2 = new B2Vec2(Dot(v, A.col1), Dot(v, A.col2));
		return u;
	}
	
	public static function MulX(T:B2Transform, v:B2Vec2) : B2Vec2
	{
		var a:B2Vec2 = MulMV(T.R, v);
		a.x += T.position.x;
		a.y += T.position.y;
		//return T.position + B2Mul(T.R, v);
		return a;
	}

	public static function MulXT(T:B2Transform, v:B2Vec2):B2Vec2
	{
		var a:B2Vec2 = SubtractVV(v, T.position);
		//return B2MulT(T.R, v - T.position);
		var tX:Float = (a.x * T.R.col1.x + a.y * T.R.col1.y );
		a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y );
		a.x = tX;
		return a;
	}

	public static function AddVV(a:B2Vec2, b:B2Vec2):B2Vec2
	{
		var v:B2Vec2 = new B2Vec2(a.x + b.x, a.y + b.y);
		return v;
	}

	public static function SubtractVV(a:B2Vec2, b:B2Vec2):B2Vec2
	{
		var v:B2Vec2 = new B2Vec2(a.x - b.x, a.y - b.y);
		return v;
	}
	
	public static function Distance(a:B2Vec2, b:B2Vec2) : Float{
		var cX:Float = a.x-b.x;
		var cY:Float = a.y-b.y;
		return Math.sqrt(cX*cX + cY*cY);
	}
	
	public static function DistanceSquared(a:B2Vec2, b:B2Vec2) : Float{
		var cX:Float = a.x-b.x;
		var cY:Float = a.y-b.y;
		return (cX*cX + cY*cY);
	}

	public static function MulFV(s:Float, a:B2Vec2):B2Vec2
	{
		var v:B2Vec2 = new B2Vec2(s * a.x, s * a.y);
		return v;
	}

	public static function AddMM(A:B2Mat22, B:B2Mat22):B2Mat22
	{
		var C:B2Mat22 = B2Mat22.FromVV(AddVV(A.col1, B.col1), AddVV(A.col2, B.col2));
		return C;
	}

	// A * B
	public static function MulMM(A:B2Mat22, B:B2Mat22):B2Mat22
	{
		var C:B2Mat22 = B2Mat22.FromVV(MulMV(A, B.col1), MulMV(A, B.col2));
		return C;
	}

	// A^T * B
	public static function MulTMM(A:B2Mat22, B:B2Mat22):B2Mat22
	{
		var c1:B2Vec2 = new B2Vec2(Dot(A.col1, B.col1), Dot(A.col2, B.col1));
		var c2:B2Vec2 = new B2Vec2(Dot(A.col1, B.col2), Dot(A.col2, B.col2));
		var C:B2Mat22 = B2Mat22.FromVV(c1, c2);
		return C;
	}

	public static function Abs(a:Float):Float
	{
		return a > 0.0 ? a : -a;
	}

	public static function AbsV(a:B2Vec2):B2Vec2
	{
		var b:B2Vec2 = new B2Vec2(Abs(a.x), Abs(a.y));
		return b;
	}

	public static function AbsM(A:B2Mat22):B2Mat22
	{
		var B:B2Mat22 = B2Mat22.FromVV(AbsV(A.col1), AbsV(A.col2));
		return B;
	}

	public static function Min(a:Float, b:Float):Float
	{
		return a < b ? a : b;
	}

	public static function MinV(a:B2Vec2, b:B2Vec2):B2Vec2
	{
		var c:B2Vec2 = new B2Vec2(Min(a.x, b.x), Min(a.y, b.y));
		return c;
	}

	public static function Max(a:Float, b:Float):Float
	{
		return a > b ? a : b;
	}

	public static function MaxV(a:B2Vec2, b:B2Vec2):B2Vec2
	{
		var c:B2Vec2 = new B2Vec2(Max(a.x, b.x), Max(a.y, b.y));
		return c;
	}

	public static function Clamp(a:Float, low:Float, high:Float):Float
	{
		return a < low ? low : ((a > high) ? high : a);
	}

	public static function ClampV(a:B2Vec2, low:B2Vec2, high:B2Vec2):B2Vec2
	{
		return MaxV(low, MinV(a, high));
	}

	public static function Swap(a:Array<Dynamic>, b:Array<Dynamic>) : Void
	{
		var tmp:Dynamic = a[0];
		a[0] = b[0];
		b[0] = tmp;
	}

	// B2Random number in range [-1,1]
	public static function Random():Float
	{
		return Math.random() * 2 - 1;
	}

	public static function RandomRange(lo:Float, hi:Float) : Float
	{
		var r:Float = Math.random();
		r = (hi - lo) * r + lo;
		return r;
	}

	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	public static function NextPowerOfTwo(x:Int):Int
	{
		x |= (x >> 1) & 0x7FFFFFFF;
		x |= (x >> 2) & 0x3FFFFFFF;
		x |= (x >> 4) & 0x0FFFFFFF;
		x |= (x >> 8) & 0x00FFFFFF;
		x |= (x >> 16)& 0x0000FFFF;
		return x + 1;
	}

	public static function IsPowerOfTwo(x:Int):Bool
	{
		var result:Bool = x > 0 && (x & (x - 1)) == 0;
		return result;
	}
	
	
	// Temp vector functions to reduce calls to 'new'
	/*static public var tempVec:B2Vec2 = new B2Vec2();
	static public var tempVec2:B2Vec2 = new B2Vec2();
	static public var tempVec3:B2Vec2 = new B2Vec2();
	static public var tempVec4:B2Vec2 = new B2Vec2();
	static public var tempVec5:B2Vec2 = new B2Vec2();
	
	static public var tempMat:B2Mat22 = new B2Mat22();	
	
	static public var tempAABB:B2AABB = new B2AABB();	*/
	
	public static var b2Vec2_zero:B2Vec2 = new B2Vec2(0.0, 0.0);
	public static var b2Mat22_identity:B2Mat22 = B2Mat22.FromVV(new B2Vec2(1.0, 0.0), new B2Vec2(0.0, 1.0));
	public static var b2Transform_identity:B2Transform = new B2Transform(b2Vec2_zero, b2Mat22_identity);
	

}
