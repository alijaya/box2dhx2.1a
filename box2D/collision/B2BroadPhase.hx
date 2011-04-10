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

import box2D.common.B2Settings;
import box2D.common.math.B2Math;
import box2D.common.math.B2Vec2;

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/


// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.

/**
* @private
*/
class B2BroadPhase implements IBroadPhase {
//public:
	
//public:
	public function new(worldAABB:B2AABB){
		m_pairManager = new B2PairManager();
		m_proxyPool = [];
		m_querySortKeys = [];
		m_queryResults = new Array();
		m_quantizationFactor = new B2Vec2();
		//B2Settings.b2Assert(worldAABB.IsValid());
		
		m_pairManager.Initialize(this);
		
		m_worldAABB = worldAABB;
		
		m_proxyCount = 0;
		
		// bounds array
		m_bounds = new Vector<Vector<B2Bound>>();
		for (i in 0...2){
			m_bounds[i] = new Vector<B2Bound>();
		}
		
		//B2Vec2 d = worldAABB.upperBound - worldAABB.lowerBound;
		var dX:Float = worldAABB.upperBound.x - worldAABB.lowerBound.x;
		var dY:Float = worldAABB.upperBound.y - worldAABB.lowerBound.y;
		
		m_quantizationFactor.x = B2Settings.USHRT_MAX / dX;
		m_quantizationFactor.y = B2Settings.USHRT_MAX / dY;
		
		m_timeStamp = 1;
		m_queryResultCount = 0;
	}
	//~B2BroadPhase();
	
	// Use this to see if your proxy is in range. If it is not in range,
	// it should be destroyed. Otherwise you may get O(m^2) pairs, where m
	// is the number of proxies that are out of range.
	public function InRange(aabb:B2AABB):Bool{
		//B2Vec2 d = B2Max(aabb.lowerBound - m_worldAABB.upperBound, m_worldAABB.lowerBound - aabb.upperBound);
		var dX:Float;
		var dY:Float;
		var d2X:Float;
		var d2Y:Float;
		
		dX = aabb.lowerBound.x;
		dY = aabb.lowerBound.y;
		dX -= m_worldAABB.upperBound.x;
		dY -= m_worldAABB.upperBound.y;
		
		d2X = m_worldAABB.lowerBound.x;
		d2Y = m_worldAABB.lowerBound.y;
		d2X -= aabb.upperBound.x;
		d2Y -= aabb.upperBound.y;
		
		dX = B2Math.Max(dX, d2X);
		dY = B2Math.Max(dY, d2Y);
		
		return B2Math.Max(dX, dY) < 0.0;
	}

	// Create and destroy proxies. These call Flush first.
	public function CreateProxy(aabb:B2AABB, userData:Dynamic):Dynamic{
		var index:Int;
		var proxy:B2Proxy;
		var i:Int;
		var j:Int;
		
		//B2Settings.b2Assert(m_proxyCount < B2_maxProxies);
		//B2Settings.b2Assert(m_freeProxy != B2Pair.B2_nullProxy);
		
		if (m_freeProxy == null)
		{
			// As all proxies are allocated, m_proxyCount == m_proxyPool.length
			m_freeProxy = m_proxyPool[m_proxyCount] = new B2Proxy();
			m_freeProxy.next = null;
			m_freeProxy.timeStamp = 0;
			m_freeProxy.overlapCount = b2_invalid;
			m_freeProxy.userData = null;
			
			for (i in 0...2)
			{
				j = m_proxyCount * 2;
				m_bounds[i][j++] = new B2Bound();
				m_bounds[i][j] = new B2Bound();
			}
			
		}
		
		proxy = m_freeProxy;
		m_freeProxy = proxy.next;
		
		proxy.overlapCount = 0;
		proxy.userData = userData;
		
		var boundCount:Int = 2 * m_proxyCount;
		
		var lowerValues:Vector<Int> = new Vector<Int>();
		var upperValues:Vector<Int> = new Vector<Int>();
		ComputeBounds(lowerValues, upperValues, aabb);
		
		for (axis in 0...2)
		{
			var bounds:Vector<B2Bound> = m_bounds[axis];
			var lowerIndex:Int = 0;
			var upperIndex:Int = 0;
			var lowerIndexOut:Vector<Int> = new Vector<Int>();
			lowerIndexOut.push(lowerIndex);
			var upperIndexOut:Vector<Int> = new Vector<Int>();
			upperIndexOut.push(upperIndex);
			QueryAxis(lowerIndexOut, upperIndexOut, lowerValues[axis], upperValues[axis], bounds, boundCount, axis);
			lowerIndex = lowerIndexOut[0];
			upperIndex = upperIndexOut[0];

			var t = bounds.pop();
			var tb = bounds.splice(upperIndex, bounds.length - 1);
			bounds.push(t);
			bounds = bounds.concat(tb);
			
			t = bounds.pop();
			tb = bounds.splice(lowerIndex, bounds.length - 1);
			bounds.push(t);
			bounds = bounds.concat(tb);
			
			// The upper index has increased because of the lower bound insertion.
			++upperIndex;
			
			// Copy in the new bounds.
			var tBound1:B2Bound = bounds[lowerIndex];
			var tBound2:B2Bound = bounds[upperIndex];
			tBound1.value = lowerValues[axis];
			tBound1.proxy = proxy;
			tBound2.value = upperValues[axis];
			tBound2.proxy = proxy;
			
			var tBoundAS3:B2Bound = bounds[lowerIndex-1];
			tBound1.stabbingCount = lowerIndex == 0 ? 0 : tBoundAS3.stabbingCount;
			tBoundAS3 = bounds[upperIndex-1];
			tBound2.stabbingCount = tBoundAS3.stabbingCount;
			
			// Adjust the stabbing count between the new bounds.
			index = lowerIndex;
			while (index < upperIndex)
			{
				tBoundAS3 = bounds[index];
				tBoundAS3.stabbingCount++;
				++index;
			}
			
			// Adjust the all the affected bound indices.
			index = lowerIndex;
			while (index < boundCount + 2)
			{
				tBound1 = bounds[index];
				var proxy2:B2Proxy = tBound1.proxy;
				if (tBound1.IsLower())
				{
					proxy2.lowerBounds[axis] = index;
				}
				else
				{
					proxy2.upperBounds[axis] = index;
				}
				++index;
			}
		}
		
		++m_proxyCount;
		
		//B2Settings.b2Assert(m_queryResultCount < B2Settings.b2_maxProxies);
		
		for (i in 0...m_queryResultCount)
		{
			//B2Settings.b2Assert(m_queryResults[i] < B2_maxProxies);
			//B2Settings.b2Assert(m_proxyPool[m_queryResults[i]].IsValid());
			
			m_pairManager.AddBufferedPair(proxy, m_queryResults[i]);
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		return proxy;
	}
	
	public function DestroyProxy(proxy_:Dynamic) : Void {
		var proxy:B2Proxy = cast( proxy_, B2Proxy);
		var tBound1:B2Bound;
		var tBound2:B2Bound;
		
		//B2Settings.b2Assert(proxy.IsValid());
		
		var boundCount:Int = 2 * m_proxyCount;
		
		for (axis in 0...2)
		{
			var bounds:Vector<B2Bound> = m_bounds[axis];
			
			var lowerIndex:Int = proxy.lowerBounds[axis];
			var upperIndex:Int = proxy.upperBounds[axis];
			tBound1 = bounds[lowerIndex];
			var lowerValue:Int = tBound1.value;
			tBound2 = bounds[upperIndex];
			var upperValue:Int = tBound2.value;
			
			bounds.splice(upperIndex, 1);
			bounds.splice(lowerIndex, 1);
			bounds.push(tBound1);
			bounds.push(tBound2);
			
			
			// Fix bound indices.
			var tEnd:Int = boundCount - 2;
			var index:Int = lowerIndex;
			while (index < tEnd)
			{
				tBound1 = bounds[index];
				var proxy2:B2Proxy = tBound1.proxy;
				if (tBound1.IsLower())
				{
					proxy2.lowerBounds[axis] = index;
				}
				else
				{
					proxy2.upperBounds[axis] = index;
				}
				++index;
			}
			
			// Fix stabbing count.
			tEnd = upperIndex - 1;
			var index2:Int = lowerIndex;
			while (index2 < tEnd)
			{
				tBound1 = bounds[index2];
				tBound1.stabbingCount--;
				++index2;
			}
			
			// Query for pairs to be removed. lowerIndex and upperIndex are not needed.
			// make lowerIndex and upper output using an array and do this for others if compiler doesn't pick them up
			var ignore:Vector<Int> = new Vector<Int>();
			QueryAxis(ignore, ignore, lowerValue, upperValue, bounds, boundCount - 2, axis);
		}
		
		//B2Settings.b2Assert(m_queryResultCount < B2Settings.b2_maxProxies);
		
		for (i in 0...m_queryResultCount)
		{
			//B2Settings.b2Assert(m_proxyPool[m_queryResults[i]].IsValid());
			
			m_pairManager.RemoveBufferedPair(proxy, m_queryResults[i]);
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		// Return the proxy to the pool.
		proxy.userData = null;
		proxy.overlapCount = b2_invalid;
		proxy.lowerBounds[0] = b2_invalid;
		proxy.lowerBounds[1] = b2_invalid;
		proxy.upperBounds[0] = b2_invalid;
		proxy.upperBounds[1] = b2_invalid;
		
		proxy.next = m_freeProxy;
		m_freeProxy = proxy;
		--m_proxyCount;
	}


	// Call MoveProxy as many times as you like, then when you are done
	// call Commit to finalized the proxy pairs (for your time step).
	public function MoveProxy(proxy_:Dynamic, aabb:B2AABB, displacement:B2Vec2) : Void {
		var proxy:B2Proxy = cast( proxy_, B2Proxy);
		
		var as3arr:Vector<Int>;
		var as3int:Int;
		
		var axis:Int;
		var index:Int;
		var bound:B2Bound;
		var prevBound:B2Bound;
		var nextBound:B2Bound;
		var nextProxyId:Int;
		var nextProxy:B2Proxy;
		var prevProxy:B2Proxy;
		
		if (proxy == null)
		{
			//B2Settings.b2Assert(false);
			return;
		}
		
		if (aabb.IsValid() == false)
		{
			//B2Settings.b2Assert(false);
			return;
		}
		
		var boundCount:Int = 2 * m_proxyCount;
		
		// Get new bound values
		var newValues:B2BoundValues = new B2BoundValues();
		ComputeBounds(newValues.lowerValues, newValues.upperValues, aabb);
		
		// Get old bound values
		var oldValues:B2BoundValues = new B2BoundValues();
		for (axis in 0...2)
		{
			bound = m_bounds[axis][proxy.lowerBounds[axis]];
			oldValues.lowerValues[axis] = bound.value;
			bound = m_bounds[axis][proxy.upperBounds[axis]];
			oldValues.upperValues[axis] = bound.value;
		}
		
		for (axis in 0...2)
		{
			var bounds:Vector<B2Bound> = m_bounds[axis];
			
			var lowerIndex:Int = proxy.lowerBounds[axis];
			var upperIndex:Int = proxy.upperBounds[axis];
			
			var lowerValue:Int = newValues.lowerValues[axis];
			var upperValue:Int = newValues.upperValues[axis];
			
			bound = bounds[lowerIndex];
			var deltaLower:Int = lowerValue - bound.value;
			bound.value = lowerValue;
			
			bound = bounds[upperIndex];
			var deltaUpper:Int = upperValue - bound.value;
			bound.value = upperValue;
			
			//
			// Expanding adds overlaps
			//
			
			// Should we move the lower bound down?
			if (deltaLower < 0)
			{
				index = lowerIndex;
				while (index > 0 && lowerValue < (cast( bounds[index-1], B2Bound)).value)
				{
					bound = bounds[index];
					prevBound = bounds[index - 1];
					
					prevProxy = prevBound.proxy;
					
					prevBound.stabbingCount++;
					
					if (prevBound.IsUpper() == true)
					{
						if (TestOverlapBound(newValues, prevProxy))
						{
							m_pairManager.AddBufferedPair(proxy, prevProxy);
						}
						
						//prevProxy.upperBounds[axis]++;
						as3arr = prevProxy.upperBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					else
					{
						//prevProxy.lowerBounds[axis]++;
						as3arr = prevProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					
					//proxy.lowerBounds[axis]--;
					as3arr = proxy.lowerBounds;
					as3int = as3arr[axis];
					as3int--;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:B2Bound = bound;
					//bound = prevEdge;
					//prevEdge = temp;
					bound.Swap(prevBound);
					//B2Math.Swap(bound, prevEdge);
					--index;
				}
			}
			
			// Should we move the upper bound up?
			if (deltaUpper > 0)
			{
				index = upperIndex;
				while (index < boundCount-1 && (cast( bounds[index+1], B2Bound)).value <= upperValue)
				{
					bound = bounds[ index ];
					nextBound = bounds[index + 1];
					nextProxy = nextBound.proxy;
					
					nextBound.stabbingCount++;
					
					if (nextBound.IsLower() == true)
					{
						if (TestOverlapBound(newValues, nextProxy))
						{
							m_pairManager.AddBufferedPair(proxy, nextProxy);
						}
						
						//nextProxy.lowerBounds[axis]--;
						as3arr = nextProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					else
					{
						//nextProxy.upperBounds[axis]--;
						as3arr = nextProxy.upperBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					
					//proxy.upperBounds[axis]++;
					as3arr = proxy.upperBounds;
					as3int = as3arr[axis];
					as3int++;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:B2Bound = bound;
					//bound = nextEdge;
					//nextEdge = temp;
					bound.Swap(nextBound);
					//B2Math.Swap(bound, nextEdge);
					index++;
				}
			}
			
			//
			// Shrinking removes overlaps
			//
			
			// Should we move the lower bound up?
			if (deltaLower > 0)
			{
				index = lowerIndex;
				while (index < boundCount-1 && (cast( bounds[index+1], B2Bound)).value <= lowerValue)
				{
					bound = bounds[ index ];
					nextBound = bounds[index + 1];
					
					nextProxy = nextBound.proxy;
					
					nextBound.stabbingCount--;
					
					if (nextBound.IsUpper())
					{
						if (TestOverlapBound(oldValues, nextProxy))
						{
							m_pairManager.RemoveBufferedPair(proxy, nextProxy);
						}
						
						//nextProxy.upperBounds[axis]--;
						as3arr = nextProxy.upperBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					else
					{
						//nextProxy.lowerBounds[axis]--;
						as3arr = nextProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					
					//proxy.lowerBounds[axis]++;
					as3arr = proxy.lowerBounds;
					as3int = as3arr[axis];
					as3int++;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:B2Bound = bound;
					//bound = nextEdge;
					//nextEdge = temp;
					bound.Swap(nextBound);
					//B2Math.Swap(bound, nextEdge);
					index++;
				}
			}
			
			// Should we move the upper bound down?
			if (deltaUpper < 0)
			{
				index = upperIndex;
				while (index > 0 && upperValue < (cast( bounds[index-1], B2Bound)).value)
				{
					bound = bounds[index];
					prevBound = bounds[index - 1];
					
					prevProxy = prevBound.proxy;
					
					prevBound.stabbingCount--;
					
					if (prevBound.IsLower() == true)
					{
						if (TestOverlapBound(oldValues, prevProxy))
						{
							m_pairManager.RemoveBufferedPair(proxy, prevProxy);
						}
						
						//prevProxy.lowerBounds[axis]++;
						as3arr = prevProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					else
					{
						//prevProxy.upperBounds[axis]++;
						as3arr = prevProxy.upperBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					
					//proxy.upperBounds[axis]--;
					as3arr = proxy.upperBounds;
					as3int = as3arr[axis];
					as3int--;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:B2Bound = bound;
					//bound = prevEdge;
					//prevEdge = temp;
					bound.Swap(prevBound);
					//B2Math.Swap(bound, prevEdge);
					index--;
				}
			}
		}
	}
	
	public function UpdatePairs(cb:Dynamic) : Void{
		m_pairManager.Commit(cb);
	}

	public function TestOverlap(proxyA:Dynamic, proxyB:Dynamic):Bool
	{
		var proxyA_:B2Proxy = cast( proxyA, B2Proxy);
		var proxyB_:B2Proxy = cast( proxyB, B2Proxy);
		if ( proxyA_.lowerBounds[0] > proxyB_.upperBounds[0]) return false;
		if ( proxyB_.lowerBounds[0] > proxyA_.upperBounds[0]) return false;
		if ( proxyA_.lowerBounds[1] > proxyB_.upperBounds[1]) return false;
		if ( proxyB_.lowerBounds[1] > proxyA_.upperBounds[1]) return false;
		return true;
	}
	
	/**
	 * Get user data from a proxy. Returns null if the proxy is invalid.
	 */
	public function GetUserData(proxy:Dynamic):Dynamic
	{
		return (cast( proxy, B2Proxy)).userData;
	}
	
	/**
	 * Get the AABB for a proxy.
	 */
	public function GetFatAABB(proxy_:Dynamic):B2AABB
	{
		var aabb:B2AABB = new B2AABB();
		var proxy:B2Proxy = cast( proxy_, B2Proxy);
		aabb.lowerBound.x = m_worldAABB.lowerBound.x +  m_bounds[0][proxy.lowerBounds[0]].value  / m_quantizationFactor.x;
		aabb.lowerBound.y = m_worldAABB.lowerBound.y +  m_bounds[1][proxy.lowerBounds[1]].value  / m_quantizationFactor.y;
		aabb.upperBound.x = m_worldAABB.lowerBound.x +  m_bounds[0][proxy.upperBounds[0]].value  / m_quantizationFactor.x;
		aabb.upperBound.y = m_worldAABB.lowerBound.y +  m_bounds[1][proxy.upperBounds[1]].value  / m_quantizationFactor.y;
		return aabb;
	}
	
	/**
	 * Get the number of proxies.
	 */
	public function GetProxyCount():Int
	{
		return m_proxyCount;
	}
		
	
	/**
	 * Query an AABB for overlapping proxies. The callback class
	 * is called for each proxy that overlaps the supplied AABB.
	 */
	public function Query(cb:Dynamic, aabb:B2AABB):Void
	{
		var lowerValues:Vector<Int> = new Vector<Int>();
		var upperValues:Vector<Int> = new Vector<Int>();
		ComputeBounds(lowerValues, upperValues, aabb);
		
		var lowerIndex:Int = 0;
		var upperIndex:Int = 0;
		var lowerIndexOut:Vector<Int> = new Vector<Int>();
		lowerIndexOut.push(lowerIndex);
		var upperIndexOut:Vector<Int> = new Vector<Int>();
		upperIndexOut.push(upperIndex);
		QueryAxis(lowerIndexOut, upperIndexOut, lowerValues[0], upperValues[0], m_bounds[0], 2*m_proxyCount, 0);
		QueryAxis(lowerIndexOut, upperIndexOut, lowerValues[1], upperValues[1], m_bounds[1], 2*m_proxyCount, 1);
		
		//B2Settings.b2Assert(m_queryResultCount < B2Settings.b2_maxProxies);
		
		// TODO: Don't be lazy, transform QueryAxis to directly call callback
		for (i in 0...m_queryResultCount)
		{
			var proxy:B2Proxy =  m_queryResults[i];
			//B2Settings.b2Assert(proxy.IsValid());
			if (!cb(proxy))
			{
				break;
			}
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
	}

	public function Validate() : Void{
		var pair:B2Pair;
		var proxy1:B2Proxy;
		var proxy2:B2Proxy;
		var overlap:Bool;
		
		for (axis in 0...2)
		{
			var bounds:Vector<B2Bound> = m_bounds[axis];
			
			var boundCount:Int = 2 * m_proxyCount;
			var stabbingCount:Int = 0;
			
			for (i in 0...boundCount)
			{
				var bound:B2Bound = bounds[i];
				//B2Settings.b2Assert(i == 0 || bounds[i-1].value <= bound->value);
				//B2Settings.b2Assert(bound->proxyId != b2_nullProxy);
				//B2Settings.b2Assert(m_proxyPool[bound->proxyId].IsValid());
				
				if (bound.IsLower() == true)
				{
					//B2Settings.b2Assert(m_proxyPool[bound.proxyId].lowerBounds[axis] == i);
					stabbingCount++;
				}
				else
				{
					//B2Settings.b2Assert(m_proxyPool[bound.proxyId].upperBounds[axis] == i);
					stabbingCount--;
				}
				
				//B2Settings.b2Assert(bound.stabbingCount == stabbingCount);
			}
		}
		
	}

	public function Rebalance(iterations:Int):Void
	{
		// Do nothing
	}

	
	/**
	 * @inheritDoc
	 */
	public function RayCast(cb:Dynamic, input:B2RayCastInput):Void
	{
		var subInput:B2RayCastInput = new B2RayCastInput();
		subInput.p1.SetV(input.p1);
		subInput.p2.SetV(input.p2);
		subInput.maxFraction = input.maxFraction;
		
		
		var dx:Float = (input.p2.x-input.p1.x)*m_quantizationFactor.x;
		var dy:Float = (input.p2.y-input.p1.y)*m_quantizationFactor.y;
		
		var sx:Int = dx<-box2D.common.math.B2Math.MIN_VALUE ? -1 : (dx>box2D.common.math.B2Math.MIN_VALUE ? 1 : 0);
		var sy:Int = dy<-box2D.common.math.B2Math.MIN_VALUE ? -1 : (dy>box2D.common.math.B2Math.MIN_VALUE ? 1 : 0);
		
		//B2Settings.b2Assert(sx!=0||sy!=0);
		
		var p1x:Float = m_quantizationFactor.x * (input.p1.x - m_worldAABB.lowerBound.x);
		var p1y:Float = m_quantizationFactor.y * (input.p1.y - m_worldAABB.lowerBound.y);
		
		var startValues:Array<Dynamic> = new Array();
		var startValues2:Array<Dynamic> = new Array();
		startValues[0]=Std.int(p1x) & (B2Settings.USHRT_MAX - 1);
		startValues[1]=Std.int(p1y) & (B2Settings.USHRT_MAX - 1);
		startValues2[0]=startValues[0]+1;
		startValues2[1]=startValues[1]+1;
		
		var startIndices:Array<Dynamic> = new Array();
		
		var xIndex:Int;
		var yIndex:Int;
		
		var proxy:B2Proxy;
		
		
		//First deal with all the proxies that contain segment.p1
		var lowerIndex:Int = 0;
		var upperIndex:Int = 0;
		var lowerIndexOut:Vector<Int> = new Vector<Int>(); 
		lowerIndexOut.push(lowerIndex);
		var upperIndexOut:Vector<Int> = new Vector<Int>();
		upperIndexOut.push(upperIndex);
		QueryAxis(lowerIndexOut, upperIndexOut, startValues[0], startValues2[0], m_bounds[0], 2*m_proxyCount, 0);
		if(sx>=0)	xIndex = upperIndexOut[0]-1;
		else		xIndex = lowerIndexOut[0];
		QueryAxis(lowerIndexOut, upperIndexOut, startValues[1], startValues2[1], m_bounds[1], 2*m_proxyCount, 1);
		if(sy>=0)	yIndex = upperIndexOut[0]-1;
		else		yIndex = lowerIndexOut[0];
			
		// Callback for starting proxies:
		for (i in 0...m_queryResultCount) {
			subInput.maxFraction = cb(m_queryResults[i], subInput);
		}
		
		//Now work through the rest of the segment
		while (true)
		{
			var xProgress:Float = 0;
			var yProgress:Float = 0;
			//Move on to next bound
			xIndex += sx >= 0?1: -1;
			if(xIndex<0||xIndex>=m_proxyCount*2)
				break;
			if(sx!=0){
				xProgress = (m_bounds[0][xIndex].value - p1x) / dx;
			}
			//Move on to next bound
			yIndex += sy >= 0?1: -1;
			if(yIndex<0||yIndex>=m_proxyCount*2)
				break;
			if(sy!=0){
				yProgress = (m_bounds[1][yIndex].value - p1y) / dy;	
			}
			while (true)
			{	
				if(sy==0||(sx!=0&&xProgress<yProgress)){
					if(xProgress>subInput.maxFraction)
						break;
					
					//Check that we are entering a proxy, not leaving
					if(sx>0?m_bounds[0][xIndex].IsLower():m_bounds[0][xIndex].IsUpper()){
						//Check the other axis of the proxy
						proxy = m_bounds[0][xIndex].proxy;
						if(sy>=0){
							if(proxy.lowerBounds[1]<=yIndex-1&&proxy.upperBounds[1]>=yIndex){
								//Add the proxy
								subInput.maxFraction = cb(proxy, subInput);
							}
						}else{
							if(proxy.lowerBounds[1]<=yIndex&&proxy.upperBounds[1]>=yIndex+1){
								//Add the proxy
								subInput.maxFraction = cb(proxy, subInput);
							}
						}
					}
					
					//Early out
					if(subInput.maxFraction==0)
						break;
					
					//Move on to the next bound
					if(sx>0){
						xIndex++;
						if(xIndex==m_proxyCount*2)
							break;
					}else{
						xIndex--;
						if(xIndex<0)
							break;
					}
					xProgress = (m_bounds[0][xIndex].value - p1x) / dx;
				}else{
					if(yProgress>subInput.maxFraction)
						break;
					
					//Check that we are entering a proxy, not leaving
					if(sy>0?m_bounds[1][yIndex].IsLower():m_bounds[1][yIndex].IsUpper()){
						//Check the other axis of the proxy
						proxy = m_bounds[1][yIndex].proxy;
						if(sx>=0){
							if(proxy.lowerBounds[0]<=xIndex-1&&proxy.upperBounds[0]>=xIndex){
								//Add the proxy
								subInput.maxFraction = cb(proxy, subInput);
							}
						}else{
							if(proxy.lowerBounds[0]<=xIndex&&proxy.upperBounds[0]>=xIndex+1){
								//Add the proxy
								subInput.maxFraction = cb(proxy, subInput);
							}
						}
					}
					
					//Early out
					if(subInput.maxFraction==0)
						break;
					
					//Move on to the next bound
					if(sy>0){
						yIndex++;
						if(yIndex==m_proxyCount*2)
							break;
					}else{
						yIndex--;
						if(yIndex<0)
							break;
					}
					yProgress = (m_bounds[1][yIndex].value - p1y) / dy;
				}
			}
			break;
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		return;
	}
	
//private:
	private function ComputeBounds(lowerValues:Vector<Int>, upperValues:Vector<Int>, aabb:B2AABB) : Void
	{
		//B2Settings.b2Assert(aabb.upperBound.x >= aabb.lowerBound.x);
		//B2Settings.b2Assert(aabb.upperBound.y >= aabb.lowerBound.y);
		
		//var minVertex:B2Vec2 = B2Math.ClampV(aabb.minVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
		var minVertexX:Float = aabb.lowerBound.x;
		var minVertexY:Float = aabb.lowerBound.y;
		minVertexX = B2Math.Min(minVertexX, m_worldAABB.upperBound.x);
		minVertexY = B2Math.Min(minVertexY, m_worldAABB.upperBound.y);
		minVertexX = B2Math.Max(minVertexX, m_worldAABB.lowerBound.x);
		minVertexY = B2Math.Max(minVertexY, m_worldAABB.lowerBound.y);
		
		//var maxVertex:B2Vec2 = B2Math.ClampV(aabb.maxVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
		var maxVertexX:Float = aabb.upperBound.x;
		var maxVertexY:Float = aabb.upperBound.y;
		maxVertexX = B2Math.Min(maxVertexX, m_worldAABB.upperBound.x);
		maxVertexY = B2Math.Min(maxVertexY, m_worldAABB.upperBound.y);
		maxVertexX = B2Math.Max(maxVertexX, m_worldAABB.lowerBound.x);
		maxVertexY = B2Math.Max(maxVertexY, m_worldAABB.lowerBound.y);
		
		// Bump lower bounds downs and upper bounds up. This ensures correct sorting of
		// lower/upper bounds that would have equal values.
		// TODO_ERIN implement fast float to uint16 conversion.
		lowerValues[0] = Std.int(m_quantizationFactor.x * (minVertexX - m_worldAABB.lowerBound.x)) & (B2Settings.USHRT_MAX - 1);
		upperValues[0] = (Std.int(m_quantizationFactor.x * (maxVertexX - m_worldAABB.lowerBound.x))& 0x0000ffff) | 1;
		
		lowerValues[1] = Std.int(m_quantizationFactor.y * (minVertexY - m_worldAABB.lowerBound.y)) & (B2Settings.USHRT_MAX - 1);
		upperValues[1] = (Std.int(m_quantizationFactor.y * (maxVertexY - m_worldAABB.lowerBound.y))& 0x0000ffff) | 1;
	}

	// This one is only used for validation.
	function TestOverlapValidate(p1:B2Proxy, p2:B2Proxy):Bool{
		
		for (axis in 0...2)
		{
			var bounds:Vector<B2Bound> = m_bounds[axis];
			
			//B2Settings.b2Assert(p1.lowerBounds[axis] < 2 * m_proxyCount);
			//B2Settings.b2Assert(p1.upperBounds[axis] < 2 * m_proxyCount);
			//B2Settings.b2Assert(p2.lowerBounds[axis] < 2 * m_proxyCount);
			//B2Settings.b2Assert(p2.upperBounds[axis] < 2 * m_proxyCount);
			
			var bound1:B2Bound = bounds[p1.lowerBounds[axis]];
			var bound2:B2Bound = bounds[p2.upperBounds[axis]];
			if (bound1.value > bound2.value)
				return false;
			
			bound1 = bounds[p1.upperBounds[axis]];
			bound2 = bounds[p2.lowerBounds[axis]];
			if (bound1.value < bound2.value)
				return false;
		}
		
		return true;
	}
	
	public function TestOverlapBound(b:B2BoundValues, p:B2Proxy):Bool
	{
		for (axis in 0...2)
		{
			var bounds:Vector<B2Bound> = m_bounds[axis];
			
			//B2Settings.b2Assert(p.lowerBounds[axis] < 2 * m_proxyCount);
			//B2Settings.b2Assert(p.upperBounds[axis] < 2 * m_proxyCount);
			
			var bound:B2Bound = bounds[p.upperBounds[axis]];
			if (b.lowerValues[axis] > bound.value)
				return false;
			
			bound = bounds[p.lowerBounds[axis]];
			if (b.upperValues[axis] < bound.value)
				return false;
		}
		
		return true;
	}

	private function QueryAxis(lowerQueryOut:Vector<Int>, upperQueryOut:Vector<Int>, lowerValue:Int, upperValue:Int, bounds:Vector<B2Bound>, boundCount:Int, axis:Int) : Void{
		
		var lowerQuery:Int = BinarySearch(bounds, boundCount, lowerValue);
		var upperQuery:Int = BinarySearch(bounds, boundCount, upperValue);
		var bound: B2Bound;
		
		// Easy case: lowerQuery <= lowerIndex(i) < upperQuery
		// Solution: search query range for min bounds.
		var j:Int = lowerQuery;
		while (j < upperQuery)
		{
			bound = bounds[j];
			if (bound.IsLower())
			{
				IncrementOverlapCount(bound.proxy);
			}
			++j;
		}
		
		// Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
		// Solution: use the stabbing count to search down the bound array.
		if (lowerQuery > 0)
		{
			var i:Int = lowerQuery - 1;
			bound = bounds[i];
			var s:Int = bound.stabbingCount;
			
			// Find the s overlaps.
			while (s != 0)
			{
				//B2Settings.b2Assert(i >= 0);
				bound = bounds[i];
				if (bound.IsLower())
				{
					var proxy:B2Proxy = bound.proxy;
					if (lowerQuery <= proxy.upperBounds[axis])
					{
						IncrementOverlapCount(bound.proxy);
						--s;
					}
				}
				--i;
			}
		}
		
		lowerQueryOut[0] = lowerQuery;
		upperQueryOut[0] = upperQuery;
	}

	function IncrementOverlapCount(proxy:B2Proxy) : Void{
		if (proxy.timeStamp < m_timeStamp)
		{
			proxy.timeStamp = m_timeStamp;
			proxy.overlapCount = 1;
		}
		else
		{
			proxy.overlapCount = 2;
			//B2Settings.b2Assert(m_queryResultCount < B2Settings.b2_maxProxies);
			m_queryResults[m_queryResultCount] = proxy;
			++m_queryResultCount;
		}
	}
	function IncrementTimeStamp() : Void{
		if (m_timeStamp == B2Settings.USHRT_MAX)
		{
			for (i in 0...m_proxyPool.length)
			{
				(cast( m_proxyPool[i], B2Proxy)).timeStamp = 0;
			}
			m_timeStamp = 1;
		}
		else
		{
			++m_timeStamp;
		}
	}
	
	var m_pairManager:B2PairManager ;

	var m_proxyPool:Array<Dynamic> ;
	var m_freeProxy:B2Proxy;

	var m_bounds:Vector<Vector<B2Bound>> ;

	var m_querySortKeys:Array<Dynamic> ;
	var m_queryResults:Array<Dynamic> ;
	var m_queryResultCount:Int;

	var m_worldAABB:B2AABB;
	var m_quantizationFactor:B2Vec2 ;
	var m_proxyCount:Int;
	var m_timeStamp:Int;

	public static var s_validate:Bool = false;
	
	public static var b2_invalid:Int = B2Settings.USHRT_MAX;
	public static var b2_nullEdge:Int = B2Settings.USHRT_MAX;


	static public function BinarySearch(bounds:Vector<B2Bound>, count:Int, value:Int):Int
	{
		var low:Int = 0;
		var high:Int = count - 1;
		while (low <= high)
		{
			var mid:Int = Std.int((low + high) / 2);
			var bound:B2Bound = bounds[mid];
			if (bound.value > value)
			{
				high = mid - 1;
			}
			else if (bound.value < value)
			{
				low = mid + 1;
			}
			else
			{
				return mid;
			}
		}
		
		return low;
	}
	
	
}
