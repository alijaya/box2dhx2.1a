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

/**
* @private
*/
class B2PairManager
 {
//public:
	
//public:
	public function new(){
		m_pairs = new Array();
		m_pairBuffer = new Array();
		m_pairCount = 0;
		m_pairBufferCount = 0;
		m_freePair = null;
	}
	//~B2PairManager();
	
	public function Initialize(broadPhase:B2BroadPhase) : Void{
		m_broadPhase = broadPhase;
	}
	
	/*
	As proxies are created and moved, many pairs are created and destroyed. Even worse, the same
	pair may be added and removed multiple times in a single time step of the physics engine. To reduce
	traffic in the pair manager, we try to avoid destroying pairs in the pair manager until the
	end of the physics step. This is done by buffering all the RemovePair requests. AddPair
	requests are processed immediately because we need the hash table entry for quick lookup.

	All user user callbacks are delayed until the buffered pairs are confirmed in Commit.
	This is very important because the user callbacks may be very expensive and client logic
	may be harmed if pairs are added and removed within the same time step.

	Buffer a pair for addition.
	We may add a pair that is not in the pair manager or pair buffer.
	We may add a pair that is already in the pair manager and pair buffer.
	If the added pair is not a new pair, then it must be in the pair buffer (because RemovePair was called).
	*/
	public function AddBufferedPair(proxy1:B2Proxy, proxy2:B2Proxy) : Void{
		//B2Settings.b2Assert(proxy1 && proxy2);
		
		var pair:B2Pair = AddPair(proxy1, proxy2);
		
		// If this pair is not in the pair buffer ...
		if (pair.IsBuffered() == false)
		{
			// This must be a newly added pair.
			//B2Settings.b2Assert(pair.IsFinal() == false);
			
			// Add it to the pair buffer.
			pair.SetBuffered();
			m_pairBuffer[m_pairBufferCount] = pair;
			++m_pairBufferCount;
			//B2Settings.b2Assert(m_pairBufferCount <= m_pairCount);
		}
		
		// Confirm this pair for the subsequent call to Commit.
		pair.ClearRemoved();
		
		if (B2BroadPhase.s_validate)
		{
			ValidateBuffer();
		}
	}
	
	// Buffer a pair for removal.
	public function RemoveBufferedPair(proxy1:B2Proxy, proxy2:B2Proxy) : Void{
		//B2Settings.b2Assert(proxy1 && proxy2);
		
		var pair:B2Pair = Find(proxy1, proxy2);
		
		if (pair == null)
		{
			// The pair never existed. This is legal (due to collision filtering).
			return;
		}
		
		// If this pair is not in the pair buffer ...
		if (pair.IsBuffered() == false)
		{
			// This must be an old pair.
			//B2Settings.b2Assert(pair.IsFinal() == true);
			
			pair.SetBuffered();
			m_pairBuffer[m_pairBufferCount] = pair;
			++m_pairBufferCount;
			
			//B2Settings.b2Assert(m_pairBufferCount <= m_pairCount);
		}
		
		pair.SetRemoved();
		
		if (B2BroadPhase.s_validate)
		{
			ValidateBuffer();
		}
	}
	
	public function Commit(cb:Dynamic) : Void{
		var i:Int;
		
		var removeCount:Int = 0;
		
		for (i in 0...m_pairBufferCount)
		{
			var pair:B2Pair = m_pairBuffer[i];
			//B2Settings.b2Assert(pair.IsBuffered());
			pair.ClearBuffered();
			
			//B2Settings.b2Assert(pair.proxy1 && pair.proxy2);
			
			var proxy1:B2Proxy = pair.proxy1;
			var proxy2:B2Proxy = pair.proxy2;
			
			//B2Settings.b2Assert(proxy1.IsValid());
			//B2Settings.b2Assert(proxy2.IsValid());
			
			if (pair.IsRemoved())
			{
				// It is possible a pair was added then removed before a commit. Therefore,
				// we should be careful not to tell the user the pair was removed when the
				// the user didn't receive a matching add.
				//if (pair.IsFinal() == true)
				//{
				//	m_callback.PairRemoved(proxy1.userData, proxy2.userData, pair.userData);
				//}
				
				// Store the ids so we can actually remove the pair below.
				//m_pairBuffer[removeCount] = pair;
				//++removeCount;
			}
			else
			{
				//B2Settings.b2Assert(m_broadPhase.TestOverlap(proxy1, proxy2) == true);
				
				if (pair.IsFinal() == false)
				{
					//pair.userData = m_callback.PairAdded(proxy1.userData, proxy2.userData);
					//pair.SetFinal();
					cb(proxy1.userData, proxy2.userData);
				}
			}
		}
		
		//for (i = 0; i < removeCount; ++i)
		//{
		//	pair = m_pairBuffer[i]
		//	RemovePair(pair.proxy1, pair.proxy2);
		//}
		
		m_pairBufferCount = 0;
		
		if (B2BroadPhase.s_validate)
		{
			ValidateTable();
		}	
	}

//private:

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	function AddPair(proxy1:B2Proxy, proxy2:B2Proxy):B2Pair
	{
		var pair:B2Pair = proxy1.pairs.get(proxy2);
		
		if (pair != null)
			return pair;
		
		if (m_freePair == null)
		{
			m_freePair = new B2Pair();
			m_pairs.push(m_freePair);
		}
		pair = m_freePair;
		m_freePair = pair.next;
		
		pair.proxy1 = proxy1;
		pair.proxy2 = proxy2;
		pair.status = 0;
		pair.userData = null;
		pair.next = null;
		
		proxy1.pairs.set(proxy2, pair);
		proxy2.pairs.set(proxy1, pair);
				
		++m_pairCount;
		
		return pair;
	}

	// Remove a pair, return the pair's userData.
	function RemovePair(proxy1:B2Proxy, proxy2:B2Proxy):Dynamic
	{
		//B2Settings.b2Assert(m_pairCount > 0);
		
		var pair:B2Pair = proxy1.pairs.get(proxy2);
		
		if (pair == null)
		{
			//B2Settings.b2Assert(false);
			return null;
		}
		
		var userData:Dynamic = pair.userData;
		
		proxy1.pairs.delete(proxy2);
		proxy2.pairs.delete(proxy1);
		
		// Scrub
		pair.next = m_freePair;
		pair.proxy1 = null;
		pair.proxy2 = null;
		pair.userData = null;
		pair.status = 0;
		
		m_freePair = pair;
		--m_pairCount;
		return userData;
	}

	function Find(proxy1:B2Proxy, proxy2:B2Proxy):B2Pair{
		
		return proxy1.pairs.get(proxy2);
	}
	
	function ValidateBuffer() : Void{
		// DEBUG
	}
	
	function ValidateTable() : Void{
		// DEBUG
	}

//public:
	public var m_broadPhase:B2BroadPhase;
	public var m_pairs:Array<Dynamic>;
	public var m_freePair:B2Pair;
	public var m_pairCount:Int;
	
	public var m_pairBuffer:Array<Dynamic>;
	public var m_pairBufferCount:Int;
	
}

