package box2D.collision;

import flash.Vector;

import box2D.common.math.B2Vec2;
	
/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 */
class B2DynamicTreeBroadPhase implements IBroadPhase {
	/**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * UpdatePairs is called.
	 */
	public function new() { 
		m_tree = new B2DynamicTree();
		m_moveBuffer = new Vector<B2DynamicTreeNode>();
		m_pairBuffer = new Vector<B2DynamicTreePair>();
		m_pairCount = 0;
	}
	
	/**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * UpdatePairs is called.
	 */
	public function CreateProxy(aabb:B2AABB, userData:Dynamic):Dynamic
	{
		var proxy:B2DynamicTreeNode = m_tree.CreateProxy(aabb, userData);
		++m_proxyCount;
		BufferMove(proxy);
		return proxy;
	}
	
	/**
	 * Destroy a proxy. It is up to the client to remove any pairs.
	 */
	public function DestroyProxy(proxy:Dynamic):Void
	{
		UnBufferMove(proxy);
		--m_proxyCount;
		m_tree.DestroyProxy(proxy);
	}
	
	/**
	 * Call MoveProxy as many times as you like, then when you are done
	 * call UpdatePairs to finalized the proxy pairs (for your time step).
	 */
	public function MoveProxy(proxy:Dynamic, aabb:B2AABB, displacement:B2Vec2):Void
	{
		var buffer:Bool = m_tree.MoveProxy(proxy, aabb, displacement);
		if (buffer)
		{
			BufferMove(proxy);
		}
	}
	
	public function TestOverlap(proxyA:Dynamic, proxyB:Dynamic):Bool
	{
		var aabbA:B2AABB = m_tree.GetFatAABB(proxyA);
		var aabbB:B2AABB = m_tree.GetFatAABB(proxyB);
		return aabbA.TestOverlap(aabbB);
	}
	
	/**
	 * Get user data from a proxy. Returns null if the proxy is invalid.
	 */
	public function GetUserData(proxy:Dynamic):Dynamic
	{
		return m_tree.GetUserData(proxy);
	}
	
	/**
	 * Get the AABB for a proxy.
	 */
	public function GetFatAABB(proxy:Dynamic):B2AABB
	{
		return m_tree.GetFatAABB(proxy);
	}
	
	/**
	 * Get the number of proxies.
	 */
	public function GetProxyCount():Int
	{
		return m_proxyCount;
	}
	
	/**
	 * Update the pairs. This results in pair callbacks. This can only add pairs.
	 */
	public function UpdatePairs(cb:Dynamic):Void
	{
		m_pairCount = 0;
		// Perform tree queries for all moving queries
		for (queryProxy in m_moveBuffer)
		{
			var self = this;
			var queryCallback = function(proxy:B2DynamicTreeNode):Bool
			{
				// A proxy cannot form a pair with itself.
				if (proxy == queryProxy)
					return true;
					
				// Grow the pair buffer as needed
				if (self.m_pairCount == cast self.m_pairBuffer.length)
				{
					self.m_pairBuffer[self.m_pairCount] = new B2DynamicTreePair();
				}
				
				var pair:B2DynamicTreePair = self.m_pairBuffer[self.m_pairCount];
				pair.proxyA = proxy;
				pair.proxyB = queryProxy;
				++self.m_pairCount;
				
				return true;
			}
			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			var fatAABB:B2AABB = m_tree.GetFatAABB(queryProxy);
			m_tree.Query(queryCallback, fatAABB);
		}
		
		// Reset move buffer
		m_moveBuffer.length = 0;
		
		// Sort the pair buffer to expose duplicates.
		// TODO: Something more sensible
		//m_pairBuffer.sort(ComparePairs);
		
		// Send the pair buffer
		var i:Int = 0;
		while (i < m_pairCount)
		{
			var primaryPair:B2DynamicTreePair = m_pairBuffer[i];
			var userDataA:Dynamic = m_tree.GetUserData(primaryPair.proxyA);
			var userDataB:Dynamic = m_tree.GetUserData(primaryPair.proxyB);
			cb(userDataA, userDataB);
			++i;
			
			// Skip any duplicate pairs
			while (i < m_pairCount)
			{
				var pair:B2DynamicTreePair = m_pairBuffer[i];
				if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB)
				{
					break;
				}
				++i;
			}
		}
	}
	
	/**
	 * @inheritDoc
	 */
	public function Query(cb:Dynamic, aabb:B2AABB):Void
	{
		m_tree.Query(cb, aabb);
	}
	
	/**
	 * @inheritDoc
	 */
	public function RayCast(cb:Dynamic, input:B2RayCastInput):Void
	{
		m_tree.RayCast(cb, input);
	}
	
	
	public function Validate():Void
	{
		//TODO_BORIS
	}
	
	public function Rebalance(iterations:Int):Void
	{
		m_tree.Rebalance(iterations);
	}
	
	
	// Private ///////////////
	
	function BufferMove(proxy:B2DynamicTreeNode):Void
	{
		m_moveBuffer[m_moveBuffer.length] = proxy;
	}
	
	function UnBufferMove(proxy:B2DynamicTreeNode):Void
	{
		var i:Int = m_moveBuffer.indexOf(proxy);
		m_moveBuffer.splice(i, 1);
	}
	
	function ComparePairs(pair1:B2DynamicTreePair, pair2:B2DynamicTreePair):Int
	{
		//TODO_BORIS:
		// We cannot consistently sort objects easily in AS3
		// The caller of this needs replacing with a different method.
		return 0;
	}
	var m_tree:B2DynamicTree ;
	var m_proxyCount:Int;
	private var m_moveBuffer:Vector<B2DynamicTreeNode>;
	
	private var m_pairBuffer:Vector<B2DynamicTreePair>;
	var m_pairCount:Int ;
}
	
