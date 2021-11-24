#include "test.h"
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <stack>
#include <random>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>

// ------------------------------------------------------------------------------------------------

class RaycastCallback : public b2RayCastCallback
{
public:

	// These are the box2D return values that control, whether or not the raycast continues after a collision is encountered
	// (it's briefly mentioned in the box2D RayCastCallback::ReportFixture documentation):
	//
	// return				-1 = ignore this fixture and continue
	// return				 0 = terminate the ray cast
	// return	fraction (0,1) = clip the ray to this point
	// return				 1 = don't clip the ray and continue
	virtual float ReportFixture( b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float fraction ) override
	{
		m_collisionHit = true;
		m_point = point;
		m_normal = normal;
		m_fraction = fraction;

		return 0.5f;
	}

	void Reset()
	{
		m_collisionHit = false;
		m_point = b2Vec2_zero;
		m_normal = b2Vec2_zero;
		m_fraction = 0;
	}

	bool	m_collisionHit{ false };
	b2Vec2	m_point{ b2Vec2_zero };
	b2Vec2	m_normal{ b2Vec2_zero };
	float	m_fraction{ 0 };
};

// ------------------------------------------------------------------------------------------------

class Tire
{
public:
	Tire( b2World* world, b2Vec2 startingPosition = b2Vec2_zero ) 
	{
		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set( startingPosition.x + 0.0f, startingPosition.y + 3.0 );

		m_body = world->CreateBody( &bodyDef );

		b2PolygonShape polygonShape;
		polygonShape.SetAsBox( 0.65f, 1.25f );
		m_body->CreateFixture( &polygonShape, 1 );
	}

	~Tire() 
	{
		m_body->GetWorld()->DestroyBody( m_body );
	}

	void update( bool showDebug )
	{
		// Reduce lateral velocity to avoid the zero-gravity-spinning-into-space effect
		// (reduce to almost zero instead of removing it altogether, as the latter
		// makes implementing drifting much more complicated and also creates this
		// very jarring effect of instant traction once the player releases the drift/handbrake button)
		const b2Vec2 currentRightNormal = m_body->GetWorldVector( b2Vec2( 1, 0 ) );
		const float lateralVelocity = b2Dot( currentRightNormal, m_body->GetLinearVelocity() );

		float reductionCoefficient{ 0.9f };
		//float reductionCoefficient{ 0 }; // try it, see if anything good comes out of it

		if( glfwGetKey( g_mainWindow, GLFW_KEY_LEFT_SHIFT ) == GLFW_PRESS )
		{
			reductionCoefficient = 0.05f;
		}

		const b2Vec2 scaledLateralVelocityVector = reductionCoefficient * lateralVelocity * currentRightNormal;
		const b2Vec2 impulse = m_body->GetMass() * -scaledLateralVelocityVector;
		m_body->ApplyLinearImpulse( impulse, m_body->GetWorldCenter(), true );

		if ( showDebug )
		{
			m_lastLateralVelocityVector = scaledLateralVelocityVector;
		}

		// apply backwards drag
		b2Vec2 currentForwardNormal = m_body->GetWorldVector( b2Vec2( 0, 1 ) );
		const float forwardVelocity = b2Dot( currentForwardNormal, m_body->GetLinearVelocity() ) ;

		b2Vec2 forwardVelocityVector = forwardVelocity * currentForwardNormal;

		const float currentForwardSpeed = forwardVelocityVector.Normalize();
		const float dragForceMagnitude = -2 * currentForwardSpeed;
		m_body->ApplyForce( dragForceMagnitude * currentForwardNormal, m_body->GetWorldCenter(), true );

		// don't apply forward force if the fictional handbrake is pressed
		if ( glfwGetKey( g_mainWindow, GLFW_KEY_LEFT_SHIFT ) != GLFW_PRESS )
		{
			if ( glfwGetKey( g_mainWindow, GLFW_KEY_W ) == GLFW_PRESS )
			{
				b2Vec2 force = m_body->GetWorldVector( b2Vec2( 0.0f, m_forwardForce ) );
				b2Vec2 point = m_body->GetWorldPoint( b2Vec2( 0.0f, -3.0f ) );
				m_body->ApplyForce( force, point, true );
			}

			if ( glfwGetKey( g_mainWindow, GLFW_KEY_S ) == GLFW_PRESS )
			{
				const b2Vec2 force = m_body->GetWorldVector( b2Vec2( 0.0f, m_backwardForce ) );
				const b2Vec2 point = m_body->GetWorldPoint( b2Vec2( 0.0f, 3.0f ) );
				m_body->ApplyForce( force, point, true );
			}
		}

		if ( m_canTurn )
		{
			if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) == GLFW_PRESS )
			{
				m_body->ApplyTorque( m_turnForce, true );
			}

			if ( glfwGetKey( g_mainWindow, GLFW_KEY_D ) == GLFW_PRESS )
			{
				m_body->ApplyTorque( -m_turnForce, true );
			}
		}
	}

	b2Body* m_body;
	bool	m_canTurn{ false };

	b2Vec2	m_lastLateralVelocityVector;

private:
	float m_forwardForce{ 150 };
	float m_backwardForce{ -40 };

	float m_turnForce{ 15 };

	float m_maxLateralVelocity{ 2 };
};

// ------------------------------------------------------------------------------------------------

class Car
{
public:
	Car( b2World* world , b2Vec2 startingPosition = b2Vec2_zero )
	{
		//create car body
		b2BodyDef bodyDef;
		bodyDef.position = startingPosition;
		bodyDef.type = b2_dynamicBody;
		m_body = world->CreateBody( &bodyDef );

		b2Vec2 vertices[ 8 ];
		vertices[ 0 ].Set( 2.25, 0 );
		vertices[ 1 ].Set( 2.5, 2.5 );
		vertices[ 2 ].Set( 2.5, 8 );
		vertices[ 3 ].Set( 2.25, 10 );
		vertices[ 4 ].Set( -2.25, 10 );
		vertices[ 5 ].Set( -2.5, 8 );
		vertices[ 6 ].Set( -2.5, 2.5 );
		vertices[ 7 ].Set( -2.25, 0 );
		b2PolygonShape polygonShape;
		polygonShape.Set( vertices, 8 );
		m_fixture = m_body->CreateFixture( &polygonShape, 0.1f );

		b2RevoluteJointDef jointDef;
		jointDef.bodyA = m_body;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = 0;
		jointDef.upperAngle = 0;
		jointDef.localAnchorB.SetZero();

		Tire* tire = new Tire( world, startingPosition );
		jointDef.bodyB = tire->m_body;
		jointDef.localAnchorA.Set( -2, 9 );
		m_frontLeftJoint = static_cast< b2RevoluteJoint* >( world->CreateJoint( &jointDef ) );
		tire->m_canTurn = true;
		m_tires.push_back( tire );


		tire = new Tire( world, startingPosition );
		jointDef.bodyB = tire->m_body;
		jointDef.localAnchorA.Set( 2, 9 );
		m_frontRightJoint = static_cast< b2RevoluteJoint* >( world->CreateJoint( &jointDef ) );
		tire->m_canTurn = true;
		m_tires.push_back( tire );

		tire = new Tire( world, startingPosition );
		jointDef.bodyB = tire->m_body;
		jointDef.localAnchorA.Set( -2, 1 );
		world->CreateJoint( &jointDef );
		m_tires.push_back( tire );

		tire = new Tire( world, startingPosition );
		jointDef.bodyB = tire->m_body;
		jointDef.localAnchorA.Set( 2, 1 );
		world->CreateJoint( &jointDef );
		m_tires.push_back( tire );

		m_world = world;
	}

	~Car()
	{
		m_body->GetWorld()->DestroyBody( m_body );
	}

	void update( bool showDebug )
	{
		// Enable/disable self-driving (wandering)
		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS )
		{
			m_wanderingEnabled = !m_wanderingEnabled;
		}

		// Turn logic
		// 
		// Car turning is done by adjusting
		// each joint, but force is applied per-wheel;
		// 
		// Hence, turning is applied first, here,
		// and then directional force is applied 
		// in the update function of each tire.

		const float angle = m_turnAngle * M_PI / 180;
		const float turnSpeed = m_turnRate * M_PI / 180;

		const float turnPerTimeStep = turnSpeed / 60.0f;

		float desiredAngle{ 0 };

		if ( !m_wanderingEnabled )
		{
			if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) == GLFW_PRESS )
			{
				desiredAngle = angle;
			}

			if ( glfwGetKey( g_mainWindow, GLFW_KEY_D ) == GLFW_PRESS )
			{
				desiredAngle = -angle;
			}
		}
		else
		{
			desiredAngle = wander( showDebug );
		}

		const float currentAngle = m_frontLeftJoint->GetJointAngle();
		float angleToTurn = desiredAngle - currentAngle;
		angleToTurn = b2Clamp( angleToTurn, -turnPerTimeStep, turnPerTimeStep );

		float newAngle = currentAngle + angleToTurn;
		m_frontLeftJoint->SetLimits( newAngle, newAngle );
		m_frontRightJoint->SetLimits( newAngle, newAngle );

		for ( int i = 0; i < m_tires.size(); ++i )
		{
			m_tires[ i ]->update( showDebug );
		}
	}

	/// <summary>
	/// Wandering logic
	/// </summary>
	/// 
	/// <param name="showDebug"> Flag to enable drawing debug vectors for rays, velocity etc. </param>
	/// <returns> A turning angle to pass down to the tires </returns>
	float wander( bool showDebug )
	{
		b2Vec2 wanderVelocity = m_nextTarget;

		// If there's no next target, wander randomly 
		if ( m_nextTarget == b2Vec2_zero )
		{
			// Get a point in front of the car
			wanderVelocity = m_body->GetWorldVector( m_wanderDirection );

			// Calculate a point on a circle around the wanderPoint
			m_wanderAngle += RandomFloat( -0.1f, 0.1f );

			float x = m_wanderRadius * cos( m_wanderAngle );
			float y = m_wanderRadius * sin( m_wanderAngle );

			// Add the current velocity and the new wander vector
			const b2Vec2 circleVector{ x,y };
			wanderVelocity += circleVector;
		}

		const b2Vec2 avoidanceVector = avoidCollisions( showDebug );
		if ( avoidanceVector != b2Vec2_zero )
		{
			wanderVelocity += avoidanceVector;
		}

		// Clamp the velocity to a max magnitude
		float wanderVelocityMagnitude = wanderVelocity.Normalize();
		wanderVelocityMagnitude = b2Clamp( wanderVelocityMagnitude, -m_maxVelocity, m_maxVelocity );
		wanderVelocity *= wanderVelocityMagnitude;

		// Reduce velocity gradually, if within arrival distance of next target
		if ( m_nextTarget != b2Vec2_zero )
		{
			const b2Vec2 distanceToTarget = m_nextTarget - m_body->GetPosition();
			if ( distanceToTarget.Length() < m_arrivalDistance )
			{
				float wanderPointMagnitude = wanderVelocity.Normalize();
				wanderVelocity *= m_maxVelocity * ( distanceToTarget.Length() / m_arrivalDistance );
			}
		}

		m_body->ApplyForce( wanderVelocity, m_body->GetWorldPoint( b2Vec2( 0.0f, -3.0f ) ), true );
		
		// calculate turn angle vector to return to tires
		// (turn angle = angle between wanderVelocity & car forward vector)
		double turnAngle{ 0 };
		b2Vec2 forwardVector = m_body->GetWorldVector( m_wanderDirection );
		double dot = b2Dot( wanderVelocity, forwardVector );

		const double wLength = wanderVelocity.Length();
		const double fLength = forwardVector.Length();
		const double mag = wLength * fLength;

		turnAngle = acos( dot / mag );

		// clamp turn angle within some realistic car tire angles
		turnAngle = b2Clamp( turnAngle, -M_PI / 2.8f, M_PI / 2.8f );
		
		// FIXME LH:	this is a bit hacky - basically the angle is always positive from the 
		//				above calcualtion, but the rotation that needs to be applied to the tires
		//				is +ve to the right, and -ve to the left, so we fudge it;
		//				Is there a way to get it so it just goes negative and we don't need to do this?
		b2Vec2 turnDirection = wanderVelocity - forwardVector;
		b2Vec2 localTurnDirection = m_body->GetLocalVector( turnDirection );
		if( localTurnDirection.x > 0 && turnAngle > 0 )
		{
			turnAngle *= -1;
		}

		if ( showDebug )
		{
			m_debugWanderVector = wanderVelocity;
			m_debugVelocityVector = m_body->GetLinearVelocity();
			m_debugTurnAngle = turnAngle;
		}

		return turnAngle;
	}


	/// ------------------------------------------------------------------------
	b2Vec2 checkRaycastCollision( RaycastCallback* callback, bool showDebug )
	{
		if ( callback && callback->m_collisionHit )
		{
			// the avoidance vector is essentially just
			// the normal of the colliding
			const b2Vec2 avoidVector = m_collisionAvoidanceForce * callback->m_normal;

			if ( showDebug )
			{
				m_debugAvoidVector = avoidVector;
				m_debugAvoidPoint = callback->m_point;
			}

			callback->Reset();

			return avoidVector;
		}

		return b2Vec2_zero;
	}


	/// ------------------------------------------------------------------------
	/// <summary>
	/// Collision avoidance logic
	/// </summary>
	/// 
	/// <param name="showDebug"> Flag to show debug values & vectors </param>
	/// <returns> 
	/// Avoidance vector to add to current movement/velocity vector 
	/// (in this case it gets added to the wander vector)
	/// </returns>
	/// ------------------------------------------------------------------------
	b2Vec2 avoidCollisions( bool showDebug )
	{	
		const b2Vec2& carPosition = m_body->GetPosition();

		// get a point in front, on the left, and on the right of the car
		// cast ray from that point to a position in front/left/right of car
		b2Vec2 f1 = carPosition + m_body->GetWorldVector( m_carFront );

		const b2Vec2 ray( m_carFront.x, m_carFront.y + m_rayLength );
		b2Vec2 f2 = carPosition + m_body->GetWorldVector( ray );

		if ( showDebug )
		{
			m_debugFrontRay = m_body->GetWorldVector( ray );
		}

		b2Vec2 r1 = carPosition + m_body->GetWorldVector( m_carRightFront );

		const b2Vec2 rightRay(m_carRightFront.x + m_rayLength, m_carRightFront.y + m_rayLength );
		b2Vec2 r2 = carPosition + m_body->GetWorldVector( rightRay );

		if ( showDebug )
		{
			m_debugRightRay = m_body->GetWorldVector( rightRay );
		}

		b2Vec2 l1 = carPosition + m_body->GetWorldVector( m_carLeftFront );

		const b2Vec2 leftRay( m_carLeftFront.x - m_rayLength, m_carLeftFront.y + m_rayLength );
		b2Vec2 l2 = carPosition + m_body->GetWorldVector( leftRay );

		if ( showDebug )
		{
			m_debugLeftRay = m_body->GetWorldVector( leftRay );
		}

		// cast rays between f/r/l points 1 & 2 and catch any collisions 
		// using the passed-in callback class overloads
		m_world->RayCast( &m_raycastCallback, f1, f2 );
		m_world->RayCast( &m_rightRaycastCallback, r1, r2 );
		m_world->RayCast( &m_leftRaycastCallback, l1, l2 );

		// compile the final avoidance vector from the collisions caught in the 3 callbacks
		b2Vec2 avoidVector{ b2Vec2_zero };

		if ( showDebug )
		{
			m_debugAvoidVector = b2Vec2_zero;
		}

		avoidVector += checkRaycastCollision( &m_raycastCallback, showDebug );
		avoidVector += checkRaycastCollision( &m_leftRaycastCallback, showDebug );
		avoidVector += checkRaycastCollision( &m_rightRaycastCallback, showDebug );

		return avoidVector;
	}

	
	RaycastCallback		m_raycastCallback;
	RaycastCallback		m_leftRaycastCallback;
	RaycastCallback		m_rightRaycastCallback;

	std::vector<Tire*>	m_tires;

	// DEBUG values (used for drawing visual elements when showDebug == true)
	b2Vec2				m_debugFrontRay{ b2Vec2_zero };
	b2Vec2				m_debugRightRay{ b2Vec2_zero };
	b2Vec2				m_debugLeftRay{ b2Vec2_zero };

	b2Vec2				m_debugVelocityVector{ b2Vec2_zero };
	b2Vec2				m_debugWanderVector{ b2Vec2_zero };

	b2Vec2				m_debugAvoidPoint{ b2Vec2_zero };
	b2Vec2				m_debugAvoidVector{ b2Vec2_zero };

	// NON-DEBUG values
	b2Vec2				m_carFront{ 0, 10 };
	b2Vec2				m_carRightFront{ 2.5, 7 };
	b2Vec2				m_carLeftFront{ -2.5, 7 };
	
	b2Vec2				m_wanderDirection{ 0, 38 };

	b2Vec2				m_nextTarget{ b2Vec2_zero };
	b2Vec2				m_distanceToNextTarget{ b2Vec2_zero };

	b2World*			m_world{ nullptr };

	b2RevoluteJoint*	m_frontLeftJoint{ nullptr };
	b2RevoluteJoint*	m_frontRightJoint{ nullptr };

	b2Body*				m_body{ nullptr };
	b2Fixture*			m_fixture{ nullptr };

	int					m_turnRate{ 200 };
	int					m_turnAngle{ 32 };

	float				m_rayLength{ 10 };
	float				m_collisionAvoidanceForce{ 19 };
	float				m_wanderRadius{ 5 };

	float				m_arrivalDistance{ 25 };

	float				m_wanderAngle{ -M_PI / 2 };

	float				m_maxVelocity{ 38 };

	// DEBUG values (used for drawing visual elements when showDebug == true)
	float				m_debugTurnAngle{ 0 };

	bool				m_wanderingEnabled{ false };
};


// ------------------------------------------------------------------------------------------------
class Track;

namespace constants
{
	constexpr int8_t TRACK_POINT_COUNT{ 15 };
}

// ------------------------------------------------------------------------------------------------

class Rule
{

public:
	// Pass whole track to each rule, as rules might need to edit
	// different properties of the track
	virtual void Process( Track* track ) const = 0;
};

// ------------------------------------------------------------------------------------------------

class Track
{

public:

	void Create( b2Body* ground, b2EdgeShape* groundShape, b2FixtureDef* groundFixture, bool showDebug = false )
	{
		if ( ground && groundShape && groundFixture )
		{
			GeneratePoints( showDebug );
			ApplyRules();
			Draw( ground, groundShape, groundFixture );
		}
	}

	void GeneratePoints( bool showDebug = false )
	{
		// Random number generation in C++ seems to be a pretty wide topic, so 
		// feel free to use a different method if preferred, I just wanted to avoid the 
		// RandomFloat() testbed function and use a seeded way of generating the track so it's reproducable.

		std::random_device rd;

		const uint32_t seed = rd();	
		if ( showDebug )
		{
			m_debugSeed = seed;
		}

		std::mt19937 generator( seed );
		std::uniform_int_distribution<int32_t> distribution( -m_maxWidthHeight, m_maxWidthHeight );

		for ( size_t i{ 0 }; i < constants::TRACK_POINT_COUNT; ++i )
		{
			m_points[ i ].x = distribution( generator );
			m_points[ i ].y = distribution( generator );
		}
	}

	void ApplyRules()
	{
		for ( const Rule* rule : m_rules )
		{
			rule->Process( this );
		}
	}

	void FindParallelPoints( std::vector<b2Vec2>& parallelPoints, const b2Vec2& p1, const b2Vec2& p2, const float parallelDistance )
	{
		// Get distance vector between two points 
		// (from current point to prevPoint)
		b2Vec2 prev = p2 - p1;

		// Normalize to get direction
		prev.Normalize();

		// Rotate above unit vector counterclockwise 90 deg
		// (the rotation itself is achieved using rotation matrices 
		// more info: https://math.stackexchange.com/questions/363652/understanding-rotation-matrices)
		b2Vec2 rotatedPrev( -prev.y, prev.x );

		// Move rotated vector by the m_roadSize to get a point
		// parallel to the convex hull points so we can draw the
		// outer side of the road
		const b2Vec2 parallelP1 = p1 + parallelDistance * rotatedPrev;

		parallelPoints.push_back( parallelP1 );

		// Now get distance vector in the opposite direction and
		// rotate clockwise 90 deg to get a parallel point at the other end,
		// also on the outer side of the road
		b2Vec2 current = p1 - p2;
		current.Normalize();

		b2Vec2 rotatedCurrent( current.y, -current.x );
		const b2Vec2 parallelP2 = p2 + parallelDistance * rotatedCurrent;


		// All of these parallel points are stored
		// on a parallel stack that will be drawn at the end
		parallelPoints.push_back( parallelP2 );
	}
	
	void Draw( b2Body* ground, b2EdgeShape* groundShape, b2FixtureDef* groundFixture )
	{
		// For drawing the actual fixters,
		// 
		// I opted for a more generic approach,
		// leaving the rules internally to do as they wish with the 
		// passed-in Track's points array. Then here, we just
		// go through the resultant points array and draw the track 
		// from the points, in order (e.g. they come sorted in a round track,
		// after the ConvexHullRule has applied it's Graham Scan
		// convex hull algorithm, but the track doesn't need to know this).
		//
		// There's a code architecture conversation about how 
		// to do this correctly, so don't take this approach as cannon.
		
		std::vector<b2Vec2> outerPoints;
		b2Vec2 prev{ m_points[0] };

		for ( auto& point : m_points )
		{
			if ( point != prev && point != b2Vec2_zero )
			{
				FindParallelPoints( outerPoints, prev, point, m_roadSize );

				groundShape->SetTwoSided( prev, point );
				ground->CreateFixture( groundFixture );
			}

			prev = point;
		}

		// Need to make sure the start of the track is
		// present twice, so it's present for the first
		// and last lines to be drawn

		outerPoints.push_back( outerPoints[ 0 ] );
		prev = outerPoints[ 0 ];

		for ( auto& outerPoint : outerPoints )
		{
			if ( outerPoint != prev && outerPoint != b2Vec2_zero )
			{
				groundShape->SetTwoSided( prev, outerPoint );
				ground->CreateFixture( groundFixture );
			}

			prev = outerPoint;
		}

		m_startingPoint = b2Vec2( m_points[0].x, m_points[0].y - m_roadSize/4 );
	}

	b2Vec2		m_startingPoint{ b2Vec2_zero };
	b2Vec2		m_points[ constants::TRACK_POINT_COUNT ];

	Rule*		m_rules[ 1 ];

	float		m_roadSize{ 35.0f };
	int32_t		m_maxWidthHeight{ 150 };
	uint32_t	m_debugSeed{ 0 };
};


/// ------------------------------------------------------------------------------------------------
/// <summary>
/// Rule override that applies a Graham Scan (Convex Hull Algorithm) to sort the
/// outer-most counterclockwise points in a continuous track.
/// 
/// See https://www.youtube.com/watch?v=B2AJoQSZf4M for the most best run-through out there
/// </summary>
/// ------------------------------------------------------------------------------------------------
class ConvexHullRule : public Rule
{

public:
	virtual void Process( Track* track ) const override
	{
		// Find smallest Y point
		const b2Vec2 minY = *(std::min_element( std::begin( track->m_points ), std::end( track->m_points ), 
			[]( const b2Vec2& a, const b2Vec2& b ) -> bool
			{
				return a.y < b.y;
			}
		));

		// Sort points in ascending order by the angle the vector
		// between a given point and minY makes with the x axis
		std::sort( std::begin( track->m_points ), std::end( track->m_points ),
			[&]( const b2Vec2& a, const b2Vec2& b ) -> bool
			{
				// Calculate the two compared angles using atan2. For reference:
				// https://en.wikipedia.org/wiki/Atan2
				// https://stackoverflow.com/questions/283406/what-is-the-difference-between-atan-and-atan2-in-c

				const double aAngle = atan2( a.y - minY.y, a.x - minY.x );
				const double bAngle = atan2( b.y - minY.y, b.x - minY.x );

				return aAngle < bAngle;
			}
		);


		// Lastly, iterate through the sorted points and check each point for
		// the direction of the angle it makes with the vector created by the previous
		// two points (see the Graham Scan video in the comment block for the class).
		// Any counterclockwise turning angles are picked and added to a stack with the
		// final set of points that complete the closed polygon. 
		std::stack<b2Vec2> trackPoints;
		
		trackPoints.push( track->m_points[ 0 ] );
		trackPoints.push( track->m_points[ 1 ] );

		for ( size_t i = 2; i < constants::TRACK_POINT_COUNT; ++i )
		{
			const b2Vec2& next = track->m_points[ i ];

			b2Vec2& point = trackPoints.top();
			trackPoints.pop();

			// Checking the clockwise direction of the angle can be done using a cross product,
			// as the resultant value's sign will show the angle's direction.
			// Box2D has a 2D cross product, which returns a scalar, whose sign can be used.
			while ( !trackPoints.empty() && b2Cross( point - trackPoints.top(), next - point ) <= 0 )
			{
				point = trackPoints.top();
				trackPoints.pop();
			}

			trackPoints.push( point );
			trackPoints.push( next );
		}

		// Add the first point at the end of the stack again to close the circle
		trackPoints.push( track->m_points[ 0 ] );

		// Copy over the points back to the original array from the stack;
		// If the stack is finished - set point to default value so we don't draw
		// the remaining points. As the stack is a subset of all the points,
		// this should always fit within the TRACK_POINT_COUNT
		for ( size_t j = 0; j < constants::TRACK_POINT_COUNT; ++j )
		{
			if ( !trackPoints.empty() )
			{
				track->m_points[ j ] = trackPoints.top();
				trackPoints.pop();
			}
			else
			{
				track->m_points[ j ] = b2Vec2_zero;
			}
		}
	}
};



// ------------------------------------------------------------------------------------------------

class CarDemo : public Test
{
public:
	
	CarDemo()
	{
		m_world->SetGravity( b2Vec2( 0.0f, 0.0f ) );

		const float k_restitution = 0.4f;
		
		// track setup
		b2Body* ground;

		b2BodyDef bd;
		bd.position.Set( 0.0f, 20.0f );
		ground = m_world->CreateBody( &bd );

		b2EdgeShape shape;

		b2FixtureDef sd;
		sd.shape = &shape;
		sd.density = 1.0f;
		sd.restitution = k_restitution;

		Track track;

		// Assigning the rule this way is, generally, a bit sloppy, so don't do this outside of small-scope code.
		// 
		// In the real world - this should be done through some sort of constructor or function, which manages rule array size, does sanity checks etc.
		// And the member variable is also private.
		ConvexHullRule convexHullRule;
		track.m_rules[ 0 ] = &convexHullRule;
		track.Create( ground, &shape, &sd, m_showDebug );

		if ( m_showDebug )
		{
			m_debugSeed = track.m_debugSeed;
		}

		m_car = new Car( m_world, track.m_startingPoint );
	}

	void Step( Settings& settings ) override
	{
		g_debugDraw.DrawString( 5, m_textLine, "Forward (W), Turn (A) and (D), Backwards (S), Handbrake (LShift)" );
		m_textLine += m_textIncrement;

		g_debugDraw.DrawString( 5, m_textLine, "Self-drive mode(TM) on/off (G)" );
		m_textLine += m_textIncrement;

		g_debugDraw.DrawString( 5, m_textLine, "Set target if in self-drive mode(Left Mouse click)" );
		m_textLine += m_textIncrement;

		m_car->update( m_showDebug );


		if ( m_showDebug )
		{
			g_debugDraw.DrawString( 5, m_textLine, "Track seed: %d", m_debugSeed);
			m_textLine += m_textIncrement;

			// a bunch of debug vectors for the wandering & velocity vectors, as well as the 3 raycasts for collision detection
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition(), m_car->m_body->GetPosition() + m_car->m_debugWanderVector, b2Color( 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition() + m_car->m_body->GetWorldVector( m_car->m_carFront ), m_car->m_body->GetPosition() + m_car->m_debugFrontRay, b2Color( 1, 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition() + m_car->m_body->GetWorldVector( m_car->m_carRightFront ), m_car->m_body->GetPosition() + m_car->m_debugRightRay, b2Color( 1, 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition() + m_car->m_body->GetWorldVector( m_car->m_carLeftFront ), m_car->m_body->GetPosition() + m_car->m_debugLeftRay, b2Color( 1, 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition(), m_car->m_body->GetPosition() + m_car->m_debugVelocityVector, b2Color( 1, 1, 255, 1 ) );

			// straight line between the car and the next target, if there is one
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition(), m_car->m_nextTarget, b2Color( 1, 1, 255, 1 ) );

			// a vector showing the force applied to the car, when avoiding collision (from collision point outwards, usually)
			g_debugDraw.DrawSegment( m_car->m_debugAvoidPoint, m_car->m_debugAvoidPoint + m_car->m_debugAvoidVector, b2Color( 1, 255, 1, 1 ) );

			// vectors showing the lateral velocity forces applied to tires, when turning/driving around
			m_textLine += m_textIncrement;
			uint8_t index{ 0 };
			for ( Tire* tire : m_car->m_tires )
			{
				g_debugDraw.DrawString( 10, m_textLine, "Current lateralVelocityVector for car %d: %.3f, %.3f", index, tire->m_lastLateralVelocityVector.x, tire->m_lastLateralVelocityVector.y );
				m_textLine += m_textIncrement;

				g_debugDraw.DrawSegment( tire->m_body->GetPosition(), tire->m_body->GetPosition() + tire->m_lastLateralVelocityVector, b2Color( 255, 1, 1 ) );
			}

			// wandering behaviour debug angles
			g_debugDraw.DrawString( 10, m_textLine, "Current WanderAngle: %.3f (Rad), %.3f (Deg); Current TurnAngle: %.3f (Rad), %.3f (Deg)", m_car->m_wanderAngle, m_car->m_wanderAngle * ( 180 / M_PI ), m_car->m_debugTurnAngle, m_car->m_debugTurnAngle * (180 / M_PI) );
			m_textLine += m_textIncrement;
		}

		Test::Step( settings );
	}

	/// <summary>
	/// Mouse click override to set car target manually if needed
	/// </summary>
	/// <param name="p"> Mouse position </param>
	//virtual void MouseDown( const b2Vec2& p ) override
	//{
	//	Test::MouseDown( p );
	//	m_car->m_nextTarget = p;
	//}

	static Test* Create()
	{
		return new CarDemo;
	}

	Car*		m_car{ nullptr };
	uint32_t	m_debugSeed{ 0 };
	bool		m_showDebug{ true };
};

static int testIndex = RegisterTest( "Custom", "Car Demo", CarDemo::Create );

#endif // USE_MATH_DEFINES