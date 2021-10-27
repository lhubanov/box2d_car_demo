#include "test.h"
#include <vector>
#include <string>

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
	virtual float ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float fraction) override
	{
		m_collisionHit = true;
		m_point = point;
		m_normal = normal;
		m_fraction = fraction;

		// FIXME: LH:	try stopping raycast & catching return value
		//				try continuing raycast in some form & just creating the raycast in the constructor maybe?
		return 0.5f;
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
	Tire( b2World* world ) 
	{
		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set( 0.0f, 3.0 );

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
		// very jarring effect of instant traction once the player releases the drift button)
		const b2Vec2 currentRightNormal = m_body->GetWorldVector( b2Vec2( 1, 0 ) );
		const float lateralVelocity = b2Dot( currentRightNormal, m_body->GetLinearVelocity() );

		float reductionCoefficient{ 0.9f };
		//float reductionCoefficient{ 0 };
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
	Car( b2World* world )
	{
		//create car body
		b2BodyDef bodyDef;
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

		Tire* tire = new Tire( world );
		jointDef.bodyB = tire->m_body;
		jointDef.localAnchorA.Set( -2, 9 );
		m_frontLeftJoint = static_cast< b2RevoluteJoint* >( world->CreateJoint( &jointDef ) );
		tire->m_canTurn = true;
		m_tires.push_back( tire );


		tire = new Tire( world );
		jointDef.bodyB = tire->m_body;
		jointDef.localAnchorA.Set( 2, 9 );
		m_frontRightJoint = static_cast< b2RevoluteJoint* >( world->CreateJoint( &jointDef ) );
		tire->m_canTurn = true;
		m_tires.push_back( tire );

		tire = new Tire( world );
		jointDef.bodyB = tire->m_body;
		jointDef.localAnchorA.Set( -2, 1 );
		world->CreateJoint( &jointDef );
		m_tires.push_back( tire );

		tire = new Tire( world );
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

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) == GLFW_PRESS )
		{
			desiredAngle = angle;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_D ) == GLFW_PRESS )
		{
			desiredAngle = -angle;
		}

		// FIXME: LH: add an if here, so we don't always just take the wander angle
		desiredAngle = wander( showDebug );

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

	// FIXME: LH: these all need renaming - e.g. wanderPoint is just velocity
	float wander( bool showDebug )
	{
		b2Vec2 wanderPoint = m_nextTarget;

		// if there's no next target, wander randomly 
		if ( m_nextTarget == b2Vec2_zero )
		{
			// get a point in front of the car
			wanderPoint = m_body->GetWorldVector( m_wanderLocalPoint );

			// calculate a point on a circle around the wanderPoint
			m_wanderAngle += RandomFloat( -0.1f, 0.1f );

			float x = m_wanderRadius * cos( m_wanderAngle );
			float y = m_wanderRadius * sin( m_wanderAngle );

			// add the current velocity and the new wander vector
			const b2Vec2 circleVector{ x,y };
			wanderPoint += circleVector;
		}

		const b2Vec2 avoidanceVector = avoidCollisions( showDebug );
		if ( avoidanceVector != b2Vec2_zero )
		{
			wanderPoint += avoidanceVector;
		}

		float wanderPointMagnitude = wanderPoint.Normalize();
		wanderPointMagnitude = b2Clamp( wanderPointMagnitude, -m_maxVelocity, m_maxVelocity );
		wanderPoint *= wanderPointMagnitude;

		//arrival force calculations here?
		if ( m_nextTarget != b2Vec2_zero )
		{
			const b2Vec2 distanceToTarget = m_nextTarget - m_body->GetPosition();
			if ( distanceToTarget.Length() < m_arrivalDistance )
			{
				float wanderPointMagnitude = wanderPoint.Normalize();
				wanderPoint *= m_maxVelocity * ( distanceToTarget.Length() / m_arrivalDistance );
			}
		}

		m_body->ApplyForce( wanderPoint , m_body->GetWorldPoint( b2Vec2( 0.0f, -3.0f ) ), true );

		// calculate turn angle vector to return to tires
		// (turn angle = angle between wanderVector & car forward vector) 
		double turnAngle{ 0 };
		b2Vec2 forwardVector = m_body->GetWorldVector( m_wanderLocalPoint );
		double dot = b2Dot( wanderPoint, forwardVector );

		double wLength = wanderPoint.Length();
		double fLength = forwardVector.Length();

		double mag = wLength * fLength;

		turnAngle = acos( dot / mag );//(wanderPoint.Length() * forwardVector.Length()) );

		turnAngle = b2Clamp( turnAngle, -M_PI / 2.0f, M_PI / 2.0f );
		
		// FIXME: LH:	this is a bit hacky - basically the angle is always positive from the 
		//				above calcualtion, but the rotation that needs to be applied to the tires
		//				is +ve to the right, and -ve to the left, so we fudge it
		b2Vec2 turnDirection = wanderPoint - forwardVector;
		b2Vec2 localTurnDirection = m_body->GetLocalVector( turnDirection );
		if( localTurnDirection.x > 0 && turnAngle > 0 )
		{
			turnAngle *= -1;
		}

		if ( showDebug )
		{
			m_debugWanderVector = wanderPoint;
			m_debugVelocityVector = m_body->GetLinearVelocity();
			m_debugTurnAngle = turnAngle;
		}

		return turnAngle;
	}

	b2Vec2 avoidCollisions( bool showDebug )
	{
		// get a point in front, on the left, and on the right of the car
		// cast ray from that point to a position in front/left/right of car
		b2Vec2 f1 = m_body->GetPosition() + m_body->GetWorldVector( m_carFront );

		const b2Vec2 ray( m_carFront.x, m_carFront.y + m_rayLength );
		b2Vec2 f2 = m_body->GetPosition() + m_body->GetWorldVector( ray );

		if ( showDebug )
		{
			m_debugFrontRay = m_body->GetWorldVector( ray );
		}

		b2Vec2 r1 = m_body->GetPosition() + m_body->GetWorldVector( m_carRightFront );

		const b2Vec2 rightRay(m_carRightFront.x + m_rayLength, m_carRightFront.y + m_rayLength );
		b2Vec2 r2 = m_body->GetPosition() + m_body->GetWorldVector( rightRay );

		if ( showDebug )
		{
			m_debugRightRay = m_body->GetWorldVector( rightRay );
		}

		b2Vec2 l1 = m_body->GetPosition() + m_body->GetWorldVector( m_carLeftFront );

		const b2Vec2 leftRay( m_carLeftFront.x - m_rayLength, m_carLeftFront.y + m_rayLength );
		b2Vec2 l2 = m_body->GetPosition() + m_body->GetWorldVector( leftRay );

		if ( showDebug )
		{
			m_debugLeftRay = m_body->GetWorldVector( leftRay );
		}

		m_world->RayCast( &m_raycastCallback, f1, f2 );
		m_world->RayCast( &m_rightRaycastCallback, r1, r2 );
		m_world->RayCast( &m_leftRaycastCallback, l1, l2 );

		auto checkRaycastCollision = [&]( RaycastCallback* callback ) -> b2Vec2
		{
			if ( callback->m_collisionHit )
			{
				b2Vec2 velocity = m_body->GetLinearVelocity();

				//const float avoidScalar = b2Dot( m_raycastCallback.m_normal, m_raycastCallback.m_point );
				//const b2Vec2 avoidVector = avoidScalar * m_raycastCallback.m_point;
				
				const b2Vec2 avoidVector = m_collisionAvoidanceForce * callback->m_normal;

				if ( showDebug )
				{
					m_debugAvoidVector = avoidVector;
					m_debugAvoidPoint = callback->m_point;
				}

				// FIXME: LH: maybe do .Reset() & reset all the members as well?
				callback->m_collisionHit = false;

				return avoidVector;
			}

			return b2Vec2_zero;
		};

		b2Vec2 avoidVector{ b2Vec2_zero };

		if ( showDebug )
		{
			m_debugAvoidVector = b2Vec2_zero;
		}

		avoidVector += checkRaycastCollision( &m_raycastCallback );
		avoidVector += checkRaycastCollision( &m_leftRaycastCallback );
		avoidVector += checkRaycastCollision( &m_rightRaycastCallback );

		return avoidVector;
	}

	// FIXME: LH: clear some of these debug values up when done
	
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
	
	b2Vec2				m_wanderLocalPoint{ 0, 30 };

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
	float				m_collisionAvoidanceForce{ 13 };
	float				m_wanderRadius{ 5 };

	float				m_arrivalDistance{ 25 };

	float				m_wanderAngle{ -M_PI / 2 };

	float				m_maxVelocity{ 30 };

	// DEBUG values (used for drawing visual elements when showDebug == true)
	float				m_debugTurnAngle{ 0 };
};

class CarDemo : public Test
{
public:
	
	CarDemo()
	{
		m_world->SetGravity( b2Vec2( 0.0f, 0.0f ) );

		const float k_restitution = 0.4f;

		// track setup
		b2Body* ground;
		{
			b2BodyDef bd;
			bd.position.Set( 0.0f, 20.0f );
			ground = m_world->CreateBody( &bd );

			b2EdgeShape shape;

			b2FixtureDef sd;
			sd.shape = &shape;
			sd.density = 1.0f;
			sd.restitution = k_restitution;

			b2Vec2 vertices[ 4 ];
			vertices[ 0 ].Set( -55.0f, -20.0f );
			vertices[ 1 ].Set( -55.0f, 20.0f );
			vertices[ 2 ].Set( -75.0f, -20.0f );
			vertices[ 3 ].Set( -75.0f, 20.0f );
			b2PolygonShape polygonShape;
			polygonShape.Set( vertices, 4 );

			b2FixtureDef polygon;
			polygon.shape = &polygonShape;
			polygon.density = 0.0f;
			polygon.restitution = k_restitution;

			ground->CreateFixture( &polygon );


			shape.SetTwoSided( b2Vec2( -110.0f, -100.0f ), b2Vec2( -110.0f, 100.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -75.0f, -75.0f ), b2Vec2( -75.0f, 75.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -40.0f, -100.0f ), b2Vec2( -40.0f, -40.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -110.0f, -100.0f ), b2Vec2( -40.0f, -100.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -55.0f, -20.0f ), b2Vec2( -55.0f, 20.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -10.0f, -20.0f ), b2Vec2( -10.0f, 20.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -55.0f, -20.0f ), b2Vec2( -10.0f, -20.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -55.0f, 20.0f ), b2Vec2( -10.0f, 20.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( 25.0f, -40.0f ), b2Vec2( 25.0f, 100.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -40.0f, 40.0f ), b2Vec2( 25.0f, 40.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -40.0f, -40.0f ), b2Vec2( 25.0f, -40.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -75.0f, 75.0f ), b2Vec2( 0.0f, 75.0f ) );
			ground->CreateFixture( &sd );

			shape.SetTwoSided( b2Vec2( -110.0f, 100.0f ), b2Vec2( 25.0f, 100.0f ) );
			ground->CreateFixture( &sd );

			// add a cavalcade of pushable objects in that lil' nook and cranny in the middle of the track
			//for ( int32 i = 0; i < 300; ++i )
			//{
			//	b2CircleShape circleShape;
			//	circleShape.m_p.SetZero();
			//	circleShape.m_radius = 0.8f;
			//	
			//	b2BodyDef bd;
			//	bd.type = b2_dynamicBody;
			//	bd.position = b2Vec2( RandomFloat( -75.0f, -55.0f ), RandomFloat( 0.0f, 40.0f ) );
			//	b2Body* body = m_world->CreateBody( &bd );

			//	b2MassData mass;
			//	mass.center = bd.position;
			//	mass.mass = 10000.0f;

			//	body->SetMassData( &mass );
			//	body->CreateFixture( &circleShape, 0.01f );
			//}
		}

		{
			m_car = new Car( m_world );
		}
	}

	void Step( Settings& settings ) override
	{
		g_debugDraw.DrawString( 5, m_textLine, "Forward (W), Turn (A) and (D), Backwards (S), Handbrake (LShift)" );
		m_textLine += m_textIncrement;

		m_car->update( showDebug );


		if ( showDebug )
		{
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition(), m_car->m_body->GetPosition() + m_car->m_debugWanderVector, b2Color( 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition() + m_car->m_body->GetWorldVector( m_car->m_carFront ), m_car->m_body->GetPosition() + m_car->m_debugFrontRay, b2Color( 1, 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition() + m_car->m_body->GetWorldVector( m_car->m_carRightFront ), m_car->m_body->GetPosition() + m_car->m_debugRightRay, b2Color( 1, 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition() + m_car->m_body->GetWorldVector( m_car->m_carLeftFront ), m_car->m_body->GetPosition() + m_car->m_debugLeftRay, b2Color( 1, 255, 1, 1 ) );
			g_debugDraw.DrawSegment( m_car->m_body->GetPosition(), m_car->m_body->GetPosition() + m_car->m_debugVelocityVector, b2Color( 1, 1, 255, 1 ) );

			g_debugDraw.DrawSegment( m_car->m_body->GetPosition(), m_car->m_nextTarget, b2Color( 1, 1, 255, 1 ) );

			g_debugDraw.DrawSegment( m_car->m_debugAvoidPoint, m_car->m_debugAvoidPoint + m_car->m_debugAvoidVector, b2Color( 1, 255, 1, 1 ) );

			uint8_t index{ 0 };
			for ( Tire* tire : m_car->m_tires )
			{
				g_debugDraw.DrawString( 10, m_textLine, "Current lateralVelocityVector for car %d: %.3f, %.3f", index, tire->m_lastLateralVelocityVector.x, tire->m_lastLateralVelocityVector.y );
				m_textLine += m_textIncrement;

				g_debugDraw.DrawSegment( tire->m_body->GetPosition(), tire->m_body->GetPosition() + tire->m_lastLateralVelocityVector, b2Color( 255, 1, 1 ) );
			}

			g_debugDraw.DrawString( 10, m_textLine, "Current WanderAngle: %.3f (Rad), %.3f (Deg); Current TurnAngle: %.3f (Rad), %.3f (Deg)", m_car->m_wanderAngle, m_car->m_wanderAngle * ( 180 / M_PI ), m_car->m_debugTurnAngle, m_car->m_debugTurnAngle * (180 / M_PI) );
			m_textLine += m_textIncrement;

			g_debugDraw.DrawString( 10, m_textLine, "Car next target: %.3f, %.3f", m_car->m_nextTarget.x, m_car->m_nextTarget.y );
			m_textLine += m_textIncrement;
		}

		Test::Step( settings );
	}

	virtual void MouseDown( const b2Vec2& p ) override
	{
		Test::MouseDown( p );

		m_car->m_nextTarget = p;
	}

	static Test* Create()
	{
		return new CarDemo;
	}

	Car* m_car;

	bool showDebug{ true };
};

static int testIndex = RegisterTest( "Custom", "Car Demo", CarDemo::Create );

#endif // USE_MATH_DEFINES