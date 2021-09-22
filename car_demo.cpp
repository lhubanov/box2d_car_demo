#include "test.h"
#include <vector>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>

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

	void update()
	{
		// lateral velocity murder so floating around doesn't happen
		const b2Vec2 currentRightNormal = m_body->GetWorldVector( b2Vec2( 1, 0 ) );
		const float lateralVelocity = b2Dot( currentRightNormal, m_body->GetLinearVelocity() );
		const b2Vec2 lateralVelocityVector = lateralVelocity * currentRightNormal;
		
		const b2Vec2 impulse = m_body->GetMass() * -lateralVelocityVector;
		m_body->ApplyLinearImpulse( impulse, m_body->GetWorldCenter(), true );

		// apply drag
		b2Vec2 currentForwardNormal = m_body->GetWorldVector( b2Vec2( 0, 1 ) );
		b2Dot( currentForwardNormal, m_body->GetLinearVelocity() ) * currentForwardNormal;

		const float currentForwardSpeed = currentForwardNormal.Normalize();
		const float dragForceMagnitude = -2 * currentForwardSpeed;
		m_body->ApplyForce( dragForceMagnitude * currentForwardNormal, m_body->GetWorldCenter(), true );

		// apply directional force, based on input;
		// Note: can optimise this by passing turn state, after handling input in the car's update().

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_W ) == GLFW_PRESS )
		{
			b2Vec2 force = m_body->GetWorldVector( b2Vec2( 0.0f, m_forwardForce ) );
			b2Vec2 point = m_body->GetWorldPoint( b2Vec2( 0.0f, -3.0f ) );
			m_body->ApplyForce( force, point, true );
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

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_S ) == GLFW_PRESS )
		{
			const b2Vec2 force = m_body->GetWorldVector( b2Vec2( 0.0f, m_backwardForce ) );
			const b2Vec2 point = m_body->GetWorldPoint( b2Vec2( 0.0f, 3.0f ) );
			m_body->ApplyForce( force, point, true );
		}

	}

	b2Body* m_body;
	bool	m_canTurn{ false };

private:
	float m_forwardForce{ 50 };
	float m_backwardForce{ -30 };

	float m_turnForce{ 10 };
};

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
		b2Fixture* fixture = m_body->CreateFixture( &polygonShape, 0.1f );

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
	}

	~Car()
	{
		m_body->GetWorld()->DestroyBody( m_body );
	}

	void update()
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

		const float currentAngle = m_frontLeftJoint->GetJointAngle();
		float angleToTurn = desiredAngle - currentAngle;
		angleToTurn = b2Clamp( angleToTurn, -turnPerTimeStep, turnPerTimeStep );

		float newAngle = currentAngle + angleToTurn;
		m_frontLeftJoint->SetLimits( newAngle, newAngle );
		m_frontRightJoint->SetLimits( newAngle, newAngle );


		for ( int i = 0; i < m_tires.size(); ++i )
		{
			m_tires[ i ]->update();
		}
	}

	std::vector<Tire*> m_tires;

	b2RevoluteJoint* m_frontLeftJoint;
	b2RevoluteJoint* m_frontRightJoint;

	b2Body* m_body;

	int m_turnRate{ 200 };
	int m_turnAngle{ 35 };
};

class CarDemo : public Test
{
public:
	
	CarDemo()
	{
		m_world->SetGravity( b2Vec2( 0.0f, 0.0f ) );

		const float k_restitution = 0.4f;

		b2Body* ground;
		{
			b2BodyDef bd;
			bd.position.Set( 0.0f, 20.0f );
			ground = m_world->CreateBody( &bd );

			b2EdgeShape shape;

			b2FixtureDef sd;
			sd.shape = &shape;
			sd.density = 0.0f;
			sd.restitution = k_restitution;

			// Left vertical
			shape.SetTwoSided( b2Vec2( -40.0f, -40.0f ), b2Vec2( -40.0f, 40.0f ) );
			ground->CreateFixture( &sd );

			// Right vertical
			shape.SetTwoSided( b2Vec2( 40.0f, -40.0f ), b2Vec2( 40.0f, 40.0f ) );
			ground->CreateFixture( &sd );

			// Top horizontal
			shape.SetTwoSided( b2Vec2( -40.0f, 40.0f ), b2Vec2( 40.0f, 40.0f ) );
			ground->CreateFixture( &sd );

			// Bottom horizontal
			shape.SetTwoSided( b2Vec2( -40.0f, -40.0f ), b2Vec2( 40.0f, -40.0f ) );
			ground->CreateFixture( &sd );
		}

		{
			m_car = new Car( m_world );
		}
	}

	void Step( Settings& settings ) override
	{
		g_debugDraw.DrawString( 5, m_textLine, "Forward (W), Turn (A) and (D), Backwards (S)" );
		m_textLine += m_textIncrement;

		m_car->update();

		Test::Step( settings );
	}

	static Test* Create()
	{
		return new CarDemo;
	}

	Car* m_car;
};

static int testIndex = RegisterTest( "Custom", "Car Demo", CarDemo::Create );

#endif // USE_MATH_DEFINES