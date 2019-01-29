/*
	This struct can be used to represent the state of a
	rigid body in 3D space
*/

#ifndef EAE6320_PHYSICS_SRIGIDBODYSTATE_H
#define EAE6320_PHYSICS_SRIGIDBODYSTATE_H

// Includes
//=========

#include <cfloat>
#include <Engine/Math/cQuaternion.h>
#include <Engine/Math/sVector.h>

// Struct Declaration
//===================

namespace eae6320
{
	namespace Physics
	{
		
		struct AABB {
			Math::sVector center;
			Math::sVector extends;
		};
		
		struct sRigidBodyState
		{
			// Data
			//=====

			Math::sVector position;	// In arbitrary units determined by the applicaton's convention
			Math::sVector velocity;	// Distance per-second
			Math::sVector acceleration;	// Distance per-second^2
			Math::cQuaternion orientation;
			Math::sVector angularVelocity_axis_local = Math::sVector( 0.0f, 1.0f, 0.0f );	// In local space (not world space)
			float angularSpeed = 0.0f;	// Radians per-second (positive values rotate right-handed, negative rotate left-handed)

			float euler_x = 0.0f; //in degrees;
			float euler_y = 0.0f;
			float euler_z = 0.0f;

			float axis_X_velocity = 0.0f;//degrees per second
			float axis_Y_velocity = 0.0f;
			float axis_Z_velocity = 0.0f;

			AABB boundingBox;

			// Interface
			//==========

			void Update( const float i_secondCountToIntegrate );
			void UpdatePosition(const float i_secondCountToIntegrate);
			void UpdateVelocity(const float i_secondCountToIntegrate);
			void UpdateOrientation(const float i_secondCountToIntegrate);
			Math::sVector PredictFuturePosition( const float i_secondCountToExtrapolate ) const;
			Math::cQuaternion PredictFutureOrientation( const float i_secondCountToExtrapolate ) const;
		};
	}
}

#endif	// EAE6320_PHYSICS_SRIGIDBODYSTATE_H
