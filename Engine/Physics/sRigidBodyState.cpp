// Includes
//=========

#include "sRigidBodyState.h"
#include "Engine/Math/Functions.h"
// Interface
//==========

void eae6320::Physics::sRigidBodyState::Update( const float i_secondCountToIntegrate )
{
	// Update position
	{
		position += velocity * i_secondCountToIntegrate;
	}
	// Update velocity
	{
		velocity += acceleration * i_secondCountToIntegrate;
	}
	// Update orientation
	{
		//const auto rotation = Math::cQuaternion( angularSpeed * i_secondCountToIntegrate, angularVelocity_axis_local );
		euler_x = euler_x + axis_X_velocity * i_secondCountToIntegrate;
		if (euler_x > 360) euler_x = euler_x - 360;
		if (euler_x < 0) euler_x = euler_x + 360;

		euler_y = euler_y + axis_Y_velocity * i_secondCountToIntegrate;
		if (euler_y > 360) euler_y = euler_y - 360;
		if (euler_y < 0) euler_y = euler_y + 360;

		euler_z = euler_z + axis_Z_velocity * i_secondCountToIntegrate;
		if (euler_z > 360) euler_z = euler_z - 360;
		if (euler_z < 0) euler_z = euler_z + 360;
		
		const auto rotation_x = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_x), Math::sVector(1, 0, 0));
		const auto rotation_y = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_y), Math::sVector(0, 1, 0));
		const auto rotation_z = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_z), Math::sVector(0, 0, 1));

		const auto rotation = rotation_y * rotation_x * rotation_z;

		orientation = rotation;
		orientation.Normalize();
	}
}
void eae6320::Physics::sRigidBodyState::UpdatePosition(const float i_secondCountToIntegrate) {
	position += velocity * i_secondCountToIntegrate;
}

void eae6320::Physics::sRigidBodyState::UpdateVelocity(const float i_secondCountToIntegrate) {
	velocity += acceleration * i_secondCountToIntegrate;
}
void eae6320::Physics::sRigidBodyState::UpdateOrientation(const float i_secondCountToIntegrate) {
	//const auto rotation = Math::cQuaternion( angularSpeed * i_secondCountToIntegrate, angularVelocity_axis_local );
	euler_x = euler_x + axis_X_velocity * i_secondCountToIntegrate;
	if (euler_x > 360) euler_x = euler_x - 360;
	if (euler_x < 0) euler_x = euler_x + 360;

	euler_y = euler_y + axis_Y_velocity * i_secondCountToIntegrate;
	if (euler_y > 360) euler_y = euler_y - 360;
	if (euler_y < 0) euler_y = euler_y + 360;

	euler_z = euler_z + axis_Z_velocity * i_secondCountToIntegrate;
	if (euler_z > 360) euler_z = euler_z - 360;
	if (euler_z < 0) euler_z = euler_z + 360;

	const auto rotation_x = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_x), Math::sVector(1, 0, 0));
	const auto rotation_y = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_y), Math::sVector(0, 1, 0));
	const auto rotation_z = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_z), Math::sVector(0, 0, 1));

	const auto rotation = rotation_y * rotation_x * rotation_z;

	orientation = rotation;
	orientation.Normalize();
}


eae6320::Math::sVector eae6320::Physics::sRigidBodyState::PredictFuturePosition( const float i_secondCountToExtrapolate ) const
{
	return position + (velocity * i_secondCountToExtrapolate );
}

eae6320::Math::cQuaternion eae6320::Physics::sRigidBodyState::PredictFutureOrientation( const float i_secondCountToExtrapolate ) const
{
	//const auto rotation = Math::cQuaternion( angularSpeed * i_secondCountToExtrapolate, angularVelocity_axis_local );
	const auto rotation_x = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_x + axis_X_velocity * i_secondCountToExtrapolate), Math::sVector(1, 0, 0));
	const auto rotation_y = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_y + axis_Y_velocity * i_secondCountToExtrapolate), Math::sVector(0, 1, 0));
	const auto rotation_z = Math::cQuaternion(Math::ConvertDegreesToRadians(euler_z + axis_Z_velocity * i_secondCountToExtrapolate), Math::sVector(0, 0, 1));

	const auto rotation = rotation_y * rotation_x * rotation_z;

	return rotation.GetNormalized();
}
