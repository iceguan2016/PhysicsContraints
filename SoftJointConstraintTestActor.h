#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SoftJointConstraintTestActor.generated.h"

class FSpringPart
{
public:
	/// Calculate spring properties
	///
	/// @param inDeltaTime Time step
	/// @param inInvEffectiveMass Inverse effective mass K
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C). Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inFrequency Oscillation frequency (Hz). Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inDamping Damping factor (0 = no damping, 1 = critical damping). Set to zero if you don't want to drive the constraint to zero with a spring.
	/// @param outEffectiveMass On return, this contains the new effective mass K^-1
	inline void					CalculateSpringProperties(float inDeltaTime, float inInvEffectiveMass, float inBias, float inC, float inFrequency, float inDamping, float& outEffectiveMass)
	{
		outEffectiveMass = 1.0f / inInvEffectiveMass;

		// Soft constraints as per: Soft Contraints: Reinventing The Spring - Erin Catto - GDC 2011
		if (inFrequency > 0.0f)
		{
			// Calculate angular frequency
			float omega = 2.0f * PI * inFrequency;

			// Calculate spring constant k and drag constant c (page 45)
			float k = outEffectiveMass * FMath::Square(omega);
			float c = 2.0f * outEffectiveMass * inDamping * omega;

			// Note that the calculation of beta and gamma below are based on the solution of an implicit Euler integration scheme
			// This scheme is unconditionally stable but has built in damping, so even when you set the damping ratio to 0 there will still
			// be damping. See page 16 and 32.

			// Calculate softness (gamma in the slides)
			// See page 34 and note that the gamma needs to be divided by delta time since we're working with impulses rather than forces:
			// softness = 1 / (dt * (c + dt * k))
			mSoftness = 1.0f / (inDeltaTime * (c + inDeltaTime * k));

			// Calculate bias factor (baumgarte stabilization):
			// beta = dt * k / (c + dt * k) = dt * k^2 * softness
			// b = beta / dt * C = dt * k * softness * C
			mBias = inBias + inDeltaTime * k * mSoftness * inC;

			// Update the effective mass, see post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
			// 
			// Newton's Law: 
			// M * (v2 - v1) = J^T * lambda 
			// 
			// Velocity constraint with softness and Baumgarte: 
			// J * v2 + softness * lambda + b = 0 
			// 
			// where b = beta * C / dt 
			// 
			// We know everything except v2 and lambda. 
			// 
			// First solve Newton's law for v2 in terms of lambda: 
			// 
			// v2 = v1 + M^-1 * J^T * lambda 
			// 
			// Substitute this expression into the velocity constraint: 
			// 
			// J * (v1 + M^-1 * J^T * lambda) + softness * lambda + b = 0 
			// 
			// Now collect coefficients of lambda: 
			// 
			// (J * M^-1 * J^T + softness) * lambda = - J * v1 - b 
			// 
			// Now we define: 
			// 
			// K = J * M^-1 * J^T + softness 
			// 
			// So our new effective mass is K^-1 
			outEffectiveMass = 1.0f / (inInvEffectiveMass + mSoftness);
		}
		else
		{
			mSoftness = 0.0f;
			mBias = inBias;
		}
	}

	/// Returns if this spring is active
	inline bool					IsActive() const
	{
		return mSoftness != 0.0f;
	}

	/// Get total bias b, including supplied bias and bias for spring: lambda = J v + b
	inline float				GetBias(float inTotalLambda) const
	{
		// Remainder of post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
		//
		// Each iteration we are not computing the whole impulse, we are computing an increment to the impulse and we are updating the velocity. 
		// Also, as we solve each constraint we get a perfect v2, but then some other constraint will come along and mess it up. 
		// So we want to patch up the constraint while acknowledging the accumulated impulse and the damaged velocity. 
		// To help with that we use P for the accumulated impulse and lambda as the update. Mathematically we have: 
		// 
		// M * (v2new - v2damaged) = J^T * lambda 
		// J * v2new + softness * (total_lambda + lambda) + b = 0 
		// 
		// If we solve this we get: 
		// 
		// v2new = v2damaged + M^-1 * J^T * lambda 
		// J * (v2damaged + M^-1 * J^T * lambda) + softness * total_lambda + softness * lambda + b = 0 
		// 
		// (J * M^-1 * J^T + softness) * lambda = -(J * v2damaged + softness * total_lambda + b) 
		// 
		// So our lagrange multiplier becomes:
		//
		// lambda = -K^-1 (J v + softness * total_lambda + b)
		//
		// So we return the bias: softness * total_lambda + b
		return mSoftness * inTotalLambda + mBias;
	}

private:
	float						mBias = 0.0f;
	float						mSoftness = 0.0f;
};

USTRUCT()
struct FPSDESTRUCTION_API FJointSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	FVector LimitDistance = FVector::ZeroVector;

	UPROPERTY(EditAnywhere)
	float Frequency = 4.0f;

	UPROPERTY(EditAnywhere)
	float DampingRatio = 0.0f;

	int32 GetConstraintIndex() const;
};

USTRUCT()
struct FJointSlovePair
{
	GENERATED_BODY()

	UPROPERTY()
	class UBoxComponent* ShapeA = nullptr;

	UPROPERTY()
	class UBoxComponent* ShapeB = nullptr;

	struct FRigidSloverData
	{
		bool bStatic = false;
		Chaos::FReal InvM = 1.0f;
		Chaos::FMatrix33 InvLocalInertiaTensor = FMatrix::Identity;

		// Local-space constraint settings
		Chaos::FRigidTransform3 LocalConnectorX;	// Local(CoM)-space joint connector transforms

		// World-space constraint settings
		Chaos::FVec3 ConnectorX;			// World-space joint connector positions
		Chaos::FRotation3 ConnectorR;		// World-space joint connector rotations

		Chaos::FMatrix33 InvI;				// World-space

		Chaos::FVec3		P = Chaos::FVec3::ZeroVector;
		Chaos::FRotation3	R = Chaos::FRotation3::Identity;
		Chaos::FVec3		V = Chaos::FVec3::ZeroVector;
		Chaos::FVec3		W = Chaos::FVec3::ZeroVector;

		Chaos::FVec3 ConstraintArm; // World-space constraint arm

		void IntegrateRotation(const Chaos::FVec3 dTheta)
		{
			Chaos::FReal Len = dTheta.Length();
			if (Len > 1.0e-6f)
			{
				R = FQuat(dTheta / Len, Len).GetNormalized();
			}
		}
	};

	FRigidSloverData Body[2];

	Chaos::FVec3 ConstraintLimits;
	Chaos::FVec3 ConstraintAxis[3];

	Chaos::FVec3 ConstraintHardIM; // K = JM^-1J^T with constraint axis
	Chaos::FVec3 ConstraintSoftIM;
	Chaos::FVec3 ConstraintCX = Chaos::FVec3::ZeroVector;

	bool NeedsSolve[3];

	Chaos::FReal TotalLambda = 0;

	FSpringPart SpingPart;
};

UCLASS(Blueprintable, BlueprintType)
class FPSDESTRUCTION_API ASoftJointConstraintTestActor : public AActor
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable)
	void AddJointPair(class UBoxComponent* InShapeA, class UBoxComponent* InShapeB, const FTransform& InJointWorldTrans);

	UFUNCTION(BlueprintCallable)
	void Simulate(float Dt);

	UFUNCTION(BlueprintCallable)
	void DebugDraw(float Dt);

	UPROPERTY(EditAnywhere, Category=Joint)
	FJointSettings JointSettings;

protected:
	void InitVelocityConstraints(float Dt, FJointSlovePair& InJointSloverPair);
	void SolvePositionConstraints(float Dt, FJointSlovePair& InJointSloverPair);
	void SolveVelocityConstraints(float Dt, FJointSlovePair& InJointSloverPair);

	void InitPlanarPositionConstraint(
		float Dt, 
		FJointSlovePair& InJointSloverPair,
		const int32 AxisIndex);

private:
	UPROPERTY()
	TArray<FJointSlovePair> JointPairs;
};