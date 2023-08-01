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

	UPROPERTY(EditAnywhere)
	float PositionTolerance = 0.025f;

	UPROPERTY(EditAnywhere)
	float AngleTolerance = 0.001;

	int32 GetConstraintIndex() const;
};

struct FRigidSloverData
{
	bool bStatic = false;
	TWeakObjectPtr<UPrimitiveComponent> Shape;

	Chaos::FReal InvM = 0.0f;
	Chaos::FMatrix33 InvLocalInertiaTensor = FMatrix::Identity;

	struct FState 
	{
		Chaos::FVec3		P = Chaos::FVec3::ZeroVector;
		Chaos::FRotation3	Q = Chaos::FRotation3::Identity;
		Chaos::FVec3		V = Chaos::FVec3::ZeroVector;
		Chaos::FVec3		W = Chaos::FVec3::ZeroVector;

		Chaos::FVec3		DP = Chaos::FVec3::ZeroVector;
		Chaos::FVec3		DQ = Chaos::FVec3::ZeroVector;
	};
	FState State;
	FState OldState;

	inline Chaos::FVec3&			P() { return State.P; }
	inline Chaos::FRotation3&		Q() { return State.Q; }
	inline Chaos::FVec3&			V() { return State.V; }
	inline Chaos::FVec3&			W() { return State.W; }
	inline Chaos::FVec3&			DP(){ return State.DP; }
	inline Chaos::FVec3&			DQ(){ return State.DQ; }

	inline Chaos::FVec3				CorrectedP() const { return State.P + Chaos::FVec3(State.DP); }

	inline Chaos::FRotation3		CorrectedQ() const { return (!bStatic && !State.DQ.IsZero()) ? Chaos::FRotation3::IntegrateRotationWithAngularVelocity(State.Q, Chaos::FVec3(State.DQ), Chaos::FReal(1)) : State.Q; }

	void IntegrateRotation(const Chaos::FVec3 dTheta)
	{
		Chaos::FReal Len = dTheta.Length();
		if (Len > 1.0e-6f)
		{
			State.Q = FQuat(dTheta / Len, Len).GetNormalized();
		}
	}
};

struct FRigidSolverDataContainer
{
	struct FRigidSloverHandle
	{
		FRigidSolverDataContainer* Container = nullptr;
		int32 Index = INDEX_NONE;

		FRigidSloverHandle()
		{

		}

		FRigidSloverHandle(FRigidSolverDataContainer* InContainer, int32 InIndex)
			: Container(InContainer), Index(InIndex) {
		}

		inline bool IsValid() const { return Container && Index >= 0; }

		inline FRigidSloverData* Get();

		inline FRigidSloverData& operator*();
		inline FRigidSloverData* operator->();
	};
	//typedef int32 FRigidSloverHandle;
	//static const FRigidSloverHandle InvalidSloverHandle = -1;

	inline FRigidSloverHandle AllocNewSolverData()
	{
		auto& SolverData = SolverDatas.Emplace_GetRef();
		return FRigidSloverHandle(this, SolverDatas.Num() - 1);
	}

	inline FRigidSloverHandle FindSolverDataByShape(class UPrimitiveComponent* InShape)
	{
		for(int32 i=0; InShape && i<SolverDatas.Num(); ++i)
		{
			if(SolverDatas[i].Shape == InShape)
			{
				return FRigidSloverHandle(this, i);
			}
		}
		return FRigidSloverHandle();
	}

	inline FRigidSloverData* GetSolverData(const FRigidSloverHandle& InSolverHandle)
	{
		return InSolverHandle.IsValid()? &SolverDatas[InSolverHandle.Index] : nullptr;
	}

	template<typename TLambda>
	void ForEach(TLambda Lambda)
	{
		for(int32 i=0; i<SolverDatas.Num(); ++i)
		{
			Lambda(SolverDatas[i]);
		}
	}

private:
	TArray<FRigidSloverData> SolverDatas;
};

USTRUCT()
struct FJointSlovePair
{
	GENERATED_BODY()

	UPROPERTY()
	class UPrimitiveComponent* ShapeA = nullptr;

	UPROPERTY()
	class UPrimitiveComponent* ShapeB = nullptr;

	typedef FRigidSolverDataContainer::FRigidSloverHandle FRigidSloverHandle;	
	FRigidSloverHandle BodySolver[2];

	inline FRigidSloverData& Body(int BodyIndex)
	{
		return *BodySolver[BodyIndex];
	}

	inline const Chaos::FVec3 P(int BodyIndex)
	{
		// NOTE: Joints always use the latest post-correction position and rotation. This makes the joint error calculations non-linear and more robust against explosion
		// but adds a cost because we need to apply the latest correction each time we request the latest transform
		return BodySolver[BodyIndex]->CorrectedP();
	}

	inline const Chaos::FRotation3 Q(int BodyIndex)
	{
		// NOTE: Joints always use the latest post-correction position and rotation. This makes the joint error calculations non-linear and more robust against explosion
		// but adds a cost because we need to apply the latest correction each time we request the latest transform
		return BodySolver[BodyIndex]->CorrectedQ();
	}

	inline const Chaos::FVec3 V(int BodyIndex)
	{
		return BodySolver[BodyIndex]->V();
	}

	inline const Chaos::FVec3 W(int BodyIndex)
	{
		return BodySolver[BodyIndex]->W();
	}

	void ComputeBodyState(int32 BodyIndex);

	void InitPositionConstraintDatas(
		const int32 ConstraintIndex,
		const Chaos::FVec3& ConstraintAxis,
		const Chaos::FReal& ConstraintDelta,
		const Chaos::FReal Dt,
		const Chaos::FReal ConstraintLimit,
		const Chaos::FVec3& ConstraintArm0,
		const Chaos::FVec3& ConstraintArm1);

	void InitPositionEffectiveMass(const int32 ConstraintIndex, const Chaos::FReal Dt);

	/* -----Update at spawn joint----- */
	// Local-space constraint settings
	Chaos::FRigidTransform3 LocalConnectorXs[2];	// Local(CoM)-space joint connector transforms
	/* -----End----------------------- */

	/* -----Update at beginning of the iteration----- */
	Chaos::FMatrix33		InvIs[2];		// World-space
	// Pre-calculate P and Q to avoid additional computational overhead for calling FJointSlovePair::P and FJointSlovePair::Q
	Chaos::FVec3			CurrentPs[2];	// Positions at the beginning of the iteration
	Chaos::FRotation3		CurrentQs[2];	// Rotations at the beginning of the iteration
	// dims0: ConstraintIndex, dims1: BodyIndex? 
	Chaos::FVec3			ConstraintArms[3][2]; // World-space constraint arm

	// World-space constraint settings
	Chaos::FVec3 ConnectorXs[2];			// World-space joint connector positions
	Chaos::FRotation3 ConnectorRs[2];		// World-space joint connector rotations

	Chaos::FVec3 ConstraintLimits;
	Chaos::FVec3 ConstraintAxis[3];

	Chaos::FVec3 ConstraintHardIM; // K = JM^-1J^T with constraint axis
	Chaos::FVec3 ConstraintSoftIM;
	Chaos::FVec3 ConstraintCX = Chaos::FVec3::ZeroVector;

	Chaos::FReal ConstraintLambda[3] = { 0 };

	// Tolerances below which we stop solving
	Chaos::FReal PositionTolerance;					// Distance error below which we consider a constraint or drive solved
	Chaos::FReal AngleTolerance;						// Angle error below which we consider a constraint or drive solved
	/* -----End-------------------------------------- */
	

	FSpringPart SpingPart;

	FJointSettings JointSettings;
};

UCLASS(Blueprintable, BlueprintType)
class FPSDESTRUCTION_API ASoftJointConstraintTestActor : public AActor
{
	GENERATED_BODY()

public:
	using FRigidSloverHandle = FRigidSolverDataContainer::FRigidSloverHandle;

	UFUNCTION(BlueprintCallable)
	void AddJointPair(class UPrimitiveComponent* InShapeA, class UPrimitiveComponent* InShapeB, const FTransform& InJointWorldTrans, bool bShapeAStatic=false);

	UFUNCTION(BlueprintCallable)
	void Simulate(float Dt);

	UFUNCTION(BlueprintCallable)
	void DebugDraw(float Dt);

	UPROPERTY(EditAnywhere, Category = Joint)
	bool bEnableReset = false;

	UPROPERTY(EditAnywhere, Category=Joint)
	FJointSettings JointSettings;

	UPROPERTY(EditAnywhere, Category = Joint)
	bool bSoftPositionConstraint = false;
	UPROPERTY(EditAnywhere, Category = Joint)
	bool bSoftVelocityConstraint = false;
	UPROPERTY(EditAnywhere, Category = Joint)
	bool bSolvePosition = false;
	UPROPERTY(EditAnywhere, Category = Joint)
	bool bSolveVelocity = true;
	UPROPERTY(EditAnywhere, Category = Joint)
	int32 NumPositionIterations = 8;
	UPROPERTY(EditAnywhere, Category = Joint)
	int32 NumVelocityIterations = 1;

protected:
	void AdvanceOneStep(float Dt);

	void ApplyAxisPositionConstraint(float Dt, int32 ConstraintIndex, FJointSlovePair& Joint);
	void SolvePositionConstraints(float Dt, 
		int32 ConstraintIndex,
		const Chaos::FReal DeltaPosition, 
		FJointSlovePair& Joint);
	void SolvePositionConstraintsSoft(float Dt,
		int32 ConstraintIndex,
		const Chaos::FReal DeltaPosition,
		FJointSlovePair& Joint);

	void ApplyAxisVelocityConstraint(float Dt,
		int32 ConstraintIndex,
		FJointSlovePair& Joint);
	void SolveLinearVelocityConstraints(float Dt, 
		int32 ConstraintIndex, 
		FJointSlovePair& Joint,
		const Chaos::FReal TargetVel);
	void SolveVelocityConstraintsSoft(float Dt, int32 ConstraintIndex, FJointSlovePair& Joint);

	void InitPlanarPositionConstraint(
		float Dt, 
		FJointSlovePair& Joint,
		const int32 AxisIndex);

	void InitSphericalPositionConstraint(
		float Dt,
		FJointSlovePair& Joint);

	void ApplyCorrections(FJointSlovePair& Joint);

private:
	FRigidSolverDataContainer RigidSloverDataContainer;

	UPROPERTY()
	TArray<FJointSlovePair> JointPairs;

	int StepNum = 0;
};