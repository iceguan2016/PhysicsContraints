#include "SoftJointConstraintTestActor.h"
#include "Components/BoxComponent.h"

Chaos::FVec3 CMtoM(const Chaos::FVec3& InVec)
{
	//return InVec * 0.01;
	return InVec;
}

Chaos::FVec3 MtoCM(const Chaos::FVec3& InVec)
{
	//return InVec * 100;
	return InVec;
}

const Chaos::FVec3 Gravity{ 0, 0, -980 };

int32 FJointSettings::GetConstraintIndex() const
{
	int32 ConstraintIndex = INDEX_NONE;
	if (LimitDistance[0] > 0.01f)
	{
		ConstraintIndex = 0;
	}
	else if (LimitDistance[1] > 0.01f)
	{
		ConstraintIndex = 1;
	}
	else
	{
		ConstraintIndex = 2;
	}

	return ConstraintIndex;
}

void ASoftJointConstraintTestActor::AddJointPair(
	class UBoxComponent* InShapeA, class UBoxComponent* InShapeB, const FTransform& InJointWorldTrans)
{
	if(!InShapeA || !InShapeB)
	{
		return;
	}

	auto& Pair = JointPairs.Emplace_GetRef();
	Pair.ShapeA = InShapeA;
	Pair.ShapeB = InShapeB;

	auto InitBody = [&InJointWorldTrans](FJointSlovePair::FRigidSloverData& b, class UBoxComponent* shape) {
		auto BodyTransform = shape->GetComponentTransform();
		// GetScaledBoxExtent return Half Extent
		auto Extent = CMtoM(shape->GetScaledBoxExtent() * 2.0);
		//auto Extent = FVector{ 1.5, 1.5, 1.5f };
		
		constexpr Chaos::FReal Density = 0.001; // kg/cm^3

		if(!b.bStatic)
		{
			Chaos::FReal Mass = (Extent.X * Extent.Y * Extent.Z) * Density;
			//Chaos::FReal Mass = 3375;
			b.LocalConnectorX = InJointWorldTrans.GetRelativeTransform(BodyTransform);
			b.LocalConnectorX.SetLocation(CMtoM(b.LocalConnectorX.GetLocation()));
			Chaos::FMatrix33 I = FMatrix::Identity;
			I.M[0][0] = 1.0 / 12 * Mass * (FMath::Square(Extent.Y) + FMath::Square(Extent.Z));
			I.M[1][1] = 1.0 / 12 * Mass * (FMath::Square(Extent.X) + FMath::Square(Extent.Z));
			I.M[2][2] = 1.0 / 12 * Mass * (FMath::Square(Extent.X) + FMath::Square(Extent.Y));

			b.InvM = 1.0 / Mass;
			b.InvLocalInertiaTensor = I.Inverse();
		}
		else
		{
			b.InvM = 0.0f;
			b.InvLocalInertiaTensor.M[0][0] = 0;
			b.InvLocalInertiaTensor.M[1][1] = 0;
			b.InvLocalInertiaTensor.M[2][2] = 0;
		}

		b.P = CMtoM(BodyTransform.GetLocation());
		b.R = BodyTransform.GetRotation();
	};

	auto& b0 = Pair.Body[0];
	auto& b1 = Pair.Body[1];

	b0.bStatic = true;
	b1.bStatic = false;

	InitBody(b0, InShapeA);
	InitBody(b1, InShapeB);
}

void ASoftJointConstraintTestActor::Simulate(float Dt)
{
	int32 ConstraintIndex = JointSettings.GetConstraintIndex();
	if(ConstraintIndex == INDEX_NONE) return;

	constexpr int32 NumPositionIterations = 8;
	constexpr int32 NumVelocityIterations = 1;

	// Integrate
	{

		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& JointPair = JointPairs[i];

			auto& b0 = JointPair.Body[0];
			auto& b1 = JointPair.Body[1];


			// Integrate Velocity
			{
				if (!b0.bStatic)
				{
					b0.V += Dt * Gravity;
				}

				if (!b1.bStatic)
				{
					b1.V += Dt * Gravity;
				}
			}

			auto IntegratePosition = [Dt](FJointSlovePair::FRigidSloverData& InBody) {
				InBody.P += Dt * InBody.V;

#if 0
				Chaos::FRotation3 aux_q = FQuat{ InBody.W.X, InBody.W.Y, InBody.W.Z, 0.0 };
				Chaos::FRotation3 q0 = aux_q * InBody.R;

				Chaos::FRotation3 q = InBody.R;
				q.X += 0.5 * q0.X;
				q.Y += 0.5 * q0.Y;
				q.Z += 0.5 * q0.Z;
				q.W += 0.5 * q0.W;

				q.Normalize();
				InBody.R = q;
#else
				InBody.IntegrateRotation(InBody.W * Dt);
#endif
			};


			// Integrate Position
			{
				if (!b0.bStatic) IntegratePosition(b0);
				if (!b1.bStatic) IntegratePosition(b1);
			}

			// Init
			InitVelocityConstraints(Dt, JointPair);
		}
	}

	// Solve position
	{
		for(int32 Itr=0; Itr<NumPositionIterations; ++Itr)
		{
			for (int32 i = 0; i < JointPairs.Num(); ++i)
			{
				auto& JointPair = JointPairs[i];

				auto& b0 = JointPair.Body[0];
				auto& b1 = JointPair.Body[1];

				ApplyAxisPositionConstraint(Dt, ConstraintIndex, JointPair);
			}
		}

		auto SetImplicitVelocity = [](FJointSlovePair::FRigidSloverData& InSolveData, Chaos::FReal Dt)
		{
			if (!InSolveData.bStatic && (Dt != Chaos::FReal(0)))
			{
				const Chaos::FReal InvDt = Chaos::FReal(1) / Chaos::FReal(Dt);
				InSolveData.V = InSolveData.V + InSolveData.DP * InvDt;
				InSolveData.W = InSolveData.W + InSolveData.DQ * InvDt;
			}
		};

		// SetImplicitVelocity
		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& JointPair = JointPairs[i];

			auto& b0 = JointPair.Body[0];
			auto& b1 = JointPair.Body[1];

			SetImplicitVelocity(b0, Dt);
			SetImplicitVelocity(b1, Dt);
		}
	}

	// Solve velocity
	{
		for (int32 Itr = 0; Itr < NumPositionIterations; ++Itr)
		{
			for (int32 i = 0; i < JointPairs.Num(); ++i)
			{
				auto& JointPair = JointPairs[i];

				auto& b0 = JointPair.Body[0];
				auto& b1 = JointPair.Body[1];

				SolveVelocityConstraints(Dt, JointPair);
			}
		}
	}

	// Update shape transform
	{
		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& JointPair = JointPairs[i];

			auto& b0 = JointPair.Body[0];
			auto& b1 = JointPair.Body[1];

			{
				auto SetShapeTransform = [](const FJointSlovePair::FRigidSloverData& b, UBoxComponent* shape) {
					shape->SetWorldLocationAndRotation(MtoCM(b.P), b.R);
				};

				if (!b0.bStatic) SetShapeTransform(b0, JointPair.ShapeA);
				if (!b1.bStatic) SetShapeTransform(b1, JointPair.ShapeB);
			}
		}
	}
}

void ASoftJointConstraintTestActor::DebugDraw(float Dt)
{
	int32 ConstraintIndex = JointSettings.GetConstraintIndex();

	for (int32 i = 0; i < JointPairs.Num(); ++i)
	{
		auto& JointPair = JointPairs[i];

		auto& b0 = JointPair.Body[0];
		auto& b1 = JointPair.Body[1];

		// Debug draw
		{
			auto World = GetWorld();
			auto DebugDrawBody = [&World](const FJointSlovePair::FRigidSloverData& b, const FColor& c) {
				auto p0 = MtoCM(b.P);
				auto p1 = MtoCM(b.P + b.ConstraintArm);
				DrawDebugLine(World, p0, p1, c, false, -1, 0, 1.0f);

				DrawDebugBox(World, MtoCM(b.ConnectorX), FVector{2.0, 2.0f, 2.0f}, FQuat::Identity, c, false, -1, 0, 1.0f);
			};

			FColor Color0 = FColor::Red;
			FColor Color1 = FColor::Green;

			DebugDrawBody(b0, Color0);
			DebugDrawBody(b1, Color1);

			auto dp = JointPair.SolvePositionDeltaLambda[ConstraintIndex] * JointPair.ConstraintAxis[ConstraintIndex];
			if(!b0.bStatic)
			{
				const Chaos::FVec3 dV0dt = b0.InvM * dp;
				DrawDebugLine(World, MtoCM(b0.ConnectorX), MtoCM(b0.ConnectorX - dV0dt), Color0, false, -1, 0, 1.0f);
			}
			if (!b1.bStatic)
			{
				const Chaos::FVec3 dV1dt = b1.InvM * dp;
				DrawDebugLine(World, MtoCM(b1.ConnectorX), MtoCM(b1.ConnectorX + dV1dt), Color1, false, -1, 0, 1.0f);
			}
		}
	}
}

void ASoftJointConstraintTestActor::InitVelocityConstraints(float Dt, FJointSlovePair& InJointSloverPair)
{
	auto& b0 = InJointSloverPair.Body[0];
	auto& b1 = InJointSloverPair.Body[1];

	// Position
	b0.ConnectorX = b0.P + b0.R.RotateVector(b0.LocalConnectorX.GetLocation());
	b1.ConnectorX = b1.P + b1.R.RotateVector(b1.LocalConnectorX.GetLocation());
	// Rotation
	b0.ConnectorR = b0.R * b0.LocalConnectorX.GetRotation();
	b1.ConnectorR = b1.R * b1.LocalConnectorX.GetRotation();
	// 

	b0.DP = Chaos::FVec3::ZeroVector;
	b1.DP = Chaos::FVec3::ZeroVector;
	b0.DQ = Chaos::FVec3::ZeroVector;
	b1.DQ = Chaos::FVec3::ZeroVector;

	InJointSloverPair.TotalLambda = 0;
	InJointSloverPair.NeedsSolve[0] = InJointSloverPair.NeedsSolve[1] = InJointSloverPair.NeedsSolve[2] = false;

	auto CalcuteInvI = [](FJointSlovePair::FRigidSloverData& b) {
		auto M = b.R.ToMatrix();
		auto M_T = M.GetTransposed();
		b.InvI = M * b.InvLocalInertiaTensor * M_T;
	};

	CalcuteInvI(InJointSloverPair.Body[0]);
	CalcuteInvI(InJointSloverPair.Body[1]);

	int32 ConstraintIndex = JointSettings.GetConstraintIndex();

	if(ConstraintIndex != INDEX_NONE)
	{
		InitPlanarPositionConstraint(Dt, InJointSloverPair, ConstraintIndex);

		if(InJointSloverPair.NeedsSolve[ConstraintIndex])
		{
			float EffectiveMass = 0;
			auto K = InJointSloverPair.ConstraintHardIM[ConstraintIndex];
			InJointSloverPair.SpingPart.CalculateSpringProperties(
				Dt,
				K,
				1.0f,
				InJointSloverPair.ConstraintCX[ConstraintIndex],
				JointSettings.Frequency,
				JointSettings.DampingRatio,
				EffectiveMass);
			InJointSloverPair.ConstraintSoftIM[ConstraintIndex] = 1.0 / EffectiveMass;
		}
	}
}

void ASoftJointConstraintTestActor::ApplyAxisPositionConstraint(float Dt, int32 ConstraintIndex, FJointSlovePair& InJointSloverPair)
{
	if (ConstraintIndex == INDEX_NONE) 
	{
		return;
	}

	auto& b0 = InJointSloverPair.Body[0];
	auto& b1 = InJointSloverPair.Body[1];

	const Chaos::FVec3 CX = b1.DP - b0.DP + Chaos::FVec3::CrossProduct(b1.DQ, b1.ConstraintArm) - Chaos::FVec3::CrossProduct(b0.DQ, b0.ConstraintArm);

	Chaos::FReal DeltaPosition = InJointSloverPair.ConstraintCX[ConstraintIndex] + Chaos::FVec3::DotProduct(CX, InJointSloverPair.ConstraintAxis[ConstraintIndex]);

	bool NeedsSolve = false;
	if (DeltaPosition > InJointSloverPair.ConstraintLimits[ConstraintIndex])
	{
		DeltaPosition -= InJointSloverPair.ConstraintLimits[ConstraintIndex];
		NeedsSolve = true;
	}
	else if (DeltaPosition < -InJointSloverPair.ConstraintLimits[ConstraintIndex])
	{
		DeltaPosition += InJointSloverPair.ConstraintLimits[ConstraintIndex];
		NeedsSolve = true;
	}

	if(NeedsSolve)
	{
		SolvePositionConstraints(Dt, ConstraintIndex, DeltaPosition, InJointSloverPair);
	}
}

void ASoftJointConstraintTestActor::InitPlanarPositionConstraint(float Dt, FJointSlovePair& InJointSloverPair, const int32 AxisIndex)
{
	const auto& bX0 = InJointSloverPair.Body[0].P;
	const auto& cR0 = InJointSloverPair.Body[0].ConnectorR;
	const auto& cX0 = InJointSloverPair.Body[0].ConnectorX;

	const auto& bX1 = InJointSloverPair.Body[1].P;
	const auto& cR1 = InJointSloverPair.Body[1].ConnectorR;
	const auto& cX1 = InJointSloverPair.Body[1].ConnectorX;

	const Chaos::FMatrix33 R0M = cR0.ToMatrix();
	Chaos::FVec3 Axis = R0M.GetAxis(AxisIndex);
	Chaos::FReal Delta = Chaos::FVec3::DotProduct(cX1 - cX0, Axis);

	// Calculate points relative to body, "Constraints Derivation for Rigid Body Simulation in 3D. equation 55" 
	// r0 + u = (cX0 - bX1) + (cX1 - cX0) = cX1 - bX1
	const Chaos::FVec3 ConstraintArm0 = cX1 - bX0;
	// r1 = cX1 - bX1
	const Chaos::FVec3 ConstraintArm1 = cX1 - bX1;

	InJointSloverPair.ConstraintLimits[AxisIndex] = JointSettings.LimitDistance[AxisIndex];
	InJointSloverPair.ConstraintAxis[AxisIndex] = Axis;

	auto CX = (cX1 - cX0).Dot(Axis);
	InJointSloverPair.ConstraintCX[AxisIndex] = CX;

	InJointSloverPair.Body[0].ConstraintArm = ConstraintArm0;
	InJointSloverPair.Body[1].ConstraintArm = ConstraintArm1;

	//【金山文档】 Github PositionBasedDynamics源码分析 Section: Constraint mass matrix
	// https://kdocs.cn/l/chHP9pOUPhv3
	const Chaos::FVec3 AngularAxis0 = ConstraintArm0.Cross(Axis);
	const Chaos::FVec3 IA0 = InJointSloverPair.Body[0].InvI * AngularAxis0;
	const Chaos::FReal II0 = IA0.Dot(AngularAxis0);

	const Chaos::FVec3 AngularAxis1 = ConstraintArm1.Cross(Axis);
	const Chaos::FVec3 IA1 = InJointSloverPair.Body[1].InvI * AngularAxis1;
	const Chaos::FReal II1 = IA1.Dot(AngularAxis1);

	InJointSloverPair.ConstraintHardIM[AxisIndex] = InJointSloverPair.Body[0].InvM + II0 + InJointSloverPair.Body[1].InvM + II1;
}


void ASoftJointConstraintTestActor::SolvePositionConstraints(float Dt,
	int32 ConstraintIndex, const Chaos::FReal DeltaPosition, FJointSlovePair& InJointSloverPair)
{
	if(!bSolvePosition) return;

	if (ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = InJointSloverPair.Body[0];
		auto& b1 = InJointSloverPair.Body[1];

		auto C = DeltaPosition;
		auto effective_mass = 1.0 / InJointSloverPair.ConstraintHardIM[ConstraintIndex];
		auto delta_lambda = -effective_mass * C;

		InJointSloverPair.SolvePositionDeltaLambda[ConstraintIndex] = delta_lambda;
		
		auto dp = delta_lambda * InJointSloverPair.ConstraintAxis[ConstraintIndex];

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = b0.InvI * (b0.ConstraintArm.Cross(dp));

			b0.DP -= dV0dt;
			b0.DQ -= dW0dt;
			
			//b0.P -= dV0dt;
			//b0.IntegrateRotation(-dW0dt);
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = b1.InvI * (b1.ConstraintArm.Cross(dp));

			b1.DP += dV1dt;
			b1.DQ += dW1dt;

			//b1.P += dV1dt;
			//b1.IntegrateRotation(dW1dt);
		}
	}
}

void ASoftJointConstraintTestActor::SolveVelocityConstraints(float Dt, FJointSlovePair& InJointSloverPair)
{
	if(!bSolveVelocity) return;

	int32 ConstraintIndex = JointSettings.GetConstraintIndex();

	if(ConstraintIndex != INDEX_NONE && InJointSloverPair.NeedsSolve[ConstraintIndex])
	{
		auto& b0 = InJointSloverPair.Body[0];
		auto& b1 = InJointSloverPair.Body[1];

		const auto& Axis = InJointSloverPair.ConstraintAxis[ConstraintIndex];

		Chaos::FVec3 v0 = b0.V + b0.W.Cross(b0.ConstraintArm);
		Chaos::FVec3 v1 = b1.V + b1.W.Cross(b1.ConstraintArm);

		Chaos::FReal jv = Axis.Dot(v1 - v0);

		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		Chaos::FReal effective_mass = 1.0 / InJointSloverPair.ConstraintSoftIM[ConstraintIndex];
		float lambda = -effective_mass * (jv + InJointSloverPair.SpingPart.GetBias(InJointSloverPair.TotalLambda));
		InJointSloverPair.TotalLambda += lambda; // Store accumulated impulse

		// apply lambda
		const Chaos::FVec3 dp = lambda * Axis;

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0 = b0.InvM * dp;
			const Chaos::FVec3 dW0 = b0.InvI * (b0.ConstraintArm.Cross(dp));

			b0.V -= dV0;
			b0.W -= dW0;
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1 = b1.InvM * dp;
			const Chaos::FVec3 dW1 = b1.InvI * (b1.ConstraintArm.Cross(dp));

			b1.V += dV1;
			b1.W += dW1;
		}
	}
}
