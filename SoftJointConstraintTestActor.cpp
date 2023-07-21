#include "SoftJointConstraintTestActor.h"
#include "Components/BoxComponent.h"

Chaos::FVec3 CMtoM(const Chaos::FVec3& InVec)
{
	return InVec * 0.01;
}

Chaos::FVec3 MtoCM(const Chaos::FVec3& InVec)
{
	return InVec * 100;
}

const Chaos::FVec3 Gravity{ 0, 0, -9.80 };

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
	else if (LimitDistance[2] > 0.01f)
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
		auto Extent = CMtoM(shape->GetScaledBoxExtent());
		
		constexpr Chaos::FReal Density = 1000; // kg/m^3

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

	for(int32 i=0; i<JointPairs.Num(); ++i)
	{
		auto& JointPair = JointPairs[i];

		auto& b0 = JointPair.Body[0];
		auto& b1 = JointPair.Body[1];

		// Integrate Velocity
		{
			if(!b0.bStatic)
			{
				b0.V += Dt * Gravity;
			}

			if(!b1.bStatic)
			{
				b1.V += Dt * Gravity;
			}
		}

		InitVelocityConstraints(Dt, JointPair);

		SolveVelocityConstraints(Dt, JointPair);

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

		SolvePositionConstraints(Dt, JointPair);

		// Update shape transform
		{
			auto SetShapeTransform = [](const FJointSlovePair::FRigidSloverData& b, UBoxComponent* shape) {
				shape->SetWorldLocationAndRotation(MtoCM(b.P), b.R);
			};

			if (!b0.bStatic) SetShapeTransform(b0, JointPair.ShapeA);
			if (!b1.bStatic) SetShapeTransform(b1, JointPair.ShapeB);
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
	// Position
	InJointSloverPair.Body[0].ConnectorX = InJointSloverPair.Body[0].P
		+ InJointSloverPair.Body[0].R.RotateVector(InJointSloverPair.Body[0].LocalConnectorX.GetLocation());
	InJointSloverPair.Body[1].ConnectorX = InJointSloverPair.Body[1].P
		+ InJointSloverPair.Body[1].R.RotateVector(InJointSloverPair.Body[1].LocalConnectorX.GetLocation());
	// Rotation
	InJointSloverPair.Body[0].ConnectorR = InJointSloverPair.Body[0].R * InJointSloverPair.Body[0].LocalConnectorX.GetRotation();
	InJointSloverPair.Body[1].ConnectorR = InJointSloverPair.Body[1].R * InJointSloverPair.Body[1].LocalConnectorX.GetRotation();
	// 

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

	InJointSloverPair.ConstraintLimits[AxisIndex] = Delta;
	InJointSloverPair.ConstraintAxis[AxisIndex] = Axis;

	auto CX = (cX1 - cX0).Dot(Axis);
	Chaos::FReal DeltaPosition = CX;

	bool NeedsSolve = false;
	if (JointSettings.LimitDistance[AxisIndex])
	{
		if (DeltaPosition > JointSettings.LimitDistance[AxisIndex])
		{
			DeltaPosition -= JointSettings.LimitDistance[AxisIndex];
			NeedsSolve = true;
		}
		else if (DeltaPosition < -JointSettings.LimitDistance[AxisIndex])
		{
			DeltaPosition += JointSettings.LimitDistance[AxisIndex];
			NeedsSolve = true;
		}
	}

	InJointSloverPair.NeedsSolve[AxisIndex] = NeedsSolve;
	InJointSloverPair.ConstraintCX[AxisIndex] = DeltaPosition;

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


void ASoftJointConstraintTestActor::SolvePositionConstraints(float Dt, FJointSlovePair& InJointSloverPair)
{
	if(!bSolvePosition) return;

	int32 ConstraintIndex = JointSettings.GetConstraintIndex();

	if (ConstraintIndex != INDEX_NONE && InJointSloverPair.NeedsSolve[ConstraintIndex])
	{
		auto& b0 = InJointSloverPair.Body[0];
		auto& b1 = InJointSloverPair.Body[1];

		auto C = InJointSloverPair.ConstraintCX[ConstraintIndex];
		auto effective_mass = 1.0 / InJointSloverPair.ConstraintHardIM[ConstraintIndex];
		auto delta_lambda = -effective_mass * C;

		InJointSloverPair.SolvePositionDeltaLambda[ConstraintIndex] = delta_lambda;
		
		auto dp = delta_lambda * InJointSloverPair.ConstraintAxis[ConstraintIndex];

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = b0.InvI * (b0.ConstraintArm.Cross(dp));

			b0.P -= dV0dt;
			b0.IntegrateRotation(-dW0dt);
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = b1.InvI * (b1.ConstraintArm.Cross(dp));

			b1.P += dV1dt;
			b1.IntegrateRotation(dW1dt);
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
