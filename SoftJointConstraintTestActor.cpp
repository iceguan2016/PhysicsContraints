#include "SoftJointConstraintTestActor.h"
#include "Components/BoxComponent.h"
#include "Chaos/Utilities.h"

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

		b.LocalConnectorX = InJointWorldTrans.GetRelativeTransform(BodyTransform);
		b.LocalConnectorX.SetLocation(CMtoM(b.LocalConnectorX.GetLocation()));

		if(!b.bStatic)
		{
		#if 0
			Chaos::FReal Mass = (Extent.X * Extent.Y * Extent.Z) * Density;
			//Chaos::FReal Mass = 3375;
			Chaos::FMatrix33 I = FMatrix::Identity;
			I.M[0][0] = 1.0 / 12 * Mass * (FMath::Square(Extent.Y) + FMath::Square(Extent.Z));
			I.M[1][1] = 1.0 / 12 * Mass * (FMath::Square(Extent.X) + FMath::Square(Extent.Z));
			I.M[2][2] = 1.0 / 12 * Mass * (FMath::Square(Extent.X) + FMath::Square(Extent.Y));

			b.InvM = 1.0 / Mass;
			b.InvLocalInertiaTensor = I.Inverse();
		#else
			Chaos::FMatrix33 I = FMatrix::Identity;
			I.M[0][0] = I.M[1][1] = I.M[2][2] = 2.53711187e-06;

			b.InvM = 1.0 / 65.148582458496094;
			b.InvLocalInertiaTensor = I;
		#endif
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

void ASoftJointConstraintTestActor::Simulate(float InDeltaTime)
{
	if(JointPairs.Num() <= 0)
	{
		return;
	}

	constexpr float FixedSimDeltaTime = 0.01f;

	float TotalSimTime = 0;
	while(TotalSimTime < InDeltaTime)
	{
		float Dt = FMath::Min(FixedSimDeltaTime, InDeltaTime - TotalSimTime);

		auto Body1PrevLoc = JointPairs[0].Body[1].P;
		auto Body1PrevRot = JointPairs[0].Body[1].R;

		AdvanceOneStep(Dt);

		auto Body1PostLoc = JointPairs[0].Body[1].P;
		auto Body1PostRot = JointPairs[0].Body[1].R;

		if(FMath::Abs(Body1PostLoc.Z - Body1PrevLoc.Z) > 100.0f)
		{
			JointPairs[0].Body[1].P = Body1PrevLoc;
			JointPairs[0].Body[1].R = Body1PrevRot;

			AdvanceOneStep(Dt);
		}

		TotalSimTime += Dt;
	}
}

void ASoftJointConstraintTestActor::AdvanceOneStep(float Dt)
{
	int32 ConstraintIndex = JointSettings.GetConstraintIndex();
	if (ConstraintIndex == INDEX_NONE) return;

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

			// Apply gravity acceleration
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

			auto IntegratePositionAndRotation = [Dt](FJointSlovePair::FRigidSloverData& InBody) {
				InBody.P += Dt * InBody.V;

#if 1
				InBody.R = Chaos::FRotation3::IntegrateRotationWithAngularVelocity(InBody.R, InBody.W, Dt);
#else
				InBody.IntegrateRotation(InBody.W * Dt);
#endif
			};


			// Integrate Position
			{
				if (!b0.bStatic) IntegratePositionAndRotation(b0);
				if (!b1.bStatic) IntegratePositionAndRotation(b1);
			}

			// Init
			InitPositionConstraints(Dt, ConstraintIndex, JointPair);
		}
	}

	// Solve position
	{
		for (int32 Itr = 0; Itr < NumPositionIterations; ++Itr)
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
				InSolveData.V += InSolveData.DP * InvDt;
				InSolveData.W += InSolveData.DQ * InvDt;
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
		for (int32 Itr = 0; Itr < NumVelocityIterations; ++Itr)
		{
			for (int32 i = 0; i < JointPairs.Num(); ++i)
			{
				auto& JointPair = JointPairs[i];

				auto& b0 = JointPair.Body[0];
				auto& b1 = JointPair.Body[1];

				ApplyAxisVelocityConstraint(Dt, ConstraintIndex, JointPair);
			}
		}
	}

	// Update shape transform
	{
		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& JointPair = JointPairs[i];

			ApplyCorrections(JointPair);

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
				//DrawDebugLine(World, p0, p1, c, false, -1, 0, 1.0f);

				//DrawDebugBox(World, MtoCM(b.ConnectorX), FVector{2.0, 2.0f, 2.0f}, FQuat::Identity, c, false, -1, 0, 1.0f);
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

			DrawDebugLine(World, MtoCM(b0.ConnectorX), MtoCM(b1.ConnectorX), FColor::Blue, false, -1, 0, 1.0f);

			auto P = MtoCM((b0.ConnectorX + b1.ConnectorX) * 0.5f);
			auto Dist = JointPair.ConstraintCX[ConstraintIndex];
			DrawDebugString(World, P, FString::Printf(TEXT("Dist:%.3fcm"), Dist), nullptr, FColor::Red, 0, false, 1.5f);
		}
	}
}

void ASoftJointConstraintTestActor::InitPositionConstraints(float Dt, int32 ConstraintIndex, FJointSlovePair& InJointSloverPair)
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

	InJointSloverPair.ConstraintLambda[ConstraintIndex] = 0;

	auto CalcuteInvI = [](FJointSlovePair::FRigidSloverData& b) {
		auto M = b.R.ToMatrix();
		auto M_T = M.GetTransposed();
		b.InvI = M * b.InvLocalInertiaTensor * M_T;
	};

	CalcuteInvI(InJointSloverPair.Body[0]);
	CalcuteInvI(InJointSloverPair.Body[1]);

	if(ConstraintIndex != INDEX_NONE)
	{
		InitPlanarPositionConstraint(Dt, InJointSloverPair, ConstraintIndex);

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
		if(bSoftConstraint)
		{
			SolvePositionConstraintsSoft(Dt, ConstraintIndex, DeltaPosition, InJointSloverPair);
		}
		else
		{
			SolvePositionConstraints(Dt, ConstraintIndex, DeltaPosition, InJointSloverPair);
		}
	}
}

void ASoftJointConstraintTestActor::InitPlanarPositionConstraint(float Dt, FJointSlovePair& InJointSloverPair, const int32 AxisIndex)
{
	auto& b0 = InJointSloverPair.Body[0];
	auto& b1 = InJointSloverPair.Body[1];

	const auto& bX0 = b0.P;
	const auto& cR0 = b0.ConnectorR;
	const auto& cX0 = b0.ConnectorX;

	const auto& bX1 = b1.P;
	const auto& cR1 = b1.ConnectorR;
	const auto& cX1 = b1.ConnectorX;

	const Chaos::FMatrix33 R0M = cR0.ToMatrix();
	Chaos::FVec3 Axis = R0M.GetAxis(AxisIndex);
	Chaos::FReal Delta = Chaos::FVec3::DotProduct(cX1 - cX0, Axis);

	// Calculate points relative to body, "Constraints Derivation for Rigid Body Simulation in 3D. equation 55" 
	// r0 + u = (cX0 - bX1) + (cX1 - cX0) = cX1 - bX0
	const Chaos::FVec3 ConstraintArm0 = cX1 - bX0;
	// r1 = cX1 - bX1
	const Chaos::FVec3 ConstraintArm1 = cX1 - bX1;

	InJointSloverPair.ConstraintLimits[AxisIndex] = JointSettings.LimitDistance[AxisIndex];
	InJointSloverPair.ConstraintAxis[AxisIndex] = Axis;

	auto CX = (cX1 - cX0).Dot(Axis);
	InJointSloverPair.ConstraintCX[AxisIndex] = CX;

	b0.ConstraintArm = ConstraintArm0;
	b1.ConstraintArm = ConstraintArm1;

	//【金山文档】 Github PositionBasedDynamics源码分析 Section: Constraint mass matrix

	// vector are column major
	// II = (rxn)^T * InvI * (rxn)
	// K = J^T * InvM * J = m0^-1 + II0 + m1^-1 + II1
	// 1. calculate 'rxn'
	const Chaos::FVec3 AngularAxis0 = ConstraintArm0.Cross(Axis);
	const Chaos::FVec3 AngularAxis1 = ConstraintArm1.Cross(Axis);

	// 2. calculate 'InvI * (rxn)'
	// const Chaos::FVec3 IA0 = InJointSloverPair.Body[0].InvI.TransformVector(AngularAxis0);
	const Chaos::FVec3 IA0 = Chaos::Utilities::Multiply(b0.InvI, AngularAxis0);
	// const Chaos::FVec3 IA1 = InJointSloverPair.Body[1].InvI.TransformVector(AngularAxis1);
	const Chaos::FVec3 IA1 = Chaos::Utilities::Multiply(b1.InvI, AngularAxis1);
	// 3. calculate '(rxn)^T * InvI * (rxn)'
	const Chaos::FReal II0 = Chaos::FVec3::DotProduct(AngularAxis0, IA0);
	const Chaos::FReal II1 = Chaos::FVec3::DotProduct(AngularAxis1, IA1);

	InJointSloverPair.ConstraintHardIM[AxisIndex] = InJointSloverPair.Body[0].InvM + II0 + InJointSloverPair.Body[1].InvM + II1;
}


void ASoftJointConstraintTestActor::ApplyCorrections(FJointSlovePair& InJointSloverPair)
{
	auto& b0 = InJointSloverPair.Body[0];
	auto& b1 = InJointSloverPair.Body[1];

	auto ApplyCorrection = [](FJointSlovePair::FRigidSloverData& b) {
		b.P += b.DP;
		b.R = !b.DQ.IsZero() ? Chaos::FRotation3::IntegrateRotationWithAngularVelocity(b.R, Chaos::FVec3(b.DQ), Chaos::FReal(1)) : b.R;
	};

	if (!b0.bStatic) ApplyCorrection(b0);
	if (!b1.bStatic) ApplyCorrection(b1);
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
	
		InJointSloverPair.ConstraintLambda[ConstraintIndex] += delta_lambda;
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

void ASoftJointConstraintTestActor::SolvePositionConstraintsSoft(float Dt, 
	int32 ConstraintIndex, const Chaos::FReal DeltaPosition, FJointSlovePair& InJointSloverPair)
{
	if (!bSolvePosition) return;

	if (ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = InJointSloverPair.Body[0];
		auto& b1 = InJointSloverPair.Body[1];

		const auto& Axis = InJointSloverPair.ConstraintAxis[ConstraintIndex];

		Chaos::FReal total_lambda = InJointSloverPair.ConstraintLambda[ConstraintIndex];
		Chaos::FReal k_hard = InJointSloverPair.ConstraintHardIM[ConstraintIndex];
		Chaos::FReal effective_mass_hard = 1.0 / k_hard;

		Chaos::FReal omega = 2.0f * PI * JointSettings.Frequency;

		// Calculate spring constant k and drag constant c (page 45)
		Chaos::FReal k = effective_mass_hard * FMath::Square(omega);
		Chaos::FReal c = 2.0f * effective_mass_hard * JointSettings.DampingRatio * omega;

		// Ks = M_hard_eff * k * h^2
		// Kd = M_hard_eff * c * h
		Chaos::FReal Ks = effective_mass_hard * k * Dt * Dt;
		Chaos::FReal Kd = effective_mass_hard * c * Dt;

		Chaos::FVec3 v0 = b0.V + b0.W.Cross(b0.ConstraintArm);
		Chaos::FVec3 v1 = b1.V + b1.W.Cross(b1.ConstraintArm);

		// Cdot*h = v_target*h - jv*h
		// v_target = 0
		Chaos::FReal Cdot_x_h = Axis.Dot(v1 - v0) * Dt;

		// DeltaPosition = C2 + Cdelta

		// (Khard*(Kd+Ks)+1)*delta_lambda = -(Ks*(C2 + Cdelta) + Kd*Cdot*h - Lambda1)
		Chaos::FReal delta_lambda = -(Ks*DeltaPosition + Kd*Cdot_x_h - total_lambda) / (k_hard * (Kd + Ks) + 1);

		InJointSloverPair.ConstraintLambda[ConstraintIndex] += delta_lambda;

		// dp = J_T * delta_lambda
		Chaos::FVec3 dp = delta_lambda * Axis;

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = b0.InvI * (b0.ConstraintArm.Cross(dp));

			b0.DP -= dV0dt;
			b0.DQ -= dW0dt;
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = b1.InvI * (b1.ConstraintArm.Cross(dp));

			b1.DP += dV1dt;
			b1.DQ += dW1dt;
		}
	}
}

void ASoftJointConstraintTestActor::ApplyAxisVelocityConstraint(float Dt, int32 ConstraintIndex, FJointSlovePair& InJointSloverPair)
{
	if(!bSolveVelocity) return;

	if (FMath::Abs(InJointSloverPair.ConstraintLambda[ConstraintIndex]) > UE_SMALL_NUMBER)
	{
		Chaos::FReal TargetVel = 0.0f;
		/*if (InJointSloverPair.ConstraintRestitution[ConstraintIndex] != 0.0f)
		{
			const FReal InitVel = InitConstraintAxisLinearVelocities[ConstraintIndex];
			TargetVel = InitVel > Chaos_Joint_LinearVelocityThresholdToApplyRestitution ?
				-PositionConstraints.ConstraintRestitution[ConstraintIndex] * InitVel : 0.0f;
		}*/

		if(bSoftConstraint)
		{
			SolveVelocityConstraintsSoft(Dt, ConstraintIndex, InJointSloverPair);
		}
		else
		{
			SolveLinearVelocityConstraints(Dt, ConstraintIndex, InJointSloverPair, TargetVel);
		}
	}
}

void ASoftJointConstraintTestActor::SolveLinearVelocityConstraints(float Dt, 
	int32 ConstraintIndex, FJointSlovePair& InJointSloverPair, const Chaos::FReal TargetVel)
{
	auto& b0 = InJointSloverPair.Body[0];
	auto& b1 = InJointSloverPair.Body[1];

	const Chaos::FVec3 CV0 = b0.V + b0.W.Cross(b0.ConstraintArm);
	const Chaos::FVec3 CV1 = b1.V + b1.W.Cross(b1.ConstraintArm);
	const Chaos::FVec3 CV = CV1 - CV0;
	// K * DeltaLamda = -JV
	const Chaos::FReal DeltaLambda = -(CV.Dot(InJointSloverPair.ConstraintAxis[ConstraintIndex]) - TargetVel) / InJointSloverPair.ConstraintHardIM[ConstraintIndex];

	const Chaos::FVec3 MDV = DeltaLambda * InJointSloverPair.ConstraintAxis[ConstraintIndex];

	if (!b0.bStatic)
	{
		const Chaos::FVec3 DV0 = b0.InvM * MDV;
		const Chaos::FVec3 DW0 = b0.InvI * (b0.ConstraintArm.Cross(MDV));;

		b0.V -= DV0;
		b0.W -= DW0;
	}
	if (!b1.bStatic)
	{
		const Chaos::FVec3 DV1 = b1.InvM * MDV;
		const Chaos::FVec3 DW1 = b1.InvI * (b1.ConstraintArm.Cross(MDV));;

		b1.V += DV1;
		b1.W += DW1;
	}

	{
		const Chaos::FVec3 _CV0 = b0.V + b0.W.Cross(b0.ConstraintArm);
		const Chaos::FVec3 _CV1 = b1.V + b1.W.Cross(b1.ConstraintArm);
		const Chaos::FVec3 _CV = _CV1 - _CV0;

		auto Cdot = _CV.Dot(InJointSloverPair.ConstraintAxis[ConstraintIndex]);
		if(Cdot == TargetVel)
		{
			int32 Stop = 0;
		}
	}
}

void ASoftJointConstraintTestActor::SolveVelocityConstraintsSoft(float Dt, int32 ConstraintIndex, FJointSlovePair& InJointSloverPair)
{
	if(!bSolveVelocity) return;

	if(ConstraintIndex != INDEX_NONE)
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
		float lambda = -effective_mass * (jv + InJointSloverPair.SpingPart.GetBias(InJointSloverPair.ConstraintLambda[ConstraintIndex]));
		InJointSloverPair.ConstraintLambda[ConstraintIndex] += lambda; // Store accumulated impulse

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
