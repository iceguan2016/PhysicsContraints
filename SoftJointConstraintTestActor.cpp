#include "SoftJointConstraintTestActor.h"
#include "Components/BoxComponent.h"
#include "Chaos/Utilities.h"

/* -----FRigidSloverHandle----- */
FRigidSloverData* FRigidSolverDataContainer::FRigidSloverHandle::Get()
{
	return IsValid()? Container->GetSolverData(*this) : nullptr;
}

FRigidSloverData& FRigidSolverDataContainer::FRigidSloverHandle::operator*()
{
	check(IsValid());
	return *Get();
}

FRigidSloverData* FRigidSolverDataContainer::FRigidSloverHandle::operator->()
{
	check(IsValid());
	return Get();
}

/* -----FJointSlovePair----- */
void FJointSlovePair::ComputeBodyState(int32 BodyIndex)
{
	CurrentPs[BodyIndex] = P(BodyIndex);
	CurrentQs[BodyIndex] = Q(BodyIndex);
	ConnectorXs[BodyIndex] = CurrentPs[BodyIndex] + CurrentQs[BodyIndex] * LocalConnectorXs[BodyIndex].GetTranslation();
	ConnectorRs[BodyIndex] = CurrentQs[BodyIndex] * LocalConnectorXs[BodyIndex].GetRotation();
}


/* -----ASoftJointConstraintTestActor----- */
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
	class UPrimitiveComponent* InShapeA, 
	class UPrimitiveComponent* InShapeB, 
	const FTransform& InJointWorldTrans, 
	bool bShapeAStatic)
{
	if(!InShapeA || !InShapeB)
	{
		return;
	}

	auto InitJointBody = [&InJointWorldTrans](
		FJointSlovePair& Joint,
		int32 bodyIndex,
		FRigidSloverData& Body,
		class UPrimitiveComponent* shape) {
		auto BodyTransform = shape->GetComponentTransform();

		constexpr Chaos::FReal Density = 0.001; // kg/cm^3

		Joint.LocalConnectorXs[bodyIndex] = InJointWorldTrans.GetRelativeTransform(BodyTransform);
		Joint.LocalConnectorXs[bodyIndex].SetLocation(CMtoM(Joint.LocalConnectorXs[bodyIndex].GetLocation()));

		if (!Body.bStatic)
		{
#if 0
			// GetScaledBoxExtent return Half Extent
			auto Extent = CMtoM(shape->GetScaledBoxExtent() * 2.0);
			//auto Extent = FVector{ 1.5, 1.5, 1.5f };

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

			Body.InvM = 1.0 / 65.148582458496094;
			Body.InvLocalInertiaTensor = I;
#endif
		}
		else
		{
			Body.InvM = 0.0f;
			Body.InvLocalInertiaTensor.M[0][0] = 0;
			Body.InvLocalInertiaTensor.M[1][1] = 0;
			Body.InvLocalInertiaTensor.M[2][2] = 0;
		}

		Body.P() = CMtoM(BodyTransform.GetLocation());
		Body.Q() = BodyTransform.GetRotation();
	};

	auto& Pair = JointPairs.Emplace_GetRef();

	auto FindOrCreateSolverData = [&](int32 bodyIndex, class UPrimitiveComponent* InShape, bool bStatic)->FRigidSloverHandle {
		auto SolverDataHandle = RigidSloverDataContainer.FindSolverDataByShape(InShape);
		if(SolverDataHandle.IsValid())
		{
			return SolverDataHandle;
		}
		else
		{
			// Create new one
			auto NewSolverDataHandle = RigidSloverDataContainer.AllocNewSolverData();
			auto& SolverData = *NewSolverDataHandle;

			SolverData.bStatic = bStatic;
			SolverData.Shape = InShape;
			InitJointBody(Pair, bodyIndex, SolverData, InShape);

			return NewSolverDataHandle;
		}
	};

	Pair.ShapeA = InShapeA;
	Pair.ShapeB = InShapeB;
	Pair.BodySolver[0] = FindOrCreateSolverData(0, InShapeA, bShapeAStatic);
	Pair.BodySolver[1] = FindOrCreateSolverData(1, InShapeB, false);
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

		AdvanceOneStep(Dt);

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
		RigidSloverDataContainer.ForEach([&](FRigidSloverData& InSolverData) {
			auto IntegratePositionAndRotation = [Dt](FRigidSloverData& InBody) {
				// Apply gravity acceleration
				InBody.V() += Dt * Gravity;
				// Update position
				InBody.P() += Dt * InBody.V();
				// Update rotation
#if 1
				InBody.Q() = Chaos::FRotation3::IntegrateRotationWithAngularVelocity(InBody.Q(), InBody.W(), Dt);
#else
				InBody.IntegrateRotation(InBody.W() * Dt);
#endif
			};

			// Integrate Position
			if (!InSolverData.bStatic)
			{
				IntegratePositionAndRotation(InSolverData);
			}

			// Initialize solver data
			InSolverData.DP() = Chaos::FVec3::ZeroVector;
			InSolverData.DP() = Chaos::FVec3::ZeroVector;
			InSolverData.DQ() = Chaos::FVec3::ZeroVector;
			InSolverData.DQ() = Chaos::FVec3::ZeroVector;
		});
	}

	// Solve position
	{
		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& Joint = JointPairs[i];

			// 1.Cache P/Q to CurrentPs/CurrentQs for each body
			// 2.Update connect X and R for each body
			Joint.ComputeBodyState(0);
			Joint.ComputeBodyState(1);
			// 3.Update InvIs
			auto UpdateInvI = [&Joint](int32 BodyIndex) {
				auto& Body = Joint.Body(BodyIndex);
				auto M = Joint.CurrentQs[BodyIndex].ToMatrix();
				auto M_T = M.GetTransposed();
				Joint.InvIs[BodyIndex] = M * Body.InvLocalInertiaTensor * M_T;
			};

			UpdateInvI(0);
			UpdateInvI(1);

			// 4.Init position constraints
			InitPositionConstraints(Dt, ConstraintIndex, Joint);
		}

		for (int32 Itr = 0; Itr < NumPositionIterations; ++Itr)
		{
			for (int32 i = 0; i < JointPairs.Num(); ++i)
			{
				auto& JointPair = JointPairs[i];

				ApplyAxisPositionConstraint(Dt, ConstraintIndex, JointPair);
			}
		}

		auto SetImplicitVelocity = [](FRigidSloverData& InSolveData, Chaos::FReal Dt)
		{
			if (!InSolveData.bStatic && (Dt != Chaos::FReal(0)))
			{
				const Chaos::FReal InvDt = Chaos::FReal(1) / Chaos::FReal(Dt);
				InSolveData.V() += InSolveData.DP() * InvDt;
				InSolveData.W() += InSolveData.DQ() * InvDt;
			}
		};

		// SetImplicitVelocity
		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& Joint = JointPairs[i];

			auto& b0 = Joint.Body(0);
			auto& b1 = Joint.Body(1);

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
				auto& Joint = JointPairs[i];

				ApplyAxisVelocityConstraint(Dt, ConstraintIndex, Joint);
			}
		}
	}

	// Update shape transform
	{
		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& Joint = JointPairs[i];

			ApplyCorrections(Joint);

			auto& b0 = Joint.Body(0);
			auto& b1 = Joint.Body(1);

			{
				auto SetShapeTransform = [](FRigidSloverData& b, UPrimitiveComponent* shape) {
					shape->SetWorldLocationAndRotation(MtoCM(b.P()), b.Q());
				};

				if (!b0.bStatic) SetShapeTransform(b0, Joint.ShapeA);
				if (!b1.bStatic) SetShapeTransform(b1, Joint.ShapeB);
			}
		}
	}
}

void ASoftJointConstraintTestActor::DebugDraw(float Dt)
{
	int32 ConstraintIndex = JointSettings.GetConstraintIndex();

	for (int32 i = 0; i < JointPairs.Num(); ++i)
	{
		auto& Joint = JointPairs[i];

		auto& b0 = Joint.Body(0);
		auto& b1 = Joint.Body(1);

		// Debug draw
		{
			auto World = GetWorld();
			auto DebugDrawBody = [&Joint, &World](int32 bodyIndex, FRigidSloverData& b, const FColor& c) {
				auto p0 = MtoCM(b.P());
				auto p1 = MtoCM(b.P() + Joint.ConstraintArms[bodyIndex]);
				//DrawDebugLine(World, p0, p1, c, false, -1, 0, 1.0f);

				//DrawDebugBox(World, MtoCM(b.ConnectorX), FVector{2.0, 2.0f, 2.0f}, FQuat::Identity, c, false, -1, 0, 1.0f);
			};

			FColor Color0 = FColor::Red;
			FColor Color1 = FColor::Green;

			DebugDrawBody(0, b0, Color0);
			DebugDrawBody(1, b1, Color1);

			auto dp = Joint.SolvePositionDeltaLambda[ConstraintIndex] * Joint.ConstraintAxis[ConstraintIndex];
			if(!b0.bStatic)
			{
				const Chaos::FVec3 dV0dt = b0.InvM * dp;
				DrawDebugLine(World, MtoCM(Joint.ConnectorXs[0]), MtoCM(Joint.ConnectorXs[0] - dV0dt), Color0, false, -1, 0, 1.0f);
			}
			if (!b1.bStatic)
			{
				const Chaos::FVec3 dV1dt = b1.InvM * dp;
				DrawDebugLine(World, MtoCM(Joint.ConnectorXs[1]), MtoCM(Joint.ConnectorXs[1] + dV1dt), Color1, false, -1, 0, 1.0f);
			}

			DrawDebugLine(World, MtoCM(Joint.ConnectorXs[0]), MtoCM(Joint.ConnectorXs[1]), FColor::Blue, false, -1, 0, 1.0f);

			auto P = MtoCM((Joint.ConnectorXs[0] + Joint.ConnectorXs[1]) * 0.5f);
			auto Dist = Joint.ConstraintCX[ConstraintIndex];
			DrawDebugString(World, P, FString::Printf(TEXT("Dist:%.3fcm"), Dist), nullptr, FColor::Red, 0, false, 1.5f);
		}
	}
}

void ASoftJointConstraintTestActor::InitPositionConstraints(float Dt, int32 ConstraintIndex, FJointSlovePair& Joint)
{
	Joint.ConstraintLambda[ConstraintIndex] = 0;

	if(ConstraintIndex != INDEX_NONE)
	{
		InitPlanarPositionConstraint(Dt, Joint, ConstraintIndex);

		float EffectiveMass = 0;
		auto K = Joint.ConstraintHardIM[ConstraintIndex];
		Joint.SpingPart.CalculateSpringProperties(
			Dt,
			K,
			1.0f,
			Joint.ConstraintCX[ConstraintIndex],
			JointSettings.Frequency,
			JointSettings.DampingRatio,
			EffectiveMass);
		Joint.ConstraintSoftIM[ConstraintIndex] = 1.0 / EffectiveMass;
	}
}

void ASoftJointConstraintTestActor::ApplyAxisPositionConstraint(float Dt, int32 ConstraintIndex, FJointSlovePair& Joint)
{
	if (ConstraintIndex == INDEX_NONE) 
	{
		return;
	}

	auto& b0 = Joint.Body(0);
	auto& b1 = Joint.Body(1);

	const Chaos::FVec3 CX = b1.DP() - b0.DP() + Chaos::FVec3::CrossProduct(b1.DQ(), Joint.ConstraintArms[1]) - Chaos::FVec3::CrossProduct(b0.DQ(), Joint.ConstraintArms[0]);

	Chaos::FReal DeltaPosition = Joint.ConstraintCX[ConstraintIndex] + Chaos::FVec3::DotProduct(CX, Joint.ConstraintAxis[ConstraintIndex]);

	bool NeedsSolve = false;
	if (DeltaPosition > Joint.ConstraintLimits[ConstraintIndex])
	{
		DeltaPosition -= Joint.ConstraintLimits[ConstraintIndex];
		NeedsSolve = true;
	}
	else if (DeltaPosition < -Joint.ConstraintLimits[ConstraintIndex])
	{
		DeltaPosition += Joint.ConstraintLimits[ConstraintIndex];
		NeedsSolve = true;
	}

	if(NeedsSolve)
	{
		if(bSoftPositionConstraint)
		{
			SolvePositionConstraintsSoft(Dt, ConstraintIndex, DeltaPosition, Joint);
		}
		else
		{
			SolvePositionConstraints(Dt, ConstraintIndex, DeltaPosition, Joint);
		}
	}
}

void ASoftJointConstraintTestActor::InitPlanarPositionConstraint(float Dt, FJointSlovePair& Joint, const int32 AxisIndex)
{
	// 要用修正后的位置计算，P+DP
	const auto& bX0 = Joint.CurrentPs[0];
	const auto& cR0 = Joint.ConnectorRs[0];
	const auto& cX0 = Joint.ConnectorXs[0];

	const auto& bX1 = Joint.CurrentPs[1];
	const auto& cR1 = Joint.ConnectorRs[1];
	const auto& cX1 = Joint.ConnectorXs[1];

	const Chaos::FMatrix33 R0M = cR0.ToMatrix();
	Chaos::FVec3 Axis = R0M.GetAxis(AxisIndex);
	Chaos::FReal Delta = Chaos::FVec3::DotProduct(cX1 - cX0, Axis);

	// Calculate points relative to body, "Constraints Derivation for Rigid Body Simulation in 3D. equation 55" 
	// r0 + u = (cX0 - bX1) + (cX1 - cX0) = cX1 - bX0
	const Chaos::FVec3 ConstraintArm0 = cX1 - bX0;
	// r1 = cX1 - bX1
	const Chaos::FVec3 ConstraintArm1 = cX1 - bX1;

	Joint.ConstraintLimits[AxisIndex] = JointSettings.LimitDistance[AxisIndex];
	Joint.ConstraintAxis[AxisIndex] = Axis;

	auto CX = (cX1 - cX0).Dot(Axis);
	Joint.ConstraintCX[AxisIndex] = CX;

	Joint.ConstraintArms[0] = ConstraintArm0;
	Joint.ConstraintArms[1] = ConstraintArm1;

	//【金山文档】 Github PositionBasedDynamics源码分析 Section: Constraint mass matrix

	// vector are column major
	// II = (rxn)^T * InvI * (rxn)
	// K = J^T * InvM * J = m0^-1 + II0 + m1^-1 + II1
	// 1. calculate 'rxn'
	const Chaos::FVec3 AngularAxis0 = ConstraintArm0.Cross(Axis);
	const Chaos::FVec3 AngularAxis1 = ConstraintArm1.Cross(Axis);

	// 2. calculate 'InvI * (rxn)'
	// const Chaos::FVec3 IA0 = Joint.Body[0].InvI.TransformVector(AngularAxis0);
	const Chaos::FVec3 IA0 = Chaos::Utilities::Multiply(Joint.InvIs[0], AngularAxis0);
	// const Chaos::FVec3 IA1 = Joint.Body[1].InvI.TransformVector(AngularAxis1);
	const Chaos::FVec3 IA1 = Chaos::Utilities::Multiply(Joint.InvIs[1], AngularAxis1);
	// 3. calculate '(rxn)^T * InvI * (rxn)'
	const Chaos::FReal II0 = Chaos::FVec3::DotProduct(AngularAxis0, IA0);
	const Chaos::FReal II1 = Chaos::FVec3::DotProduct(AngularAxis1, IA1);

	Joint.ConstraintHardIM[AxisIndex] = Joint.Body(0).InvM + II0 + Joint.Body(1).InvM + II1;
}


void ASoftJointConstraintTestActor::ApplyCorrections(FJointSlovePair& Joint)
{
	auto ApplyCorrection = [&Joint](int BodyIndex) {
		Joint.Body(BodyIndex).P() = Joint.P(BodyIndex);
		Joint.Body(BodyIndex).Q() = Joint.Q(BodyIndex);
	};

	ApplyCorrection(0);
	ApplyCorrection(1);
}

void ASoftJointConstraintTestActor::SolvePositionConstraints(float Dt,
	int32 ConstraintIndex, const Chaos::FReal DeltaPosition, FJointSlovePair& Joint)
{
	if(!bSolvePosition) return;

	if (ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = Joint.Body(0);
		auto& b1 = Joint.Body(1);

		auto C = DeltaPosition;
		auto effective_mass = 1.0 / Joint.ConstraintHardIM[ConstraintIndex];
		auto delta_lambda = -effective_mass * C;
	
		Joint.ConstraintLambda[ConstraintIndex] += delta_lambda;
		Joint.SolvePositionDeltaLambda[ConstraintIndex] = delta_lambda;
		
		auto dp = delta_lambda * Joint.ConstraintAxis[ConstraintIndex];

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = Joint.InvIs[0] * (Joint.ConstraintArms[0].Cross(dp));

			b0.DP() -= dV0dt;
			b0.DQ() -= dW0dt;
			
			//b0.P -= dV0dt;
			//b0.IntegrateRotation(-dW0dt);
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = Joint.InvIs[1] * (Joint.ConstraintArms[1].Cross(dp));

			b1.DP() += dV1dt;
			b1.DQ() += dW1dt;

			//b1.P += dV1dt;
			//b1.IntegrateRotation(dW1dt);
		}
	}
}

void ASoftJointConstraintTestActor::SolvePositionConstraintsSoft(float Dt, 
	int32 ConstraintIndex, const Chaos::FReal DeltaPosition, FJointSlovePair& Joint)
{
	if (!bSolvePosition) return;

	if (ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = Joint.Body(0);
		auto& b1 = Joint.Body(1);

		const auto& Axis = Joint.ConstraintAxis[ConstraintIndex];

		Chaos::FReal total_lambda = Joint.ConstraintLambda[ConstraintIndex];
		Chaos::FReal k_hard = Joint.ConstraintHardIM[ConstraintIndex];
		Chaos::FReal effective_mass_hard = 1.0 / k_hard;

		Chaos::FReal omega = 2.0f * PI * JointSettings.Frequency;

		// Calculate spring constant k and drag constant c (page 45)
		Chaos::FReal k = effective_mass_hard * FMath::Square(omega);
		Chaos::FReal c = 2.0f * effective_mass_hard * JointSettings.DampingRatio * omega;

		// Ks = M_hard_eff * k * h^2
		// Kd = M_hard_eff * c * h
		Chaos::FReal Ks = effective_mass_hard * k * Dt * Dt;
		Chaos::FReal Kd = effective_mass_hard * c * Dt;

		Chaos::FVec3 v0 = b0.V() + b0.W().Cross(Joint.ConstraintArms[0]);
		Chaos::FVec3 v1 = b1.V() + b1.W().Cross(Joint.ConstraintArms[1]);

		// Cdot*h = v_target*h - jv*h
		// v_target = 0
		Chaos::FReal Cdot_x_h = Axis.Dot(v1 - v0) * Dt;

		// DeltaPosition = C2 + Cdelta

		// (Khard*(Kd+Ks)+1)*delta_lambda = -(Ks*(C2 + Cdelta) + Kd*Cdot*h - Lambda1)
		Chaos::FReal delta_lambda = -(Ks*DeltaPosition + Kd*Cdot_x_h - total_lambda) / (k_hard * (Kd + Ks) + 1);

		Joint.ConstraintLambda[ConstraintIndex] += delta_lambda;

		// dp = J_T * delta_lambda
		Chaos::FVec3 dp = delta_lambda * Axis;

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = Joint.InvIs[0] * (Joint.ConstraintArms[0].Cross(dp));

			b0.DP() -= dV0dt;
			b0.DQ() -= dW0dt;
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = Joint.InvIs[1] * (Joint.ConstraintArms[1].Cross(dp));

			b1.DP() += dV1dt;
			b1.DQ() += dW1dt;
		}
	}
}

void ASoftJointConstraintTestActor::ApplyAxisVelocityConstraint(float Dt, int32 ConstraintIndex, FJointSlovePair& Joint)
{
	if(!bSolveVelocity) return;

	if (FMath::Abs(Joint.ConstraintLambda[ConstraintIndex]) > UE_SMALL_NUMBER)
	{
		Chaos::FReal TargetVel = 0.0f;
		/*if (Joint.ConstraintRestitution[ConstraintIndex] != 0.0f)
		{
			const FReal InitVel = InitConstraintAxisLinearVelocities[ConstraintIndex];
			TargetVel = InitVel > Chaos_Joint_LinearVelocityThresholdToApplyRestitution ?
				-PositionConstraints.ConstraintRestitution[ConstraintIndex] * InitVel : 0.0f;
		}*/

		if(bSoftVelocityConstraint)
		{
			SolveVelocityConstraintsSoft(Dt, ConstraintIndex, Joint);
		}
		else
		{
			SolveLinearVelocityConstraints(Dt, ConstraintIndex, Joint, TargetVel);
		}
	}
}

void ASoftJointConstraintTestActor::SolveLinearVelocityConstraints(float Dt, 
	int32 ConstraintIndex, FJointSlovePair& Joint, const Chaos::FReal TargetVel)
{
	auto& b0 = Joint.Body(0);
	auto& b1 = Joint.Body(1);

	const Chaos::FVec3 CV0 = b0.V() + b0.W().Cross(Joint.ConstraintArms[0]);
	const Chaos::FVec3 CV1 = b1.V() + b1.W().Cross(Joint.ConstraintArms[1]);
	const Chaos::FVec3 CV = CV1 - CV0;
	// K * DeltaLamda = -JV
	const Chaos::FReal DeltaLambda = -(CV.Dot(Joint.ConstraintAxis[ConstraintIndex]) - TargetVel) / Joint.ConstraintHardIM[ConstraintIndex];

	const Chaos::FVec3 MDV = DeltaLambda * Joint.ConstraintAxis[ConstraintIndex];

	if (!b0.bStatic)
	{
		const Chaos::FVec3 DV0 = b0.InvM * MDV;
		const Chaos::FVec3 DW0 = Joint.InvIs[0] * (Joint.ConstraintArms[0].Cross(MDV));;

		b0.V() -= DV0;
		b0.W() -= DW0;
	}
	if (!b1.bStatic)
	{
		const Chaos::FVec3 DV1 = b1.InvM * MDV;
		const Chaos::FVec3 DW1 = Joint.InvIs[1] * (Joint.ConstraintArms[1].Cross(MDV));;

		b1.V() += DV1;
		b1.W() += DW1;
	}
}

void ASoftJointConstraintTestActor::SolveVelocityConstraintsSoft(float Dt, int32 ConstraintIndex, FJointSlovePair& Joint)
{
	if(!bSolveVelocity) return;

	if(ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = Joint.Body(0);
		auto& b1 = Joint.Body(1);

		const auto& Axis = Joint.ConstraintAxis[ConstraintIndex];

		Chaos::FVec3 v0 = b0.V() + b0.W().Cross(Joint.ConstraintArms[0]);
		Chaos::FVec3 v1 = b1.V() + b1.W().Cross(Joint.ConstraintArms[1]);

		Chaos::FReal jv = Axis.Dot(v1 - v0);

		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		Chaos::FReal effective_mass = 1.0 / Joint.ConstraintSoftIM[ConstraintIndex];
		float lambda = -effective_mass * (jv + Joint.SpingPart.GetBias(Joint.ConstraintLambda[ConstraintIndex]));
		Joint.ConstraintLambda[ConstraintIndex] += lambda; // Store accumulated impulse

		// apply lambda
		const Chaos::FVec3 dp = lambda * Axis;

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0 = b0.InvM * dp;
			const Chaos::FVec3 dW0 = Joint.InvIs[0] * (Joint.ConstraintArms[0].Cross(dp));

			b0.V() -= dV0;
			b0.W() -= dW0;
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1 = b1.InvM * dp;
			const Chaos::FVec3 dW1 = Joint.InvIs[1] * (Joint.ConstraintArms[1].Cross(dp));

			b1.V() += dV1;
			b1.W() += dW1;
		}
	}
}
