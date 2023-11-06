#include "SoftJointConstraintTestActor.h"
#include "Components/BoxComponent.h"
#include "Chaos/Utilities.h"
#include "Chaos/Rotation.h"

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


void FJointSlovePair::InitPositionConstraintDatas(
	const int32 ConstraintIndex, 
	const Chaos::FVec3& _ConstraintAxis, 
	const Chaos::FReal& ConstraintDelta, 
	const Chaos::FReal Dt, 
	const Chaos::FReal ConstraintLimit, 
	const Chaos::FVec3& ConstraintArm0, 
	const Chaos::FVec3& ConstraintArm1)
{
	this->ConstraintAxis[ConstraintIndex] = _ConstraintAxis;
	this->ConstraintLimits[ConstraintIndex] = ConstraintLimit;
	this->ConstraintCX[ConstraintIndex] = ConstraintDelta;
	this->ConstraintArms[ConstraintIndex][0] = ConstraintArm0;
	this->ConstraintArms[ConstraintIndex][1] = ConstraintArm1;

	InitPositionEffectiveMass(ConstraintIndex, Dt);
}

void FJointSlovePair::InitPositionEffectiveMass(
	const int32 ConstraintIndex, 
	const Chaos::FReal Dt)
{
	const auto& ConstraintArm0 = ConstraintArms[ConstraintIndex][0];
	const auto& ConstraintArm1 = ConstraintArms[ConstraintIndex][1];
	const auto& _ConstraintAxis = this->ConstraintAxis[ConstraintIndex];

	//【金山文档】 Github PositionBasedDynamics源码分析 Section: Constraint mass matrix

	// vector are column major
	// II = (rxn)^T * InvI * (rxn)
	// K = J^T * InvM * J = m0^-1 + II0 + m1^-1 + II1
	// 1. calculate 'rxn'
	const Chaos::FVec3 AngularAxis0 = ConstraintArm0.Cross(_ConstraintAxis);
	const Chaos::FVec3 AngularAxis1 = ConstraintArm1.Cross(_ConstraintAxis);

	// 2. calculate 'InvI * (rxn)'
	// const Chaos::FVec3 IA0 = Joint.Body[0].InvI.TransformVector(AngularAxis0);
	const Chaos::FVec3 IA0 = Chaos::Utilities::Multiply(InvIs[0], AngularAxis0);
	// const Chaos::FVec3 IA1 = Joint.Body[1].InvI.TransformVector(AngularAxis1);
	const Chaos::FVec3 IA1 = Chaos::Utilities::Multiply(InvIs[1], AngularAxis1);
	// 3. calculate '(rxn)^T * InvI * (rxn)'
	const Chaos::FReal II0 = Chaos::FVec3::DotProduct(AngularAxis0, IA0);
	const Chaos::FReal II1 = Chaos::FVec3::DotProduct(AngularAxis1, IA1);

	// hard mass
	ConstraintHardIM[ConstraintIndex] = Body(0).InvM + II0 + Body(1).InvM + II1;

	// Soft mass
	float EffectiveMass = 0;
	auto K = ConstraintHardIM[ConstraintIndex];
	SpingPart[ConstraintIndex].CalculateSpringProperties(
		Dt,
		K,
		1.0f,
		ConstraintCX[ConstraintIndex],
		JointSettings.Frequency,
		JointSettings.DampingRatio,
		EffectiveMass);
	ConstraintSoftIM[ConstraintIndex] = 1.0 / EffectiveMass;
}

void FJointSlovePair::ApplyAxisPositionConstraint(float Dt, int32 ConstraintIndex)
{
	if (ConstraintIndex == INDEX_NONE)
	{
		return;
	}

	auto& b0 = Body(0);
	auto& b1 = Body(1);

	const Chaos::FVec3 CX = b1.DP() - b0.DP() + Chaos::FVec3::CrossProduct(b1.DQ(), ConstraintArms[ConstraintIndex][1]) - Chaos::FVec3::CrossProduct(b0.DQ(), ConstraintArms[ConstraintIndex][0]);

	Chaos::FReal DeltaPosition = ConstraintCX[ConstraintIndex] + Chaos::FVec3::DotProduct(CX, ConstraintAxis[ConstraintIndex]);

	bool NeedsSolve = false;
	if (DeltaPosition > ConstraintLimits[ConstraintIndex])
	{
		DeltaPosition -= ConstraintLimits[ConstraintIndex];
		NeedsSolve = true;
	}
	else if (DeltaPosition < -ConstraintLimits[ConstraintIndex])
	{
		DeltaPosition += ConstraintLimits[ConstraintIndex];
		NeedsSolve = true;
	}

	if (NeedsSolve /*&& FMath::Abs(DeltaPosition) > Joint.PositionTolerance*/)
	{
		if (JointSettings.bSoftPositionConstraint)
		{
			SolvePositionConstraintsSoft(Dt, ConstraintIndex, DeltaPosition);
		}
		else
		{
			SolvePositionConstraints(Dt, ConstraintIndex, DeltaPosition);
		}
	}
}

void FJointSlovePair::InitPlanarPositionConstraint(float Dt, const int32 AxisIndex)
{
	// 要用修正后的位置计算，P+DP
	const auto& bX0 = CurrentPs[0];
	const auto& cR0 = ConnectorRs[0];
	const auto& cX0 = ConnectorXs[0];

	const auto& bX1 = CurrentPs[1];
	const auto& cR1 = ConnectorRs[1];
	const auto& cX1 = ConnectorXs[1];

	// Calculate points relative to body, "Constraints Derivation for Rigid Body Simulation in 3D. equation 55" 
	// r0 + u = (cX0 - bX1) + (cX1 - cX0) = cX1 - bX0
	const Chaos::FVec3 ConstraintArm0 = ConnectorXs[1] - CurrentPs[0];
	// r1 = cX1 - bX1
	const Chaos::FVec3 ConstraintArm1 = ConnectorXs[1] - CurrentPs[1];

	const Chaos::FMatrix33 R0M = cR0.ToMatrix();
	Chaos::FVec3 Axis = R0M.GetAxis(AxisIndex);
	Chaos::FReal Delta = Chaos::FVec3::DotProduct(cX1 - cX0, Axis);

	InitPositionConstraintDatas(AxisIndex, Axis, Delta, Dt, JointSettings.LimitDistance[AxisIndex], ConstraintArm0, ConstraintArm1);
}


void FJointSlovePair::InitSphericalPositionConstraint(float Dt)
{
	const auto& bX0 = CurrentPs[0];
	const auto& cR0 = ConnectorRs[0];
	const auto& cX0 = ConnectorXs[0];

	const auto& bX1 = CurrentPs[1];
	const auto& cR1 = ConnectorRs[1];
	const auto& cX1 = ConnectorXs[1];

	Chaos::FVec3 SphereAxis0 = Chaos::FVec3::UnitX();
	Chaos::FReal SphereDelta0 = 0;

	const Chaos::FVec3 DX = cX1 - cX0;
	const Chaos::FReal DXLen = DX.Size();
	if (DXLen > UE_KINDA_SMALL_NUMBER)
	{
		SphereAxis0 = DX / DXLen;
		SphereDelta0 = DXLen;
	}

	const Chaos::FVec3 SphereAxis1 = (FMath::Abs(FMath::Abs(SphereAxis0.Dot(Chaos::FVec3(1, 0, 0)) - 1.0)) > UE_SMALL_NUMBER) ? SphereAxis0.Cross(Chaos::FVec3(1, 0, 0)) :
		(FMath::Abs(FMath::Abs(SphereAxis0.Dot(Chaos::FVec3(0, 1, 0)) - 1.0)) > UE_SMALL_NUMBER) ? SphereAxis0.Cross(Chaos::FVec3(0, 1, 0)) : SphereAxis0.Cross(Chaos::FVec3(0, 0, 1));
	const Chaos::FVec3 SphereAxis2 = SphereAxis0.Cross(SphereAxis1);

	// Using Connector1 for both conserves angular momentum and avoid having 
	// too much torque applied onto the COM. But can only be used for limited constraints
	const Chaos::FVec3 ConstraintArm0 = ConnectorXs[1] - CurrentPs[0];
	const Chaos::FVec3 ConstraintArm1 = ConnectorXs[1] - CurrentPs[1];

	InitPositionConstraintDatas(0, SphereAxis0, SphereDelta0, Dt, JointSettings.LimitDistance[0], ConstraintArm0, ConstraintArm1);
	InitPositionConstraintDatas(1, SphereAxis1, 0, Dt, JointSettings.LimitDistance[1], ConstraintArm0, ConstraintArm1);
	InitPositionConstraintDatas(2, SphereAxis2, 0, Dt, JointSettings.LimitDistance[2], ConstraintArm0, ConstraintArm1);
}

void FJointSlovePair::ApplyCorrections()
{
	auto ApplyCorrection = [&](int BodyIndex) {
		Body(BodyIndex).P() = P(BodyIndex);
		Body(BodyIndex).Q() = Q(BodyIndex);
		//Joint.Body(BodyIndex).DP() = Chaos::FVec3::ZeroVector;
		//Joint.Body(BodyIndex).DQ() = Chaos::FVec3::ZeroVector;
	};

	ApplyCorrection(0);
	ApplyCorrection(1);
}

void FJointSlovePair::SolvePositionConstraints(float Dt,
	int32 ConstraintIndex, const Chaos::FReal DeltaPosition)
{
	if (!JointSettings.bSolvePosition) return;

	if (ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = Body(0);
		auto& b1 = Body(1);

		auto C = DeltaPosition;
		auto effective_mass = 1.0 / ConstraintHardIM[ConstraintIndex];
		auto delta_lambda = -effective_mass * C;

		ConstraintLambda[ConstraintIndex] += delta_lambda;

		auto dp = delta_lambda * ConstraintAxis[ConstraintIndex];

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = InvIs[0] * (ConstraintArms[ConstraintIndex][0].Cross(dp));

			b0.DP() -= dV0dt;
			b0.DQ() -= dW0dt;

			//b0.P -= dV0dt;
			//b0.IntegrateRotation(-dW0dt);
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = InvIs[1] * (ConstraintArms[ConstraintIndex][1].Cross(dp));

			b1.DP() += dV1dt;
			b1.DQ() += dW1dt;

			//b1.P += dV1dt;
			//b1.IntegrateRotation(dW1dt);
		}
	}
}

void FJointSlovePair::SolvePositionConstraintsSoft(float Dt,
	int32 ConstraintIndex, const Chaos::FReal DeltaPosition)
{
	if (!JointSettings.bSolvePosition) return;

	if (ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = Body(0);
		auto& b1 = Body(1);

		const auto& Axis = ConstraintAxis[ConstraintIndex];

		Chaos::FReal total_lambda = ConstraintLambda[ConstraintIndex];
		Chaos::FReal k_hard = ConstraintHardIM[ConstraintIndex];
		Chaos::FReal effective_mass_hard = 1.0 / k_hard;

		Chaos::FReal omega = 2.0f * PI * JointSettings.Frequency;

		// Calculate spring constant k and drag constant c (page 45)
		Chaos::FReal k = effective_mass_hard * FMath::Square(omega);
		Chaos::FReal c = 2.0f * effective_mass_hard * JointSettings.DampingRatio * omega;

		// Ks = M_hard_eff * k * h^2
		// Kd = M_hard_eff * c * h
		Chaos::FReal Ks = effective_mass_hard * k * Dt * Dt;
		Chaos::FReal Kd = effective_mass_hard * c * Dt;

		Chaos::FVec3 v0 = b0.V() + b0.W().Cross(ConstraintArms[ConstraintIndex][0]);
		Chaos::FVec3 v1 = b1.V() + b1.W().Cross(ConstraintArms[ConstraintIndex][1]);

#if 1
		// Cdot*h = Jv*h - v_target*h 
		// v_target = 0
		Chaos::FReal Cdot_x_h = Axis.Dot(v1 - v0) * Dt;

		// DeltaPosition = C2 + Cdelta

		// (Khard*(Kd+Ks)+1)*delta_lambda = -(Ks*(C2 + Cdelta) + Kd*Cdot*h + Lambda1)
		Chaos::FReal delta_lambda = -(Ks * DeltaPosition + Kd * Cdot_x_h + total_lambda) / (k_hard * (Kd + Ks) + 1);

		ConstraintLambda[ConstraintIndex] += delta_lambda;

		// dp = J_T * delta_lambda
		Chaos::FVec3 dp = delta_lambda * Axis;

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = InvIs[0] * (ConstraintArms[ConstraintIndex][0].Cross(dp));

			b0.DP() -= dV0dt;
			b0.DQ() -= dW0dt;
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = InvIs[1] * (ConstraintArms[ConstraintIndex][1].Cross(dp));

			b1.DP() += dV1dt;
			b1.DQ() += dW1dt;
		}
#else
		Chaos::FReal Cdot_x_h = Axis.Dot(v0 - v1) * Dt;
		Chaos::FReal delta_lambda = (Ks * DeltaPosition - Kd * Cdot_x_h - total_lambda) / (k_hard * (Kd + Ks) + 1);
		ConstraintLambda[ConstraintIndex] += delta_lambda;

		// dp = J_T * delta_lambda
		Chaos::FVec3 dp = delta_lambda * Axis;

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0dt = b0.InvM * dp;
			const Chaos::FVec3 dW0dt = InvIs[0] * (ConstraintArms[ConstraintIndex][0].Cross(dp));

			b0.DP() += dV0dt;
			b0.DQ() += dW0dt;
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1dt = b1.InvM * dp;
			const Chaos::FVec3 dW1dt = InvIs[1] * (ConstraintArms[ConstraintIndex][1].Cross(dp));

			b1.DP() -= dV1dt;
			b1.DQ() -= dW1dt;
		}
#endif
	}
}

void FJointSlovePair::ApplyAxisVelocityConstraint(float Dt, int32 ConstraintIndex)
{
	if (!JointSettings.bSolveVelocity) return;

	if (!JointSettings.bSolvePosition || FMath::Abs(ConstraintLambda[ConstraintIndex]) > UE_SMALL_NUMBER)
	{
		Chaos::FReal TargetVel = 0.0f;
		/*if (ConstraintRestitution[ConstraintIndex] != 0.0f)
		{
			const FReal InitVel = InitConstraintAxisLinearVelocities[ConstraintIndex];
			TargetVel = InitVel > Chaos_Joint_LinearVelocityThresholdToApplyRestitution ?
				-PositionConstraints.ConstraintRestitution[ConstraintIndex] * InitVel : 0.0f;
		}*/

		if (JointSettings.bSoftVelocityConstraint)
		{
			SolveVelocityConstraintsSoft(Dt, ConstraintIndex);
		}
		else
		{
			SolveLinearVelocityConstraints(Dt, ConstraintIndex, TargetVel);
		}
	}
}

void FJointSlovePair::SolveLinearVelocityConstraints(float Dt,
	int32 ConstraintIndex, const Chaos::FReal TargetVel)
{
	auto& b0 = Body(0);
	auto& b1 = Body(1);

	const Chaos::FVec3 CV0 = b0.V() + b0.W().Cross(ConstraintArms[ConstraintIndex][0]);
	const Chaos::FVec3 CV1 = b1.V() + b1.W().Cross(ConstraintArms[ConstraintIndex][1]);
	const Chaos::FVec3 CV = CV1 - CV0;
	// K * DeltaLamda = -JV
	const Chaos::FReal DeltaLambda = -(CV.Dot(ConstraintAxis[ConstraintIndex]) - TargetVel) / ConstraintHardIM[ConstraintIndex];

	const Chaos::FVec3 MDV = DeltaLambda * ConstraintAxis[ConstraintIndex];

	if (!b0.bStatic)
	{
		const Chaos::FVec3 DV0 = b0.InvM * MDV;
		const Chaos::FVec3 DW0 = InvIs[0] * (ConstraintArms[ConstraintIndex][0].Cross(MDV));;

		b0.V() -= DV0;
		b0.W() -= DW0;
	}
	if (!b1.bStatic)
	{
		const Chaos::FVec3 DV1 = b1.InvM * MDV;
		const Chaos::FVec3 DW1 = InvIs[1] * (ConstraintArms[ConstraintIndex][1].Cross(MDV));;

		b1.V() += DV1;
		b1.W() += DW1;
	}
}

void FJointSlovePair::SolveVelocityConstraintsSoft(float Dt, int32 ConstraintIndex)
{
	if (!JointSettings.bSolveVelocity) return;

	if (ConstraintIndex != INDEX_NONE)
	{
		auto& b0 = Body(0);
		auto& b1 = Body(1);

		const auto& Axis = ConstraintAxis[ConstraintIndex];

		Chaos::FVec3 v0 = b0.V() + b0.W().Cross(ConstraintArms[ConstraintIndex][0]);
		Chaos::FVec3 v1 = b1.V() + b1.W().Cross(ConstraintArms[ConstraintIndex][1]);

		Chaos::FReal jv = Axis.Dot(v1 - v0);

		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		Chaos::FReal effective_mass = 1.0 / ConstraintSoftIM[ConstraintIndex];
		float lambda = -effective_mass * (jv + SpingPart[ConstraintIndex].GetBias(ConstraintLambda[ConstraintIndex]));
		ConstraintLambda[ConstraintIndex] += lambda; // Store accumulated impulse

		// apply lambda
		const Chaos::FVec3 dp = lambda * Axis;

		if (!b0.bStatic)
		{
			const Chaos::FVec3 dV0 = b0.InvM * dp;
			const Chaos::FVec3 dW0 = InvIs[0] * (ConstraintArms[ConstraintIndex][0].Cross(dp));

			b0.V() -= dV0;
			b0.W() -= dW0;
		}
		if (!b1.bStatic)
		{
			const Chaos::FVec3 dV1 = b1.InvM * dp;
			const Chaos::FVec3 dW1 = InvIs[1] * (ConstraintArms[ConstraintIndex][1].Cross(dp));

			b1.V() += dV1;
			b1.W() += dW1;
		}
	}
}


/* -----ASoftJointConstraintTestActor----- */

const Chaos::FVec3 Gravity{ 0, 0, -980 };

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

	Chaos::FVec3 cX = InJointWorldTrans.GetLocation();
	Chaos::FRotation3 cR = InJointWorldTrans.GetRotation();

	auto InitBodyInfo = [](FRigidSloverData& Body, class UPrimitiveComponent* shape) {
		auto bodyWorldTrans = shape->GetComponentTransform();

		Chaos::FVec3 bX = bodyWorldTrans.GetLocation();
		Chaos::FRotation3 bR = bodyWorldTrans.GetRotation();

		constexpr Chaos::FReal Density = 0.001; // kg/cm^3

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

		Body.P() = bX;
		Body.Q() = bR;

		Body.OldState = Body.State;
	};

	auto InitJointInfo = [&cX, &cR](
		FJointSlovePair& Joint,
		FRigidSloverData& Body,
		int32 BodyIndex) {

		Chaos::FVec3 bX = Body.P();
		Chaos::FRotation3 bR = Body.Q();

		// Can't use GetRelativeTransform directly here, because scaling will affect it!
		//Joint.LocalConnectorXs[bodyIndex] = InJointWorldTrans.GetRelativeTransform(BodyTransform);
		Joint.LocalConnectorXs[BodyIndex].SetLocation(bR.Inverse().RotateVector(cX - bX));
		Joint.LocalConnectorXs[BodyIndex].SetRotation(bR.Inverse() * cR);
	};

	auto& Joint = JointPairs.Emplace_GetRef();

	auto FindOrCreateSolverData = [&](int32 bodyIndex, class UPrimitiveComponent* InShape, bool bStatic)->FRigidSloverHandle {
		auto SolverDataHandle = RigidSloverDataContainer.FindSolverDataByShape(InShape);
		if(SolverDataHandle.IsValid())
		{
			// !! Don't forget to call the InitJointInfo function
			InitJointInfo(Joint, *SolverDataHandle, bodyIndex);
			return SolverDataHandle;
		}
		else
		{
			// Create new one
			auto NewSolverDataHandle = RigidSloverDataContainer.AllocNewSolverData();
			auto& SolverData = *NewSolverDataHandle;

			SolverData.bStatic = bStatic;
			SolverData.Shape = InShape;

			InitBodyInfo(SolverData, InShape);
			InitJointInfo(Joint, SolverData, bodyIndex);

			return NewSolverDataHandle;
		}
	};

	Joint.JointSettings = JointSettings;
	Joint.ShapeA = InShapeA;
	Joint.ShapeB = InShapeB;
	Joint.BodySolver[0] = FindOrCreateSolverData(0, InShapeA, bShapeAStatic);
	Joint.BodySolver[1] = FindOrCreateSolverData(1, InShapeB, false);
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
	/*int32 ConstraintIndex = JointSettings.GetConstraintIndex();
	if (ConstraintIndex == INDEX_NONE) return;*/

	++StepNum;

	// Integrate
	{
		int32 Index = 0;
		bool bReset = false;
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

			++Index;
			// Integrate Position
			if (!InSolverData.bStatic)
			{
				const auto& OldState = InSolverData.OldState;
				const auto& NewState = InSolverData.State;

				auto PosDiff = NewState.P - OldState.P;
				auto VelDiff = NewState.V - OldState.V;
				auto AnglDiff = NewState.W - OldState.W;

				if(PosDiff.Length() > 100.0 
					|| VelDiff.Length() > 1000.0f 
					|| NewState.DP.Length() > 2.0)
				{
					// 重新模拟
					if (bEnableReset) bReset = true;
				}

				/*UE_LOG(LogTemp, Log, TEXT("StepNum:%d Index:%d P:%s, Q:%s, V:%s, W:%s"), 
					StepNum, 
					Index, 
					*InSolverData.P().ToString(),
					*InSolverData.Q().ToString(),
					*InSolverData.V().ToString(),
					*InSolverData.W().ToString());*/

				IntegratePositionAndRotation(InSolverData);
			}
		});

		// for debug use
		RigidSloverDataContainer.ForEach([&](FRigidSloverData& InSolverData) {
			if (bReset)
			{
				InSolverData.State = InSolverData.OldState;
			}
			else
			{
				InSolverData.OldState = InSolverData.State;
			}

			InSolverData.DP() = Chaos::FVec3::Zero();
			InSolverData.DQ() = Chaos::FVec3::Zero();
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
			// 5.Update constraint arm for each body
			/*for(int32 j=0; j<3; ++j)
			{
				Joint.InitPlanarPositionConstraint(Dt, j);
			}*/
			Joint.InitSphericalPositionConstraint(Dt);

			// 6.Clear constraint lambda
			Joint.ConstraintLambda[0] = Joint.ConstraintLambda[1] = Joint.ConstraintLambda[2] = 0;

			// 7.Update Position and Rotation Tolerance
			const Chaos::FReal ToleranceScale = FMath::Min(1.f, 60.f * 60.f * Dt * Dt);
			Joint.PositionTolerance = ToleranceScale * Joint.JointSettings.PositionTolerance;
			Joint.AngleTolerance = ToleranceScale * Joint.JointSettings.AngleTolerance;
		}

		for (int32 Itr = 0; Itr < NumPositionIterations; ++Itr)
		{
			for (int32 i = 0; i < JointPairs.Num(); ++i)
			{
				auto& Joint = JointPairs[i];

				for (int32 j = 0; j < 3; ++j)
				{
					Joint.ApplyAxisPositionConstraint(Dt, j);
				}
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

		// !! Do not call the SetImplicitVelocity function repeatedly
		RigidSloverDataContainer.ForEach([&](FRigidSloverData& Body) {
			SetImplicitVelocity(Body, Dt);
		});
	}

	// Solve velocity
	{
		for (int32 Itr = 0; Itr < NumVelocityIterations; ++Itr)
		{
			for (int32 i = 0; i < JointPairs.Num(); ++i)
			{
				auto& Joint = JointPairs[i];

				for (int32 j = 0; j < 3; ++j)
				{
					Joint.ApplyAxisVelocityConstraint(Dt, j);
				}
			}

			// Check
			{
				for (int32 i = 0; i < JointPairs.Num(); ++i)
				{
					auto& Joint = JointPairs[i];

					auto& b0 = Joint.Body(0);
					auto& b1 = Joint.Body(1);

					for (int32 j = 0; j < 3; ++j)
					{
						if (FMath::Abs(Joint.ConstraintLambda[j]) > UE_SMALL_NUMBER)
						{
							const Chaos::FVec3 CV0 = b0.V() + b0.W().Cross(Joint.ConstraintArms[j][0]);
							const Chaos::FVec3 CV1 = b1.V() + b1.W().Cross(Joint.ConstraintArms[j][1]);
							const Chaos::FVec3 CV = CV1 - CV0;

							Chaos::FReal Cdot = Joint.ConstraintAxis[j].Dot(CV);
							if (!FMath::IsNearlyZero(Cdot))
							{
								int stop = 0;
							}
						}
					}
				}
			}
		}
	}

	// Update shape transform
	{
		for (int32 i = 0; i < JointPairs.Num(); ++i)
		{
			auto& Joint = JointPairs[i];

			Joint.ApplyCorrections();

			auto& b0 = Joint.Body(0);
			auto& b1 = Joint.Body(1);

			if(true)
			{
				auto SetShapeTransform = [](FRigidSloverData& b, UPrimitiveComponent* shape) {
					shape->SetWorldLocationAndRotation(b.P(), b.Q());
				};

				if (!b0.bStatic) SetShapeTransform(b0, Joint.ShapeA);
				if (!b1.bStatic) SetShapeTransform(b1, Joint.ShapeB);
			}
		}
	}
}

void ASoftJointConstraintTestActor::DebugDraw(float Dt)
{
	for (int32 i = 0; i < JointPairs.Num(); ++i)
	{
		auto& Joint = JointPairs[i];

		auto& b0 = Joint.Body(0);
		auto& b1 = Joint.Body(1);

		// Debug draw
		{
			auto World = GetWorld();
			auto DebugDrawBody = [&Joint, &World](int32 bodyIndex, FRigidSloverData& b, const FColor& c) {
				auto p0 = b.P();
				//auto p1 = b.P() + Joint.ConstraintArms[ConstraintIndex][bodyIndex];
				//DrawDebugLine(World, p0, p1, c, false, -1, 0, 1.0f);

				//DrawDebugBox(World, b.ConnectorX, FVector{2.0, 2.0f, 2.0f}, FQuat::Identity, c, false, -1, 0, 1.0f);

				auto p2 = p0 + b.V();
				DrawDebugLine(World, p0, p2, c, false, -1, 0, 3.0f);
			};

			FColor Color0 = FColor::Red;
			FColor Color1 = FColor::Green;

			DebugDrawBody(0, b0, Color0);
			DebugDrawBody(1, b1, Color1);

			DrawDebugLine(World, Joint.ConnectorXs[0], Joint.ConnectorXs[1], FColor::Blue, false, -1, 0, 1.0f);

			auto P = (Joint.ConnectorXs[0] + Joint.ConnectorXs[1]) * 0.5f;
			DrawDebugString(World, P, FString::Printf(TEXT("Dist:%s"), *Joint.ConstraintCX.ToString()), nullptr, FColor::Red, 0, false, 1.5f);
		}
	}

	auto World = GetWorld();
	RigidSloverDataContainer.ForEach([&](FRigidSloverData& InSolverData) {
		FVector Center = InSolverData.P();
		FQuat Roation = InSolverData.Q();
		FVector Extent{ 32, 32, 32 };

		//DrawDebugBox(World, Center, Extent, Roation, FColor::Red, false, -1, 0, 3.0f);
	});
}