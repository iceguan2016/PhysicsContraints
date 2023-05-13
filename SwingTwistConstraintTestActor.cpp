#include "SwingTwistConstraintTestActor.h"

void ASwingTwistConstraintTestActor::InitSwingTwistLimit(const FTransform& InBody1, const FTransform& InBody2, const FTransform& InConnect)
{
	FVector TwistAxis1, TwistAxis2;
	FVector PlaneAxis1, PlaneAxis2;

	TwistAxis1 = TwistAxis2 = InConnect.TransformVector(FVector::UnitX());
	PlaneAxis1 = PlaneAxis2 = InConnect.TransformVector(FVector::UnitY());

	FVector normal_axis1 = PlaneAxis1.Cross(TwistAxis1);
	FMatrix c_to_b1(TwistAxis1, normal_axis1, PlaneAxis1, FVector::ZeroVector);
	mConstraintToBody1 = c_to_b1.Rotator().Quaternion();
	mLocalSpacePosition1 = InBody1.GetLocation();

	mLocalSpacePosition1 = InBody1.InverseTransformPosition(InConnect.GetLocation());
	mConstraintToBody1 = InBody1.GetRotation().Inverse() * mConstraintToBody1;

	// Calculate rotation needed to go from constraint space to body2 local space
	FVector normal_axis2 = PlaneAxis2.Cross(TwistAxis2);
	FMatrix c_to_b2(TwistAxis2, normal_axis2, PlaneAxis2, FVector::ZeroVector);
	mConstraintToBody2 = c_to_b2.Rotator().Quaternion();
	mLocalSpacePosition2 = InBody2.GetLocation();

	mLocalSpacePosition2 = InBody2.InverseTransformPosition(InConnect.GetLocation());
	mConstraintToBody2 = InBody2.GetRotation().Inverse() * mConstraintToBody2;

	mConnectInitTransform = InConnect;
	mBody2InitTransform = InBody2;
}

void ASwingTwistConstraintTestActor::UpdateSwingTwistLimit(const FTransform& InBody1, const FTransform& InBody2)
{
	auto World = GetWorld();
	if (!World) return;

	auto DrawTransform = [&](const FVector& InLocation, const FQuat& InRotation, const float InLength) {
		auto X = InRotation.RotateVector(FVector::UnitX());
		auto Y = InRotation.RotateVector(FVector::UnitY());
		auto Z = InRotation.RotateVector(FVector::UnitZ());

		DrawDebugLine(World, InLocation, InLocation + X * InLength, FColor::Red);
		DrawDebugLine(World, InLocation, InLocation + Y * InLength, FColor::Green);
		DrawDebugLine(World, InLocation, InLocation + Z * InLength, FColor::Blue);
	};

	auto ToSwingAndTwist = [](const FQuat& InQuat, FQuat& OutTwist, FQuat& OutSwing) {
		float x = InQuat.X, y = InQuat.Y, z = InQuat.Z, w = InQuat.W;
		float s = sqrt(FMath::Square(w) + FMath::Square(x));
		if (s != 0.0f)
		{
			OutTwist = FQuat(x / s, 0, 0, w / s);
			OutSwing = FQuat(0, (w * y - x * z) / s, (w * z + x * y) / s, s);
		}
		else
		{
			// If both x and w are zero, this must be a 180 degree rotation around either y or z
			OutTwist = FQuat::Identity;
			OutSwing = InQuat;
		}
	};

	auto DrawEllipseOnSphere = [&](
		const FVector& InCenter, 
		const FVector& InNormal, 
		const FVector& InX, 
		const FVector& InY, 
		float InRadiansX, 
		float InRadiansY, 
		float InRadius, 
		const FColor& InColor) {
			int n = DrawSegmentNum;
			float theta = 0;
			float dTheta = 2.0f * PI / n;

			FVector prevV;
			FVector prevV2;
			FVector prevV3;

			for (int i=0; i<n+1; ++i) {
				FVector rotVec = FVector::ZeroVector;
				FQuat rotQ = FQuat::Identity;

				float rx = FMath::Cos(theta) * InRadiansX;
				float ry = FMath::Sin(theta) * InRadiansY;

				float halfRotAng = FMath::Sqrt(rx * rx + ry * ry);
				float rotSin = FMath::Sin(halfRotAng * 0.5);
				float rotCos = FMath::Cos(halfRotAng * 0.5);
				rotVec += (InX * rx)+(InY * ry);
				rotVec *= (1 / halfRotAng * rotSin);
				rotQ = FQuat(rotVec.X, rotVec.Y, rotVec.Z, rotCos);

				FVector v = InNormal * InRadius;
				v = InCenter + rotQ.RotateVector(v);

				FVector v2 = InCenter + rotVec * InRadius;

				FVector local_v3 = mConnectInitTransform.InverseTransformPosition(v);
				FVector v3 = mConnectInitTransform.TransformPosition(FVector{ 0, local_v3.Y, local_v3.Z });
			
				if(bDrawSwingPath)
				{
					DrawDebugPoint(World, v, 5.0f, FColor::Red);
				}
				if(bDrawLimitEllipse)
				{
					DrawDebugPoint(World, v2, 5.0f, FColor::Magenta);
				}
				if (bDrawPoseEllipse)
				{
					DrawDebugPoint(World, v3, 5.0f, FColor::Orange);
				}

				if (i >= 1) {
					// »æÖÆSwing°Ú¶¯Ô¼Êø¹ì¼£
					if(bDrawSwingPath)
					{
						DrawDebugLine(World, prevV, v, InColor);
					}
					// »æÖÆSwingÖáµÄÍÖÔ²
					if (bDrawLimitEllipse)
					{
						DrawDebugLine(World, prevV2, v2, FColor::Cyan);
					}
					// »æÖÆ°Ú¶¯×ËÌ¬µÄÍÖÔ²
					if (bDrawPoseEllipse)
					{
						DrawDebugLine(World, prevV3, v3, FColor::Orange);
					}
				}

				prevV = v;
				prevV2 = v2;
				prevV3 = v3;
				theta += dTheta;
			}

	};

	FQuat constraint_body1_to_world = InBody1.GetRotation() * mConstraintToBody1;
	FQuat constraint_body2_to_world = InBody2.GetRotation() * mConstraintToBody2;
	FQuat q = constraint_body1_to_world.Inverse() * constraint_body2_to_world;

	FQuat q_swing, q_twist;
	ToSwingAndTwist(q, q_twist, q_swing);

	FVector q_swing_axis;
	float q_swing_angle;
	q_swing.ToAxisAndAngle(q_swing_axis, q_swing_angle);

	{
		DrawTransform(InBody1.GetLocation(), InBody1.GetRotation(), 100.0f);
		DrawTransform(InBody2.GetLocation(), InBody2.GetRotation(), 100.0f);

		DrawTransform(InBody1.TransformPosition(mLocalSpacePosition1), constraint_body1_to_world, 30.0f);
		DrawTransform(InBody2.TransformPosition(mLocalSpacePosition2), constraint_body2_to_world, 10.0f);

		DrawEllipseOnSphere(
			InBody1.TransformPosition(mLocalSpacePosition1), 
			constraint_body1_to_world.RotateVector(FVector::UnitX()),
			constraint_body1_to_world.RotateVector(FVector::UnitY()),
			constraint_body1_to_world.RotateVector(FVector::UnitZ()),
			FMath::DegreesToRadians(SwingAngleYLimit),
			FMath::DegreesToRadians(SwingAngleZLimit),
			SwingAngleRadius,
			FColor::Yellow
		);
	}
}

void ASwingTwistConstraintTestActor::UpdateSwingPose(UPrimitiveComponent* InBody1, UPrimitiveComponent* InBody2, float InEllipseTheta)
{
	if(!InBody1 || !InBody2) return;

	auto World = GetWorld();
	if (!World) return;

	auto Body1Trans = InBody1->GetComponentToWorld();
	auto Body2Trans = InBody2->GetComponentToWorld();

	float RadiansY = FMath::DegreesToRadians(SwingAngleYLimit);
	float RadiansZ = FMath::DegreesToRadians(SwingAngleZLimit);

	FQuat constraint_body1_to_world = Body1Trans.GetRotation() * mConstraintToBody1;
	FVector X = constraint_body1_to_world.RotateVector(FVector::UnitX());
	FVector Y = constraint_body1_to_world.RotateVector(FVector::UnitY());
	FVector Z = constraint_body1_to_world.RotateVector(FVector::UnitZ());

	FVector rotVec = FVector::ZeroVector;
	FQuat rotQ = FQuat::Identity;

	float ry = FMath::Cos(InEllipseTheta) * RadiansY;
	float rz = FMath::Sin(InEllipseTheta) * RadiansZ;

	float halfRotAng = FMath::Sqrt(ry * ry + rz * rz);
	float halfRotAngDegree = FMath::RadiansToDegrees(halfRotAng);
	float rotSin = FMath::Sin(halfRotAng * 0.5);
	float rotCos = FMath::Cos(halfRotAng * 0.5);
	rotVec += (Y * ry) + (Z * rz);
	rotVec *= (1 / halfRotAng * rotSin);
	rotQ = FQuat(rotVec.X, rotVec.Y, rotVec.Z, rotCos);

	FVector ConstraintWorldPos = mBody2InitTransform.TransformPosition(mLocalSpacePosition2);
	FVector WorldSpacePosition2 = mBody2InitTransform.TransformVector(-mLocalSpacePosition2);

	FTransform NewBody2Trans;
	NewBody2Trans.SetLocation(ConstraintWorldPos + rotQ.RotateVector(WorldSpacePosition2));
	//NewBody2Trans.SetLocation(mBody2InitTransform.GetLocation());
	NewBody2Trans.SetRotation(rotQ * mBody2InitTransform.GetRotation());
	//NewBody2Trans.SetRotation(mBody2InitTransform.GetRotation());
	NewBody2Trans.SetScale3D(mBody2InitTransform.GetScale3D());

	InBody2->SetWorldTransform(NewBody2Trans);

	FVector Center = Body1Trans.TransformPosition(mLocalSpacePosition1);

	// »æÖÆtwistÖá
	FVector S = NewBody2Trans.GetLocation();
	FVector E = S + (ConstraintWorldPos - S).GetSafeNormal() * 500.0f;
	DrawDebugLine(World, S, E, FColor::Yellow);

	// »æÖÆÐý×ªÖá
	S = Center;
	E = S + rotVec * SwingAngleRadius;
	DrawDebugLine(World, S, E, FColor::Cyan);

	// »æÖÆ×ËÌ¬Î»
	FVector v = X * SwingAngleRadius;
	v = Center + rotQ.RotateVector(v);

	FVector local_v3 = mConnectInitTransform.InverseTransformPosition(v);
	FVector v3 = mConnectInitTransform.TransformPosition(FVector{ 0, local_v3.Y, local_v3.Z });

	S = Center;
	E = v3;
	DrawDebugLine(World, S, E, FColor::Orange);
}

