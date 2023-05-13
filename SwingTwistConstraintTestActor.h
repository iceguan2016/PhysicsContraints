#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SwingTwistConstraintTestActor.generated.h"

UCLASS(Blueprintable, BlueprintType)
class FPSDESTRUCTION_API ASwingTwistConstraintTestActor : public AActor
{
	GENERATED_BODY()

public:
	
	UFUNCTION(BlueprintCallable)
	void InitSwingTwistLimit(const FTransform& InBody1, const FTransform& InBody2, const FTransform& InConnect);

	UFUNCTION(BlueprintCallable)
	void UpdateSwingTwistLimit(const FTransform& InBody1, const FTransform& InBody2);

	UFUNCTION(BlueprintCallable)
	void UpdateSwingPose(UPrimitiveComponent* InBody1, UPrimitiveComponent* InBody2, float InEllipseTheta);

	UPROPERTY(EditAnywhere)
	float SwingAngleYLimit = 120;

	UPROPERTY(EditAnywhere)
	float SwingAngleZLimit = 40;

	UPROPERTY(EditAnywhere)
	float SwingAngleRadius = 40;

	UPROPERTY(EditAnywhere)
	int DrawSegmentNum = 30;

	UPROPERTY(EditAnywhere)
	bool bDrawSwingPath = true;

	UPROPERTY(EditAnywhere)
	bool bDrawPoseEllipse = true;

	UPROPERTY(EditAnywhere)
	bool bDrawLimitEllipse = true;

private:
	FQuat mConstraintToBody1;
	FVector mLocalSpacePosition1;

	FQuat mConstraintToBody2;
	FVector mLocalSpacePosition2;

	FTransform mBody2InitTransform;
	FTransform mConnectInitTransform;
};