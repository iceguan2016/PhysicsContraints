#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DeformableConstraintTestActor.generated.h"

class FDeformableMesh 
{
public:
	struct FTetModel
	{
		TArray<float>	verts;
		TArray<int32>	tet_indics;
		TArray<int32>	tet_edge_indics;
		TArray<int32>	tet_surf_tri_indics;
	};
	FDeformableMesh(FTetModel& InTetModel, const FTransform& InWorldTransform, const FBox& InSimulateBounds)
		: tet_model(MoveTemp(InTetModel)), simulate_bounds(InSimulateBounds)
	{
		InitPhysics(InWorldTransform);
	}

	const FTetModel& GetTetModel() const { return tet_model; }

	// Volume constraint solve result, (v0, v1, v2, v3) new position
	using FVolumeConstraintPositionData = TTuple<FVector, FVector, FVector, FVector>;
	using FVolumeConstraintInvMassData	= TTuple<float, float, float, float>;
	FVolumeConstraintPositionData ProjectSingleVolumeConstraintDamping(
		const float InDamping,
		const float InInitVolume,
		const FVolumeConstraintPositionData& InPositions,
		const FVolumeConstraintInvMassData& InInvMass,
		const float InDt,
		float &InLamada
	)
	{
		FVolumeConstraintPositionData OutPositions = InPositions;

		auto p0 = InPositions.Get<0>();
		auto p1 = InPositions.Get<1>();
		auto p2 = InPositions.Get<2>();
		auto p3 = InPositions.Get<3>();

		auto w0 = InInvMass.Get<0>();
		auto w1 = InInvMass.Get<1>();
		auto w2 = InInvMass.Get<2>();
		auto w3 = InInvMass.Get<3>();

		float Vol = GetTetVolume(InPositions);
		float C = Vol - InInitVolume;

		int32 volIdOrder[][3] =
		{ 
			{1, 3, 2}, 
			{0, 2, 3}, 
			{0, 3, 1}, 
			{0, 1, 2}
		};

		/*const auto grad0 = (1. / 6.) * (p1 - p2).Cross(p3 - p2);
		const auto grad1 = (1. / 6.) * (p2 - p0).Cross(p3 - p0);
		const auto grad2 = (1. / 6.) * (p0 - p1).Cross(p3 - p1);
		const auto grad3 = (1. / 6.) * (p1 - p0).Cross(p2 - p0);*/

		const auto grad0 = (1. / 6.) * (p3 - p1).Cross(p2 - p1);
		const auto grad1 = (1. / 6.) * (p2 - p0).Cross(p3 - p0);
		const auto grad2 = (1. / 6.) * (p3 - p0).Cross(p1 - p0);
		const auto grad3 = (1. / 6.) * (p1 - p0).Cross(p2 - p0);

		auto const weighted_sum_of_gradients = w0 * grad0.SizeSquared() + w1 * grad1.SizeSquared() +
			w2 * grad2.SizeSquared() + w3 * grad3.SizeSquared();

		if (weighted_sum_of_gradients < 1e-5)
			return OutPositions;

		const auto alpha_tilde = InDamping / (InDt * InDt);
		const auto delta_lamada = - (C + alpha_tilde * InLamada) / (weighted_sum_of_gradients + alpha_tilde);

		InLamada += delta_lamada;

		OutPositions.Get<0>() += w0 * grad0 * delta_lamada;
		OutPositions.Get<1>() += w1 * grad1 * delta_lamada;
		OutPositions.Get<2>() += w2 * grad2 * delta_lamada;
		OutPositions.Get<3>() += w3 * grad3 * delta_lamada;

		float Vol2 = GetTetVolume(OutPositions);
		float C2 = Vol2 - InInitVolume;
		if (FMath::Abs(C2) > FMath::Abs(C))
		{
			int stop = 0;
		}

		IsInSimulateBounds(OutPositions.Get<0>());
		IsInSimulateBounds(OutPositions.Get<1>());
		IsInSimulateBounds(OutPositions.Get<2>());
		IsInSimulateBounds(OutPositions.Get<3>());

		return OutPositions;
	}

	using FEdgeContraintPositionData = TTuple<FVector, FVector>;
	using FEdgeContraintInvMassData	 = TTuple<float, float>;
	FEdgeContraintPositionData ProjectSingleDistanceConstraintDamping(
		const float InDamping,
		const float InInitLength,
		const FEdgeContraintPositionData& InPositions,
		const FEdgeContraintInvMassData& InInvMass,
		const float InDt,
		float &InLamada
	)
	{
		FEdgeContraintPositionData OutPositions = InPositions;

		auto p0 = InPositions.Get<0>();
		auto p1 = InPositions.Get<1>();

		auto w0 = InInvMass.Get<0>();
		auto w1 = InInvMass.Get<1>();

		auto const n = (p0 - p1).GetSafeNormal();

		float Distance = FVector::Distance(p0, p1);
		float C = Distance - InInitLength;
		
		const auto weighted_sum_of_gradients = w0 + w1;
		const auto alpha_tilde = InDamping / (InDt * InDt);
		const auto delta_lamada = - (C + alpha_tilde * InLamada) / (weighted_sum_of_gradients + alpha_tilde);

		InLamada += delta_lamada;
		OutPositions.Get<0>() += w0 * n * delta_lamada;
		OutPositions.Get<1>() += w1 * -n * delta_lamada;

		float Distance2 = FVector::Distance(OutPositions.Get<0>(), OutPositions.Get<1>());
		float C2 = Distance2 - InInitLength;
		if (FMath::Abs(C2) > FMath::Abs(C))
		{
			int stop = 0;
		}

		IsInSimulateBounds(OutPositions.Get<0>());
		IsInSimulateBounds(OutPositions.Get<1>());

		return OutPositions;
	}

	void Solve(
		const double timestep,
		const int32 iterations,
		const int32 substeps,
		const float damping,
		const bool is_volume_constraint = true,
		const bool is_edge_constraint = true)
	{
		const auto gravity = FVector(0.0f, 0.0f, -980.0f);

		auto const num_iterations = iterations / substeps;
		double dt = timestep / static_cast<double>(substeps);

		int32 num_vertices = pos.Num();
		int32 num_volume_constraints = tet_model.tet_indics.Num() / 4;
		int32 num_edge_constraints = tet_model.tet_edge_indics.Num() / 2;

		for (auto s = 0; s < substeps; ++s)
		{
			for (auto m = 0; m < num_vertices; ++m)
			{
				vel[m] += gravity * dt;
				pos[m] += vel[m] * dt;

				// simple handle collision
				if (pos[m].Z < 0.0)
				{
					pos[m] = prev_pos[m];
					pos[m].Z = 0;
				}
			}

			for (auto m = 0; m < volume_lamada.Num(); ++m) volume_lamada[m] = 0;
			for (auto m = 0; m < edge_lamada.Num(); ++m) edge_lamada[m] = 0;

			// sequential gauss seidel type solve
			for (auto n = 0; n < num_iterations; ++n)
			{
				// edge constraints
				if (is_edge_constraint)
				{
					for (auto j = 0; j < num_edge_constraints; ++j)
					{
						auto edge_vert_offset = j * 2;

						auto id0 = tet_model.tet_edge_indics[edge_vert_offset + 0];
						auto id1 = tet_model.tet_edge_indics[edge_vert_offset + 1];

						FEdgeContraintPositionData position_data(pos[id0], pos[id1]);
						FEdgeContraintInvMassData invmass_data(pos_inv_mass[id0], pos_inv_mass[id1]);

						const auto init_edge_length = init_edge_lengths[j];

						auto out_positions = ProjectSingleDistanceConstraintDamping(
							damping, init_edge_length, position_data, invmass_data, dt, edge_lamada[j]);

						pos[id0] = out_positions.Get<0>();
						pos[id1] = out_positions.Get<1>();
					}
				}

				// volume constraints
				if (is_volume_constraint)
				{
					for (auto j = 0; j < num_volume_constraints; ++j)
					{
						auto tet_vert_offset = j * 4;

						auto id0 = tet_model.tet_indics[tet_vert_offset + 0];
						auto id1 = tet_model.tet_indics[tet_vert_offset + 1];
						auto id2 = tet_model.tet_indics[tet_vert_offset + 2];
						auto id3 = tet_model.tet_indics[tet_vert_offset + 3];

						FVolumeConstraintPositionData position_data(
							pos[id0], pos[id1], pos[id2], pos[id3]);
						FVolumeConstraintInvMassData invmass_data(
							pos_inv_mass[id0], pos_inv_mass[id1], pos_inv_mass[id2], pos_inv_mass[id3]);
						const float init_volume = init_tet_volumes[j];

						auto out_positions = ProjectSingleVolumeConstraintDamping(
							damping, init_volume, position_data, invmass_data, dt, volume_lamada[j]);

						pos[id0] = out_positions.Get<0>();
						pos[id1] = out_positions.Get<1>();
						pos[id2] = out_positions.Get<2>();
						pos[id3] = out_positions.Get<3>();
					}
				}
			}

			// update solution
			for (auto i = 0; i < num_vertices; ++i)
			{
				vel[i] = (pos[i] - prev_pos[i]) / dt;
				prev_pos[i] = pos[i];
			}
		}
	}

	void Render(UWorld* InWorld)
	{
		if (!InWorld) return;

		int32 num_surf_tri = tet_model.tet_surf_tri_indics.Num() / 3;

		DrawDebugMesh(InWorld, pos, tet_model.tet_surf_tri_indics, FColor(0x99, 0xff, 0xcc), false, -1, 0);

		for (auto i = 0; i < num_surf_tri; ++i)
		{
			auto tri_vert_offset = i * 3;
			auto id0 = tet_model.tet_surf_tri_indics[tri_vert_offset + 0];
			auto id1 = tet_model.tet_surf_tri_indics[tri_vert_offset + 1];
			auto id2 = tet_model.tet_surf_tri_indics[tri_vert_offset + 2];

			auto &p0 = pos[id0];
			auto &p1 = pos[id1];
			auto & p2 = pos[id2];

			DrawDebugLine(InWorld, p0, p1, FColor::Black, false, -1, 0, 1.0f);
			DrawDebugLine(InWorld, p1, p2, FColor::Black, false, -1, 0, 1.0f);
			DrawDebugLine(InWorld, p2, p0, FColor::Black, false, -1, 0, 1.0f);
		}

		auto Center = simulate_bounds.GetCenter();
		auto HalfExtent = simulate_bounds.GetExtent();
		DrawDebugBox(InWorld, Center, HalfExtent, FQuat::Identity, FColor::Green, false, -1, 0, 2.0f);
	}

protected:
	float GetTetVolume(const FVolumeConstraintPositionData& InPositions)
	{
		auto p0 = InPositions.Get<0>();
		auto p1 = InPositions.Get<1>();
		auto p2 = InPositions.Get<2>();
		auto p3 = InPositions.Get<3>();

		auto const vol = (1. / 6.) * (p1 - p0).Cross(p2 - p0).Dot(p3 - p0);
		return FMath::Abs(vol);
	}

	bool InitPhysics(const FTransform& InWorldTransform)
	{
		constexpr Chaos::FReal Density = 0.001; // kg/cm^3

		int32 vert_num = tet_model.verts.Num() / 3;
		int32 tet_num = tet_model.tet_indics.Num() / 4;
		int32 edge_num = tet_model.tet_edge_indics.Num() / 2;
		int32 tri_num = tet_model.tet_surf_tri_indics.Num() / 3;

		init_tet_volumes.SetNum(tet_num);
		init_edge_lengths.SetNum(edge_num);
		pos_inv_mass.SetNum(vert_num);

		pos.Empty(vert_num);
		prev_pos.Empty(vert_num);
		for (auto i = 0; i < vert_num; ++i)
		{
			FVector local_pos(tet_model.verts[i*3 + 0], tet_model.verts[i * 3 + 1], tet_model.verts[i * 3 + 2]);
			// convert cm to m
			local_pos *= 100.0;
			// convert to world position
			FVector world_pos = InWorldTransform.TransformPosition(local_pos);
			pos.Add(world_pos);
			prev_pos.Add(world_pos);
		}

		vel.SetNumZeroed(vert_num);

		volume_lamada.SetNumZeroed(tet_num);
		edge_lamada.SetNumZeroed(edge_num);


		for (int32 i = 0; i < tet_num; i++) 
		{
			int32 tet_vert_offset = i * 4;
			int32 id0 = tet_model.tet_indics[tet_vert_offset + 0];
			int32 id1 = tet_model.tet_indics[tet_vert_offset + 1];
			int32 id2 = tet_model.tet_indics[tet_vert_offset + 2];
			int32 id3 = tet_model.tet_indics[tet_vert_offset + 3];

			FVolumeConstraintPositionData tet_positions(
				pos[id0],
				pos[id1],
				pos[id2],
				pos[id3]
			);

			float Vol = GetTetVolume(tet_positions);
			init_tet_volumes[i] = Vol;
			float inv_mass = Vol > 0.0f? 1.0f / ((Vol* Density) / 4.0f) : 0.0f;
			pos_inv_mass[id0] += inv_mass;
			pos_inv_mass[id1] += inv_mass;
			pos_inv_mass[id2] += inv_mass;
			pos_inv_mass[id3] += inv_mass;
		}

		for (int32 i = 0; i < edge_num; i++) 
		{
			int32 edge_vert_offset = i * 2;
			int32 id0 = tet_model.tet_edge_indics[edge_vert_offset + 0];
			int32 id1 = tet_model.tet_edge_indics[edge_vert_offset + 1];

			init_edge_lengths[i] = FVector::Distance(pos[id0], pos[id1]);
		}

		return true;
	}

	bool IsInSimulateBounds(const FVector& p) const
	{
		auto center = simulate_bounds.GetCenter();
		auto half_extent = simulate_bounds.GetExtent();

		auto half_p = p - center;
		bool is_in_bounds = FMath::Abs(half_p.X) < half_extent.X &&
							FMath::Abs(half_p.Y) < half_extent.Y &&
							FMath::Abs(half_p.Z) < half_extent.Z;
		if(!is_in_bounds)
		{
			int stop = 0;
		}

		return is_in_bounds;
	}

protected:
	FTetModel tet_model;
	
	// initialize data
	TArray<float> init_tet_volumes;
	TArray<float> init_edge_lengths;

	// dynamic
	TArray<float>	pos_inv_mass;
	TArray<FVector> pos;
	TArray<FVector> prev_pos;
	TArray<FVector> vel;
	TArray<float>	volume_lamada;
	TArray<float>	edge_lamada;

	// for debug
	FBox simulate_bounds;
};

UCLASS(Blueprintable, BlueprintType)
class ADeformableConstraintTestActor : public AActor
{
	GENERATED_BODY()

public:
	ADeformableConstraintTestActor();

	using FDeformableMeshPtr = TSharedPtr<FDeformableMesh>;
	FDeformableMeshPtr CreateDeformableMesh();

	virtual void BeginPlay() override;

	UFUNCTION(BlueprintCallable)
	void Simulate(float DeltaSeconds);

	UFUNCTION(BlueprintCallable)
	void Render();

	UPROPERTY(EditAnywhere)
	FTransform TetWorldTransform;

	UPROPERTY(EditAnywhere)
	FVector	SimulateBoxExtent { 100, 100, 100};

	UPROPERTY(EditAnywhere)
	bool bEnableSolve = true;

	UPROPERTY(EditAnywhere)
	bool bEnableVolumeConstraintSolve = true;

	UPROPERTY(EditAnywhere)
	bool bEnableEdgeConstraintSolve = true;

	UPROPERTY(EditAnywhere)
	float SolveDumping = 0.0f;

protected:
	FDeformableMeshPtr DeformableMesh;
};