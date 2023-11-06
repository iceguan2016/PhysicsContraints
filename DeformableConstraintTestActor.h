#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DeformableConstraintTestActor.generated.h"

class FDeformableMesh 
{
public:
	struct FTetModel
	{
		TArray<FVector> verts;
		TArray<int32>	tet_indics;
		TArray<int32>	tet_edge_indics;
		TArray<int32>	tet_surf_tri_indics;
	};
	FDeformableMesh(const FTetModel& InTetModel)
		: tet_model(InTetModel)
	{

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
		const float InDt
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

		const auto grad0 = (1. / 6.) * (p1 - p2).Cross(p3 - p2);
		const auto grad1 = (1. / 6.) * (p2 - p0).Cross(p3 - p0);
		const auto grad2 = (1. / 6.) * (p0 - p1).Cross(p3 - p1);
		const auto grad3 = (1. / 6.) * (p1 - p0).Cross(p2 - p0);

		auto const weighted_sum_of_gradients = w0 * grad0.SizeSquared() + w1 * grad1.SizeSquared() +
			w2 * grad2.SizeSquared() + w3 * grad3.SizeSquared();

		if (weighted_sum_of_gradients < 1e-5)
			return OutPositions;

		const auto alpha_tilde = InDamping / (InDt * InDt);
		const auto delta_lamada = - C / (weighted_sum_of_gradients + alpha_tilde);

		OutPositions.Get<0>() += w0 * grad0 * delta_lamada;
		OutPositions.Get<1>() += w1 * grad1 * delta_lamada;
		OutPositions.Get<2>() += w2 * grad2 * delta_lamada;
		OutPositions.Get<3>() += w3 * grad3 * delta_lamada;

		return OutPositions;
	}

	using FEdgeContraintPositionData = TTuple<FVector, FVector>;
	using FEdgeContraintInvMassData	 = TTuple<float, float>;
	FEdgeContraintPositionData ProjectSingleDistanceConstraintDamping(
		const float InDamping,
		const float InInitLength,
		const FEdgeContraintPositionData& InPositions,
		const FEdgeContraintInvMassData& InInvMass,
		const float InDt
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
		const auto delta_lamada = - C / (weighted_sum_of_gradients + alpha_tilde);

		OutPositions.Get<0>() += w0 * n * delta_lamada;
		OutPositions.Get<1>() += w1 * -n * delta_lamada;

		return OutPositions;
	}

	void Solve(
		const double timestep,
		const int32 iterations,
		const int32 substeps,
		const float damping)
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
				pos[m] += vel[m];
			}

			// sequential gauss seidel type solve
			for (auto n = 0; n < num_iterations; ++n)
			{
				// volume constraints
				for (auto j = 0; j < num_volume_constraints; ++j)
				{
					auto index_tet = j * 4;

					auto id0 = tet_model.tet_indics[index_tet + 0];
					auto id1 = tet_model.tet_indics[index_tet + 1];
					auto id2 = tet_model.tet_indics[index_tet + 2];
					auto id3 = tet_model.tet_indics[index_tet + 3];

					FVolumeConstraintPositionData position_data(
						pos[id0], pos[id1], pos[id2], pos[id3]);
					FVolumeConstraintInvMassData invmass_data(
						pos_inv_mass[id0], pos_inv_mass[id1], pos_inv_mass[id2], pos_inv_mass[id3]);
					const float init_volume = init_tet_volumes[index_tet];

					auto out_positions = ProjectSingleVolumeConstraintDamping(damping, init_volume, position_data, invmass_data, dt);

					pos[id0] = out_positions.Get<0>();
					pos[id1] = out_positions.Get<1>();
					pos[id2] = out_positions.Get<2>();
					pos[id3] = out_positions.Get<3>();
				}

				// edge constraints
				for (auto j = 0; j < num_edge_constraints; ++j)
				{
					auto index_edge = j * 2;

					auto id0 = tet_model.tet_edge_indics[index_edge + 0];
					auto id1 = tet_model.tet_edge_indics[index_edge + 1];

					FEdgeContraintPositionData position_data(pos[id0], pos[id1]);
					FEdgeContraintInvMassData invmass_data(pos_inv_mass[id0], pos_inv_mass[id1]);

					const auto init_edge_length = init_edge_lengths[index_edge];

					auto out_positions = ProjectSingleDistanceConstraintDamping(damping, init_edge_length, position_data, invmass_data, dt);

					pos[id0] = out_positions.Get<0>();
					pos[id1] = out_positions.Get<1>();
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

	bool InitPhysics()
	{
		int32 vert_num = tet_model.verts.Num();
		int32 tet_num = tet_model.tet_indics.Num() / 4;
		int32 edge_num = tet_model.tet_edge_indics.Num() / 2;
		int32 tri_num = tet_model.tet_surf_tri_indics.Num() / 2;

		init_tet_volumes.SetNum(tet_num);
		init_edge_lengths.SetNum(edge_num);
		pos_inv_mass.SetNum(vert_num);

		pos = tet_model.verts;
		prev_pos = tet_model.verts;

		vel.SetNumZeroed(vert_num);


		for (int32 i = 0; i < tet_num; i++) 
		{
			int32 tet_index = i * 4;
			FVolumeConstraintPositionData tet_positions(
				tet_model.verts[tet_index + 0],
				tet_model.verts[tet_index + 1],
				tet_model.verts[tet_index + 2],
				tet_model.verts[tet_index + 3]
			);

			float Vol = GetTetVolume(tet_positions);
			init_tet_volumes[i] = Vol;
			float inv_mass = Vol > 0.0f? 1.0f / (Vol / 4.0f) : 0.0f;
			pos_inv_mass[tet_index + 0] += inv_mass;
			pos_inv_mass[tet_index + 1] += inv_mass;
			pos_inv_mass[tet_index + 2] += inv_mass;
			pos_inv_mass[tet_index + 3] += inv_mass;
		}

		for (int32 i = 0; i < edge_num; i++) 
		{
			int32 edge_index = i * 2;
			int32 id0 = tet_model.tet_edge_indics[edge_index + 0];
			int32 id1 = tet_model.tet_edge_indics[edge_index + 1];
			init_edge_lengths[edge_index] = FVector::Distance(tet_model.verts[id0], tet_model.verts[id1]);
		}
	}

protected:
	FTetModel tet_model;
	
	// initialize data
	TArray<float> init_tet_volumes;
	TArray<float> init_edge_lengths;

	// dynamic
	TArray<float> pos_inv_mass;
	TArray<FVector> pos;
	TArray<FVector> prev_pos;
	TArray<FVector> vel;
};

UCLASS(Blueprintable, BlueprintType)
class ADeformableConstraintTestActor : public AActor
{
	GENERATED_BODY()

public:
};