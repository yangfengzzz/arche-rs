mod components;
mod entity;

pub use components::*;
pub use entity::*;

use bevy::{core::FixedTimestep, prelude::*};

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

#[derive(Debug)]
pub struct Gravity(pub Vec2);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec2::new(0., -9.81))
    }
}

pub const DELTA_TIME: f32 = 1. / 60.;

pub fn startup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let sphere = meshes.add(Mesh::from(shape::Icosphere {
        radius: 0.5,
        subdivisions: 4,
    }));

    let white = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        ..Default::default()
    });

    // Left particle
    commands
        .spawn_bundle(PbrBundle {
            mesh: sphere.clone(),
            material: white.clone(),
            ..Default::default()
        })
        .insert_bundle(ParticleBundle::new_with_pos_and_vel(
            Vec2::new(-2., 0.),
            Vec2::new(2., 0.),
        ));

    // Right particle
    commands
        .spawn_bundle(PbrBundle {
            mesh: sphere.clone(),
            material: white.clone(),
            ..Default::default()
        })
        .insert_bundle(ParticleBundle::new_with_pos_and_vel(
            Vec2::new(2., 0.),
            Vec2::new(-2., 0.),
        ));

    commands.spawn_bundle(OrthographicCameraBundle {
        transform: Transform::from_translation(Vec3::new(0., 0., 100.)),
        orthographic_projection: OrthographicProjection {
            scale: 0.01,
            ..Default::default()
        },
        ..OrthographicCameraBundle::new_3d()
    });
}

//--------------------------------------------------------------------------------------------------
#[derive(SystemLabel, Debug, Hash, PartialEq, Eq, Clone)]
enum Step {
    CollectCollisionPairs,
    Integrate,
    SolvePositions,
    UpdateVelocities,
    SolveVelocities,
}

fn collect_collision_pairs() {}

fn integrate(mut query: Query<(&mut Pos, &mut PrevPos, &mut Vel, &Mass)>, gravity: Res<Gravity>) {
    for (mut pos, mut prev_pos, mut vel, mass) in query.iter_mut() {
        prev_pos.0 = pos.0;

        let gravitation_force = mass.0 * gravity.0;
        let external_forces = gravitation_force;
        vel.0 += DELTA_TIME * external_forces / mass.0;
        pos.0 += DELTA_TIME * vel.0;
    }
}

fn solve_pos() {}

fn update_vel(mut query: Query<(&Pos, &PrevPos, &mut Vel)>) {
    for (pos, prev_pos, mut vel) in query.iter_mut() {
        vel.0 = (pos.0 - prev_pos.0) / DELTA_TIME;
    }
}

fn solve_vel() {}

/// Copies positions from the physics world to bevy Transforms
fn sync_transforms(mut query: Query<(&mut Transform, &Pos)>) {
    for (mut transform, pos) in query.iter_mut() {
        transform.translation = pos.0.extend(0.);
    }
}

#[derive(Debug, Default)]
pub struct XPBDPlugin;

impl Plugin for XPBDPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Gravity>();
        app.add_stage_before(
            CoreStage::Update,
            FixedUpdateStage,
            SystemStage::parallel()
                .with_run_criteria(FixedTimestep::step(DELTA_TIME as f64))
                .with_system(
                    collect_collision_pairs
                        .label(Step::CollectCollisionPairs)
                        .before(Step::Integrate),
                )
                .with_system(integrate.label(Step::Integrate))
                .with_system(solve_pos.label(Step::SolvePositions).after(Step::Integrate))
                .with_system(
                    update_vel
                        .label(Step::UpdateVelocities)
                        .after(Step::SolvePositions),
                )
                .with_system(
                    solve_vel
                        .label(Step::SolveVelocities)
                        .after(Step::UpdateVelocities),
                )
                .with_system(sync_transforms.after(Step::SolveVelocities)),
        );
    }
}