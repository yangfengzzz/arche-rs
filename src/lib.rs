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
        ))
        .insert(Mass(3.));

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
        ))
        .insert(Mass(1.));

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

fn integrate(
    mut query: Query<(&mut Pos, &mut PrevPos, &mut Vel, &mut PreSolveVel, &Mass)>,
    gravity: Res<Gravity>,
) {
    for (mut pos, mut prev_pos, mut vel, mut pre_solve_vel, mass) in query.iter_mut() {
        prev_pos.0 = pos.0;

        let gravitation_force = mass.0 * gravity.0;
        let external_forces = gravitation_force;
        vel.0 += DELTA_TIME * external_forces / mass.0;
        pos.0 += DELTA_TIME * vel.0;
        pre_solve_vel.0 = vel.0; // <-- new
    }
}

fn solve_pos(
    mut query: Query<(Entity, &mut Pos, &CircleCollider, &Mass)>,
    mut contacts: ResMut<Contacts>,
) {
    contacts.0.clear();
    let mut iter = query.iter_combinations_mut();
    while let Some(
        [(entity_a, mut pos_a, circle_a, mass_a), (entity_b, mut pos_b, circle_b, mass_b)],
    ) = iter.fetch_next()
    {
        let ab = pos_b.0 - pos_a.0;
        let combined_radius = circle_a.radius + circle_b.radius;
        let ab_sqr_len = ab.length_squared();
        if ab_sqr_len < combined_radius * combined_radius {
            contacts.0.push((entity_a, entity_b));
            let ab_length = ab_sqr_len.sqrt();
            let penetration_depth = combined_radius - ab_length;
            let n = ab / ab_length;

            let w_a = 1. / mass_a.0;
            let w_b = 1. / mass_b.0;
            let w_sum = w_a + w_b;

            pos_a.0 -= n * penetration_depth * w_a / w_sum;
            pos_b.0 += n * penetration_depth * w_b / w_sum;
        }
    }
}

fn solve_vel(
    query: Query<(&mut Vel, &PreSolveVel, &Pos, &Mass, &Restitution)>,
    contacts: Res<Contacts>,
) {
    for (entity_a, entity_b) in contacts.0.iter().cloned() {
        let (
            (mut vel_a, pre_solve_vel_a, pos_a, mass_a, restitution_a),
            (mut vel_b, pre_solve_vel_b, pos_b, mass_b, restitution_b),
        ) = unsafe {
            // Ensure safety
            assert!(entity_a != entity_b);
            (
                query.get_unchecked(entity_a).unwrap(),
                query.get_unchecked(entity_b).unwrap(),
            )
        };
        let n = (pos_b.0 - pos_a.0).normalize();
        let pre_solve_relative_vel = pre_solve_vel_a.0 - pre_solve_vel_b.0;
        let pre_solve_normal_vel = Vec2::dot(pre_solve_relative_vel, n);

        let relative_vel = vel_a.0 - vel_b.0;
        let normal_vel = Vec2::dot(relative_vel, n);
        let restitution = (restitution_a.0 + restitution_b.0) / 2.;

        let w_a = 1. / mass_a.0;
        let w_b = 1. / mass_b.0;
        let w_sum = w_a + w_b;

        vel_a.0 += n * (-normal_vel - restitution * pre_solve_normal_vel) * w_a / w_sum;
        vel_b.0 -= n * (-normal_vel - restitution * pre_solve_normal_vel) * w_b / w_sum;
    }
}

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
        app.init_resource::<Gravity>()
            .init_resource::<Contacts>()
            .add_stage_before(
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
                    solve_vel
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