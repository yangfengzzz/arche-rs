mod components;
mod entity;
mod resources;

pub use components::*;
pub use entity::*;
pub use resources::*;

use bevy::{core::FixedTimestep, prelude::*};

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

pub const SUBSTEPS: i32 = 10;
pub const DELTA_TIME: f32 = 1. / 60. / SUBSTEPS as f32;

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

fn collect_collision_pairs(
    query: Query<(Entity, &Pos, &CircleCollider)>,
    mut collision_pairs: ResMut<CollisionPairs>,
) {
    collision_pairs.0.clear();
    unsafe {
        for (entity_a, pos_a, circle_a) in query.iter_unsafe() {
            for (entity_b, pos_b, circle_b) in query.iter_unsafe() {
                // Ensure safety
                if entity_a <= entity_b {
                    continue;
                }

                let ab = pos_b.0 - pos_a.0;
                let combined_radius = circle_a.radius + circle_b.radius;

                let ab_sqr_len = ab.length_squared();
                if ab_sqr_len < combined_radius * combined_radius {
                    collision_pairs.0.push((entity_a, entity_b));
                }
            }
        }
    }
}

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

fn clear_contacts(mut contacts: ResMut<Contacts>, mut static_contacts: ResMut<StaticContacts>) {
    contacts.0.clear();
    static_contacts.0.clear();
}

fn solve_pos(
    query: Query<(&mut Pos, &CircleCollider, &Mass)>,
    collision_pairs: Res<CollisionPairs>,
) {
    for (entity_a, entity_b) in collision_pairs.0.iter() {
        let (
            (mut pos_a, circle_a, mass_a),
            (mut pos_b, circle_b, mass_b),
        ) = unsafe {
            assert_ne!(entity_a, entity_b); // Ensure we don't violate memory constraints
            (
                query.get_unchecked(*entity_a).unwrap(),
                query.get_unchecked(*entity_b).unwrap(),
            )
        };
        let ab = pos_b.0 - pos_a.0;
        let combined_radius = circle_a.radius + circle_b.radius;
        let ab_sqr_len = ab.length_squared();
        if ab_sqr_len < combined_radius * combined_radius {
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

fn solve_pos_statics(
    mut dynamics: Query<(Entity, &mut Pos, &CircleCollider), With<Mass>>,
    statics: Query<(Entity, &Pos, &CircleCollider), Without<Mass>>,
    mut contacts: ResMut<StaticContacts>,
) {
    for (entity_a, mut pos_a, circle_a) in dynamics.iter_mut() {
        for (entity_b, pos_b, circle_b) in statics.iter() {
            let ab = pos_b.0 - pos_a.0;
            let combined_radius = circle_a.radius + circle_b.radius;
            let ab_sqr_len = ab.length_squared();
            if ab_sqr_len < combined_radius * combined_radius {
                let ab_length = ab_sqr_len.sqrt();
                let penetration_depth = combined_radius - ab_length;
                let n = ab / ab_length;
                pos_a.0 -= n * penetration_depth;
                contacts.0.push((entity_a, entity_b, n));
            }
        }
    }
}

fn solve_pos_static_boxes(
    mut dynamics: Query<(Entity, &mut Pos, &CircleCollider), With<Mass>>,
    statics: Query<(Entity, &Pos, &BoxCollider), Without<Mass>>,
    mut contacts: ResMut<StaticContacts>,
) {
    for (entity_a, mut pos_a, circle_a) in dynamics.iter_mut() {
        for (entity_b, pos_b, box_b) in statics.iter() {
            let box_to_circle = pos_a.0 - pos_b.0;
            let box_to_circle_abs = box_to_circle.abs();
            let half_extents = box_b.size / 2.;
            let corner_to_center = box_to_circle_abs - half_extents;
            let r = circle_a.radius;
            if corner_to_center.x > r || corner_to_center.y > r {
                continue;
            }

            let s = box_to_circle.signum();

            let (n, penetration_depth) = if corner_to_center.x > 0. && corner_to_center.y > 0. {
                // Corner case
                let corner_to_center_sqr = corner_to_center.length_squared();
                if corner_to_center_sqr > r * r {
                    continue;
                }
                let corner_dist = corner_to_center_sqr.sqrt();
                let penetration_depth = r - corner_dist;
                let n = corner_to_center / corner_dist * -s;
                (n, penetration_depth)
            } else if corner_to_center.x > corner_to_center.y {
                // Closer to vertical edge
                (Vec2::X * -s.x, -corner_to_center.x + r)
            } else {
                (Vec2::Y * -s.y, -corner_to_center.y + r)
            };

            pos_a.0 -= n * penetration_depth;
            contacts.0.push((entity_a, entity_b, n));
        }
    }
}

fn solve_vel(
    query: Query<(&mut Vel, &PreSolveVel, &Mass, &Restitution)>,
    contacts: Res<Contacts>,
) {
    for (entity_a, entity_b, n) in contacts.0.iter().cloned() {
        let (
            (mut vel_a, pre_solve_vel_a, mass_a, restitution_a),
            (mut vel_b, pre_solve_vel_b, mass_b, restitution_b),
        ) = unsafe {
            // Ensure safety
            assert_ne!(entity_a, entity_b);
            (
                query.get_unchecked(entity_a).unwrap(),
                query.get_unchecked(entity_b).unwrap(),
            )
        };
        let pre_solve_relative_vel = pre_solve_vel_a.0 - pre_solve_vel_b.0;
        let pre_solve_normal_vel = Vec2::dot(pre_solve_relative_vel, n);

        let relative_vel = vel_a.0 - vel_b.0;
        let normal_vel = Vec2::dot(relative_vel, n);
        let restitution = (restitution_a.0 + restitution_b.0) / 2.;

        let w_a = 1. / mass_a.0;
        let w_b = 1. / mass_b.0;
        let w_sum = w_a + w_b;

        let restitution_velocity = (-restitution * pre_solve_normal_vel).min(0.);
        let vel_impulse = n * ((-normal_vel + restitution_velocity) / w_sum);

        vel_a.0 += vel_impulse * w_a;
        vel_b.0 -= vel_impulse * w_b;
    }
}

fn solve_vel_statics(
    mut dynamics: Query<(&mut Vel, &PreSolveVel, &Restitution), With<Mass>>,
    statics: Query<&Restitution, Without<Mass>>,
    contacts: Res<StaticContacts>,
) {
    for (entity_a, entity_b, n) in contacts.0.iter().cloned() {
        let (mut vel_a, pre_solve_vel_a, restitution_a) = dynamics.get_mut(entity_a).unwrap();
        let restitution_b = statics.get(entity_b).unwrap();
        let pre_solve_normal_vel = Vec2::dot(pre_solve_vel_a.0, n);
        let normal_vel = Vec2::dot(vel_a.0, n);
        let restitution = (restitution_a.0 + restitution_b.0) / 2.;
        vel_a.0 += n * (-normal_vel + (-restitution * pre_solve_normal_vel).min(0.));
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
            .init_resource::<CollisionPairs>()
            .init_resource::<Contacts>()
            .init_resource::<StaticContacts>()
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
                    .with_system(clear_contacts.before(Step::SolvePositions))
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolvePositions)
                            .after(Step::Integrate)
                            .with_system(solve_pos)
                            .with_system(solve_pos_statics)
                            .with_system(solve_pos_static_boxes),
                    )
                    .with_system(
                        solve_vel
                            .label(Step::UpdateVelocities)
                            .after(Step::SolvePositions),
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolveVelocities)
                            .after(Step::UpdateVelocities)
                            .with_system(solve_vel)
                            .with_system(solve_vel_statics),
                    )
                    .with_system(sync_transforms.after(Step::SolveVelocities)),
            );
    }
}
