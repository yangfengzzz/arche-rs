mod components;
mod contact;
mod entity;
mod resources;
mod utils;
mod rotation;

pub use components::*;
pub use entity::*;
pub use resources::*;
pub use rotation::*;
use utils::*;
use contact::Contact;

use bevy::{prelude::*, ecs::schedule::*};

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

pub const DELTA_TIME: f32 = 1. / 60.;
pub const NUM_SUBSTEPS: u32 = 10;
pub const SUB_DT: f32 = DELTA_TIME / NUM_SUBSTEPS as f32;

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
    query: Query<(Entity, &Aabb)>,
    mut collision_pairs: ResMut<CollisionPairs>,
) {
    collision_pairs.0.clear();

    unsafe {
        for (entity_a, aabb_a) in query.iter_unsafe() {
            for (entity_b, aabb_b) in query.iter_unsafe() {
                // Ensure safety
                if entity_a <= entity_b {
                    continue;
                }
                if aabb_a.intersects(aabb_b) {
                    collision_pairs.0.push((entity_a, entity_b));
                }
            }
        }
    }
}

#[derive(Default)]
pub struct LoopState {
    pub(crate) has_added_time: bool,
    pub(crate) accumulator: f32,
    pub(crate) substepping: bool,
    pub(crate) current_substep: u32,
    pub(crate) queued_steps: u32,
    pub paused: bool,
}

impl LoopState {
    pub fn step(&mut self) {
        self.queued_steps += 1;
    }
    pub fn pause(&mut self) {
        self.paused = true;
    }
    pub fn resume(&mut self) {
        self.paused = false;
    }
}

pub fn pause(mut loop_state: ResMut<LoopState>) {
    loop_state.pause();
}

pub fn resume(mut loop_state: ResMut<LoopState>) {
    loop_state.resume();
}

fn run_criteria(time: Res<Time>, mut state: ResMut<LoopState>) -> ShouldRun {
    if !state.has_added_time {
        state.has_added_time = true;
        state.accumulator += time.delta_seconds();
    }

    if state.substepping {
        state.current_substep += 1;

        if state.current_substep < NUM_SUBSTEPS {
            return ShouldRun::YesAndCheckAgain;
        } else {
            // We finished a whole step
            state.accumulator -= DELTA_TIME;
            state.current_substep = 0;
            state.substepping = false;
        }
    }

    if state.accumulator >= DELTA_TIME {
        state.substepping = true;
        state.current_substep = 0;
        ShouldRun::YesAndCheckAgain
    } else {
        state.has_added_time = false;
        ShouldRun::No
    }
}

fn first_substep(state: Res<LoopState>) -> ShouldRun {
    if state.current_substep == 0 {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn last_substep(state: Res<LoopState>) -> ShouldRun {
    if state.current_substep == NUM_SUBSTEPS - 1 {
        ShouldRun::Yes
    } else {
        ShouldRun::No
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
        vel.0 += SUB_DT * external_forces / mass.0;
        pos.0 += SUB_DT * vel.0;
        pre_solve_vel.0 = vel.0;
    }
}

fn integrate_rot(mut query: Query<(&mut Rot, &mut PrevRot)>) {
    for (mut rot, mut prev_rot) in query.iter_mut() {
        prev_rot.0 = *rot;
        rot += SUB_DT * ang_vel.0;
    }
}

fn update_vel(mut query: Query<(&Pos, &PrevPos, &mut Vel)>) {
    for (pos, prev_pos, mut vel) in query.iter_mut() {
        vel.0 = (pos.0 - prev_pos.0) / SUB_DT;
    }
}

fn update_ang_vel(mut query: Query<(&Rot, &PrevRot, &mut AngVel)>) {
    for (rot, prev_rot, mut ang_vel) in query.iter_mut() {
        ang_vel.0 = (prev_rot.0.inv().mul(*rot)).as_radians() / SUB_DT;
    }
}

fn update_aabb_circle(mut query: Query<(&mut Aabb, &Pos, &CircleCollider)>) {
    for (mut aabb, pos, circle) in query.iter_mut() {
        let half_extents = Vec2::splat(circle.radius);
        aabb.min = pos.0 - half_extents;
        aabb.max = pos.0 + half_extents;
    }
}

fn update_aabb_box(mut query: Query<(&mut Aabb, &Pos, &Rot, &Vel, &BoxCollider)>) {
    for (mut aabb, pos, rot, vel, r#box) in query.iter_mut() {
        let sin = rot.sin().abs();
        let cos = rot.cos().abs();
        let box_w = r#box.size.x;
        let box_h = r#box.size.y;
        let w = box_h * sin + box_w * cos;
        let h = box_w * sin + box_h * cos;
        let margin = COLLISION_PAIR_VEL_MARGIN_FACTOR * vel.0.length();
        let half_extents = Vec2::new(w / 2., h / 2.) + Vec2::splat(margin);
        aabb.min = pos.0 - half_extents;
        aabb.max = pos.0 + half_extents;
    }
}

/// Solves overlap between two dynamic bodies according to their masses
fn constrain_body_positions(
    pos_a: &mut Pos,
    pos_b: &mut Pos,
    mass_a: &Mass,
    mass_b: &Mass,
    n: Vec2,
    penetration_depth: f32,
) {
    let w_a = 1. / mass_a.0;
    let w_b = 1. / mass_b.0;
    let w_sum = w_a + w_b;
    let pos_impulse = n * (-penetration_depth / w_sum);
    pos_a.0 += pos_impulse * w_a;
    pos_b.0 -= pos_impulse * w_b;
}

/// Solve a overlap between a dynamic object and a static object
fn constrain_body_position(pos: &mut Pos, normal: Vec2, penetration_depth: f32) {
    pos.0 -= normal * penetration_depth;
}

fn solve_pos(
    mut query: Query<(&mut Pos, &CircleCollider, &Mass)>,
    mut contacts: ResMut<Contacts>,
    collision_pairs: Res<CollisionPairs>,
) {
    debug!("  solve_pos");
    for (entity_a, entity_b) in collision_pairs.0.iter().cloned() {
        if let Ok((
                      (mut pos_a, circle_a, mass_a),
                      (mut pos_b, circle_b, mass_b))
        ) = query.get_pair_mut(entity_a, entity_b) {
            if let Some(Contact {
                            normal,
                            penetration,
                        }) = contact::ball_ball(pos_a.0, circle_a.radius, pos_b.0, circle_b.radius)
            {
                constrain_body_positions(&mut pos_a, &mut pos_b, mass_a, mass_b, normal, penetration);
                contacts.0.push((entity_a, entity_b, normal));
            }
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
            if let Some(Contact {
                            normal,
                            penetration,
                        }) = contact::ball_ball(pos_a.0, circle_a.radius, pos_b.0, circle_b.radius)
            {
                constrain_body_position(&mut pos_a, normal, penetration);
                contacts.0.push((entity_a, entity_b, normal));
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
            if let Some(Contact {
                            normal,
                            penetration,
                        }) = contact::ball_box(pos_a.0, circle_a.radius, pos_b.0, box_b.size)
            {
                constrain_body_position(&mut pos_a, normal, penetration);
                contacts.0.push((entity_a, entity_b, normal));
            }
        }
    }
}

fn solve_pos_box_box(
    mut query: Query<(&mut Pos, &Rot, &BoxCollider, &Mass)>,
    mut contacts: ResMut<Contacts>,
    collision_pairs: Res<CollisionPairs>,
) {
    for (entity_a, entity_b) in collision_pairs.0.iter().cloned() {
        if let Ok(((mut pos_a, rot_a, box_a, mass_a), (mut pos_b, rot_b, box_b, mass_b))) =
        query.get_pair_mut(entity_a, entity_b)
        {
            if let Some(Contact {
                            normal,
                            penetration,
                        }) = contact::box_box(pos_a.0, *rot_a, box_a.size,
                                              pos_b.0, *rot_b, box_b.size)
            {
                constrain_body_positions(
                    &mut pos_a,
                    &mut pos_b,
                    mass_a,
                    mass_b,
                    normal,
                    penetration,
                );
                contacts.0.push((entity_a, entity_b, normal));
            }
        }
    }
}

fn solve_pos_static_box_box(
    mut dynamics: Query<(Entity, &mut Pos, &Rot, &BoxCollider), With<Mass>>,
    statics: Query<(Entity, &Pos, &Rot, &BoxCollider), Without<Mass>>,
    mut contacts: ResMut<StaticContacts>,
) {
    for (entity_a, mut pos_a, rot_a, box_a) in dynamics.iter_mut() {
        for (entity_b, pos_b, rot_b, box_b) in statics.iter() {
            if let Some(Contact {
                            normal,
                            penetration,
                        }) = contact::box_box(pos_a.0, *rot_a, box_a.size, pos_b.0, *rot_b, box_b.size)
            {
                constrain_body_position(&mut pos_a, normal, penetration);
                contacts.0.push((entity_a, entity_b, normal));
            }
        }
    }
}

fn solve_vel(
    mut query: Query<(&mut Vel, &PreSolveVel, &Mass, &Restitution)>,
    contacts: Res<Contacts>,
) {
    for (entity_a, entity_b, n) in contacts.0.iter().cloned() {
        let (
            (mut vel_a, pre_solve_vel_a, mass_a, restitution_a),
            (mut vel_b, pre_solve_vel_b, mass_b, restitution_b),
        ) = query.get_pair_mut(entity_a, entity_b).unwrap();
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

/// Copies positions and rotations from the physics world to bevy Transforms
fn sync_transforms(mut query: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in query.iter_mut() {
        transform.translation = pos.0.extend(0.);
        transform.rotation = (*rot).into();
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
            .init_resource::<LoopState>()
            .add_stage_before(
                CoreStage::Update,
                FixedUpdateStage,
                SystemStage::parallel()
                    .with_run_criteria(run_criteria)
                    .with_system_set(
                        SystemSet::new()
                            .before(Step::CollectCollisionPairs)
                            .with_system(update_aabb_box)
                            .with_system(update_aabb_circle)
                    )
                    .with_system(
                        collect_collision_pairs
                            .with_run_criteria(first_substep)
                            .label(Step::CollectCollisionPairs)
                            .before(Step::Integrate),
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::Integrate)
                            .with_system(integrate)
                            .with_system(integrate_rot),
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolvePositions)
                            .after(Step::Integrate)
                            .with_system(solve_pos)
                            .with_system(solve_pos_box_box)
                            .with_system(solve_pos_statics)
                            .with_system(solve_pos_static_boxes)
                            .with_system(solve_pos_static_box_box),
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::UpdateVelocities)
                            .after(Step::SolvePositions)
                            .with_system(update_vel)
                            .with_system(update_ang_vel)
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolveVelocities)
                            .after(Step::UpdateVelocities)
                            .with_system(solve_vel)
                            .with_system(solve_vel_statics)
                    )
                    .with_system(
                        sync_transforms
                            .with_run_criteria(last_substep)
                            .after(Step::SolveVelocities),
                    ),
            );
    }
}
