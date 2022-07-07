use bevy::{core::FixedTimestep, prelude::*};
use arche_rs::*;
use rand::random;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.8, 0.8, 0.9)))
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(XPBDPlugin::default())
        .add_startup_system(startup)
        .add_system_set(
            SystemSet::new()
                .with_run_criteria(FixedTimestep::step(1. / 2.))
                .with_system(spawn_boxes),
        )
        .add_system(despawn_boxes)
        .run();
}

struct Materials {
    blue: Handle<StandardMaterial>,
}

struct Meshes {
    quad: Handle<Mesh>,
}

fn startup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let blue = materials.add(StandardMaterial {
        base_color: Color::rgb(0.4, 0.4, 0.6),
        unlit: true,
        ..Default::default()
    });

    let quad = meshes.add(Mesh::from(shape::Quad::new(Vec2::ONE)));

    let size = Vec2::new(10., 2.);
    commands
        .spawn_bundle(PbrBundle {
            mesh: quad.clone(),
            material: blue.clone(),
            transform: Transform::from_scale(size.extend(1.)),
            ..Default::default()
        })
        .insert_bundle(StaticBoxBundle {
            pos: Pos(Vec2::new(0., -3.)),
            rot: Rot::from_degrees(0.),
            collider: BoxCollider { size },
            ..Default::default()
        });

    commands.insert_resource(Meshes { quad });
    commands.insert_resource(Materials { blue });

    commands.spawn_bundle(OrthographicCameraBundle {
        transform: Transform::from_translation(Vec3::new(0., 0., 100.)),
        orthographic_projection: bevy::render::camera::OrthographicProjection {
            scale: 0.01,
            ..Default::default()
        },
        ..OrthographicCameraBundle::new_3d()
    });
}

fn spawn_boxes(mut commands: Commands, materials: Res<Materials>, meshes: ResMut<Meshes>) {
    let size = Vec2::splat(0.3);
    let pos = Vec2::new(random::<f32>() - 0.5, random::<f32>() - 0.5) * 0.5 + Vec2::Y * 3.;
    let vel = Vec2::new(random::<f32>() - 0.5, random::<f32>() - 0.5);
    let rot = random::<Rot>();
    let ang_vel = random::<f32>() * 2. - 1.;
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.quad.clone(),
            material: materials.blue.clone(),
            transform: Transform {
                scale: size.extend(1.),
                translation: pos.extend(0.),
                rotation: rot.into(),
                ..Default::default()
            },
            ..Default::default()
        })
        .insert_bundle(DynamicBoxBundle {
            collider: BoxCollider { size },
            ..DynamicBoxBundle::new_with_pos_and_vel_and_rot_and_ang_vel(pos, vel, rot, ang_vel)
        });
}

fn despawn_boxes(mut commands: Commands, query: Query<(Entity, &Pos)>) {
    for (entity, pos) in query.iter() {
        if pos.0.y < -20. {
            commands.entity(entity).despawn();
        }
    }
}