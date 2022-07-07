use bevy::prelude::*;
use arche_rs::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(XPBDPlugin::default())
        .insert_resource(Gravity(Vec2::ZERO))
        .add_startup_system(startup)
        .run();
}

fn startup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let white = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        ..Default::default()
    });

    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Quad::new(Vec2::ONE))),
            material: white.clone(),
            ..Default::default()
        })
        .insert_bundle(DynamicBoxBundle {
            rot: Rot::from_degrees(45.),
            ..Default::default()
        })
        .insert(Mass(1.));

    commands.spawn_bundle(OrthographicCameraBundle {
        transform: Transform::from_translation(Vec3::new(0., 0., 100.)),
        orthographic_projection: bevy::render::camera::OrthographicProjection {
            scale: 0.01,
            ..Default::default()
        },
        ..OrthographicCameraBundle::new_3d()
    });
}