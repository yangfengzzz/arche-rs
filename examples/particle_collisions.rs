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
