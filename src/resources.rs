use bevy::prelude::*;

#[derive(Default, Debug)]
pub struct Contacts(pub Vec<(Entity, Entity, Vec2)>);

#[derive(Default, Debug)]
pub struct StaticContacts(pub Vec<(Entity, Entity, Vec2)>);

#[derive(Debug, Default)]
pub(crate) struct CollisionPairs(pub Vec<(Entity, Entity)>);

#[derive(Debug)]
pub struct Gravity(pub Vec2);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec2::new(0., -9.81))
    }
}