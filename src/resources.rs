use bevy::prelude::*;

#[derive(Debug, Clone)]
pub struct BodyContact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub r_a: Vec2,
    pub r_b: Vec2,
    pub normal: Vec2,
}

#[derive(Default, Debug)]
pub struct Contacts(pub Vec<BodyContact>);

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